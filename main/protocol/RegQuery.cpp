/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "RegQuery.h"

#include "MagSens.h"
#include "nmea_util.h"
#include "Clock.h"
#include "comm/DeviceMgr.h"
#include "comm/Messages.h"
#include "Version.h"
#include "SetupNG.h"
#include "sensor.h"
#include "logdef.h"

#include <cstring>
#include <string>


extern InterfaceCtrl* CAN;


// The CAn device regitration query protocol. Those commands/queries are supported:
//
// - Registration query: Send a request to a potential peer on the bus grabbing the master role,
//      and deleivers the expected registration response. Until a peer responds the drive will continue sending a query 
//      every second.
//      As long as no registration to a master is established the drive can only be commanded through the physical
//      buttons.
//      The request is sent on a high id to give precedence to all other traffic on the bus. The registration is not time 
//      critical. The registration query is sent on the CAN id=0x7f0. (== b0111 1111 0000, the ones are recessive on CAN and thus
//      most other telegrams will have precedence). The master response is expected on the same id.
//      The request is equipped with a unique token that consists of an arbitrary sequence of three characters. It is used as identification 
//      of the masters response, because all client registration are happening on the same CAN id.
//      In case multiple peers respond with different port/id information, the information of the first valid response will be used.
//      An optional secondary master should just pick up the id information from the primary master and would be able to command the 
//      drive as well.
//      Second parameter is a string with the letters "MAGSENS" to identify the desired protocol to talk, and thus the kind of devcie
//      requesting.
//   $PJPREG, <token>, <protocol type>\r\n
//
// The expected master response
// - Registration query accepted: A master respons with the query token, a drive id to listen, and a master id to respond messages to.
//   $PJMACC, <token>, <drive id>, <master id>*<CRC>\r\n
//

RegQuery::RegQuery(int mp, ProtocolState &sm, DataLink &dl)
  : ProtocolItf(DeviceId::MASTER_DEV, mp, sm, dl),
    Clock_I(200) // generates a time-out callback ca. every two seconds
{
    // Once stored random token
    char buf[20];
    sprintf(buf, "%03d", reg_token.get());
    int len = strlen(buf);
    strcpy(Q_TOKEN, &buf[len-3]);

    Clock::start(this); // register query kicker time-out
    ESP_LOGI(FNAME, "time %ldmsec", Clock::getMillis());
}

RegQuery::~RegQuery()
{
    Clock::stop(this); // deregister time-out
}

datalink_action_t RegQuery::nextByte(const char c)
{
    int pos = _sm._frame.size() - 1; // c already in the buffer
    datalink_action_t ret = NOACTION;
    ESP_LOGD(FNAME, "state %d, pos %d next char %c", _sm._state, pos, c);
    switch(_sm._state) {
    case START_TOKEN:
        if ( c == '$' ) {
            _sm._state = HEADER;
            ESP_LOGD(FNAME, "Msg START_TOKEN");
        }
        break;
    case HEADER:
        NMEA::incrCRC(_sm._crc,c);
        if ( pos < 3 ) { break; }
        if ( _sm._frame.substr(1,3) != "PJM" ) {
            _sm._state = START_TOKEN;
            break;
        }
        _sm._state = PAYLOAD;
        ESP_LOGD(FNAME, "Msg HEADER");
        break;
    case PAYLOAD:
        if ( c == '*' ) {
            _sm._state = CHECK_CRC1; // Expecting a CRC to check
            break;
        }
        if ( c != '\r' && c != '\n' ) {
            ESP_LOGD(FNAME, "Msg PAYLOAD");
            NMEA::incrCRC(_sm._crc,c);
            break;
        }
        _sm._state = COMPLETE;
        break;
    case CHECK_CRC1:
        _crc_buf[0] = c;
        _sm._state = CHECK_CRC2;
        break;
    case CHECK_CRC2:
    {
        _crc_buf[1] = c;
        _crc_buf[2] = '\0';
        char read_crc = (char)strtol(_crc_buf, NULL, 16);
        ESP_LOGD(FNAME, "Msg CRC %s/%x - %x", _crc_buf, read_crc, _sm._crc);
        if ( read_crc != _sm._crc ) {
            _sm._state = START_TOKEN;
            break;
        }
        _sm._state = COMPLETE;
        // Fall through 
    }
    case STOP_TOKEN:
    case COMPLETE:
    {
        _sm._state = START_TOKEN; // restart parsing
        ESP_LOGI(FNAME, "Msg complete %s", _sm._frame.c_str());
        if ( _sm._frame.compare(4, 3, "ACC") == 0 ) {
            ret = registration();
        }
        else if ( _sm._frame.compare(4, 3, "NAC") == 0 ) {
            // lol
            ESP_LOGI(FNAME, "Received not accepted");
        }
        break;
    }
    default:
        break;
    }
    return ret;
}

datalink_action_t RegQuery::registration()
{
    // grab token from e.g. message "$PJMACC, 123, j_id, m_id"
    if ( _sm._frame.size() < 12 ) {
        return NOACTION;
    }
    int pos = 8;
    std::string tail = NMEA::extractWord(_sm._frame, pos);
    ESP_LOGI(FNAME, "MS token >%s<", tail.c_str());

    // check on correct token
    if ( tail.compare(0, 3, Q_TOKEN) != 0 ) {
        return NOACTION;
    }

    // read the device id
    int c_id = std::stoi(NMEA::extractWord(_sm._frame, pos));

    // read the master id
    int m_id = std::stoi(NMEA::extractWord(_sm._frame, pos));

    // simple check
    if ( c_id > 0 && c_id < 0x7ff 
        && m_id > 0 && m_id < 0x7ff ) {
        // success
        ESP_LOGI(FNAME, "MS registered (CH:%d)", c_id);
        MAG = static_cast<MagSens*>(DEVMAN->addDevice(MASTER_DEV, MAGSENS_P, c_id, m_id, CAN_BUS));

        Clock::stop(this);
        return NOACTION;
    }
    else {
        ESP_LOGE(FNAME, "MS registration failed (ID=%d, MasterID=%d)", c_id, m_id);
    }
    return NOACTION;
}


//
// Transmitter routine
//
bool RegQuery::sendRegistrationQuery()
{
    Message* msg = newMessage();

    msg->buffer = "$PJPREG, ";
    msg->buffer += Q_TOKEN;
    msg->buffer += ", MAGSENS\r\n";
    ESP_LOGI(FNAME, "MS send request: %s", msg->buffer.c_str());
    return DEV::Send(msg);
}

void RegQuery::tick()
{
    if ( _nr_trials++ < MAX_NR_TRIALS ) {
        sendRegistrationQuery();
    }
    else {
        Clock::stop(this);
    }
}
