

#include "MagSens.h"

#include "sensor.h"
#include "nmea_util.h"
#include "comm/Messages.h"
#include "SetupNG.h"
#include "Version.h"
#include "logdef.h"

// MagSens NMEA protocol is just a simple one. Those queries are supported:
// - Hello and version query:
//   $PMH\r\n
//
// - The stream request: (to be compatible a stream is picked up straight after boot-up)
//   $PMS<stream_type=r/c/..>\r\n
//
// - Transfer new calibration:
//   $PMC ... *<CRC>\r\n
//
// - Kill stream request:
//   $PMK\r\n
//
// - Anounce firmware update:
//   $PMU <length>*<CRC>\r\n
//
// The MagSens responses
// - Version:
//   $PMV <release_number>, <build_dateandtime>\r\n

datalink_action_t MagSens::nextByte(const char c)
{
    int pos = _sm._frame.size() - 1; // c already in the buffer
    datalink_action_t ret = NOACTION;
    ESP_LOGI(FNAME, "state %d, pos %d next char %c", _sm._state, pos, c);
    switch(_sm._state) {
    case START_TOKEN:
        if ( c == '$' ) { // 0x24
            _sm._state = HEADER;
            ESP_LOGI(FNAME, "Msg START_TOKEN");
        }
        break;
    case HEADER:
        NMEA::incrCRC(_sm._crc,c);
        if ( pos < 3 ) { break; }
        if ( _sm._frame.substr(1,2) != "PM" ) {
            _sm._state = START_TOKEN;
            break;
        }
        _sm._state = PAYLOAD;
        break;
    case PAYLOAD:
        if ( c == '*' ) {
            _sm._state = CHECK_CRC1; // Expecting a CRC to check
            break;
        }
        if ( c != '\r' && c != '\n' ) {
            ESP_LOGI(FNAME, "Msg PAYLOAD");
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
        switch (_sm._frame[3]) {
            case 'H':
                Version();
                break;
            case 'C':
                parseCalibration();
                break;
            case 'S':
                startStream();
                break;
            case 'K':
                killStream();
                break;
            case 'U':
                prepareUpdate();
                ret = NXT_PROTO;
                break;
            default:
                break;
        }
        break;
    }
    default:
        break;
    }
    return ret;
}

datalink_action_t MagSens::nextStreamChunk(const char *cptr, int count)
{
    return NOACTION;
}

void MagSens::Version()
{
    ESP_LOGI(FNAME,"Pm Version");
    ::Version myVersion;
    Message* msg = newMessage();

    msg->buffer = "$PMV " + std::to_string(Version::RELEASE_NR) + ", " + myVersion.version() + "\r\n";
    DEV::Send(msg);
}

void MagSens::parseCalibration()
{
    ESP_LOGI(FNAME,"PM Calib");
    // Fixme: Content to come
}

void MagSens::startStream()
{
    ESP_LOGI(FNAME,"PM Start Stream");
    if ( _sm._frame[4] == 'r' ) {
        stream_status = RAW_STREAM;
    }
    else if ( _sm._frame[4] == 'c' ) {
        stream_status = CALIBRATED;
    }
}

void MagSens::killStream()
{
    ESP_LOGI(FNAME,"PM Kill Stream");
    stream_status = STREAM_OFF;
}

void MagSens::prepareUpdate()
{
    stream_status = STREAM_OFF;
    
    sscanf(_sm._frame.c_str(), "$PMU %d*", &_updateSize);
    ESP_LOGI(FNAME,"PM Update size %d", _updateSize);
}

