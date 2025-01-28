

#include "MagSens.h"

#include "sensor.h"
#include "nmea_util.h"
#include "comm/Messages.h"
#include "comm/DataLink.h"
// #include "SetupNG.h"
#include "Version.h"
#include "logdef.h"

// MagSens NMEA protocol is just a simple one. Those queries are supported:
// - Hello and version query:
//   $PMSH\r\n
//
// - The stream request: (to be compatible a stream is picked up straight after boot-up)
//   $PMSS<stream_type=r/c/..>\r\n
//
// - Transfer new calibration:
//   $PMSC, ... *<CRC>\r\n
//
// - Kill stream request:
//   $PMSK\r\n
//
// - Anounce firmware update:
//   $PMSU, <length>, <packet_size>*<CRC>\r\n
//
// The MagSens responses
// - Version:
//   $PMSV, <release_number>, <build_dateandtime>\r\n
//
// - Firmware packet confirmation:
//   $PMSC, <enum>\r\n

datalink_action_t MagSens::nextByte(const char c)
{
    int pos = _sm._frame.size() - 1; // c already in the buffer
    datalink_action_t ret = NOACTION;
    ESP_LOGD(FNAME, "state %d, pos %d next char %c", _sm._state, pos, c);
    switch(_sm._state) {
    case START_TOKEN:
        if ( c == '$' ) { // 0x24
            _sm._state = HEADER;
            ESP_LOGD(FNAME, "Msg START_TOKEN");
        }
        break;
    case HEADER:
        NMEA::incrCRC(_sm._crc,c);
        if ( pos < 3 ) { break; }
        if ( _sm._frame.substr(1,3) != "PMS" ) {
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
        break;
    }
    case STOP_TOKEN:
    case COMPLETE:
    {
        _sm._state = START_TOKEN; // restart parsing
        ESP_LOGI(FNAME, "Msg complete %s", _sm._frame.c_str());
        switch (_sm._frame[4]) {
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
    if ( _updateHandle ) {
        if ( _buff_fill + count > _updPackSize ) {
            _packEnum++;
            if ( esp_ota_write(_updateHandle, _uptBuffer, _buff_fill) ) {
                confirmPacket(_packEnum);
            }
            _buff_fill = 0;
        }
        memcpy(_uptBuffer+_buff_fill, cptr, count);
        _buff_fill += count;
        _bytesReceived += count;
        if ( (_bytesReceived % 300) == 0 ) {
            ESP_LOGI(FNAME, "received %d", _bytesReceived);
        }
    }
    if ( _bytesReceived >= _updateSize ) {
        ESP_LOGI(FNAME, "Received complete update.");
        if ( _buff_fill > 0 ) {
            esp_ota_write(_updateHandle, _uptBuffer, _buff_fill);
        }
        if (esp_ota_end(_updateHandle) == ESP_OK)
        {
            _updateHandle = 0;
            // Lets update the partition
            if (esp_ota_set_boot_partition(_updatePartition) == ESP_OK)
            {
                const esp_partition_t *boot_partition = esp_ota_get_boot_partition();

                ESP_LOGI(FNAME, "Next boot partition subtype %d at offset 0x%x", boot_partition->subtype, boot_partition->address);

            }
            else
            {
                ESP_LOGE(FNAME, "\r\n\r\n !!! Flashed Error !!!\r\n");
            }
        }
        ESP_LOGI(FNAME, "rebooting ..");
        delay(1000);
        esp_restart();
        // _binary = false;
        // return GO_NMEA;
    }
    return NOACTION;
}

void MagSens::Version()
{
    ESP_LOGI(FNAME,"PMS Version");
    ::Version myVersion;
    Message* msg = newMessage();

    msg->buffer = "$PMSV, " + std::to_string(Version::RELEASE_NR) + ", " + myVersion.version() + "\r\n";
    DEV::Send(msg);
}

void MagSens::parseCalibration()
{
    ESP_LOGI(FNAME,"PMS Calib");
    // Fixme: Content to come
}

void MagSens::startStream()
{
    ESP_LOGI(FNAME,"PMS Start Stream");
    if ( _sm._frame[5] == 'r' ) {
        stream_status = RAW_STREAM;
    }
    else if ( _sm._frame[5] == 'c' ) {
        stream_status = CALIBRATED;
    }
}

void MagSens::killStream()
{
    ESP_LOGI(FNAME,"PMS Kill Stream");
    stream_status = STREAM_OFF;
}

void MagSens::prepareUpdate()
{
    // $PMSU, %d, %d*
    stream_status = STREAM_OFF;
    
    // read size of update
    int pos = 5;
    _updateSize = std::stoi(NMEA::extractWord(_sm._frame, pos));
    _updPackSize = std::stoi(NMEA::extractWord(_sm._frame, pos));
    ESP_LOGI(FNAME,"PMS Update size %d", _updateSize);

    // Init the update
    _updatePartition = esp_ota_get_next_update_partition(NULL);
    ESP_LOGI(FNAME, "Next update prt.: %p", _updatePartition);
    _packEnum = 0;
    _bytesReceived = 0;
    _buff_fill = 0;
    esp_err_t res = esp_ota_begin(_updatePartition, _updateSize, &_updateHandle);
    if (res == ESP_OK) {
        _uptBuffer = (char *)malloc(_updPackSize);
    }
    else {
        ESP_LOGE(FNAME, "Failed to initiate update: %s", esp_err_to_name(res));
        _updateHandle = 0; // Let the bytes pass
    }
    // Switch to a binary version of this receiver for the next _updateSize bytes
    _binary = true;
    _dl.goBIN();
}

void MagSens::confirmPacket(int nr)
{
    // $PMSC, <enum>\r\n
    Message* msg = newMessage();
    ESP_LOGI(FNAME,"PMS confirm to did%D - %d", msg->target_id, nr);

    msg->buffer = "$PMSC, " + std::to_string(nr) + "\r\n";
    DEV::Send(msg);
}