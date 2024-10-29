

#include "MagSens.h"

#include "sensor.h"
#include "Router.h"
#include "SetupNG.h"
#include "Version.h"
#include <logdef.h>

// MagSens NMEA protocol is just a simple one. Those queries are supported:
// - Hello and version query:
//   $PMH\r\n
//
// - The stream request: ( to be compatible a raw stream is picked up after 2 minutes time-out)
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

gen_state_t MagSens::nextByte(const char c)
{
    ESP_LOGI(FNAME, "state %d, pos %d next char %c", _state, _pos, c);
    switch(_state) {
    case START_TOKEN:
    case CHECK_OK:
    case CHECK_FAILED:
    case ABORT:
        if ( c == '$' ) { // 0x24
            _state = HEADER;
            reset();
            push(c);
            _crc = 0;
        }
        ESP_LOGI(FNAME, "Msg START_TOKEN");
        break;
    case HEADER:
        if ( _pos == 1 && c != 'P' ) {
            _state = ABORT;
            break;
        }
        else if ( _pos == 2 && c != 'M' ) {
            _state = ABORT;
            break;
        }
        else if ( _pos == 3 ) {
            _state = PAYLOAD;
            ESP_LOGI(FNAME, "Msg HEADER");
        }
        else if ( _pos > 3 ) {
            _state = ABORT;
            break;
        }
        push(c);
        break;
    case PAYLOAD:
        if ( c == '*' ) {
            _state = CHECK_CRC; // Expecting a CRC to check
            break;
        }
        if ( c != '\r' && c != '\n' ) {
            ESP_LOGI(FNAME, "Msg PAYLOAD");
            if ( push(c) ) {
                break;
            }
            // Buffer is full
            _state = STOP_TOKEN;
        }
        // Fall through 
    case CHECK_CRC:
        if( _state == CHECK_CRC ) { // did we not fall through
            ESP_LOGI(FNAME, "Msg CRC %x - %x", c, _crc);
            if ( c == _crc ) {
                _state = CHECK_OK;
            }
            else {
                _state = CHECK_FAILED;
                break;
            }
        }
        // Fall through 
    case STOP_TOKEN:
    case COMPLETE:
        ESP_LOGI(FNAME, "Msg complete %c", _framebuffer[3]);
        switch (_framebuffer[3]) {
            case 'H':
                Version();
                break;
            case 'C':
                if ( _state == CHECK_OK ) {
                    parseCalibration();
                }
                break;
            case 'S':
                startStream();
                break;
            case 'K':
                killStream();
                break;
            case 'U':
                prepareUpdate();
                break;
            default:
                break;
        }
        _state = START_TOKEN; // no routing nor parsing wanted
        break;
    default:
        break;
    }
    return _state;
}

void MagSens::Version()
{
    ESP_LOGI(FNAME,"Pm Version");
    ::Version myVersion;
    char str[40];
    sprintf( str, "$PMV %d, %s\r\n", Version::RELEASE_NR, myVersion.version() );
    SString nmea(str);
    Router::forwardMsg( nmea, can_tx_q );
}

void MagSens::parseCalibration()
{
    ESP_LOGI(FNAME,"PM Calib");
    // Fixme: Content to come
}

void MagSens::startStream()
{
    ESP_LOGI(FNAME,"PM Start Stream");
    if ( _framebuffer[4] == 'r' ) {
        stream_status = RAW_STREAM;
    }
    else if ( _framebuffer[4] == 'c' ) {
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
    ESP_LOGI(FNAME,"PM Update");
    stream_status = STREAM_OFF;
    // Fixme, more to come
}

void MagSens::incrCRC(const char c)
{
    _crc ^= c;
}