
#include "canbus.h"
#include "SetupNG.h"

#include "sensor.h"
#include "RingBufCPP.h"
#include "Router.h"
#include "QMC5883L.h"


#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#include <cstring>


/*
 *  Code for a 1:1 connection between two XCVario with a fixed message ID
 *
 *
 *
 */

int CANbus::_tick = 0;
volatile bool CANbus::_ready_initialized = false;
gpio_num_t CANbus::_tx_io = CAN_BUS_TX_PIN;
gpio_num_t CANbus::_rx_io = CAN_BUS_RX_PIN;
bool       CANbus::_connected;
int        CANbus::_connected_timeout;

static TaskHandle_t cpid;
static bool force_reconnect = false;

#define MSG_ID 0x555


// install/reinstall CAN driver in corresponding mode
void CANbus::driverInstall( twai_mode_t mode, bool other_speed ){
	if( _ready_initialized ){
        driverUninstall();
	}
	twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT( _tx_io, _rx_io, mode );
	twai_timing_config_t t_config;
    int speed = can_speed.get();
    if ( other_speed ) {
        speed = (speed+1)%CAN_SPEED_MAX;
        can_speed.set(speed, false);
    }
	if( speed == CAN_SPEED_250KBIT ){
		ESP_LOGI(FNAME,"CAN rate 250KBit");
		t_config = TWAI_TIMING_CONFIG_250KBITS();
	}
	else if( speed == CAN_SPEED_500KBIT ){
		ESP_LOGI(FNAME,"CAN rate 500KBit");
		t_config = TWAI_TIMING_CONFIG_500KBITS();
	}
	else if( speed == CAN_SPEED_1MBIT ){
		ESP_LOGI(FNAME,"CAN rate 1MBit");
		t_config = TWAI_TIMING_CONFIG_1MBITS();
	}
	else{
		ESP_LOGI(FNAME,"CAN rate 1MBit for selftest");
		t_config = TWAI_TIMING_CONFIG_1MBITS();
	}

	twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    //Install TWAI driver
	if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
		ESP_LOGI(FNAME,"Driver installed OK, mode %d", mode );
	} else {
		ESP_LOGI(FNAME,"Failed to install driver");
		return;
	}
	delay(50);
	//Start TWAI driver
	if (twai_start() == ESP_OK) {
		ESP_LOGI(FNAME,"Driver started");
        delay(100);
        _ready_initialized = true;
	} else {
        twai_driver_uninstall();
		ESP_LOGI(FNAME,"Failed to start driver");
	}
}

void CANbus::driverUninstall(){
    if( _ready_initialized ){
        _ready_initialized = false;
		twai_stop();
		delay(10);
		twai_driver_uninstall();
		delay(50);
	}
}

void canTask(void *pvParameters){
	while (true) {
		if ( CANbus::tick() ) {
    		delay(500);
        }
		if( (CANbus::_tick % 100) == 0) {
			// ESP_LOGI(FNAME,"Free Heap: %d bytes", heap_caps_get_free_size(MALLOC_CAP_8BIT) );
			if( uxTaskGetStackHighWaterMark( cpid ) < 128 )
				ESP_LOGW(FNAME,"Warning canbus task stack low: %d bytes", uxTaskGetStackHighWaterMark( cpid ) );
		}
        delay(50);
	}
}


// begin CANbus, start selfTest and launch driver in normal (bidir) mode afterwards
void CANbus::begin()
{
    //Initialize configuration structures using macro initializers
	ESP_LOGI(FNAME,"CANbus::begin");
	driverInstall( TWAI_MODE_NORMAL );

	xTaskCreatePinnedToCore(&canTask, "canTask", 4096, nullptr, 8, &cpid, 0);

}

// receive message of corresponding ID
int CANbus::receive( int *id, SString& msg, int timeout ){
	twai_message_t rx;
	esp_err_t ret = twai_receive(&rx, pdMS_TO_TICKS(timeout) );
	// ESP_LOGI(FNAME,"RX CAN bus message ret=%02x", ret );
	if(ret == ESP_OK ){
		if( rx.data_length_code > 0 && rx.data_length_code <= 8 ){
            msg.append((const char*)rx.data, rx.data_length_code);
			// ESP_LOGI(FNAME,"RX CAN bus message ret=(%02x), id:%04x bytes:%d, data:%s", ret, rx.identifier, rx.data_length_code, msg );
			*id = rx.identifier;
			return rx.data_length_code;
		}
	}
	return 0;
}

// hook this into another task to save memory
bool CANbus::tick(){
	_tick++;
	if( force_reconnect ){
		force_reconnect = false;
        driverInstall( TWAI_MODE_NORMAL, true );
        _connected_timeout = 0;
		return true;
	}
	if( ! _ready_initialized ){
		ESP_LOGI(FNAME,"CANbus not ready");
		return true;
	}
	static SString msg;
	// CAN bus send tick
	if ( !can_tx_q.isEmpty() ){
		// ESP_LOGI(FNAME,"There is CAN data");
		if( _connected ){
			if( Router::pullMsg( can_tx_q, msg ) ) {
				ESP_LOGI(FNAME,"CAN TX len: %d bytes", msg.length() );
				// ESP_LOG_BUFFER_HEXDUMP(FNAME,s.c_str(),s.length(), ESP_LOG_DEBUG);
				sendNMEA( msg.c_str() );
			}
		}
	}
	msg.clear();
	// Can bus receive tick
	int id = 0;
	int bytes = receive( &id, msg, 10 );
	// ESP_LOGI(FNAME,"CAN RX id:%02x, bytes:%d, connected:%d", id, bytes, _connected );
	if( bytes ){
		_connected_timeout = 0;
	}
	else{
		_connected_timeout++;
	}
    // receive message of corresponding ID
    static SString nmea;
	if( id == 0x20 ) {     // start of nmea
		// ESP_LOGI(FNAME,"CAN RX Start of frame");
		nmea = msg;
	}
	else if( id == 0x21 ){ // segment
		// ESP_LOGI(FNAME,"CAN RX frame segment");
		nmea += msg;
	}
	else if( id == 0x22 ){
		nmea += msg;
		// ESP_LOGI(FNAME,"CAN RX, msg: %s", nmea.c_str() );
		Router::forwardMsg( nmea, can_rx_q );
	}
	if( !(_tick%4) ) {
		Router::routeCAN();
    }
    return _connected;
}


bool CANbus::sendNMEA( const char *msg ){
	if( !_ready_initialized ){
		return false;
    }
	int len=strlen(msg);
	ESP_LOGI(FNAME,"send CAN NMEA len %d, msg: %s", len, msg );
	bool ret = true;
	int id = 0x20;
	int dlen=8;
	int pos;
	for( pos=0; pos < len; pos+=8 ){
		int rest = len - pos;
		if( !sendData( id, &msg[pos], dlen ) )
			ret = false;
		if( rest > 0 && rest <= 8 ){
			dlen = rest;
			break;
		}else{
			rest = 0;
		}
		// ESP_LOGI(FNAME,"Sent id:%d pos:%d dlen %d", id, pos, dlen );
		id = 0x21;
	}
	id = 0x22;
	// ESP_LOGI(FNAME,"Sent id:%d pos:%d dlen %d", id, pos, dlen );
	if( !sendData( id, &msg[pos], dlen ) )
		ret = false;

	return ret;
}


bool CANbus::selfTest(){
	ESP_LOGI(FNAME,"CAN bus selftest");
	driverInstall( TWAI_MODE_NO_ACK );
	bool res=false;
	int id=0x100;
	for( int i=0; i<10; i++ ){
		char tx[10] = { "18273645" };
		int len = strlen(tx);
		ESP_LOGI(FNAME,"strlen %d", len );
		twai_clear_receive_queue();  // there might be data from a remote device
		if( !sendData( id, tx,len, 1 ) ){
			ESP_LOGW(FNAME,"CAN bus selftest TX FAILED");
		}
		SString msg;
		int rxid;
		delay(10);
		int bytes = receive( &rxid, msg );
		ESP_LOGI(FNAME,"RX CAN bus message bytes:%d, id:%04x, data:%s", bytes, id, msg.c_str() );
		if( bytes == 0 || rxid != id ){
			ESP_LOGW(FNAME,"CAN bus selftest RX call FAILED");
			delay(10*i);
		}
		else if( memcmp( msg.c_str(), tx, len ) == 0 ){
			res=true;
			break;
		}
	}
    driverUninstall();

    if( res ){
    	ESP_LOGW(FNAME,"CAN bus selftest TX/RX OKAY");
        return true;
    }else{
    	ESP_LOGW(FNAME,"CAN bus selftest TX/RX FAILED");
        return false;
    }
}

bool CANbus::sendData( int id, const char* msg, int length, int self )
{
	// ESP_LOGI(FNAME,"CANbus::send %d bytes, self %d", length, self );
	// ESP_LOG_BUFFER_HEXDUMP(FNAME,msg,length, ESP_LOG_INFO);

	if( _connected_timeout > 100 ){
        ESP_LOGI(FNAME,"no CAN connection, reconnecting");
        force_reconnect = true; // escalate situation
        _connected_timeout = 0;
        _connected = false;

		return false;
	}
    if( ! _ready_initialized ){
		// ESP_LOGI(FNAME,"CANbus not ready");
		return false;
	}

	twai_message_t message;
	memset( &message, 0, sizeof( message ) );
	message.identifier = id;
	message.self = self;
	message.data_length_code = length;

	for (int i = 0; i < length; i++) {
	    message.data[i] = msg[i];
	}
	// ESP_LOGI(FNAME,"TX CAN bus message id:%04x, bytes:%d, data:%s, self:%d", message.identifier, message.data_length_code, message.data, message.self );

	//Queue message for transmission
	delay(1);
	esp_err_t error = twai_transmit(&message, pdMS_TO_TICKS(100));
	if(error == ESP_OK){
		// ESP_LOGI(FNAME,"Send CAN bus message okay");
        if ( ! _connected ) {
            // on connect, put used can speed to nvs memory
            ESP_LOGI(FNAME,"CAN connected, can speed %d saved", can_speed.get());
            can_speed.set(can_speed.get());
            _connected = true;
        }
        _connected_timeout=0;
		return true;
	}
	else{
		// ESP_LOGI(FNAME,"Send CAN bus message failed, ret:%02x", error );
        _connected_timeout++;
		return false;
	}
}
