
#include "canbus.h"
#include "SetupNG.h"

#include "sensor.h"
#include "SString.h"
#include "Router.h"


#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/gpio.h>
#include <logdef.h>
#include <esp_err.h>

#include <cstring>


/*
 *  1:1 connection between two XCVario with a fixed message ID
 *
 */

#define MAGSENS_ID 0x31

int CANbus::_tick = 0;
volatile bool CANbus::_ready_initialized = false;
gpio_num_t CANbus::_tx_io = CAN_BUS_TX_PIN;
gpio_num_t CANbus::_rx_io = CAN_BUS_RX_PIN;
bool       CANbus::_connected;
int        CANbus::_connected_timeout;
TickType_t CANbus::_tx_timeout = 2;

static TaskHandle_t cpid;
static bool force_reconnect = false;

#define MSG_ID 0x555


// install/reinstall CAN driver in corresponding mode
void CANbus::driverInstall( twai_mode_t mode, bool other_speed ){
	if( _ready_initialized ){
        driverUninstall();
	}
	twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT( _tx_io, _rx_io, mode );
	ESP_LOGI(FNAME, "default alerts %X", g_config.alerts_enabled);
	g_config.alerts_enabled |= TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED;
	g_config.rx_queue_len = 15; // 1.5x the need of one NMEA sentence
	g_config.tx_queue_len = 15;
	ESP_LOGI(FNAME, "my alerts %X", g_config.alerts_enabled);

	twai_timing_config_t t_config;
	e_can_speed_t speed = CAN_SPEED_1MBIT;

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

	twai_filter_config_t f_config = {
		.acceptance_code = uint32_t(MAGSENS_ID << 21), // Shift for standard-ID (11 Bit)
		.acceptance_mask = ~(uint32_t(0x7ff << 21)), // Only pay attention to those 11 id bits (not the first two message bytes)
		.single_filter = true };

	//Install TWAI driver
	if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
		ESP_LOGI(FNAME,"Driver installed OK, mode %d, filter %04x", mode, f_config.acceptance_code );
	} else {
		ESP_LOGI(FNAME,"Failed to install driver");
		return;
	}

	//Start TWAI driver
	if (twai_start() == ESP_OK) {
		ESP_LOGI(FNAME,"Driver started");
        delay(10);
        _ready_initialized = true;
	} else {
        twai_driver_uninstall();
		ESP_LOGI(FNAME,"Failed to start driver");
	}
}

void CANbus::driverUninstall(){
    if( _ready_initialized ){
        _ready_initialized = false;
		delay(100);
		twai_stop();
		delay(100);
		twai_driver_uninstall();
		delay(100);
	}
}

void canTask(void *pvParameters){
	while (true) {
		CANbus::tick();
		if( (CANbus::_tick % 100) == 0) {
			// ESP_LOGI(FNAME,"Free Heap: %d bytes", heap_caps_get_free_size(MALLOC_CAP_8BIT) );
			if( uxTaskGetStackHighWaterMark( cpid ) < 128 )
				ESP_LOGW(FNAME,"Warning canbus task stack low: %d bytes", uxTaskGetStackHighWaterMark( cpid ) );
		}
	}
}


void CANbus::restart(){
	ESP_LOGW(FNAME,"CANbus restart");
	driverUninstall();
	driverInstall( TWAI_MODE_NORMAL );
	_connected_timeout = 0;
}

void CANbus::recover(){
	ESP_LOGW(FNAME,"CANbus recover");
	twai_stop();
	delay(100);
	twai_start();
	delay(100);
	twai_initiate_recovery();
	_connected_timeout = 0;
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
// return a terminated SString containing exactly the received chars
int CANbus::receive( int *id, SString& msg, int timeout ){
	twai_message_t rx;

	uint32_t alerts;
	twai_read_alerts(&alerts, pdMS_TO_TICKS(_tx_timeout));
	if( alerts & TWAI_ALERT_RX_QUEUE_FULL )
		ESP_LOGW(FNAME,"receive: RX QUEUE FULL alert %X", alerts );

	esp_err_t ret = twai_receive(&rx, pdMS_TO_TICKS(timeout) );
	// ESP_LOGI(FNAME,"RX CAN bus message ret=%02x len_%d ID:%d dlc:%d ext:%d rtr:%d", ret, rx.data_length_code, rx.identifier, rx.dlc_non_comp, rx.extd, rx.rtr );
	if(ret == ESP_OK ){
		// ESP_LOGI(FNAME,"RX CAN bus message ret=%02x TO:%d", ret, _connected_timeout_xcv );
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
				// ESP_LOG_BUFFER_HEXDUMP(FNAME,msg.c_str(),msg.length(), ESP_LOG_DEBUG);
				sendNMEA( msg.c_str() );
			}
		}
	}
	msg.clear();
	// Can bus receive tick
	int id = 0;
	int bytes = receive( &id, msg, 500 );
	// ESP_LOGI(FNAME,"CAN RX id:%02x, bytes:%d, connected:%d", id, bytes, _connected );
	if( bytes ){
		_connected_timeout = 0;
	}
	else{
		_connected_timeout++;
	}
    // receive message of corresponding ID
	if( id == 0x20 ) {     // start of nmea
		// ESP_LOGI(FNAME,"CAN RX NMEA chunk, len:%d msg: %s", bytes, msg.c_str() );
		// ESP_LOG_BUFFER_HEXDUMP(FNAME, msg.c_str(), msg.length(), ESP_LOG_INFO);
		//dlink.process( msg.c_str(), msg.length(), 3 );  // (char *packet, int len, int port );
	}
    return _connected;
}


bool CANbus::sendNMEA( const SString& msg ){
	if( !_ready_initialized ){
		return false;
    }
	int len = msg.length();  // Including the terminating \0 -> need to remove this one byte at RX from strlen
	ESP_LOGI(FNAME,"send CAN NMEA len %d, msg: %s", len, msg );
	bool ret = true;
	uint32_t alerts;
	twai_read_alerts(&alerts, 0); // read and clear alerts
	if( alerts != 0 ) {
		ESP_LOGW(FNAME,"Before send alerts %X", alerts);
    }
	const int chunk=8;
	int id = 0x20;
	const char *cptr = msg.c_str();
	while( len > 0 )
	{
		int dlen = std::min(chunk, len);
		// Underlaying queue does block until there is space,
		// only a timeout would return false.
		if( !sendData(id, cptr, dlen) || !ret) {
			if ( !ret ) {
				break;
			}
			delay(2);
			ret = false;
		}
		else {
			cptr += dlen;
			len -= dlen;
		}
	}
	return ret;
}


bool CANbus::selfTest(){
	ESP_LOGI(FNAME,"CAN bus selftest");
	driverInstall( TWAI_MODE_NO_ACK );
	bool res=false;
	int id=0x100;
	delay(100);
	twai_clear_receive_queue();
	for( int i=0; i<10; i++ ){
		char tx[10] = { "1827364" };
		int len = strlen(tx);
		ESP_LOGI(FNAME,"strlen %d", len );
		twai_clear_receive_queue();  // there might be data from a remote device
		if( !sendData( id, tx,len, 1 ) ){
			ESP_LOGW(FNAME,"CAN bus selftest TX FAILED");
			recover();
		}
		SString msg;
		int rxid;
		int bytes = receive( &rxid, msg, 10 );
		ESP_LOGI(FNAME,"RX CAN bus message bytes:%d, id:%04x, data:%s", bytes, id, msg.c_str() );
		if( bytes != 7 || rxid != id ){
			ESP_LOGW(FNAME,"CAN bus selftest RX call FAILED bytes:%d rxid%d recm:%s", bytes, rxid, msg.c_str() );
			delay(i);
			twai_clear_receive_queue();
		}
		else if( memcmp( msg.c_str(), tx, len ) == 0 ){
			ESP_LOGI(FNAME,"RX CAN bus OKAY");
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
		ESP_LOGI(FNAME,"CANbus not ready initialized");
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
            _connected = true;
        }
        _connected_timeout=0;
		return true;
	}
	else{
		ESP_LOGI(FNAME,"Send CAN bus message failed, ret:%02x", error );
        _connected_timeout++;
		return false;
	}
}
