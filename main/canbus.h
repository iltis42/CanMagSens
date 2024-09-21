#pragma once

#include "driver/gpio.h"
#include "driver/twai.h"


#define CAN_BUS_TX_PIN GPIO_NUM_1
#define CAN_BUS_RX_PIN GPIO_NUM_3

class SString;

class CANbus {
public:
	CANbus(){};
	~CANbus(){};
	static void begin();
	static bool sendData( int id, const char* msg, int length, int self=0 );
	static bool sendNMEA( const SString& msg );
	static int receive(  int *id, SString& msg, int timeout=5);
	static bool tick();

	static void restart();
	static void recover();
	static bool selfTest();
	static int _tick;
	static bool connected() { return _connected; };
	static bool isOkay() { return _ready_initialized; };

private:
	static void driverInstall( twai_mode_t mode, bool other_speed = false );
	static void driverUninstall();
	static volatile bool _ready_initialized;
	static gpio_num_t _tx_io;
	static gpio_num_t _rx_io;
	static bool _connected;
	static int _connected_timeout;
    static TickType_t _tx_timeout; // [msec] about two times the time for 111 bit to send
};
