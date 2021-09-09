
#include <esp_log.h>
#include <string>
#include "sdkconfig.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include <freertos/semphr.h>
#include "RingBufCPP.h"
#include <logdef.h>
#include "sensor.h"
#include "Router.h"


RingBufCPP<SString, QUEUE_SIZE> can_rx_q;
RingBufCPP<SString, QUEUE_SIZE> can_tx_q;

portMUX_TYPE btmux = portMUX_INITIALIZER_UNLOCKED;

// Utility methods to push and pull data into/from queues

#undef ESP_LOGV
#define ESP_LOGV(x,y,z);  ;

// checks if Queue is full, otherwise appends SString
bool Router::forwardMsg( SString &s, RingBufCPP<SString, QUEUE_SIZE>& q ){
	if( !q.isFull() ) {
		portENTER_CRITICAL_ISR(&btmux);
		q.add( s );
		portEXIT_CRITICAL_ISR(&btmux);
		return true;
	}
	// ESP_LOGW(FNAME,"+++ WARNING +++ dropped msg %s", s.c_str() );
	return false;
}

// gets last message from ringbuffer FIFO
bool Router::pullMsg( RingBufCPP<SString, QUEUE_SIZE>& q, SString& s ){
	if( !q.isEmpty() ){
		portENTER_CRITICAL_ISR(&btmux);
		q.pull( &s );
		portEXIT_CRITICAL_ISR(&btmux);
		return true;
	}
	return false;
}

int Router::pullMsg( RingBufCPP<SString, QUEUE_SIZE>& q, char *block ){
	int size = 0;
	if( !q.isEmpty() ){
		SString s;
		portENTER_CRITICAL_ISR(&btmux);
		q.pull(  &s );
		portEXIT_CRITICAL_ISR(&btmux);
		size = s.length();
		memcpy( block, s.c_str(), size );
	}
	block[size]=0;
	return size;
}

int Router::pullBlock( RingBufCPP<SString, QUEUE_SIZE>& q, char *block, int size ){
	int len = 0;
	while( !q.isEmpty() ){
		SString s;
		portENTER_CRITICAL_ISR(&btmux);
		q.pull(  &s );
		portEXIT_CRITICAL_ISR(&btmux);
		memcpy( block+len, s.c_str(), s.length() );
		len += s.length();
		if( (len + SSTRLEN) > size )
			break;
	}
	block[len]=0;
	return len;
}




// route messages from CAN
void Router::routeCAN(){
	SString can;
	if( pullMsg( can_rx_q, can ) ){
 // 		Protocols::parseNMEA( can.c_str() );
	}
}

