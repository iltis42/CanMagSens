
#include "Router.h"

#include "RingBufCPP.h"

#include <logdef.h>

#include <string>
#include <cstdio>
#include <freertos/semphr.h>


RingBufCPP<SString, QUEUE_SIZE> can_rx_q;
RingBufCPP<SString, QUEUE_SIZE> can_tx_q;

portMUX_TYPE btmux = portMUX_INITIALIZER_UNLOCKED;

// Utility methods to push and pull data into/from queues

#undef ESP_LOGV
#define ESP_LOGV(x,y,z);  ;
static xSemaphoreHandle qMutex=NULL;

void Router::begin(){
	qMutex = xSemaphoreCreateMutex();
}

// checks if Queue is full, otherwise appends SString
bool Router::forwardMsg( SString &s, RingBufCPP<SString, QUEUE_SIZE>& q ){
	if( !q.isFull() ) {
		xSemaphoreTake(qMutex,portMAX_DELAY );
		q.add( s );
		xSemaphoreGive(qMutex);
		return true;
	}
	// ESP_LOGW(FNAME,"+++ WARNING +++ dropped msg %s", s.c_str() );
	return false;
}

// gets last message from ringbuffer FIFO
bool Router::pullMsg( RingBufCPP<SString, QUEUE_SIZE>& q, SString& s ){
	if( !q.isEmpty() ){
		xSemaphoreTake(qMutex,portMAX_DELAY );
		q.pull( s );
		xSemaphoreGive(qMutex);
		return true;
	}
	return false;
}

int Router::pullBlock( RingBufCPP<SString, QUEUE_SIZE>& q, char *block, int size ){
	xSemaphoreTake(qMutex,portMAX_DELAY );
	int total_len = 0;
	while( !q.isEmpty() ){
		int len = q.pull( block+total_len );
		total_len += len;
		if( (total_len + SSTRLEN) > size )
			break;
	}
	block[total_len]=0;
	xSemaphoreGive(qMutex);
	return total_len;
}

