#ifndef __ROUTER_H__
#define __ROUTER_H__

#include "RingBufCPP.h"

#include <freertos/FreeRTOS.h>

#include <string>

/*
 *   Router for XCVario, to interconnect wireless (Bluetooth or WLAN), XCVario and serial S1,S2 interfaces
 *
 *
 *
 */


#define QUEUE_SIZE 8

extern RingBufCPP<SString, QUEUE_SIZE> can_rx_q;
extern RingBufCPP<SString, QUEUE_SIZE> can_tx_q;

extern portMUX_TYPE btmux;


class Router {

public:
  Router() { };
  static void begin();
  // add message to queue and return true if succeeded
  static bool forwardMsg( SString &s, RingBufCPP<SString, QUEUE_SIZE>& q );
  // gets last message from ringbuffer FIFO, return true if succeeded
  static bool pullMsg( RingBufCPP<SString, QUEUE_SIZE>& q, SString& s );
  // get one big block max 512 byte
  static int pullBlock( RingBufCPP<SString, QUEUE_SIZE>& q, char * block, int size );
  // route messages coming in from CAN interface

private:

};

#endif
