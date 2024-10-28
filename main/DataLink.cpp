
#include "DataLink.h"
#include <logdef.h>


DataLink::DataLink(){
	state = START_TOKEN;
	protocol = nullptr;
}

void DataLink::setProtocol(ProtocolItf* p)
{
	protocol = p;
}

void DataLink::process( const char *packet, int len, int port )
{
	// Special use, no data timeout, might be expected and normal
	// We just reset the protocol state machine then
	if ( packet == nullptr ) {
		protocol->timeOut();
		return;
	}

	// process every frame byte through state machine
	// ESP_LOGI(FNAME,"Port %d: RX len: %d bytes", port, len );
	// ESP_LOG_BUFFER_HEXDUMP(FNAME,packet, len, ESP_LOG_INFO);
	for (int i = 0; i < len; i++) {
		state = protocol->nextByte(packet[i]);
	}
}

