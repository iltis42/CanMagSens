#pragma once

#include "protocol/ProtocolItf.h"


/**
 *
 * Data link layer to multiplex data stream to proper protocol parser.
 *
 */



class DataLink {
	public:
		DataLink();
		void setProtocol(ProtocolItf* p);
		void process( const char *packet, int len, int port );

	private:
		enum gen_state_t state;
		ProtocolItf* protocol;
};
