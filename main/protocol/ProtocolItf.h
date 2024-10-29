#pragma once

// Supported protocol id's
enum protocol_nt {
	NO_ONE,     // Disable
	NMEA,
	MAGSENS
};

// Generic protocol state machine
enum gen_state_t {
	START_TOKEN,
	HEADER,
	PAYLOAD,
	STOP_TOKEN,
	COMPLETE,
	CHECK_CRC,
	CHECK_OK,
	CHECK_FAILED,
	ABORT
};

// Protocol parser interface
class ProtocolItf {
public:
	virtual ~ProtocolItf() {}

	static constexpr int MAX_LEN = 128;
public:
	// API
	virtual protocol_nt getProtocolId() const = 0;
	virtual gen_state_t nextByte(const char c) = 0;
	gen_state_t getState() const { return _state; }
	const char* getBuffer() { return _framebuffer; }
	int getLength() const { return _pos; }
	bool isPassThrough() const { return _pass_through; }
	virtual bool isNMEA() const { return true; }
	virtual void timeOut() {;} // called when for some time no input message

protected:
	void reset() { _pos = 0; _len = 0; _crc = 0; }
	bool push(char c) { if ( _pos < MAX_LEN ) { _framebuffer[_pos++] = c; incrCRC(c); return true;} return false; }
	virtual void incrCRC(const char c) = 0;
	enum gen_state_t _state = START_TOKEN;
	bool _pass_through = false;
	char _framebuffer[MAX_LEN];
	int  _pos = 0;
	int  _len = 0;
	char _crc = 0; // checksum
};

