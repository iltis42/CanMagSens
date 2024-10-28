#pragma once

#include "ProtocolItf.h"

class MagSens final : public ProtocolItf
{
public:
	MagSens() = default;
	virtual ~MagSens() {}
public:
	virtual protocol_nt getProtocolId() const override { return MAGSENS; }
	virtual gen_state_t nextByte(const char c) override;
	virtual bool isNMEA() const override { return true; }
	virtual void timeOut() override { _state = ABORT; reset(); }

private:
	void incrCRC(const char c) override;
	void Version();
	void parseCalibration();
	void startStream();
	void killStream();
	void prepareUpdate();
};

