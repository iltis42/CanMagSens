/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "ProtocolItf.h"

class MagSens final : public ProtocolItf
{
public:
	static constexpr int MAGCTRL_ID   = 0x30;
	static constexpr int MAGSTREAM_ID = 0x31;

    MagSens(int mp, ProtocolState &sm, DataLink &dl) : ProtocolItf(DeviceId::MAGSENS_DEV, mp, sm, dl) {}
	virtual ~MagSens() {}

public:
	ProtocolType getProtocolId() override { return MAGSENS_P; }
	datalink_action_t nextByte(const char c) override;
	datalink_action_t nextStreamChunk(const char *cptr, int count) override;

private:
	void Version();
	void parseCalibration();
	void startStream();
	void killStream();
	void prepareUpdate();
	int _updateSize = 0;
};

