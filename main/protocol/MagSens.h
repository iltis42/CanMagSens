/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "ProtocolItf.h"

#include <esp_ota_ops.h>

class MagSens final : public ProtocolItf
{
public:
	static constexpr int MAGCTRL_ID   = 0x30;
	static constexpr int MAGSTREAM_ID = 0x31;

    MagSens(int mp, ProtocolState &sm, DataLink &dl) : ProtocolItf(DeviceId::MASTER_DEV, mp, sm, dl) {}
	virtual ~MagSens() {}

public:
	ProtocolType getProtocolId() override { return MAGSENS_P; }
	datalink_action_t nextByte(const char c) override;
	datalink_action_t nextStreamChunk(const char *cptr, int count) override;
	bool isBinary() const override { return _binary; }

private:
	void Version();
	void parseCalibration();
	void startStream();
	void killStream();
	void prepareUpdate();
    void confirmPacket(int nr);
    // firmware update
    int _updateSize = 0;
	int _updPackSize = 0;
	int _bytesReceived = 0;
	int _packEnum = 0;
	char *_uptBuffer = nullptr;
	int _buff_fill = 0;
	const esp_partition_t *_updatePartition = nullptr;
	esp_ota_handle_t _updateHandle = 0;
	bool _binary = false;
};

