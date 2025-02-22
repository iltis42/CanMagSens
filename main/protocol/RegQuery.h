/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "ProtocolItf.h"
#include "ClockIntf.h"

#include <string>

class RegQuery final : public ProtocolItf, public Clock_I
{
public:
    explicit RegQuery(int mp, ProtocolState &sm, DataLink &dl);
    virtual ~RegQuery();

public:
    ProtocolType getProtocolId() override { return REGISTRATION_P; }
    datalink_action_t nextByte(const char c) override;

    static constexpr int MAX_NR_TRIALS = 20;

    // The only transmitter
    bool sendRegistrationQuery();

    // Clock tick callback
    void tick() override;

private:
    // Actions on commands
    datalink_action_t registration();
    char Q_TOKEN[4];
    int _nr_trials = 0;
};
