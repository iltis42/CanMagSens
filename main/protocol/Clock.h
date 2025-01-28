/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include <esp_timer.h>

class Clock_I;

// Clock based on esp_timer
class Clock
{
public:
    static constexpr unsigned long TICK_ATOM = 10; // msec

public:
    Clock();

    // API
    static void start(Clock_I *cb);
    static void stop(Clock_I *cb);
    static unsigned long getMillis();
    static int getSeconds();

private:
    static esp_timer_handle_t _clock_timer;
};
