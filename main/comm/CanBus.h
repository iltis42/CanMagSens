#pragma once

#include <driver/twai.h>

#include "InterfaceCtrl.h"

#include <string>

class DataLink;

typedef enum
{
    CAN_SPEED_OFF,
    CAN_SPEED_250KBIT,
    CAN_SPEED_500KBIT,
    CAN_SPEED_1MBIT,
    CAN_SPEED_MAX
} CanSpeed;

class CANbus final : public InterfaceCtrl
{
public:
    CANbus(const gpio_num_t tx, const gpio_num_t rx);
    ~CANbus() = default;
    bool begin(CanSpeed speed=CAN_SPEED_1MBIT);
    void stop();
    bool isInitialized() { return _initialized; };
    // Ctrl
    InterfaceId getId() const override { return CAN_BUS; }
    const char* getStringId() const override { return "CAN"; }
    void ConfigureIntf(int cfg) override;
    int Send(const char*, int&, int) override;


private:
    friend void CANReceiveTask(void *arg);

private:
    void recover();
    bool selfTest();
    bool sendData(int id, const char *msg, int length, int self = 0);
    void driverInstall(twai_mode_t mode, CanSpeed speed);
    void driverUninstall();
    const gpio_num_t _tx_io;
    const gpio_num_t _rx_io;
    bool _initialized = false;
    int _tx_timeout = 2; // [msec] about two times the time for 111 bit to send
    CanSpeed _speed = CAN_SPEED_1MBIT; // fixme
};

extern CANbus *CAN;
