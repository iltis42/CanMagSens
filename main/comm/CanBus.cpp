
#include "CanBus.h"

#include "Messages.h"
#include "DeviceMgr.h"
#include "DataLink.h"
#include "logdef.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <esp_task_wdt.h>
#include <driver/gpio.h>
#include <esp_err.h>

#include <cstring>


// used CAN Id's
static constexpr int CANTEST_ID = CAN_REG_PORT+1;

static TaskHandle_t rxTask = nullptr;

CANbus *CAN = 0;
static bool terminate_receiver = false;
static bool do_recover = false;

// CAN receiver task
void IRAM_ATTR CANReceiveTask(void *arg)
{
    CANbus* can = static_cast<CANbus*>(arg);
    unsigned int tick = 0;
    bool to_once = true;

    while ( ! can->isInitialized() )
    {
        ESP_LOGI(FNAME, "CANbus not ready");
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    esp_task_wdt_add(NULL);

    std::string msg; // scratch buffer
    msg.resize(10);
    do {
        // basically block on the twai receiver for ever
        twai_message_t rx;
        if (ESP_OK == twai_receive(&rx, pdMS_TO_TICKS(500)) && rx.data_length_code > 0)
        {
            msg.assign((char *)rx.data, rx.data_length_code);
            ESP_LOGD(FNAME, "CAN RX NMEA chunk, id:0x%x, len:%d msg: %s", (unsigned int)(rx.identifier), rx.data_length_code, msg.c_str());
            auto dl = can->_dlink.find(rx.identifier);
            if ( dl != can->_dlink.end() ) {
                dl->second->process(msg.data(), msg.size());
                to_once = true;
            }
        }
        else
        {
            // protocol state machine may want to react on no traffic -- once
            if ( to_once ) {
                for (auto &dl : can->_dlink ) {
                    dl.second->process(nullptr, 0);
                }
                to_once = false;
            }
        }

        if ( terminate_receiver ) { break; }
        if ( do_recover ) {
            can->recover(); // Can only do this not waiting in twai_receive
            do_recover = false;
        }

        if ((tick++ % 100) == 0)
        {
            if (uxTaskGetStackHighWaterMark(rxTask) < 128) {
                ESP_LOGW(FNAME, "Warning canbus task stack low: %d bytes", uxTaskGetStackHighWaterMark(rxTask));
            }
        }

        esp_task_wdt_reset();
    } while ( true );

    // cannot stop twai when waiting on twai_receive (->crash)
    can->driverUninstall();

    terminate_receiver = false; // handshake
    rxTask = nullptr;
    vTaskDelete(NULL);
}



CANbus::CANbus(const gpio_num_t tx, const gpio_num_t rx) :
    _tx_io(tx),
    _rx_io(rx)
{
}

void CANbus::ConfigureIntf(int cfg)
{
    // todo
}

// install/reinstall CAN driver in corresponding mode
void CANbus::driverInstall(twai_mode_t mode, CanSpeed speed)
{
    if (_initialized)
    {
        driverUninstall();
    }

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(_tx_io, _rx_io, mode);
    ESP_LOGI(FNAME, "default alerts %X", (unsigned int)(g_config.alerts_enabled));
    // g_config.alerts_enabled = TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_OFF | TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_BUS_ERROR;
    g_config.alerts_enabled = TWAI_ALERT_ALL;
    g_config.bus_off_io = GPIO_NUM_NC;
    g_config.rx_queue_len = 15;
    g_config.tx_queue_len = 15;
    ESP_LOGI(FNAME, "set alerts to %X", (unsigned int)(g_config.alerts_enabled));

    twai_timing_config_t t_config;
    _speed = speed;
    _tx_timeout = 2; // 111usec/chunk -> 2msec
    if (speed == CAN_SPEED_250KBIT)
    {
        ESP_LOGI(FNAME, "CAN rate 250KBit");
        t_config = TWAI_TIMING_CONFIG_250KBITS();
        _tx_timeout = 8; // 444usec/chunk -> 4msec
    }
    else if (speed == CAN_SPEED_500KBIT)
    {
        ESP_LOGI(FNAME, "CAN rate 500KBit");
        t_config = TWAI_TIMING_CONFIG_500KBITS();
        _tx_timeout = 4; // 222usec/chunk -> 2msec
    }
    else if (speed == CAN_SPEED_1MBIT)
    {
        ESP_LOGI(FNAME, "CAN rate 1MBit");
        t_config = TWAI_TIMING_CONFIG_1MBITS();
    }
    else
    {
        ESP_LOGI(FNAME, "CAN rate 1MBit for selftest");
        t_config = TWAI_TIMING_CONFIG_1MBITS();
    }

    // Install TWAI driver
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
    {
        ESP_LOGI(FNAME, "Driver installed OK, mode %d, filter 0x%04x", mode, (unsigned int)(f_config.acceptance_code));
    }
    else
    {
        ESP_LOGI(FNAME, "Failed to install driver");
        return;
    }

    // Start TWAI driver
    if (twai_start() == ESP_OK)
    {
        ESP_LOGI(FNAME, "Driver started");
        _initialized = true;
    }
    else
    {
        twai_driver_uninstall();
        ESP_LOGI(FNAME, "Failed to start driver");
    }
}

void CANbus::driverUninstall()
{
    if (_initialized)
    {
        _initialized = false;
        twai_stop();
        twai_driver_uninstall();
    }
}

void CANbus::recover()
{
    twai_status_info_t status_info;
    if (twai_get_status_info(&status_info) == ESP_OK)
    {
        if (status_info.state == TWAI_STATE_BUS_OFF)
        {
            // recovery is only possible in this state
            ESP_LOGW(FNAME, "CANbus recover");
            twai_initiate_recovery();
            vTaskDelay(pdMS_TO_TICKS(10));
            twai_start();
        }
    }
}

// begin CANbus, launch driver in normal mode after a selfTest
bool CANbus::begin(CanSpeed speed)
{
    ESP_LOGI(FNAME, "CAN begin");

    if ( ! _initialized && selfTest() ) {
        driverInstall(TWAI_MODE_NORMAL, CanSpeed::CAN_SPEED_1MBIT);

        terminate_receiver = false;
        xTaskCreate(&CANReceiveTask, "CanRx", 4096, this, 21, &rxTask);
    } else {
        driverUninstall();
    }
    return _initialized;
}

// terminate CANbus
void CANbus::stop()
{
    ESP_LOGI(FNAME, "CAN stop");

    // send terminate signals to tasks
    terminate_receiver = true; // for receiver

    DeleteAllDataLinks();
}


bool CANbus::selfTest()
{
    ESP_LOGI(FNAME, "CAN bus selftest");
    driverInstall(TWAI_MODE_NO_ACK, CanSpeed::CAN_SPEED_1MBIT);
    // driverInstall(TWAI_MODE_LISTEN_ONLY, CanSpeed::CAN_SPEED_1MBIT);
    
    bool res = false;
    int id = CANTEST_ID; // todo find some better idea to splatter bytes on the bus for a test
    twai_clear_receive_queue();
    for (int i = 0; i < 3; i++)
    {
        char tx[10] = {"1827364"};
        int len = strlen(tx);
        ESP_LOGI(FNAME, "strlen %d", len);
        twai_clear_receive_queue(); // there might be data from a remote device
        if ( ! sendData(id, tx, len, 1) ) {
            ESP_LOGW(FNAME, "CAN bus selftest TX FAILED");
        }

        twai_message_t rx;
        if ( (ESP_OK == twai_receive(&rx, pdMS_TO_TICKS(20)))
            && (rx.identifier == id)
            && (rx.data_length_code == len)
            && (memcmp(rx.data, tx, len) == 0) )
        {
            ESP_LOGI(FNAME, "RX CAN bus OKAY");
            res = true;
            break;
        }
        else
        {
            std::string msg((char*)rx.data, rx.data_length_code);
            ESP_LOGW(FNAME, "CAN bus selftest RX call FAILED bytes:%d rxid:%x rxmsg:%s", (unsigned int)(rx.data_length_code), (unsigned int)(rx.identifier), msg.c_str());
            twai_clear_receive_queue();
        }
    }
    driverUninstall();

    ESP_LOGW(FNAME, "CAN bus selftest TX/RX %s", res?"Ok":"failed");
    return res;
}


int CANbus::Send(const char *cptr, int &len, int port)
{
    constexpr int chunk = 8;

    int rem = len;
    while (rem > 0)
    {
        int dlen = std::min(chunk, rem);
        if ( ! sendData(port, cptr, dlen) ) {
            break;
        }
        cptr += dlen;
        rem -= dlen;
    }

    if ( rem == 0 ) {
        return 0;
    }
    else {
        if ( _error_batch > 60 ) {
            esp_restart(); // this never apeared to be solved otherwise
        }
        len = len - rem; // buffered bytes
        return (rem/chunk + 1) * _tx_timeout; // ETA to wait for next trial
    }
}

// Send, handle alerts, do max 3 retries
bool CANbus::sendData(int id, const char *msg, int length, int self)
{
    if (!_initialized)
    {
        ESP_LOGI(FNAME, "CANbus not ready initialized");
        return false;
    }

    // build message
    twai_message_t message;
    message.identifier = id;
    message.self = self;
    message.data_length_code = length;
    for (int i = 0; i < length; i++)
    {
        message.data[i] = msg[i];
    }

    // logger.format(LOG_INFO,"TX CAN bus message id:%x, bytes:%d, data:%c... self:%d", message.identifier, message.data_length_code, *message.data, message.self );
    // Queue message for transmission
    uint32_t alerts = 0;
    int retry = 3;
    esp_err_t res = ESP_OK;
    while ( retry-- > 0 )
    {
        res = twai_transmit(&message, pdMS_TO_TICKS(_tx_timeout));
        if ( res == ESP_OK ) {
            _error_batch = 0;
            break;
        }
        _error_batch++;
        ESP_LOGE(FNAME, "Transmit error: %s", esp_err_to_name(res));
        if (res == ESP_ERR_TIMEOUT) {
            ESP_LOGW(FNAME, "Transmit timeout. Message dropped.");
        }
        twai_read_alerts(&alerts, pdMS_TO_TICKS(_tx_timeout));
        ESP_LOGW(FNAME, "Tx chunk failed alerts 0x%x", (unsigned int)(alerts) );
    }
    if ( alerts != 0 )
    {
        if ( alerts & TWAI_ALERT_BUS_OFF ) {
            ESP_LOGE(FNAME, "BUS OFF alert");
            do_recover = true;
        }
        if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
            ESP_LOGW(FNAME, "RX QUEUE FULL alert");
        }
        if (alerts & TWAI_ALERT_TX_FAILED) {
            ESP_LOGW(FNAME, "TX_FAILED alert");
        }
        return false;
    }
    return (res == ESP_OK);
}
