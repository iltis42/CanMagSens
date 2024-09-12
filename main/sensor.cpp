#include "sensor.h"

#include "Version.h"
#include "SetupNG.h"
#include "canbus.h"
#include "QMC5883L.h"
#include "QMC6310U.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_system.h>
#include <driver/adc.h>
#include <driver/gpio.h>
#include "ESP32NVS.h"
#include <esp_wifi.h>
#include <logdef.h>

#include "I2Cbus.hpp"
#include <esp32/rom/miniz.h>
#include <esp32/rom/uart.h>
#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include <esp_pm.h>
#include <esp_sleep.h>

#include <cstdio>
#include <cstring>

I2C_t& i2c_0 = i2c0;  // i2c0 or i2c1

static int msgsent = 0;

// Sensor board init method. Herein all functions that make the XCVario are launched and tested.
extern "C" void  app_main(void){
	ESP_LOGI(FNAME,"app_main" );
	ESP_LOGI(FNAME,"Now init all Setup elements");
	bool setupPresent;
	SetupCommon::initSetup( setupPresent );

	esp_wifi_set_mode(WIFI_MODE_NULL);

	esp_chip_info_t chip_info;
	esp_chip_info(&chip_info);
	ESP_LOGI( FNAME,"This is ESP32 chip with %d CPU core(s), WiFi%s%s, ",
			chip_info.cores,
			(chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
					(chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
	ESP_LOGI( FNAME,"Silicon revision %d, ", chip_info.revision);
	ESP_LOGI( FNAME,"%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
			(chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    // Configure power mode
    esp_pm_config_esp32c3_t pmconf;
    pmconf.max_freq_mhz = 80;
    pmconf.min_freq_mhz = 80;
    pmconf.light_sleep_enable = false;
    esp_pm_configure(&pmconf);

	NVS.begin();
	delay( 1000 );

	Version myVersion;
	ESP_LOGI(FNAME,"Program Version %s", myVersion.version() );
	ESP_LOGI(FNAME,"Wireless ID %s", SetupCommon::getID() );

	ESP_LOGI(FNAME,"Now start CAN bus selftest" );
	if( ! CANbus::selfTest() ) {
		ESP_LOGE(FNAME,"CAN bus selftest failed" );
        // CAN might not be connected, reboot
        delay(2000);
        esp_restart();
    }
	ESP_LOGI(FNAME,"CAN bus selftest end" );
	CANbus::begin();

	// Find the proper mag sensor chip 
	QMCbase *magsens;
	while ( 1 ) {
		magsens = new QMC5883L( QMCbase::ODR_50Hz, QMC5883L::RANGE_2GAUSS, QMC5883L::OSR_512, &i2c_0 );
		if( magsens->begin(GPIO_NUM_5, GPIO_NUM_4, 100000 ) ) {
			break; // found a QMC5883L
		}
		delete magsens;

		// Try the next chip type
		magsens = new QMC6310U( QMCbase::ODR_50Hz, QMC6310U::RANGE_2GAUSS, QMC6310U::OSR1_8, &i2c_0 );
		if( magsens->begin(GPIO_NUM_5, GPIO_NUM_4, 100000 ) ) {
			break; // found a QMC6310U
		}
		delete magsens;

		ESP_LOGW(FNAME,"Failed to find a magnetic sensor");
		delay(1000);
	}
	if ( ! QMCbase::haveSensor() ) {
		ESP_LOGW(FNAME,"Magnetic sensor init failed");
	}

	// Load the nvs stored bias and scale calibration
	float x_bias = compass_x_bias.get();
	float y_bias = compass_y_bias.get();
	float z_bias = compass_z_bias.get();
	float x_scale = compass_x_scale.get();
	float y_scale = compass_y_scale.get();
	float z_scale = compass_z_scale.get();
	// Check for old scale == 0 default
	if ( x_scale == 0 || y_scale == 0 || z_scale == 0 ) {
		x_scale = y_scale = z_scale = 1.f;
	}
	// Load the chip specific gain to send data in [µT] on top
	x_scale *= magsens->getGain();
	y_scale *= magsens->getGain();
	z_scale *= magsens->getGain();

	esp_err_t ret = esp_task_wdt_add(NULL);
	if( ret != ESP_OK ) {
		ESP_LOGE(FNAME,"WDT add task failed %X", ret );
	}

	// Reduce logging from now on
	constexpr esp_log_level_t log_level = ESP_LOG_ERROR;
	ESP_LOGI( FNAME, "Log level set globally to %d", log_level);
	esp_log_level_set("*", log_level);

	const uint64_t SENSOR_PERIOD = 100 * 1000; // 100 msec in usec
	uint64_t sleep_time = SENSOR_PERIOD;
	uint64_t last_timesys = esp_timer_get_time();
	while( 1 ){
		sleep_time = SENSOR_PERIOD - (esp_timer_get_time() - last_timesys);
		if ( sleep_time > SENSOR_PERIOD ) {
				sleep_time = SENSOR_PERIOD;
		}
		esp_sleep_enable_timer_wakeup(sleep_time);
		//ESP_LOGI(FNAME,"Sleep for = %lldsec", sleep_time );
		// uint64_t before_sleep = esp_timer_get_time();
		esp_err_t err = esp_light_sleep_start();
		last_timesys = esp_timer_get_time();

		// const char *wakeup_reason;
		// esp_sleep_wakeup_cause_t wake_cause = esp_sleep_get_wakeup_cause();
		// switch (wake_cause)
		// {
		// case ESP_SLEEP_WAKEUP_EXT0:
		// 	wakeup_reason = "rtc io";
		// 	break;
		// case ESP_SLEEP_WAKEUP_EXT1:
		// 	wakeup_reason = "rtc ctl";
		// 	break;
		// case ESP_SLEEP_WAKEUP_TIMER:
		// 	wakeup_reason = "timer";
		// 	break;
		// case ESP_SLEEP_WAKEUP_GPIO:
		// 	wakeup_reason = "button";
		// 	break;
		// default:
		// 	wakeup_reason = "other";
		// 	break;
		// }
		// ESP_LOGI(FNAME, "e%d: %s woken, slept for %lldusec.", err, wakeup_reason, last_timesys-before_sleep);
		
		int16_t x,y,z;
		if( magsens->rawHeading( x,y,z) ){
			// ESP_LOGI(FNAME,"X=%d, Y=%d Z=%d", x, y, z );
			char data[6];
			data[0] = x & 0xFF;
			data[1] = (x & 0xFF00) >> 8;
			data[2] = y & 0xFF;
			data[3] = (y & 0xFF00) >> 8;
			data[4] = z & 0xFF;
			data[5] = ( z & 0xFF00 ) >> 8;
			if( CANbus::sendData( 0x031, data, 6 ) ){
				msgsent++;
				if( !(msgsent%200) ){
				   ESP_LOGI(FNAME,"CAN bus msg sent ok = %d X=%d Y=%d Z=%d", msgsent, x, y, z );
				   ESP_LOG_BUFFER_HEXDUMP(FNAME,data,6, ESP_LOG_INFO);
				}
			}
		}
		else
			ESP_LOGW(FNAME,"Magnetic sensor read failed");

        esp_err_t ret = esp_task_wdt_reset();
        if( ret != ESP_OK )
            ESP_LOGE(FNAME,"WDT reset failed %X", ret );
	}
}
