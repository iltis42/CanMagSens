#include "sensor.h"

#include "Version.h"
#include "SetupNG.h"
#include "Router.h"
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
mag_state_t stream_status = RAW_STREAM;

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
	delay( 10 );
	Router::begin();

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
	ESP_LOGI( FNAME, "Bias/Scale: (%f,%f,%f)/(%f,%f,%f)", x_bias,y_bias,z_bias, x_scale,y_scale,z_scale);

	// Load the chip specific gain to send data in [microT] on top
	ESP_LOGI( FNAME, "MagSense gain factor %f", magsens->getGain());
	x_scale *= magsens->getGain();
	y_scale *= magsens->getGain();
	z_scale *= magsens->getGain();

	// Decide on which stream to start fixme
	if ( 1 ) {
		stream_status = CALIBRATED;
	}

	esp_err_t ret = esp_task_wdt_add(NULL);
	if( ret != ESP_OK ) {
		ESP_LOGE(FNAME,"WDT add task failed %X", ret );
	}

	// Reduce logging from now on
	constexpr esp_log_level_t log_level = ESP_LOG_INFO;
	ESP_LOGI( FNAME, "Log level set globally to %d", log_level);
	esp_log_level_set("*", log_level);

	const uint64_t SENSOR_PERIOD = 100 * 1000; // 100 msec in usec
	uint64_t sleep_time = SENSOR_PERIOD;
	uint64_t wake_time = esp_timer_get_time();
	while( 1 ){

		// Timeslot to listen on incomein kill stream messages
		delay(30);

		sleep_time = SENSOR_PERIOD - (esp_timer_get_time() - wake_time);
		if ( sleep_time > SENSOR_PERIOD ) {
				sleep_time = SENSOR_PERIOD;
		}

		if ( stream_status == STREAM_OFF ) {
			// Listen on CAN for commands
			delay(sleep_time/1000);
		}
		else {
			esp_sleep_enable_timer_wakeup(sleep_time);
			//ESP_LOGI(FNAME,"Sleep for = %lldsec", sleep_time );
			// uint64_t before_sleep = esp_timer_get_time();
			esp_err_t err = esp_light_sleep_start();
		}
		wake_time = esp_timer_get_time();
		
		if ( stream_status != STREAM_OFF ) {
			int16_t data[3];
			int16_t &x=data[0], &y=data[1], &z=data[2];
			if( magsens->rawHeading( x,y,z) ){
				bool can_ok = false;
				if ( stream_status == RAW_STREAM ) {
					can_ok = CANbus::sendData( 0x031, (char *)data, 6 );
				}
				else if ( stream_status == CALIBRATED ) {
					float data[3];
					float &xf=data[0], &yf=data[1], &zf=data[2];
					xf = (x - x_bias) * x_scale; 
					yf = (y - y_bias) * y_scale; 
					zf = (z - z_bias) * z_scale; 
					can_ok = CANbus::sendData( 0x031, (char *)data, 8 );
					can_ok |= CANbus::sendData( 0x031, (char *)&zf, 4 );
				}
				if( can_ok ) {
					msgsent++;
					if( !(msgsent%200) ) {
						ESP_LOGI(FNAME,"CAN bus msg sent ok = %d X=%d Y=%d Z=%d", msgsent, x, y, z );
						// ESP_LOG_BUFFER_HEXDUMP(FNAME,data,6, ESP_LOG_INFO);
					}
				}
			}
			else {
				ESP_LOGW(FNAME,"Magnetic sensor read failed");
			}
		}

        esp_err_t ret = esp_task_wdt_reset();
        if( ret != ESP_OK )
            ESP_LOGE(FNAME,"WDT reset failed %X", ret );
	}
}
