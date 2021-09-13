#include <string>
#include "sdkconfig.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "string.h"
#include "esp_system.h"
#include <driver/adc.h>
#include <driver/gpio.h>
#include "ESP32NVS.h"
#include "esp_sleep.h"
#include <esp_wifi.h>
#include "sensor.h"
#include "Version.h"

#include "SetupNG.h"
#include <logdef.h>

#include "I2Cbus.hpp"
#include <esp32/rom/miniz.h>
// #include "soc/sens_reg.h" // needed for adc pin reset
#include "driver/gpio.h"
#include "canbus.h"
#include "QMC5883L.h"
#include "esp_task_wdt.h"

extern TaskHandle_t pxCurrentTCB;
TaskHandle_t tpid;
I2C_t& i2c_0 = i2c0;  // i2c0 or i2c1

QMC5883L magsens( QMC5883L_ADDR, ODR_50Hz, RANGE_2GAUSS, OSR_512, &i2c_0 );
static int msgsent = 0;

// Sensor board init method. Herein all functions that make the XCVario are launched and tested.
void sensor(void *args){
	esp_wifi_set_mode(WIFI_MODE_NULL);
	// esp_log_level_set("*", ESP_LOG_INFO);
	ESP_LOGI( FNAME, "Log level set globally to INFO %d",  ESP_LOG_INFO);
	esp_chip_info_t chip_info;
	esp_chip_info(&chip_info);
	ESP_LOGI( FNAME,"This is ESP32 chip with %d CPU core(s), WiFi%s%s, ",
			chip_info.cores,
			(chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
					(chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
	ESP_LOGI( FNAME,"Silicon revision %d, ", chip_info.revision);
	ESP_LOGI( FNAME,"%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
			(chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

	NVS.begin();
	delay( 1000 );

	Version myVersion;
	ESP_LOGI(FNAME,"Program Version %s", myVersion.version() );
	ESP_LOGI(FNAME,"Wireless ID %s", SetupCommon::getID() );

	ESP_LOGI(FNAME,"Now start CAN bus selftest" );
	if( CANbus::selfTest() ){
		CANbus::begin();
	}else
		ESP_LOGE(FNAME,"CAN bus selftest failed" );
	ESP_LOGI(FNAME,"CAN bus selftest end" );

	if( magsens.begin(GPIO_NUM_5, GPIO_NUM_4, 400000 ) ){
		if( !magsens.selfTest() ){
			ESP_LOGW(FNAME,"Magnetic sensor selftest failed");
		}
	}else
		ESP_LOGW(FNAME,"Magnetic sensor init failed");

	esp_err_t ret = esp_task_wdt_add(tpid);
	if( ret != ESP_OK )
		ESP_LOGE(FNAME,"WDT add task failed %X", ret );

	while( 1 ){
		delay( 50 );
		int16_t x,y,z;
		if( magsens.rawHeading( x,y,z) ){
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
				esp_err_t ret = esp_task_wdt_reset();
				if( ret != ESP_OK )
					ESP_LOGE(FNAME,"WDT reset failed %X", ret );
				if( !(msgsent%200) ){
					ESP_LOGI(FNAME,"CAN bus msg sent ok = %d X=%d Y=%d Z=%d", msgsent, x, y, z );
					ESP_LOG_BUFFER_HEXDUMP(FNAME,data,6, ESP_LOG_INFO);
				}
			}
		}
		else
			ESP_LOGW(FNAME,"Magnetic sensor read failed");

	}
}


extern "C" void  app_main(void){
	ESP_LOGI(FNAME,"app_main" );
	ESP_LOGI(FNAME,"Now init all Setup elements");
	bool setupPresent;
	SetupCommon::initSetup( setupPresent );
	esp_log_level_set("*", ESP_LOG_INFO);
	xTaskCreate(&sensor, "sensor", 4096, NULL, 14, &tpid );
	vTaskDelete( NULL );
}



