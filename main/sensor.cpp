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

// Sensor board init method. Herein all functions that make the XCVario are launched and tested.
void sensor(void *args){
	esp_wifi_set_mode(WIFI_MODE_NULL);
	// esp_log_level_set("*", ESP_LOG_INFO);
	ESP_LOGI( FNAME, "Log level set globally to INFO %d",  ESP_LOG_INFO);
	esp_chip_info_t chip_info;
	esp_chip_info(&chip_info);
	ESP_LOGI( FNAME,"This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
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

	if( CANbus::selfTest() ){
		CANbus::begin();
	}

}

extern "C" void  app_main(void){
	ESP_LOGI(FNAME,"app_main" );
	ESP_LOGI(FNAME,"Now init all Setup elements");
	bool setupPresent;
	SetupCommon::initSetup( setupPresent );
	esp_log_level_set("*", ESP_LOG_INFO);
	sensor( 0 );
	vTaskDelete( NULL );
}


