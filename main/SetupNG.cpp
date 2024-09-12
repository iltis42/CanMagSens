/*
 * Setup.cpp
 *
 *  Created on: September, 2021
 *      Author: iltis
 */

#include "SetupNG.h"

#include "ESP32NVS.h"
#include "esp32/rom/uart.h"
#include <esp32/rom/miniz.h>

#include "esp_system.h"
#include <logdef.h>

#include <string>
#include <cstdio>
#include <cmath>
#include <iostream>
#include <map>
#include <vector>


std::vector<SetupCommon *> SetupCommon::entries;
char SetupCommon::_ID[14];

SetupNG<float>          compass_x_bias( "CP_X_BIAS", 0 );
SetupNG<float>          compass_y_bias( "CP_Y_BIAS", 0 );
SetupNG<float>          compass_z_bias( "CP_Z_BIAS", 0 );
SetupNG<float>          compass_x_scale( "CP_X_SCALE", 0 );
SetupNG<float>          compass_y_scale( "CP_Y_SCALE", 0 );
SetupNG<float>          compass_z_scale( "CP_Z_SCALE", 0 );
SetupNG<int>            can_speed( "CANSPEED", CAN_SPEED_1MBIT );


SetupCommon * SetupCommon::getMember( const char * key ){
	for(int i = 0; i < entries.size(); i++ ) {
		if( std::string( key ) == std::string( entries[i]->key() )){
			ESP_LOGI(FNAME,"found key %s", entries[i]->key() );
			return entries[i];
		}
	}
	return 0;
}

void SetupCommon::syncEntry( int entry ){
	ESP_LOGI(FNAME,"SetupCommon::syncEntry( %d )", entry );
}

bool SetupCommon::factoryReset(){
	ESP_LOGI(FNAME,"\n\n******  FACTORY RESET ******");
	bool retsum = true;
	for(int i = 0; i < entries.size(); i++ ) {
		ESP_LOGI(FNAME,"i=%d %s erase", i, entries[i]->key() );
		if( entries[i]->mustReset() ){
			bool ret = entries[i]->erase();
			if( ret != true ) {
				ESP_LOGE(FNAME,"Error erasing %s", entries[i]->key() );
				retsum = false;
			}
			ret = entries[i]->init();
			if( ret != true ) {
				ESP_LOGE(FNAME,"Error init with default %s", entries[i]->key() );
				retsum = false;
			}
			else
				ESP_LOGI(FNAME,"%s successfully initialized with default", entries[i]->key() );
		}
	}
	if( retsum )
		ESP_LOGI(FNAME,"Factory reset SUCCESS");
	else
		ESP_LOGI(FNAME,"Factory reset FAILED!");
	return retsum;
}

bool SetupCommon::initSetup( bool& present ) {
	bool ret=true;
	ESP_LOGI(FNAME,"SetupCommon::initSetup()");
	esp_err_t _err = nvs_flash_init();
	if (_err == ESP_ERR_NVS_NO_FREE_PAGES) {
		ESP_LOGE(FNAME,"Error no more space in NVS: erase partition");
		const esp_partition_t* nvs_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
		_err = (esp_partition_erase_range(nvs_partition, 0, nvs_partition->size));
		if ( _err != ESP_OK ){
			ESP_LOGE(FNAME, "partition erase returned error ret=%d", _err );
			ret = false;
		}
	}

	for(int i = 0; i < entries.size(); i++ ) {
			bool ret = entries[i]->init();
			if( ret != true )
				ESP_LOGE(FNAME,"Error init with default NVS: %s", entries[i]->key() );
	}

	if( 0 ) {
		ESP_LOGI(FNAME,"\n\n******  FACTORY RESET ******");
		for(int i = 0; i < entries.size(); i++ ) {
			ESP_LOGI(FNAME,"i=%d %s erase", i, entries[i]->key() );
			if( entries[i]->mustReset() ){
				bool ret = entries[i]->erase();
				if( ret != true ) {
					ESP_LOGE(FNAME,"Error erasing %s", entries[i]->key() );
					ret = false;
				}
				ret = entries[i]->init();
				if( ret != true ) {
					ESP_LOGE(FNAME,"Error init with default %s", entries[i]->key() );
					ret = false;
				}
				else
					ESP_LOGI(FNAME,"%s successfully initialized with default", entries[i]->key() );
			}
		}
	}
	return ret;
};

char * SetupCommon::getID() {
	if( strlen( _ID ) == 0 ) {
		uint8_t mac[6];
		unsigned long  crc = 0;
		if ( esp_efuse_mac_get_default(mac) == ESP_OK ){
			crc = mz_crc32(0L, mac, 6);
		}
		sprintf( _ID, "MagSens-%03d", int(crc % 1000));
	}
	return _ID;
}
