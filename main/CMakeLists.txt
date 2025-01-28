idf_component_register(SRC_DIRS "."  "comm"  "protocol"
                       INCLUDE_DIRS "."
		       REQUIRES esp_adc_cal soc driver I2Cbus app_update nvs_flash hal )
