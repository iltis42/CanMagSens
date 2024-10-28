idf_component_register(SRC_DIRS "."  "protocol" 
                       INCLUDE_DIRS "."
		       REQUIRES esp_adc_cal soc driver I2Cbus nvs_flash hal )
