idf_component_register(SRC_DIRS "."  "comm"  "protocol"
                       INCLUDE_DIRS "."
		       REQUIRES soc driver esp_timer I2Cbus app_update spi_flash nvs_flash hal )

