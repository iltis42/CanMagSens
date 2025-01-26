#pragma once

#define delay(x) vTaskDelay(x/portTICK_PERIOD_MS)

typedef enum {
	STREAM_OFF,
	RAW_STREAM,
	CALIBRATED
} mag_state_t;

extern mag_state_t stream_status;

class MagSens;
extern MagSens* MAG;