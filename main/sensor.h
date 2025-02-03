#pragma once

typedef enum {
	STREAM_OFF,
	RAW_STREAM,
	CALIBRATED
} mag_state_t;

extern mag_state_t stream_status;

class MagSens;
extern MagSens* MAG;