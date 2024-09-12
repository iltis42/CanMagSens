/**************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************

I2C driver for the chip QMC5883L, 3-Axis Magnetic Sensor.

File: QMC5883L.cpp

Author: Axel Pauli, January 2021
Adaptions for standalone CAN sensor Eckhard Völlm

 ***************************************************************************/


#include "QMC5883L.h"
#include "SetupNG.h"
#include "sensor.h"

#include <cassert>
#include <cmath>

/* The default I2C address of this chip */
#define QMC5883L_ADDR 0x0D

/* Register numbers */
// #define REG_X_LSB 0         // Output Data Registers for magnetic sensor.
// #define REG_X_MSB 1
// #define REG_Y_LSB 2
// #define REG_Y_MSB 3
// #define REG_Z_LSB 4
// #define REG_Z_MSB 5
// #define REG_STATUS 6        // Status Register.
// #define REG_TEMP_LSB 7      // Output Data Registers for temperature.
// #define REG_TEMP_MSB 8
#define REG_CONTROL1 9      // Control Register #1.
#define REG_CONTROL2 0xA    // Control Register #2.
#define REG_RST_PERIOD 0xB  // ET/RESET Period Register.
// #define REG_RESERVED 0xC    // Reserved.
#define REG_CHIP_ID 0xD     // Chip ID register.

/* Flags for Control Register 1. */
#define MODE_STANDBY    0 // Standby mode.
#define MODE_CONTINUOUS 1 // Continuous read mode.

/* Flags for Status Register #2. */
#define INT_ENB  0x01 // Interrupt Pin Enabling
#define POL_PNT  0x40 // Pointer Roll-over
#define SOFT_RST 0x80 // Soft Reset

/*
  Creates instance for I2C connection with passing the desired parameters.
  No action is done at the bus. Note if i2cBus is not set in the constructor,
  you have to set it by calling method setBus(). The default address of the
  chip is 0x0D.
 */
QMC5883L::QMC5883L( const uint8_t odrIn, const uint8_t rangeIn, const uint16_t osrIn, I2C_t *i2cBus ) : 
	QMCbase( odrIn, rangeIn, osrIn, i2cBus )
{
	// set address to default value of chip, if it is zero.
	addr = QMC5883L_ADDR;
	ESP_LOGI( FNAME, "QMC5883L( %x )", addr );

	// Set register addresses
	REG_X_LSB = 0;
	REG_STATUS = 6;

	// The Gauss range
	microTesla_gain = 2.f/12000.f;
	if ( range == RANGE_8GAUSS ) {
		microTesla_gain = 8.f/3000.f;
	}
	// The sensors µT gain (1G = 0.1mT)
	microTesla_gain = 100.f;
}


// scan bus for I2C address
bool QMC5883L::selfTest()
{
	ESP_LOGI( FNAME, "QMC5883L selftest addr %x", addr);

	// Soft Reset
	ESP_LOGI( FNAME, "RST QMC5883L");
	esp_err_t err = i2c_bus->writeByte( addr, REG_CONTROL2, SOFT_RST );
	if( err != ESP_OK  ) {
		ESP_LOGI( FNAME, "RST QMC5883L failed");
		return false;
	}
	i2c_bus->writeByte( addr, REG_CONTROL2, 0 ); // Clear soft reset
	delay(20);

	uint8_t chipId = 0;
	// Try to read Register 0xD, it delivers the chip id 0xff for a QMC5883L
	for( int i=0; i< 10; i++ ){
		if( i2c_bus->readByte( addr, REG_CHIP_ID, &chipId ) == ESP_OK ) {
			break;
		}
		delay(5);
	}
	if( ! chipId ){
		ESP_LOGE( FNAME,"QMC5883L Read Chip ID failed" );
		return false;
	}
	if( chipId != 0xff ) {
		ESP_LOGE( FNAME, "QMC5883L self-test, detected chip ID 0x%02X is unsupported, expected 0xFF",	chipId );
		return false;
	}
	ESP_LOGI( FNAME, "QMC5883L found");
	sensor = QMC5883;

	return true;
}

/**
 * Configure the device with the set parameters and set the mode to continuous.
 * That means, the device starts working.
 */
bool QMC5883L::initialize()
{
	esp_err_t e1, e2, e3;

	ESP_LOGI( FNAME, "initialize dataRate: %d Oversampling: %d Range: %d", odr, osr, range );

	// Enable ROL_PTN, Pointer roll over function.
	e1 = writeRegister( REG_CONTROL2, POL_PNT );

	// Define SET/RESET period. Should be set to 1
	e2 = writeRegister( REG_RST_PERIOD, 1 );

	// Set mesaurement data and start it in dependency of mode bit.
	ESP_LOGI( FNAME, "initialize dataRate: %d Oversampling: %d", odr, osr );
	e3 = writeRegister( REG_CONTROL1, (osr << 6) | (range <<4) | (odr <<2) | MODE_CONTINUOUS );
	if( e1 == ESP_OK && e2 == ESP_OK && e3 == ESP_OK ) {
		ESP_LOGI( FNAME, "initialize OK");
		return true;
	}
	else{
		ESP_LOGE( FNAME, "initialize ERROR %d %d %d", e1, e2, e3 );
		return false;
	}
	return false;
}
