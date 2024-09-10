/**************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************

I2C driver for the chip QMC6310U, 3-Axis Magnetic Sensor.

File: QMC6310U.cpp

Author: Axel Pauli, January 2021
Adaptions for standalone CAN sensor Eckhard Völlm

 ***************************************************************************/


#include "QMC6310U.h"
#include "SetupNG.h"
#include "sensor.h"

#include <cassert>
#include <cmath>

/* The default I2C addresses of this chip */
#define QMC6310U_ADDR1 0x1C
#define QMC6310U_ADDR2 0x3C

// /* Register numbers */
#define REG_CHIP_ID 0
// #define REG_X_LSB 1         // Output Data Registers for magnetic sensor.
// #define REG_X_MSB 2
// #define REG_Y_LSB 3
// #define REG_Y_MSB 4
// #define REG_Z_LSB 5
// #define REG_Z_MSB 6
// #define REG_STATUS 9        // Status Register.
// #define REG_TEMP_LSB 7      // Output Data Registers for temperature.
// #define REG_TEMP_MSB 8
#define REG_CONTROL1 0x0A   // Control Register #1.
#define REG_CONTROL2 0x0B   // Control Register #2.
// #define REG_RESERVED 0xC    // Reserved.

/* Flags for Control Register 1. */
#define MODE_STANDBY    0b00 // Standby mode.
#define MODE_NORMAL     0b01
#define MODE_SINGLE     0b10
#define MODE_CONTINUOUS 0b11 // Continuous read mode.

/* Flags for Status Register #2. */
#define SOFT_RST 0x80 // Soft Reset

// Offset control
#define MODE_SET_ON_RESET_ON    0b00
#define MODE_SET_ON             0b01
#define MODE_SET_OFF_RESET_OFF  0b11

/*
  Creates instance for I2C connection with passing the desired parameters.
  No action is done at the bus. Note if i2cBus is not set in the constructor,
  you have to set it by calling method setBus(). The default address of the
  chip is 0x0D.
 */
QMC6310U::QMC6310U( const uint8_t odrIn, const uint8_t rangeIn, const uint16_t osrIn, I2C_t *i2cBus ) : 
	QMCbase( odrIn, rangeIn, osrIn, i2cBus )
{
    // set address to default value of chip
    addr = QMC6310U_ADDR1;
	ESP_LOGI( FNAME, "QMC6310U( %x )", addr );

	// Set register addresses
	REG_X_LSB = 1;
	REG_STATUS = 9;
}


// scan bus for I2C address
bool QMC6310U::selfTest()
{
	ESP_LOGI( FNAME, "QMC6310U selftest");

	uint8_t addresses[2] = { QMC6310U_ADDR1, QMC6310U_ADDR2 };
	esp_err_t err;
	// Soft Reset
	for( int a=0; a<2; a++)	{
		err = i2c_bus->writeByte( addresses[a], REG_CONTROL2, SOFT_RST );
		if( err == ESP_OK  ) {
			ESP_LOGI( FNAME, "RST QMC6310U @ addr %x succeeded", addresses[a] );
			addr = addresses[a]; // address found
			break;
		}
	}
	i2c_bus->writeByte( addr, REG_CONTROL2, 0 );  // Clear soft reset
	delay(20);

	// Try to read Register 0x00, it delivers the chip id 0x80 for a QMC6310
	ESP_LOGI( FNAME, "QMC6310 selftest check chip ID");
	uint8_t chipId = 0;
	for( int i=0; i< 10; i++ ){
		err = i2c_bus->readByte( addr, REG_CHIP_ID, &chipId );
		if( err == ESP_OK ) {
			break;
		}
		delay(5);
	}

	if( ! chipId ) {
		ESP_LOGE( FNAME,"QMC6310 Read Chip ID failed");
		return false;
	}
	if( chipId != 0x80 ) {
		ESP_LOGE( FNAME, "QMC6310 self-test, detected chip ID 0x%02X is unsupported, expected 0x80", chipId );
		return false;
	}
	ESP_LOGI( FNAME, "QMC6310 found");
	sensor = QMC6310;

	return true;
}


/**
 * Configure the device with the set parameters and set the mode to continuous.
 * That means, the device starts working.
 */
bool QMC6310U::initialize()
{
	esp_err_t e1, e2, e3;

	ESP_LOGI( FNAME, "initialize dataRate: %d Oversampling: %d Range: %d", odr, osr, range );
	// range and offset management control
	uint8_t rdtmp, temp = (range << 2) | MODE_SET_ON_RESET_ON;
	ESP_LOGI( FNAME, "initialize Reg2: %x", temp );
	e1 = i2c_bus->writeByte( addr, REG_CONTROL2, temp );
	delay(2);
	e2 = i2c_bus->readByte( addr, REG_CONTROL2, &rdtmp );
	ESP_LOGI( FNAME, "initialize Reg2 read: %x (%d)", rdtmp, e2 );
	
	temp = (osr << 4) | (odr << 2) | MODE_CONTINUOUS;
	ESP_LOGI( FNAME, "initialize Reg1: %x", temp );
	e3 = writeRegister( REG_CONTROL1, temp );
	delay(2);
	e2 = i2c_bus->readByte( addr, REG_CONTROL1, &rdtmp );
	ESP_LOGI( FNAME, "initialize Reg1 read: %x (%d)", rdtmp, e2 );

	if( e1 == ESP_OK && e3 == ESP_OK ) {
		ESP_LOGI( FNAME, "initialize QMC6310U OK");
		return true;
	}
	else{
		ESP_LOGE( FNAME, "initialize QMC6310U ERROR %d %d", e1, e3 );
		return false;
	}
	return false;
}

// On the QMC6310 PCB the chip is mounted in a different orientation
void QMC6310U::xyz_map( int16_t &xout, int16_t &yout, int16_t &zout )
{
	// Rotate 180 around Z axis
	xout = -xout;
	yout = -yout;
}

