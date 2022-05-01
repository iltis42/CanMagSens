/**************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************

I2C driver for the chip QMC6310U, 3-Axis Magnetic Sensor.

QMC6310U data sheet:

https://datasheetspdf.com/pdf-file/1309218/QST/QMC6310U/1

File: QMC6310U.cpp

Author: Axel Pauli, January 2021
Adaptions for standalone CAN sensor Eckhard Völlm

Last update: 2021-09-09

 ***************************************************************************/

// Activate/deactivate debug messages
// #define DEBUG_COMP 1

#include <cassert>
#include <cmath>
#include "QMC6310U.h"
#include "SetupNG.h"
#include "sensor.h"

/* Register numbers */

#define QMC6310_CHIP_ID_REG             0x00
#define REG_X_LSB 1         // Output Data Registers for magnetic sensor.
#define REG_X_MSB 2
#define REG_Y_LSB 3
#define REG_Y_MSB 4
#define REG_Z_LSB 5
#define REG_Z_MSB 6
#define REG_STATUS 9        // Status Register.
#define REG_TEMP_LSB 7      // Output Data Registers for temperature.
#define REG_TEMP_MSB 8
#define REG_CONTROL1 0x0A      // Control Register #1.
#define REG_CONTROL2 0x0B    // Control Register #2.

#define REG_RESERVED 0xC    // Reserved.

/* Flags for Status Register #1. */
#define STATUS_DRDY 1 // Data Ready.
#define STATUS_OVL  2  // Overflow flag.


#define SOFT_RST 0x80      // Soft Reset.

/* Flags for Control Register 1. */
#define MODE_STANDBY    0b00  // Standby mode.
#define MODE_NORMAL     0b01
#define MODE_SINGLE     0b10
#define MODE_CONTINUOUS 0b11  // Continuous read mode.

// Offset control
#define MODE_SET_ON_RESET_ON    0b00
#define MODE_SET_ON             0b01
#define MODE_SET_OFF_RESET_OFF  0b11

// Sensor state
bool QMC6310U::m_sensor = false;
// Sensor overflow flag.
bool QMC6310U::overflowWarning = false;
// Error counter
int QMC6310U::errors = 0;
int QMC6310U::totalReadErrors = 0;
Average<20> QMC6310U::filterX;
Average<20> QMC6310U::filterY;
Average<20> QMC6310U::filterZ;

/*
  Creates instance for I2C connection with passing the desired parameters.
  No action is done at the bus. Note if i2cBus is not set in the constructor,
  you have to set it by calling method setBus(). The default address of the
  chip is 0x0D.
 */
QMC6310U::QMC6310U( const uint8_t addrIn,
		const uint8_t odrIn,
		const uint8_t rangeIn,
		const uint16_t osr1In,
		const uint16_t osr2In,
		I2C_t *i2cBus ) : i2c_bus( i2cBus ), addr( addrIn ), odr( odrIn ), range( rangeIn ), osr1( osr1In ), osr2( osr2In )
{
	ESP_LOGI( FNAME, "QMC6310U( %02X )", addrIn );
	if( addrIn == 0 )
	{
		// set address to default value of chip, if it is zero.
		addr = QMC6310U_ADDR;
	}
	overflowWarning = false;
	xraw = 0;
	yraw = 0;
	zraw = 0;
}

QMC6310U::~QMC6310U()
{
}

/** Write with data part. */
esp_err_t QMC6310U::writeRegister( const uint8_t addr,	const uint8_t reg,	const uint8_t value )
{
	esp_err_t err = i2c_bus->writeByte( addr, reg, value );
	if( err != ESP_OK )	{
		ESP_LOGE( FNAME, "QMC6310U writeRegister( 0x%02X, 0x%02X, 0x%02X ) FAILED",	addr, reg, value );
		return ESP_FAIL;
	}
	return err;
}

/**
 * Read bytes from the chip.
 * Return the number of read bytes or 0 in error case.
 */
uint8_t QMC6310U::readRegister( const uint8_t addr,
		const uint8_t reg,
		const uint8_t count,
		uint8_t *data  )
{
	// read bytes from chip
	for( int i=0; i<=100; i++ ){
		esp_err_t err = i2c_bus->readBytes( addr, reg, count, data );
		if( err == ESP_OK && count == 6 ){
			return count;
		}
		else
		{
			// ESP_LOGW( FNAME,"readRegister( 0x%02X, 0x%02X, %d ) FAILED N:%d", addr, reg, count, i );
			if( i == 100 ){
				ESP_LOGW( FNAME,"100 retries read mag sensor failed also, now try to reinitialize chip");
				if( !initialize() )
					initialize();  // one retry
				err = i2c_bus->readBytes( addr, reg, count, data );
				if( err == ESP_OK && count == 6 ){
					// ESP_LOGI( FNAME,"Read after reinit ok");
					return count;
				}
			}
			// ESP_LOGW( FNAME,"retry #%d reg=%02X err=%d", i, reg, err );
			delay(2);
		}
	}
	ESP_LOGW( FNAME,"Read after init and retry failed also, return with no data, len=0");
	return 0;
}

// scan bus for I2C address
bool QMC6310U::selfTest()
{
	ESP_LOGI( FNAME, "QMC6310U selftest");
	uint8_t chipId = 0;
	// Try to read Register 0x00, it delivers the chip id 0x80 for a QMC6310
	m_sensor = false;
	ESP_LOGI( FNAME, "QMC6310 selftest chip ID");
	for( int i=0; i< 10; i++ ){
		esp_err_t err = i2c_bus->readByte( addr, QMC6310_CHIP_ID_REG, &chipId );
		if( err == ESP_OK ){
			m_sensor = true;
			break;
		}
		delay(20);
	}
	if( !m_sensor ){
		ESP_LOGE( FNAME,"QMC6310 Read Chip ID failed");
		return false;
	}
	if( chipId != 0x80 ){
		m_sensor = false;
		ESP_LOGE( FNAME, "QMC6310 self-test, detected chip ID 0x%02X is unsupported, expected 0xFF",	chipId );
		return false;
	}
	m_sensor = true;
	ESP_LOGI( FNAME, "QMC6310 selftest chip ID PASSED");
	return true;
}

/**
 * Configure the device with the set parameters and set the mode to continuous.
 * That means, the device starts working.
 */
bool QMC6310U::initialize()
{
	esp_err_t e1, e2, e3, e4;
	e1 = e2 = e3 = e4 = 0;
	uint8_t addresses[2] = { QMC6310U_ADDR, QMC6310U_ADDR2 };
	addr = addresses[1];
	bool ok=false;
	// Soft Reset
	for( int i=0; i<2; i++)	{
		addr = addresses[i];
		ESP_LOGI( FNAME, "initialize() I2C addr %x", addr );
		e1 = writeRegister( addr, REG_CONTROL1, SOFT_RST );
		if( e1 != ESP_OK  ) {
			ESP_LOGI( FNAME, "initialize() QMC6310U addr: %x REG_CONTROL2: %x write: %x failed", addr, REG_CONTROL2, SOFT_RST );
		}
		else{
			ok = true;
			break;
		}
	}
	if( ok )
		ESP_LOGI( FNAME, "initialize() QMC6310U detected addr: %x", addr );
	else
		return false;
	delay(2);
	if( !selfTest() )
		return false;

	// soft reset, range and offset management control
	e2 = writeRegister( addr, REG_CONTROL2, (range << 2) | MODE_SET_ON_RESET_ON );

	// ESP_LOGI( FNAME, "initialize() dataRate: %d Oversampling: %d", used_odr, used_osr );
	e4 = writeRegister( addr, REG_CONTROL1,	(osr1 << 6) | (osr2 <<4) | (odr <<2) | MODE_CONTINUOUS );
	if( e1 == ESP_OK || e2 == ESP_OK || e3 == ESP_OK || e4 == ESP_OK ) {
		ESP_LOGI( FNAME, "initialize() QMC6310U OK");
		return true;
	}
	else{
		ESP_LOGE( FNAME, "initialize() QMC6310U ERROR %d %d %d %d", e1,e2,e3,e4 );
		return false;
	}
	return false;
}

/**
 * Read out the registers X, Y, Z (0...5) in raw format.
 *
 * The status register bits are checked, if the heading data available, if no
 * overflow has occurred and if no data locking is shown.
 *
 * Returns true in case of success otherwise false.
 */

bool QMC6310U::begin( gpio_num_t sda, gpio_num_t scl, int i2c_clock ){
	i2c_bus->begin(sda, scl, GPIO_PULLUP_DISABLE, GPIO_PULLUP_DISABLE, i2c_clock );
	bool ret = initialize();
	return ret;
}


bool QMC6310U::rawHeading( int16_t &xout, int16_t &yout, int16_t &zout )
{
	uint8_t data[6];
	uint8_t status = 0;
	// Check, if data are available
	// as sensor is outside the housing, there may be I2C noise, so we need retries
	bool okay = false;
	// Poll status until RDY or DOR
	esp_err_t ret = ESP_OK;
	for( int i=1; i<30; i++ ){
		ret = i2c_bus->readByte( addr, REG_STATUS, &status );
		if( ret == ESP_OK ){
			if( (status & STATUS_DRDY)   ){
				okay = true;
				break;
			}
			// else
			// 	ESP_LOGW( FNAME, "No new data,  N=%d  RDY%d  DOR%d REG:%02X", i, status & STATUS_DRDY, status & STATUS_DOR, status );
		}
		else{
			delay( 2 );
			// ESP_LOGW( FNAME, "read REG_STATUS failed, N=%d  RDY%d  DOR%d", i, status & STATUS_DRDY, status & STATUS_DOR );
		}
	}
	if( okay == false )
	{
		// ESP_LOGE( FNAME, "read REG_STATUS FAILED %d", ret );
		return false;
	}

	// Precondition already checked in loop before, point only reached if there is RDY or DOR
	int count = readRegister( addr, REG_X_LSB, 6, data );
	// Data can be read in every case
	if( count == 6 )
	{
		int x = -(int)( (int16_t)(( data[1] << 8 ) | data[0]) );
		int y = (int)( (int16_t)(( data[3] << 8 ) | data[2]) );
		int z = -(int)( (int16_t)(( data[5] << 8 ) | data[4]) );

		xraw = filterX( x );
		yraw = filterY( y );
		zraw = filterZ( z );
		xout = (int16_t)xraw;
		yout = (int16_t)yraw;
		zout = (int16_t)zraw;
		ESP_LOGI( FNAME, "X:%d Y:%d Z:%d  RDY:%d OVL:%d", xraw, yraw,zraw, status & STATUS_DRDY, status & STATUS_OVL );
		return true;
	}
	ESP_LOGE( FNAME, "read Register REG_X_LSB returned count != 6, count: %d", count );
	return false;
}

/**
 * Read temperature in degree Celsius. If ok is passed, it is set to true,
 * if temperature data is valid, otherwise it is set to false.
 */
int16_t QMC6310U::temperature( bool *ok )
{
	assert( (ok != nullptr) && "Passing of NULL pointer is forbidden" );

	uint8_t data[2];
	if( readRegister( addr, REG_TEMP_LSB, 2, data ) == 0 ){
		if( ok != nullptr )
			*ok = false;
		// Nothing has been read
		return 0.0;
	}
	int16_t t = ( data[1] << 8 ) | data[0];

	if( ok != nullptr )
		*ok = true;
	return t;
}

