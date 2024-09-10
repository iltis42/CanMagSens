/**************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************

I2C driver for 3-Axis Magnetic Sensor.

File: QMCba5883L.cpp

Author: Axel Pauli, January 2021
Adaptions for standalone CAN sensor Eckhard Völlm

Last update: 2021-09-09

 ***************************************************************************/

#include "QMCbase.h"
#include "SetupNG.h"
#include "sensor.h"

#include <cassert>
#include <cmath>

/* Flags for Status Register #1. */
#define STATUS_DRDY 1 // Data Ready
#define STATUS_OVL  2 // Overflow flag
#define STATUS_DOR  4 // Data skipped for reading

// Static variables
// Found chip type
enum QMCbase::chip_type QMCbase::sensor = NoChipFound;

/*
  Creates instance for I2C connection with passing the desired parameters.
  No action is done at the bus. Note if i2cBus is not set in the constructor,
  you have to set it by calling method setBus(). The default address of the
  chip is 0x0D.
 */
QMCbase::QMCbase( const uint8_t odrIn, const uint8_t rangeIn, const uint16_t osrIn, I2C_t *i2cBus ) :
	i2c_bus( i2cBus ), odr( odrIn ), range( rangeIn ), osr( osrIn )
{
}

QMCbase::~QMCbase()
{
	if ( i2c_bus ) {
		i2c_bus->close();
	}
}


bool QMCbase::begin( gpio_num_t sda, gpio_num_t scl, int i2c_clock )
{
	i2c_bus->begin(sda, scl, GPIO_PULLUP_DISABLE, GPIO_PULLUP_DISABLE, i2c_clock );

	// Check if really a chip of this type is on the PCB
	if ( selfTest() ) {
		return initialize();
	}
	return false;
}


/** Write with data part. */
esp_err_t QMCbase::writeRegister( const uint8_t reg, const uint8_t value )
{
	esp_err_t err = i2c_bus->writeByte( addr, reg, value );
	if( err != ESP_OK ) {
		return ESP_FAIL;
	}
	return err;
}


/**
 * Read bytes from the chip.
 * Return the number of read bytes or 0 in error case.
 */
uint8_t QMCbase::readRegister( const uint8_t reg, const uint8_t count, uint8_t *data  )
{
	// read bytes from chip
	for( int i=0; i<=100; i++ ) {
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

void QMCbase::xyz_map( int16_t &xout, int16_t &yout, int16_t &zout )
{
	// keep mapping by default
}

/**
 * Read out the registers X, Y, Z (0...5) in raw format.
 *
 * The status register bits are checked, if the heading data available, if no
 * overflow has occurred and if no data locking is shown.
 *
 * Returns true in case of success otherwise false.
 */
bool QMCbase::rawHeading( int16_t &xout, int16_t &yout, int16_t &zout )
{
	uint8_t data[6];
	uint8_t status = 0;
	// Check, if data are available
	// as sensor is outside the housing, there may be I2C noise, so we need retries

	// Poll status until RDY or DOR
	for( int i=1; i<30; i++ ){
		if( i2c_bus->readByte( addr, REG_STATUS, &status ) == ESP_OK ){
			if( status & STATUS_DRDY ) {
				break;
			}
			// else
			// 	ESP_LOGW( FNAME, "No new data,  STAT=%x", i, status );
		}
		// else{
		// 	delay( 2 );
		// 	// ESP_LOGW( FNAME, "read REG_STATUS failed, N=%d  RDY%d  DOR%d", i, status & STATUS_DRDY, status & STATUS_DOR );
		// }
	}
	if( !(status & STATUS_DRDY) || (status & STATUS_OVL) )
	{
		ESP_LOGE( FNAME, "read REG_STATUS failed" );
		return false;
	}

	// Precondition already checked in loop before, point only reached if there is RDY or DOR
	int count = readRegister( REG_X_LSB, 6, data );
	// Data can be read in every case
	if( count == 6 )
	{
		xout = (int)( (int16_t)(( data[1] << 8 ) | data[0]) );
		yout = (int)( (int16_t)(( data[3] << 8 ) | data[2]) );
		zout = (int)( (int16_t)(( data[5] << 8 ) | data[4]) );
		xyz_map(xout, yout, zout);

		ESP_LOGI( FNAME, "X:%d Y:%d Z:%d  STAT:%x", xout, yout, zout, status );
		return true;
	}
	ESP_LOGE( FNAME, "read Register REG_X_LSB returned count != 6, count: %d", count );
	return false;
}

/**
 * Read temperature in degree Celsius. If ok is passed, it is set to true,
 * if temperature data is valid, otherwise it is set to false.
 */
int16_t QMCbase::temperature( bool *ok )
{
	assert( (ok != nullptr) && "Passing of NULL pointer is forbidden" );

	uint8_t data[2];
	if( readRegister( REG_TEMP_LSB, 2, data ) == 0 ){
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

int N=0;
bool holddown=false;
