/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ****************************************************************************

I2C driver for the chip QMC6310U, 3-Axis Magnetic Sensor.

QMC6310U data sheet:

https://datasheetspdf.com/pdf-file/1309218/QST/QMC6310U/1

File: QMC6310U.h

Author: Axel Pauli, January 2021

Last update: 2021-03-28

 ****************************************************************************/

#pragma once

#include <sys/time.h>
#include "esp_system.h"
#include "logdef.h"
#include "esp_log.h"
#include "I2Cbus.hpp"
#include "average.h"

/* The default I2C addresses of this chip */
#define QMC6310U_ADDR  0x1C
#define QMC6310U_ADDR2 0x3C

typedef enum e_oversampling_osr1 { OSR1_1=3, OSR1_2=2, OSR1_4=1, OSR1_8=0 } e_oversampling_osr1_t;  // Oversampling
typedef enum e_oversampling_osr2 { OSR2_8=3, OSR2_4=2, OSR2_2=1, OSR2_1=0 } e_oversampling_osr2_t;  // Downsampling

// typedef enum e_datarate_63 { ODR_10Hz=0, ODR_50Hz=1, ODR_100Hz=2, ODR_200Hz=3 } e_datarate_63_t;
typedef enum e_magn_range_63 { RANGE63_30GAUSS=0, RANGE63_12GAUSS=1, RANGE63_8GAUSS=1,RANGE63_2GAUSS=3 } e_mag_range_63_t;


class QMC6310U
{
public:
	/*
    Creates instance for I2C connection with passing the desired parameters.
    No action is done at the bus. Note if i2cBus is not set in the constructor,
    you have to set it by calling method setBus(). The default address of the
    chip is 0x0D.
	 */
	QMC6310U( const uint8_t addr,
			      const uint8_t odr,
			      const uint8_t range,
			      const uint16_t osr1,
				  const uint16_t osr2,
			      I2C_t *i2cBus=nullptr );

	~QMC6310U();

	/**
	 * Returns true, if the self test has been passed successfully, otherwise
	 * false.
	 */
	static bool haveSensor()
	{
	  return m_sensor;
	}

	bool begin( gpio_num_t sda, gpio_num_t scl, int i2c_clock );

	/** Check for reply with I2C bus address */
	bool selfTest();

	/**
	 * Configure the device with the set parameters and set the mode to continuous.
	 * That means, the device starts working.
	 * you max give output datarate (odr) and oversampling rate (osr)
	 * if not given defaults are as from constructor
	 */
	bool initialize();

	/**
	 * Read temperature in degree Celsius. Ok is set to true,
	 * if temperature data is valid, otherwise it is set to false.
	 */
	short temperature( bool *ok );

	/**
	 * Read out the registers X, Y, Z (0...5) in raw format.
	 * Returns true in case of success otherwise false.
	 */
	bool rawHeading( int16_t &xout, int16_t &yout, int16_t &zout);

	/**
	 * Return the overflow status flag. It is set to true, if any data of three
	 * axis magnetic sensor channels is out of range.
	 */
	static bool overflowFlag()
	{
	  return overflowWarning;
	}


	int getReadError(){ return totalReadErrors; };

	/**
	 * Read bytes from the chip.
	 * Return the number of read bytes or 0 in error case.
	 */
	uint8_t readRegister( const uint8_t addr, const uint8_t reg, const uint8_t count, uint8_t *data );

	esp_err_t writeRegister( const uint8_t addr, const uint8_t reg,	const uint8_t value );

private:

	/**
	 * Get time in ms since 1.1.1970
	 */
	uint64_t getMsTime()
	{
	  struct timeval tv;
	  gettimeofday( &tv, nullptr );
	  return ( tv.tv_sec * 1000 ) + ( tv.tv_usec / 1000 );
	}


	static bool m_sensor;
	I2C_t* i2c_bus;

	/** Read raw values from the chip averaged */
	int xraw, yraw, zraw;

	uint8_t addr; // chip adress
	uint8_t odr;  // output data rate
	uint8_t range; // magnetic resolution of sensor
	uint8_t osr1; // over sample ratio
	uint8_t osr2; // down sample ratio
	static bool overflowWarning;
	static int errors;
	static int   totalReadErrors;
	static Average<20> filterX;
	static Average<20> filterY;
	static Average<20> filterZ;
};
