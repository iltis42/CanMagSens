/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ****************************************************************************

I2C driver for the chip QMCbase and QMC6310, 3-Axis Magnetic Sensor.

data sheets:
https://datasheetspdf.com/pdf-file/1309218/QST/QMCbase/1
https://datasheetspdf.com/pdf-file/1309218/QST/QMC6310U/1

File: QMCbase.h

Author: Axel Pauli, January 2021

 ****************************************************************************/

#pragma once


#include <I2Cbus.hpp>
#include <hal/gpio_types.h>

#include <cstdint>




class QMCbase
{
public:
	enum { ODR_10Hz=0, ODR_50Hz=1, ODR_100Hz=2, ODR_200Hz=3 };

	/**
    Creates instance for I2C connection with passing the desired parameters.
    No action is done at the bus. Note if i2cBus is not set in the constructor,
    you have to set it by calling method setBus(). The default address of the
    chip is 0x0D.
	 */
	QMCbase( const uint8_t odr, const uint8_t range, const uint16_t osr, I2C_t *i2cBus=nullptr );
	virtual ~QMCbase();

	/**
	 * Returns type of sensor, if the self test has been passed successfully, otherwise 0.
	 * -> instantiate from this class only one specific object at a time (!)
	 */
	static int haveSensor() { return sensor; }

	// Probe, start and configure the chip for use
	bool begin( gpio_num_t sda, gpio_num_t scl, int i2c_clock );

	/**
	 * Read temperature in degree Celsius. Ok is set to true,
	 * if temperature data is valid, otherwise it is set to false.
	 */
	short temperature( bool *ok );

	/**
	 * Read out the registers X, Y, Z (0...5) in raw format.
	 * Returns true in case of success otherwise false.
	 */
	bool rawHeading( int16_t &xout, int16_t &yout, int16_t &zout );

	/**
	 * Read bytes from the chip.
	 * Return the number of read bytes or 0 in error case.
	 */
	uint8_t readRegister( const uint8_t reg, const uint8_t count, uint8_t *data );
	esp_err_t writeRegister( const uint8_t reg,	const uint8_t value );

	// The factor to multiply on rawHeading readings to get [microT]
	float getGain();

protected:
	/** Probe chip Id and check for the proper I2C bus address */
	virtual bool selfTest() = 0;

	/**
	 * Configure the device with the set parameters and set the mode to continuous.
	 * That means, the device starts working.
	 * you max give output datarate (odr) and oversampling rate (osr)
	 * if not given defaults are as from constructor
	 */
	virtual bool initialize() = 0;

	static enum chip_type {
		NoChipFound,
		QMC5883,
		QMC6310
	} sensor; // Set to when a chip got detected

	I2C_t* i2c_bus;
	// Map raw readings to defined XYZ orientation
	virtual void xyz_map( int16_t &xout, int16_t &yout, int16_t &zout );
	// virutal void xyc_reverse_map( int16_t &xout, int16_t &yout, int16_t &zout );

	uint8_t addr;  // chip adress
	uint8_t odr;   // output data rate
	uint8_t range; // magnetic resolution of sensor
	uint8_t osr;   // over sample ratio

	// Common register mappings vor the two supported chip
protected:
	uint8_t REG_X_LSB;         // Output Data Registers for magnetic sensor.
	// not used: REG_X_MSB, REG_Y_LSB, REG_Y_MSB, REG_Z_LSB, REG_Z_MSB
	uint8_t REG_STATUS;        // Status Register.
	static constexpr uint8_t REG_TEMP_LSB = 7; // Output Data Registers for temperature.
	float microTesla_gain;
};
