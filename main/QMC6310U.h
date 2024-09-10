/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ****************************************************************************

I2C driver for the chip QMC6310U, 3-Axis Magnetic Sensor.

File: QMC6310U.h

Author: Axel Pauli, January 2021

 ****************************************************************************/

#pragma once

#include "QMCbase.h"


class QMC6310U : public QMCbase
{
public:
	enum { OSR1_1=3, OSR1_2=2, OSR1_4=1, OSR1_8=0 };
	enum { RANGE_30GAUSS=0, RANGE_12GAUSS=1, RANGE_8GAUSS=2, RANGE_2GAUSS=3 };

	/**
	Creates instance for I2C connection with passing the desired parameters.
	No action is done at the bus. Note if i2cBus is not set in the constructor,
	you have to set it by calling method setBus(). The default address of the
	chip is 0x0D.
	*/
	QMC6310U( const uint8_t odr, const uint8_t range, const uint16_t osr, I2C_t *i2cBus=nullptr );
	virtual ~QMC6310U() = default;

	/** Probe for the QMC6310U chip */
	virtual bool selfTest();

	/** Configure the chip */
	virtual bool initialize();

	// Map raw readings to defined XYZ orientation
	virtual void xyz_map( int16_t &xout, int16_t &yout, int16_t &zout );
};
