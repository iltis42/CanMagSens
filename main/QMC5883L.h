/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ****************************************************************************

I2C driver for the chip QMC5883L, 3-Axis Magnetic Sensor.

File: QMC5883L.h

Author: Axel Pauli, January 2021

 ****************************************************************************/

#pragma once

#include "QMCbase.h"


class QMC5883L : public QMCbase
{
public:
	enum { OSR_512=0, OSR_256=1, OSR_128=2, OSR_64=3 };
	enum { RANGE_2GAUSS=0, RANGE_8GAUSS=1 };

	/**
	Creates instance for I2C connection with passing the desired parameters.
	No action is done at the bus. Note if i2cBus is not set in the constructor,
	you have to set it by calling method setBus(). The default address of the
	chip is 0x0D.
	 */
	QMC5883L( const uint8_t odr, const uint8_t range, const uint16_t osr, I2C_t *i2cBus=nullptr );
	virtual ~QMC5883L() = default;

	/** Probe for the QMC5883 chip */
	virtual bool selfTest();

	/** Configure the chip */
	virtual bool initialize();
};
