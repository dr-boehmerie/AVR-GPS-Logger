/*
 * Author: Michael Böhme
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef I2C_H
#define I2C_H

#include <inttypes.h>

#include "config.h"

#if I2C_ENABLE

// Funktionen für I2C
//void i2c_error(uint8_t fnr);
int8_t i2c_write (uint8_t addr, const uint8_t *data, uint8_t len);
int8_t i2c_read (uint8_t addr, uint8_t *data, uint8_t len);

#endif	// I2C_ENABLE
#endif	// I2C_H