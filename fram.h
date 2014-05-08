/*
 * Author: Michael Böhme
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef FRAM_H
#define FRAM_H

#include <inttypes.h>

#include "config.h"

#if FRAM_ENABLE

#define FRAM_CS_ACTIVE		PORTB &= ~_BV(PB2)
#define FRAM_CS_INACTIVE	PORTB |=  _BV(PB2)


#define FRAM_CMD_READ		0x03
#define FRAM_CMD_WRITE		0x02
#define FRAM_CMD_WREN		0x06
#define FRAM_CMD_WRDI		0x04
#define FRAM_CMD_READST		0x05
#define FRAM_CMD_WRITEST	0x01

// SPI initialisieren
void fram_init (void);

// Bytes lesen
int8_t fram_read (uint16_t addr, uint8_t *data, uint8_t len);

// Bytes schreiben
int8_t fram_write (uint16_t addr, const uint8_t *data, uint8_t len);

#endif	// FRAM_ENABLE
#endif	// FRAM_H
