/*
 * Author: Michael Böhme
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include <inttypes.h>
#include <stdlib.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "fram.h"


#if FRAM_ENABLE

void fram_init (void)
{
	volatile uint8_t	reg;

	// SPI Master, MSB First, Clk/64 -> 230kHz
	SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR1) /*| _BV(SPR0)*/;

	// clear SPIF bit in SPSR
	reg = SPSR;
	reg = SPDR;

	// Chipselect deaktivieren
	FRAM_CS_INACTIVE;
}

uint8_t fram_xferByte (uint8_t byte)
{
	volatile uint8_t	reg;

	// Daten ins Register schreiben
	SPDR = byte;

	// warten bis die Übertragung abgeschlossen ist
	//loop_until_bit_is_set(SPSR, SPIF);
	do {reg = SPSR;} while ((reg & _BV(SPIF)) == 0);
	// Datenregister lesen
	reg = SPDR;

	return reg;
}

int8_t fram_read (uint16_t addr, uint8_t *data, uint8_t len)
{
	// Chipselect aktivieren
	FRAM_CS_ACTIVE;

	// Kommando senden
	fram_xferByte (FRAM_CMD_READ);

	// Adresse senden
	fram_xferByte ((addr >> 8) & 0x1F);
	fram_xferByte (addr & 0xFF);

	// Daten lesen
	while (len-- > 0) {
		// Dummy senden, Ergebnis lesen und speichern
		*data++ = fram_xferByte (0xFF);
	}

	// Chipselect deaktivieren
	FRAM_CS_INACTIVE;

	return 0;
}

// Bytes schreiben
int8_t fram_write (uint16_t addr, const uint8_t *data, uint8_t len)
{
	// Chipselect aktivieren
	FRAM_CS_ACTIVE;

	// Write Enable Kommando senden
	fram_xferByte (FRAM_CMD_WREN);

	// Chipselect deaktivieren
	FRAM_CS_INACTIVE;

	// Chipselect aktivieren
	FRAM_CS_ACTIVE;

	// Kommando senden
	fram_xferByte (FRAM_CMD_WRITE);

	// Adresse senden
	fram_xferByte ((addr >> 8) & 0x1F);
	fram_xferByte (addr & 0xFF);

	// Daten lesen
	while (len-- > 0) {
		// Daten senden
		fram_xferByte (*data++);
	}

	// Chipselect deaktivieren
	FRAM_CS_INACTIVE;

	return 0;
}

#endif	// FRAM_ENABLE
