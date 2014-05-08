/*
 * Author: Michael Böhme
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef HYT_H
#define HYT_H

#include <inttypes.h>

#include "config.h"

#if HYT_ENABLE

// HYT321
#define TWI_SLA_HYT321		(0x28 << 1)

#define HYT_CMODE_BIT		(1<<7)
#define HYT_STALE_BIT		(1<<6)

#define HYT_STATUS_VALUE(_VAL_)		((_VAL_) & (HYT_CMODE_BIT | HYT_STALE_BIT))
#define HYT_CAP_VALUE(_HI_,_LO_)	(((uint16_t)((_HI_) & 0x3F) << 8) | (uint16_t)(_LO_))
#define HYT_TEMP_VALUE(_HI_,_LO_)	(((uint16_t)(_HI_) << 6) | (uint16_t)((_LO_) >> 2))

// F_CPU / 1024 = 14745600/1024 = 14400
#define HYT_TMR0_PRESCALER			(_BV(CS02) | _BV(CS00))
// 14400 / (143 + 1) = 100 Hz
#define HYT_TMR0_OCRA_VALUE			143

// Zustandsautomat
enum HYT_STATE
{
	HYT_IDLE,
	HYT_START_MEAS,
	HYT_WAIT_DONE,
	HYT_READ_DATA,
	HYT_DATA_READY,
};

// nachdem die Daten ausgewertet wurden -> wieder zurück zu idle
#define HYT_SET_STATE_IDLE()	hyt_state = HYT_IDLE


struct hyt_data
{
	uint8_t		capHigh;
	uint8_t		capLow;
	uint8_t		tempHigh;
	uint8_t		tempLow;
};

struct hyt_results
{
	int16_t		temp;
	int16_t		humid;
};

extern uint8_t				hyt_timer;		// Intervallzähler
extern volatile uint8_t		hyt_state;		// Automatenzustand (wird im OCR0A gesetzt)
extern struct hyt_results	hyt_result;		// Ergebnisspeicher

#if HYT_AVERAGE_NO > 0
extern struct hyt_results	hyt_avg;		// Mittelwerte über die letzten Ergebnisse
#endif

void hyt_init (void);

#if HYT_TIMER_ENABLE
// Zustandsautomat
uint8_t hyt_process (void);
#else
// Messung starten und Ergebnis abholen in einer Funktion
int8_t hyt_read (uint8_t *data, uint8_t len);
#endif

int16_t hyt_calcDewPoint (int16_t rh, int16_t t);

#endif	// HYT_ENABLE
#endif	// HYT_H
