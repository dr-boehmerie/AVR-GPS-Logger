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
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "i2c.h"
#include "hyt.h"


#if HYT_ENABLE

#define HYT_DEW_POINT_A				0.6112
#define HYT_DEW_POINT_B				17.368
#define HYT_DEW_POINT_C				238.88
#define HYT_DEW_POINT_SCALER		90


uint8_t				hyt_timer;		// Intervallzähler
volatile uint8_t	hyt_state;		// Automatenzustand (wird im OCR0A gesetzt)

struct hyt_data		hyt_buffer;
struct hyt_results	hyt_result;

#if HYT_AVERAGE_NO > 0

struct hyt_results	hyt_array[HYT_AVERAGE_NO];
uint8_t				hyt_index;

int16_t				hyt_temp_sum;
int16_t				hyt_humid_sum;

struct hyt_results	hyt_avg;

struct hyt_dew_point_para
{
	uint8_t		m_t;
	uint8_t		m_rh;
	uint16_t	n;
};

const struct hyt_dew_point_para	hyt_dewPointTab[][100 / 5 - 1] PROGMEM =
{
//			m_t	m_rh	n
	{
/*  5*/	{	65,	134,	3833	},
		{	70,	83,		3346	},
		{	73,	61,		3032	},
		{	75,	49,		2796	},
		{	77,	41,		2605	},
		{	79,	35,		2443	},
		{	80,	31,		2303	},
		{	81,	28,		2179	},
		{	82,	25,		2068	},
		{	83,	23,		1967	},
		{	84,	21,		1874	},
		{	85,	20,		1788	},
		{	86,	18,		1708	},
		{	86,	17,		1633	},
		{	87,	16,		1563	},
		{	88,	15,		1496	},
		{	88,	15,		1434	},
		{	89,	14,		1374	},
		{	89,	13,		1317	},
	},
	{
/* 10*/	{	64,	144,	3875	},
		{	69,	89,		3402	},
		{	72,	66,		3097	},
		{	75,	52,		2867	},
		{	77,	44,		2680	},
		{	78,	38,		2522	},
		{	80,	33,		2385	},
		{	81,	30,		2264	},
		{	82,	27,		2155	},
		{	83,	25,		2056	},
		{	84,	23,		1966	},
		{	85,	21,		1882	},
		{	85,	20,		1803	},
		{	86,	19,		1730	},
		{	87,	17,		1662	},
		{	88,	17,		1597	},
		{	88,	16,		1535	},
		{	89,	15,		1477	},
		{	89,	14,		1422	},
	},
	{
/* 15*/	{	65,	139,	3854	},
		{	70,	86,		3374	},
		{	73,	63,		3065	},
		{	75,	50,		2832	},
		{	77,	42,		2643	},
		{	78,	36,		2483	},
		{	80,	32,		2344	},
		{	81,	29,		2222	},
		{	82,	26,		2112	},
		{	83,	24,		2011	},
		{	84,	22,		1919	},
		{	85,	20,		1834	},
		{	86,	19,		1755	},
		{	86,	18,		1681	},
		{	87,	17,		1612	},
		{	88,	16,		1546	},
		{	88,	15,		1484	},
		{	89,	14,		1425	},
		{	89,	14,		1369	},
	},
	{
/* 20*/	{	64,	148,	3893	},
		{	69,	92,		3428	},
		{	72,	68,		3128	},
		{	74,	54,		2901	},
		{	76,	45,		2717	},
		{	78,	39,		2561	},
		{	79,	34,		2426	},
		{	81,	31,		2307	},
		{	82,	28,		2199	},
		{	83,	26,		2102	},
		{	84,	24,		2012	},
		{	85,	22,		1929	},
		{	85,	21,		1852	},
		{	86,	19,		1780	},
		{	87,	18,		1712	},
		{	88,	17,		1648	},
		{	88,	16,		1588	},
		{	89,	16,		1530	},
		{	89,	15,		1475	},
	},
	{
/* 25*/	{	64,	153,	3910	},
		{	69,	95,		3453	},
		{	72,	70,		3158	},
		{	74,	56,		2934	},
		{	76,	47,		2753	},
		{	78,	41,		2600	},
		{	79,	36,		2467	},
		{	80,	32,		2349	},
		{	82,	29,		2243	},
		{	83,	27,		2147	},
		{	84,	25,		2059	},
		{	84,	23,		1977	},
		{	85,	21,		1901	},
		{	86,	20,		1830	},
		{	87,	19,		1763	},
		{	88,	18,		1700	},
		{	88,	17,		1641	},
		{	89,	16,		1584	},
		{	89,	15,		1530	},
	},
	{
/* 30*/	{	63,	158,	3926	},
		{	68,	99,		3477	},
		{	72,	73,		3186	},
		{	74,	58,		2967	},
		{	76,	49,		2788	},
		{	78,	42,		2638	},
		{	79,	37,		2507	},
		{	80,	33,		2391	},
		{	81,	30,		2287	},
		{	82,	28,		2192	},
		{	83,	25,		2105	},
		{	84,	24,		2025	},
		{	85,	22,		1951	},
		{	86,	21,		1881	},
		{	87,	20,		1815	},
		{	87,	19,		1753	},
		{	88,	18,		1695	},
		{	89,	17,		1639	},
		{	89,	16,		1586	},
	},
	{
/* 35*/	{	63,	163,	3939	},
		{	68,	102,	3500	},
		{	71,	75,		3214	},
		{	74,	60,		2999	},
		{	76,	50,		2824	},
		{	77,	43,		2676	},
		{	79,	38,		2547	},
		{	80,	34,		2433	},
		{	81,	31,		2331	},
		{	82,	29,		2238	},
		{	83,	26,		2153	},
		{	84,	25,		2074	},
		{	85,	23,		2001	},
		{	86,	22,		1932	},
		{	87,	20,		1868	},
		{	87,	19,		1807	},
		{	88,	18,		1749	},
		{	89,	17,		1695	},
		{	89,	17,		1643	},
	},
	{
/* 40*/	{	63,	169,	3952	},
		{	68,	105,	3521	},
		{	71,	78,		3241	},
		{	73,	62,		3030	},
		{	75,	52,		2858	},
		{	77,	45,		2713	},
		{	79,	40,		2587	},
		{	80,	36,		2475	},
		{	81,	32,		2375	},
		{	82,	30,		2284	},
		{	83,	27,		2200	},
		{	84,	25,		2123	},
		{	85,	24,		2051	},
		{	86,	22,		1984	},
		{	87,	21,		1921	},
		{	87,	20,		1861	},
		{	88,	19,		1805	},
		{	89,	18,		1751	},
		{	89,	17,		1700	},
	},
	{
/* 45*/	{	62,	174,	3963	},
		{	67,	108,	3541	},
		{	71,	80,		3268	},
		{	73,	64,		3060	},
		{	75,	54,		2892	},
		{	77,	46,		2750	},
		{	78,	41,		2626	},
		{	80,	37,		2517	},
		{	81,	33,		2419	},
		{	82,	31,		2330	},
		{	83,	28,		2248	},
		{	84,	26,		2172	},
		{	85,	25,		2102	},
		{	86,	23,		2036	},
		{	87,	22,		1974	},
		{	87,	21,		1916	},
		{	88,	20,		1861	},
		{	89,	19,		1809	},
		{	89,	18,		1759	},
	},
	{
/* 50*/	{	62,	179,	3972	},
		{	67,	112,	3560	},
		{	70,	83,		3293	},
		{	73,	66,		3090	},
		{	75,	56,		2926	},
		{	77,	48,		2786	},
		{	78,	42,		2666	},
		{	80,	38,		2559	},
		{	81,	34,		2463	},
		{	82,	32,		2376	},
		{	83,	29,		2296	},
		{	84,	27,		2222	},
		{	85,	25,		2153	},
		{	86,	24,		2089	},
		{	87,	23,		2029	},
		{	87,	21,		1972	},
		{	88,	20,		1918	},
		{	89,	19,		1867	},
		{	89,	19,		1819	},
	},
};


#endif


void hyt_init (void)
{
#if HYT_TIMER_ENABLE
	// Intervalltimer für HYT-Abfrage
	TCCR0B = 0;
	TCCR0A = _BV(WGM01);
	TCNT0 = 0;
	OCR0A = HYT_TMR0_OCRA_VALUE;
	OCR0B = 0;
	TIMSK0 |= _BV(OCIE0A);

	// Timer starten
	TCCR0B |= HYT_TMR0_PRESCALER;
#endif
}

#if HYT_TIMER_ENABLE

/* Timer0 OCR0A */
ISR (TIMER0_COMPA_vect)
{
	// 100 Hz Timer

	// Intervallzähler inkrementieren
	if (hyt_timer < 99) {
		hyt_timer++;
	} else {
		// Intervall ist voll -> rücksetzen
		hyt_timer = 0;
		// und Lesen anstoßen, wenn frei
		if (hyt_state == HYT_IDLE)
			hyt_state = HYT_START_MEAS;
	}
}

//#endif

uint8_t hyt_process (void)
{
	int8_t	err;
	uint8_t	status;
	int32_t	vali32;

	// ist was zu tun
	switch (hyt_state)
	{
	default:	// idle
		break;

	case HYT_START_MEAS:
		// Messung starten (einfach nur ein Schreibzugriff)
		err = i2c_write (TWI_SLA_HYT321, NULL, 0);
		if (err == 0) {
			// hat geklappt -> warten auf Ergebnis
			hyt_state = HYT_WAIT_DONE;
		} else {
			// wird nochmal probiert
		}
		break;

	case HYT_WAIT_DONE:		// warten auf fertig
		// Status lesen
		err = i2c_read (TWI_SLA_HYT321, &status, 1);
		// Status testen
		if (err == 0 && (status & HYT_STALE_BIT) == 0) {
			// Messung abgeschlossen -> Daten holen
			hyt_state = HYT_READ_DATA;
		} else {
			// Fehler oder Messung noch nicht abgeschlossen
		}
		break;

	case HYT_READ_DATA:		// Daten holen
		// Daten lesen (hier ist das stale-Bit immer gesetzt)
		err = i2c_read (TWI_SLA_HYT321, (uint8_t *)&hyt_buffer, sizeof(hyt_buffer));
		if (err == 0) {
			// Ergebnisse auswerten

			// Temperatur
			vali32 = HYT_TEMP_VALUE(hyt_buffer.tempHigh, hyt_buffer.tempLow);
			// T[°C] = 165 / 2^14 * value - 40
			vali32 *= 1650;
			// aufrunden
			vali32 += (1<<14) - 1;
			vali32 >>= 14;	// valu32 /= 16384;
			vali32 -= 400;
			// merken
			hyt_result.temp = (int16_t)vali32;

			// Feuchte
			vali32 = HYT_CAP_VALUE(hyt_buffer.capHigh, hyt_buffer.capLow);
			// rH[%] = 100 / 2^14 * value
			vali32 *= 1000;
			// aufrunden
			vali32 += (1<<14) - 1;
			vali32 >>= 14;	// valu32 /= 16384;
			// merken
			hyt_result.humid = (int16_t)vali32;

#if HYT_AVERAGE_NO > 0
			// älteste Werte abziehen
			hyt_temp_sum  -= hyt_array[hyt_index].temp;
			hyt_humid_sum -= hyt_array[hyt_index].humid;
			// neue Werte merken
			hyt_array[hyt_index].temp  = hyt_result.temp;
			hyt_array[hyt_index].humid = hyt_result.humid;
			// Index dekrementieren
			if (hyt_index > 0)
				hyt_index--;
			else
				hyt_index = HYT_AVERAGE_NO - 1;
			// neue Werte zur Summe addieren
			hyt_temp_sum  += hyt_result.temp;
			hyt_humid_sum += hyt_result.humid;
			// Mittelwerte
			hyt_avg.temp  = hyt_temp_sum  / HYT_AVERAGE_NO;
			hyt_avg.humid = hyt_humid_sum / HYT_AVERAGE_NO;
#endif

			// fertig
			hyt_state = HYT_DATA_READY;
		} else {
			// Fehler -> wieder idle
			hyt_state = HYT_IDLE;
		}
		break;
	}
	// aktuellen Zustand zurückgeben
	return hyt_state;
}

#else

int8_t hyt_read (uint8_t *data, uint8_t len)
{
	int8_t	err;
	uint8_t	status, wait;

	// Messung starten (einfach nur ein Schreibzugriff)
	err = i2c_write (TWI_SLA_HYT321, NULL, 0);
	if (err == 0) {
		wait = 250;
		do {
			// kurz warten
			_delay_us (100);
			// Status lesen
			err = i2c_read (TWI_SLA_HYT321, &status, 1);
			// Status testen
			if ((status & HYT_STALE_BIT) == 0) {
				// Messung abgeschlossen -> raus
				break;
			}
			// Timeout
			if (--wait == 0)
				return -1;
			// Schleife
		} while (err == 0);

		if (err == 0) {
			// Daten lesen (hier ist das stale-Bit immer gesetzt)
			err = i2c_read (TWI_SLA_HYT321, data, len);
		}
	}
	return err;
}

#endif	// HYT_TIMER_ENABLE


int16_t hyt_calcDewPoint (int16_t rh, int16_t t)
{
	int16_t		t0, r0;
	uint16_t	a, b;

	struct hyt_dew_point_para	para;

	if (rh <= 0 || t <= 0)
		return 0;

	if (rh >= 1000)
		return t;

	if (t >= 500)
		return t;

	// 5 Grad Schritte
	t0 = t / 50;
	// 5 % Schritte
	r0 = rh / 50;

	// Parameter laden
	memcpy_P (&para, &hyt_dewPointTab[t0][r0], sizeof(para));

	// negative Quellgrößen wurden bereits abgefangen
	a = (uint16_t)para.m_t * (uint16_t)t;
	b = (uint16_t)para.m_rh * (uint16_t)rh;

	// addieren und den Faktor 10 der Quellgrößen rauswerfen
	a = (a + b) / 10;

	// wenn a kleiner als n, dann wäre das Ergebnis negativ -> 0 zurückgeben
	if (a > para.n) {
		// positiver Taupunkt
		a = a - para.n;
		// mal 10 liefert eine Kommastelle, läuft laut Tabelle auch nicht über
		a = a * 10;
		// durch den Faktor teilen
		a = a / HYT_DEW_POINT_SCALER;

		// fertig
		return (int16_t)a;

	} else {
		// 0 oder negativer Taupunkt -> 0
	}

	return 0;
}

#endif

