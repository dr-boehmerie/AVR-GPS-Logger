/*
 * Author: Michael Böhme
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include "config.h"


#if NMEA_ENABLE
#include <inttypes.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <compat/twi.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#include "nmea.h"

enum NMEA_STATE
{
	NMEA_WAIT_START,		/*!< warten auf $ */

	NMEA_START,				/*!< $ empfangen */
	NMEA_START_G,			/*!< $G empfangen */
	NMEA_START_GP,			/*!< $GP empfangen */
	NMEA_WAIT_SEPARATOR,	/*!< warten auf , */
};


// Globals

const char nmea_groups[NMEA_GRP_NO][3] PROGMEM =
{
#if NMEA_GGA_ENABLE
	/* NMEA_GRP_GPGGA	*/	{'G','G','A'},	/*!< fix information */
#endif
#if NMEA_RMC_ENABLE
	/* NMEA_GRP_GPRMC	*/	{'R','M','C'},	/*!< recommended minimum data */
#endif
#if NMEA_VTG_ENABLE
	/* NMEA_GRP_GPVTG	*/	{'V','T','G'},	/*!< vector track and speed over ground */
#endif
#if NMEA_GSA_ENABLE
	/* NMEA_GRP_GPGSA	*/	{'G','S','A'},	/*!< overall satellite data */
#endif
#if NMEA_GSV_ENABLE
	/* NMEA_GRP_GPGSV	*/	{'G','S','V'},	/*!< detailed satellite data */
#endif
};

const uint8_t nmea_result_tab[NMEA_GRP_NO][20] PROGMEM =
{
#if NMEA_GGA_ENABLE
	/* NMEA_GRP_GPGGA	*/
	{
	/* NMEA_GPGGA_0 */		NMEA_RESULT_NONE,
	/* NMEA_GPGGA_TIME */	NMEA_RESULT_NONE,
	/* NMEA_GPGGA_LAT */	NMEA_RESULT_NONE,
	/* NMEA_GPGGA_NS */		NMEA_RESULT_NONE,
	/* NMEA_GPGGA_LONG */	NMEA_RESULT_NONE,
	/* NMEA_GPGGA_EW */		NMEA_RESULT_NONE,
	/* NMEA_GPGGA_QUAL */	NMEA_RESULT_QUAL,
	/* NMEA_GPGGA_SAT_NO */	NMEA_RESULT_SAT_NO,
	/* NMEA_GPGGA_HDIL */	NMEA_RESULT_NONE,
	/* NMEA_GPGGA_ALT */	NMEA_RESULT_ALT,
	/* NMEA_GPGGA_ALTM */	NMEA_RESULT_NONE,
	/* NMEA_GPGGA_HEIGHT */	NMEA_RESULT_NONE,
	/* NMEA_GPGGA_HEIGHTM*/	NMEA_RESULT_NONE,
	/* NMEA_GPGGA_13 */		NMEA_RESULT_NONE,
	/* NMEA_GPGGA_14 */		NMEA_RESULT_NONE,
	/* NMEA_GPGGA_CKSUM */	NMEA_RESULT_NONE,
	},
#endif
#if NMEA_RMC_ENABLE
	/* NMEA_GRP_GPRMC */
	{
	/* NMEA_GPRMC_0 */		NMEA_RESULT_NONE,
	/* NMEA_GPRMC_TIME */	NMEA_RESULT_TIME,
	/* NMEA_GPRMC_STATUS */	NMEA_RESULT_STATUS,
	/* NMEA_GPRMC_LAT */	NMEA_RESULT_LAT,
	/* NMEA_GPRMC_NS */		NMEA_RESULT_LAT_NS,
	/* NMEA_GPRMC_LONG */	NMEA_RESULT_LONG,
	/* NMEA_GPRMC_EW */		NMEA_RESULT_LONG_EW,
	/* NMEA_GPRMC_SPD */	NMEA_RESULT_SPEED,
	/* NMEA_GPRMC_TRK */	NMEA_RESULT_TRACK,
	/* NMEA_GPRMC_DATE */	NMEA_RESULT_DATE,
	/* NMEA_GPRMC_MAGN */	NMEA_RESULT_NONE,
	/* NMEA_GPRMC_W */		NMEA_RESULT_NONE,
#if NMEA_GGA_ENABLE
	/* NMEA_GPRMC_QUAL */	NMEA_RESULT_NONE,
#else
	/* NMEA_GPRMC_QUAL */	NMEA_RESULT_QUAL,
#endif
	/* NMEA_GPRMC_CKSUM */	NMEA_RESULT_NONE
	},
#endif
#if NMEA_VTG_ENABLE
	/* NMEA_GRP_GPVTG	*/
	{
	},
#endif
#if NMEA_GSA_ENABLE
	/* NMEA_GRP_GPGSA	*/
	{
	},
#endif
#if NMEA_GSV_ENABLE
	/* NMEA_GRP_GPGSV	*/
	{
	},
#endif
};


// Zahlen
nmea_number_t	nmea_numbers[NMEA_RESULT_NUMBER_NO];

// Zeichenketten
char			nmea_strings[NMEA_RESULT_STRING_NO][NMEA_RESULT_STRING_LENGTH];

// Datum und Uhrzeit
nmea_date_t		nmea_date;
nmea_time_t		nmea_time;

uint8_t			nmea_param_grp = NMEA_GRP_UNKNOWN;
uint8_t			nmea_param_id = 0;

uint8_t			nmea_group_complete[NMEA_GRP_NO];

//! State of NMEA parser
static uint8_t	nmea_state = NMEA_WAIT_START;
static uint8_t	nmea_buffer_ptr = 0;

static uint8_t	nmea_buffer[16];

// Funktionen
static uint8_t nmea_findGroup (const uint8_t *buffer, uint8_t len)
{
	uint8_t	i, result;

	result = NMEA_GRP_UNKNOWN;

	// es müssen genau 3 Zeichen im Puffer stehen (Meldung ohne $GP und ,)
	if (len == 3) {
		for (i = 0; i < NMEA_GRP_NO; i++) {
			// Vergleichswert im Flash
			if (memcmp_P (buffer, &nmea_groups[i][0], 3) == 0) {
				// Treffer
				result = i;
				break;
			}
		}
	}
	return result;
}

static void nmea_parseNumber (const uint8_t *buffer, uint8_t len, uint8_t id)
{
	int16_t		integer;
	uint8_t		fraction;
	uint8_t		sign;
	uint8_t		i;
	uint8_t		dot;

	integer = 0;
	fraction = 0;

	if (len > 0) {
		dot = len;

		if (buffer[0] == '-') {
			// negativ
			sign = 1;
		} else {
			sign = 0;
		}
		// Vorkommaanteil
		for (i = sign; i < len; i++) {
			// Zeichen testen
			if (isdigit((int)buffer[i])) {
				// Ziffer
				integer *= 10;
				integer += (buffer[i] - '0');

			} else if (buffer[i] == '.') {
				// nach dem Punkt kommt der Nachkommaanteil
				dot = i;
				break;
			} else {
				// Abbruch
				break;
			}
		}
		// Vorzeichen übernehmen
		if (sign != 0)
			integer *= -1;

		// Nachkommaanteil
		if (dot < len) {
#if 0
			for (i = len - 1; i > dot; i--) {
				// Zeichen testen
				if (isdigit((int)buffer[i])) {
					// Ziffer
					fraction *= 10;
					fraction += (buffer[i] - '0');
				} else if (buffer[i] == '.') {
					// noch ein Punkt ???
					break;
				} else {
					// Abbruch
					break;
				}
			}
#else
			for (i = dot + 1; i < len; i++) {
				// Zeichen testen
				if (isdigit((int)buffer[i])) {
					// Ziffer
					fraction *= 10;
					fraction += (buffer[i] - '0');
#if 0
					// kann fraction nochmal mit 10 multipliziert
					// und 9 dazuaddiert werden ohne Überlauf?
					if (fraction > ((255 - 9) / 10)) {
						// die nächste Ziffer passt nicht mehr rein
						break;
					}
#else
					// maximal 2 Ziffern wandeln, sonst siehts komisch aus
					if (i == (dot + 1 + 1))
						break;
#endif
				} else {
					// Abbruch
					break;
				}
			}
#endif
		}
	}
	// Debug
//	nmea_numbers[NMEA_RESULT_NONE].integer = integer;
//	nmea_numbers[NMEA_RESULT_NONE].fraction = fraction;
//	nmea_numbers[NMEA_RESULT_NONE].changed = 1;

	// Ergebnis eintragen
	nmea_numbers[id].integer = integer;
	nmea_numbers[id].fraction = fraction;
	nmea_numbers[id].changed = 1;

//	return dot;
}

static void nmea_parseString (const uint8_t *buffer, uint8_t len, uint8_t id)
{
	uint8_t	i;

	// Index in den Ergebnisspeicher
	id = id - NMEA_RESULT_STRING_START;

	// begrenzen
	if (len > NMEA_RESULT_STRING_LENGTH - 1)
		len = NMEA_RESULT_STRING_LENGTH - 1;

	// Zeichen kopieren
	for (i = 0; i < len; i++)
		nmea_strings[id][i] = buffer[i];
	// abschließen
	nmea_strings[id][i] = '\0';

//	return 1;
}

static void nmea_parseDate (const uint8_t *buffer, uint8_t len, uint8_t id)
{
#if NMEA_DATE_AS_INT
	// schnell und einfach
	nmea_date.day = buffer[0] - '0';
	nmea_date.day *= 10;
	nmea_date.day += buffer[1] - '0';

	nmea_date.month = buffer[2] - '0';
	nmea_date.month *= 10;
	nmea_date.month += buffer[3] - '0';

	nmea_date.year = buffer[4] - '0';
	nmea_date.year *= 10;
	nmea_date.year += buffer[5] - '0';
#else
	// String kopieren
	uint8_t	i;

	if (len > 6)
		len = 6;
	for (i = 0; i < len; i++)
		nmea_date.str[i] = buffer[i];
	nmea_date.str[i] = '\0';
#endif
	nmea_date.changed = 1;
//	return 1;
}

static void nmea_parseTime (const uint8_t *buffer, uint8_t len, uint8_t id)
{
#if NMEA_TIME_AS_INT
	// schnell und einfach
	nmea_time.hour = buffer[0] - '0';
	nmea_time.hour *= 10;
	nmea_time.hour += buffer[1] - '0';

	nmea_time.minute = buffer[2] - '0';
	nmea_time.minute *= 10;
	nmea_time.minute += buffer[3] - '0';

	nmea_time.second = buffer[4] - '0';
	nmea_time.second *= 10;
	nmea_time.second += buffer[5] - '0';
#else
	// String kopieren
	uint8_t	i;

	if (len > 6)
		len = 6;
	for (i = 0; i < len; i++)
		nmea_time.str[i] = buffer[i];
	nmea_time.str[i] = '\0';

#endif
	nmea_time.changed = 1;
//	return 1;
}


static uint8_t nmea_parseParam (const uint8_t *buffer, uint8_t len)
{
	uint8_t	id;

	if (nmea_param_grp < NMEA_GRP_NO && nmea_param_id < 20) {
		// wenn in der Tabelle etwas eingetragen ist -> Zahl parsen und melden
		id = pgm_read_byte(&nmea_result_tab[nmea_param_grp][nmea_param_id]);

		if (id != NMEA_RESULT_NONE) {
			if (id < NMEA_RESULT_NUMBER_NO) {
				// Zahl parsen
				nmea_parseNumber (buffer, len, id);

			} else if (id < NMEA_RESULT_STRING_END) {
				// String parsen
				nmea_parseString (buffer, len, id);

			} else if (id == NMEA_RESULT_DATE) {
				// Datum
				nmea_parseDate (buffer, len, id);

			} else if (id == NMEA_RESULT_TIME) {
				// Datum
				nmea_parseTime (buffer, len, id);

			} else {
			}
			return 1;
		}
	} else {
		// was anderes
	}
	return 0;
}

int8_t nmea_parser (char data)
{
	uint8_t	old_state, new_state;
	int8_t	result;

	old_state = nmea_state;
	// wenn ein case nicht passt geht die Suche von vorn los
	new_state = NMEA_WAIT_START;
	result = 0;

	// aktueller Zustand des Parsers:
	switch (old_state)
	{
	case NMEA_WAIT_START:
		if (data == '$') {
			// Start gefunden
			new_state = NMEA_START;
			// Puffer zurücksetzen
			nmea_buffer_ptr = 0;

			// Status löschen
			nmea_param_grp = NMEA_GRP_UNKNOWN;
			nmea_param_id = 0;
		}
		break;

	case NMEA_START:
		if (data == 'G') new_state = NMEA_START_G;
		break;

	case NMEA_START_G:
		if (data == 'P') new_state = NMEA_START_GP;
		break;

	case NMEA_START_GP:
	case NMEA_WAIT_SEPARATOR:
		// warten auf Trenner oder Zeilenende
		if (data == ',' || data == '\r' || data == '\n' || data == '*') {
			// gefunden -> Puffer analysieren
			if (nmea_buffer_ptr > 0) {
				if (nmea_param_id == 0) {
					// das war der Trenner nach $GP -> was kommen hier für Daten
					nmea_param_grp = nmea_findGroup (nmea_buffer, nmea_buffer_ptr);

				} else {
					// Daten
					nmea_parseParam (nmea_buffer, nmea_buffer_ptr);
				}
				// Puffer zurücksetzen
				nmea_buffer_ptr = 0;
			}
			// nächster Parameter
			nmea_param_id++;

			if (data == '\r' || data == '\n') {
				// Zeilenende -> wieder auf Start warten
				new_state = NMEA_WAIT_START;

				// aktuelle Gruppe abgeschlossen
				if (nmea_param_grp < NMEA_GRP_NO)
					nmea_group_complete[nmea_param_grp] += 1;

			} else if (data == '*') {
				// Ende der Parameter, jetzt kommt die Prüfsumme
				// in diesem Zustand bleiben
				new_state = NMEA_WAIT_SEPARATOR;

			} else {
				// in diesem Zustand bleiben
				new_state = NMEA_WAIT_SEPARATOR;
			}
		} else if (data == '$') {
			// neuer Start ohne Zeilenende ???
			new_state = NMEA_START;
			// Puffer zurücksetzen
			nmea_buffer_ptr = 0;
			result = data;
		} else {
			// Zeichen in den Puffer schreiben
			if (nmea_buffer_ptr < (sizeof(nmea_buffer) - 1)) {
				// passt
				nmea_buffer[nmea_buffer_ptr] = data;
				nmea_buffer_ptr++;
			} else {
				// voll
			}
			// in diesem Zustand bleiben
			new_state = NMEA_WAIT_SEPARATOR;
		}
		break;

	default:
		// wenn kein case passt, ist es ein Fehler und es wird auf eine neue Zeile gewartet
		new_state = NMEA_WAIT_START;
	}

	// bei Änderung am Zustand
	if (old_state != new_state) {
		nmea_state = new_state;
	}

	return result;
}

void nmea_init (void)
{
//	memset (nmea_numbers, 0, sizeof(nmea_numbers));
//	memset (nmea_strings, ' ', sizeof(nmea_strings));
}

#endif
