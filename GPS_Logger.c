/*
 * Author: Michael Böhme
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef F_CPU
#define F_CPU		14745600UL
#endif

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

#include "config.h"
#include "i2c.h"
#include "sd_raw.h"
#include "nmea.h"
#include "hyt.h"
#include "fram.h"

#ifndef NULL
#define NULL	(void*)0
#endif

/*---------------------------Konstanten--------------------------------------*/
/*
 * Note [3]
 * TWI address for 24Cxx EEPROM:
 *
 * 1 0 1 0 E2 E1 E0 R/~W	24C01/24C02
 * 1 0 1 0 E2 E1 A8 R/~W	24C04
 * 1 0 1 0 E2 A9 A8 R/~W	24C08
 * 1 0 1 0 A10 A9 A8 R/~W	24C16
 */
#define TWI_SLA_24CXX	0xa0	/* E2 E1 E0 = 0 0 0 */

/*
 * Maximal number of iterations to wait for a device to respond for a
 * selection.  Should be large enough to allow for a pending write to
 * complete, but low enough to properly abort an infinite loop in case
 * a slave is broken or not present at all.  With 100 kHz TWI clock,
 * transfering the start condition and SLA+R/W packet takes about 10
 * µs.  The longest write period is supposed to not exceed ~ 10 ms.
 * Thus, normal operation should not require more than 100 iterations
 * to get the device to respond to a selection.
 */
#define TWI_MAX_ITER	200

/*
 * Number of bytes that can be written in a row, see comments for
 * ee24xx_write_page() below.  Some vendor's devices would accept 16,
 * but 8 seems to be the lowest common denominator.
 *
 * Note that the page size must be a power of two, this simplifies the
 * page boundary calculations below.
 */
#define PAGE_SIZE 8


// ADM1032
#define TWI_SLA_ADM1032		(0x4C << 1)

// Registeradressen für Daten Lesen
#define ADM_RD_TEMP_L		0x00
#define ADM_RD_TEMP_RH		0x01
#define ADM_RD_STATUS		0x02
#define ADM_RD_CONFIG		0x03
#define ADM_RD_CONV_RATE	0x04
#define ADM_RD_LIMIT_LH		0x05
#define ADM_RD_LIMIT_LL		0x06
#define ADM_RD_LIMIT_RHH	0x07
#define ADM_RD_LIMIT_RLH	0x08

#define ADM_RD_TEMP_RL		0x10
#define ADM_RD_OFFS_H		0x11
#define ADM_RD_OFFS_L		0x12
#define ADM_RD_LIMIT_RHL	0x13
#define ADM_RD_LIMIT_RLL	0x14

#define ADM_RD_LIMIT_RTH	0x19
#define ADM_RD_LIMIT_LTH	0x20
#define ADM_RD_HYST_TH		0x21
#define ADM_RD_CONS_ALRT	0x22

#define ADM_RD_MFG_ID		0xFE
#define ADM_RD_REV_CODE		0xFF

// Registeradressen für Daten Schreiben
#define ADM_WR_CONFIG		0x09
#define ADM_WR_CONV_RATE	0x0A
#define ADM_WR_LIMIT_LH		0x0B
#define ADM_WR_LIMIT_LL		0x0C
#define ADM_WR_LIMIT_RHH	0x0D
#define ADM_WR_LIMIT_RLH	0x0E
#define ADM_WR_ONE_SHOT		0x0F
#define ADM_WR_OFFS_H		0x11
#define ADM_WR_OFFS_L		0x12
#define ADM_WR_LIMIT_RHL	0x13
#define ADM_WR_LIMIT_RLL	0x14
#define ADM_WR_LIMIT_RTH	0x19
#define ADM_WR_LIMIT_LTH	0x20
#define ADM_WR_HYST_TH		0x21
#define ADM_WR_CONS_ALRT	0x22


// UDR Empty Interrupt
#define SET_UDRIE (UCSR0B |=  _BV(UDRIE0))
#define CLR_UDRIE (UCSR0B &= ~_BV(UDRIE0))

// UDR Recv Interrupt
#define SET_RXCIE (UCSR0B |=  _BV(RXCIE0))
#define CLR_RXCIE (UCSR0B &= ~_BV(RXCIE0))

// Baudratenregister
#define UBRR_BAUD	((F_CPU / (16 * BAUD)) - 1)


/*---------------------------Aliase für Pins und Ports-----------------------*/
#define LCD_EN_LO			PORTC &= ~_BV(PC1)
#define LCD_EN_HI			PORTC |=  _BV(PC1)
#define LCD_EN_OUT			DDRC  |=  _BV(PC1)

#define LCD_RS_LO			PORTC &= ~_BV(PC0)
#define LCD_RS_HI			PORTC |=  _BV(PC0)
#define LCD_RS_OUT			DDRC  |=  _BV(PC0)

#define LCD_D0				_BV(PB0)
#define LCD_D1				_BV(PB1)
#define LCD_D2				_BV(PD2)
#define LCD_D3				_BV(PD3)
#define LCD_D4				_BV(PD4)
#define LCD_D5				_BV(PD5)
#define LCD_D6				_BV(PD6)
#define LCD_D7				_BV(PD7)

#define LCD_MASK0			(LCD_D0 | LCD_D1)
#define LCD_MASK1			(LCD_D2 | LCD_D3 | LCD_D4 | LCD_D5 | LCD_D6 | LCD_D7)

#define LCD_PORT0			PORTB
#define LCD_PORT1			PORTD
#define LCD_DDR0			DDRB
#define LCD_DDR1			DDRD
#define LCD_DDR_OUT			{LCD_DDR0 |= LCD_MASK0; LCD_DDR1 |= LCD_MASK1;}
#define LCD_DDR_IN			{LCD_DDR0 &= ~LCD_MASK0; LCD_DDR1 &= ~LCD_MASK1;}

#define LCD_DATA0(BYTE)		{LCD_PORT0 &= ~LCD_MASK0; LCD_PORT0 |= ((BYTE) & LCD_MASK0);}
#define LCD_DATA1(BYTE)		{LCD_PORT1 &= ~LCD_MASK1; LCD_PORT1 |= ((BYTE) & LCD_MASK1);}
#define LCD_DATA(BYTE)		{LCD_DATA0(BYTE); LCD_DATA1(BYTE);}

#define LCD_LINE_LENGTH		16
#define LCD_LINES			3

#define LCD_LINE0			0
#define LCD_LINE1			LCD_LINE_LENGTH
#define LCD_LINE2			(2 * LCD_LINE_LENGTH)

#define LCD_POS_MAX			(LCD_LINES * LCD_LINE_LENGTH)

// Vdd = 3.7V (LI-ION ungeregelt)
//#define LCD_CONTRAST		8

// Vdd = 3.3V (geregelt)
#define LCD_CONTRAST		16


// SPI SD-Card definitions

#define SPI_CS_ACTIVE		PORTB &= ~_BV(PB2)
#define SPI_CS_INACTIVE		PORTB |= _BV(PB2)

#define SD_CMD0		0x40	// software reset
#define SD_CMD1		0x41	// brings card out of idle state
#define SD_CMD2		0x42	// not used in SPI mode
#define SD_CMD3		0x43	// not used in SPI mode
#define SD_CMD4		0x44	// not used in SPI mode
#define SD_CMD5		0x45	// Reserved
#define SD_CMD6		0x46	// Reserved
#define SD_CMD7		0x47	// not used in SPI mode
#define SD_CMD8		0x48	// Reserved
#define SD_CMD9		0x49	// ask card to send card speficic data (CSD)
#define SD_CMD10	0x4A	// ask card to send card identification (CID)
#define SD_CMD11	0x4B	// not used in SPI mode
#define SD_CMD12	0x4C	// stop transmission on multiple block read
#define SD_CMD13	0x4D	// ask the card to send it's status register
#define SD_CMD14	0x4E	// Reserved
#define SD_CMD15	0x4F	// not used in SPI mode
#define SD_CMD16	0x50	// sets the block length used by the memory card
#define SD_CMD17	0x51	// read single block
#define SD_CMD18	0x52	// read multiple block
#define SD_CMD19	0x53	// Reserved
#define SD_CMD20	0x54	// not used in SPI mode
#define SD_CMD21	0x55	// Reserved
#define SD_CMD22	0x56	// Reserved
#define SD_CMD23	0x57	// Reserved
#define SD_CMD24	0x58	// writes a single block
#define SD_CMD25	0x59	// writes multiple blocks
#define SD_CMD26	0x5A	// not used in SPI mode
#define SD_CMD27	0x5B	// change the bits in CSD
#define SD_CMD28	0x5C	// sets the write protection bit
#define SD_CMD29	0x5D	// clears the write protection bit
#define SD_CMD30	0x5E	// checks the write protection bit
#define SD_CMD31	0x5F	// Reserved
#define SD_CMD32	0x60	// Sets the address of the first sector of the erase group
#define SD_CMD33	0x61	// Sets the address of the last sector of the erase group
#define SD_CMD34	0x62	// removes a sector from the selected group
#define SD_CMD35	0x63	// Sets the address of the first group
#define SD_CMD36	0x64	// Sets the address of the last erase group
#define SD_CMD37	0x65	// removes a group from the selected section
#define SD_CMD38	0x66	// erase all selected groups
#define SD_CMD39	0x67	// not used in SPI mode
#define SD_CMD40	0x68	// not used in SPI mode
#define SD_CMD41	0x69	// Reserved
#define SD_CMD42	0x6A	// locks a block
// CMD43 ... CMD57 are Reserved
#define SD_CMD58	0x7A	// reads the OCR register
#define SD_CMD59	0x7B	// turns CRC off
// CMD60 ... CMD63 are not used in SPI mode

#define SD_R1_IDLE			0b00000001
#define SD_R1_ILLEGAL_CMD	0b00000100



struct sd_ident_s
{
	uint8_t		mid;		/*!< Manufacturer ID */
	char		oid[2];		/*!< OEM/Application ID */
	char		pnm[4];		/*!< Product Name */
	uint8_t		prv;		/*!< Product Revision */
	uint8_t		psn[4];		/*!< Serial Number */
	uint8_t		mdt[2];		/*!< MFG Date Code */
	uint8_t		crc;		/*!< CRC-7 */
};

struct sd_specData_s
{
	uint8_t		res_1;
	uint8_t		taac;
	uint8_t		nsac;
	uint8_t		speed;
	uint8_t		ccc_h;

	uint8_t		ccc_l:4;
	uint8_t		rd_bl_len:4;

	uint8_t		rd_bl_part:1;
	uint8_t		wr_blk_misalign:1;
	uint8_t		rd_blk_misalign:1;
	uint8_t		dsr_imp:1;
	uint8_t		res_2:2;
	uint8_t		c_size_2:2;

	uint8_t		c_size_1;

	uint8_t		c_size_0:2;
	uint8_t		vdd_rcurr_min:3;
	uint8_t		vdd_rcurr_max:3;

	uint8_t		vdd_wcurr_min:3;
	uint8_t		vdd_wcurr_max:3;
	uint8_t		c_size_mult_1:2;

	uint8_t		c_size_mult_0:1;
	uint8_t		erase_blk_en:1;
	uint8_t		sector_size_1:6;

	uint8_t		sector_size_0:1;
	uint8_t		wp_grp_size:7;

	uint8_t		wp_grp_en:1;
	uint8_t		res_3:2;
	uint8_t		r2w_factor:3;
	uint8_t		wr_bl_len_1:2;

	uint8_t		wr_bl_len_0:2;
	uint8_t		res_4:6;

	uint8_t		file_format_grp:1;
	uint8_t		copy:1;
	uint8_t		perm_wr_protect:1;
	uint8_t		tmp_wr_protect:1;
	uint8_t		file_format:2;
	uint8_t		res_5:2;

	uint8_t		crc;
};


#define GPS_STATUS_ACTIVE		_BV(0)
#define GPS_STATUS_GPS_FIX		_BV(1)
#define GPS_STATUS_DGPS_FIX		_BV(2)
#define GPS_STATUS_ESTIMATED	_BV(3)

#define GPS_STATUS_FIX			(GPS_STATUS_GPS_FIX | GPS_STATUS_DGPS_FIX | GPS_STATUS_ESTIMATED)

#define GPS_STATUS_SOUTH		_BV(4)
#define GPS_STATUS_WEST			_BV(5)


struct gps_data_log
{
	uint8_t		status;

	uint8_t		day;
	uint8_t		month;
	uint8_t		year;
	uint8_t		hour;
	uint8_t		minute;
	uint8_t		second;

	uint8_t		lon[3];
	uint8_t		lat[3];
	uint8_t		speed[2];
	uint8_t		altitude[2];

	uint8_t		temp[2];
	uint8_t		humidity[2];
};


/*---------------------------Variablen---------------------------------------*/
/*
 * Saved TWI status register, for error messages only.  We need to
 * save it in a variable, since the datasheet only guarantees the TWSR
 * register to have valid contents while the TWINT bit in TWCR is set.
 */
//volatile uint8_t twst;

char	text[LCD_LINE_LENGTH + 1];
char	vartext[LCD_LINE_LENGTH + 1];

//const char text_empty[LCD_LINE_LENGTH + 1]				PROGMEM = "                ";
//const char text_test[LCD_LINE_LENGTH + 1]				PROGMEM = " --- Test   --- ";
const char text_welcome_0[LCD_LINE_LENGTH + 1]			PROGMEM = "Environm. Sensor";
const char text_welcome_1[LCD_LINE_LENGTH + 1]			PROGMEM = "   GPS - Data   ";
const char text_welcome_2[LCD_LINE_LENGTH + 1]			PROGMEM = "   - Logger -   ";

#if NMEA_ENABLE
//const char text_nmea_position[LCD_LINE_LENGTH + 1]		PROGMEM = "   0.0     0.0  ";
const char text_nmea_lat_speed[LCD_LINE_LENGTH + 1]		PROGMEM = "   0 00\0  0.0\0 0";
const char text_nmea_long_track[LCD_LINE_LENGTH + 1]	PROGMEM = "   0 00\0  0\0   0";
#endif

#if HYT_ENABLE || ADM_ENABLE
#if HYT_ENABLE && NMEA_ENABLE
const char text_nmea_status_hyt[LCD_LINE_LENGTH + 1]	PROGMEM = "         C     %";
#else
const char text_error[LCD_LINE_LENGTH + 1]				PROGMEM = "Error:          ";
const char text_status[LCD_LINE_LENGTH + 1]				PROGMEM = "Status:         ";

const char text_localTemp[LCD_LINE_LENGTH + 1]			PROGMEM = "Loc Temp:       ";
const char text_remoteTemp[LCD_LINE_LENGTH + 1]			PROGMEM = "Rem Temp:       ";
const char text_humidity[LCD_LINE_LENGTH + 1]			PROGMEM = "Humidity:       ";
#endif
#endif

//const uint8_t	*spi_sdata;		// Pointer to certain Character in String
//uint8_t			*spi_rdata;
//uint8_t			spi_slen;
//uint8_t			spi_cts = 1;	// String send complete bit

#if SD_CARD_ENABLE
uint8_t			sd_cmd[6];
uint8_t			sd_response;
//struct sd_ident_s		sd_ident;
//struct sd_specData_s	sd_specData;

struct sd_raw_info	sd_info;
uint8_t				sd_data[128];
#endif

#if UART_ENABLE
#if UART_RECV_ENABLE
//! RX buffer for uart.
uint8_t UART_RxBuffer[UART_RX_BUFFER_SIZE];
//! RX buffer pointer.
volatile uint8_t UART_RxHead = 0;
uint8_t UART_RxTail = 0;
#endif

#if UART_SEND_ENABLE
//! TX buffer for uart.
uint8_t UART_TxBuffer[UART_TX_BUFFER_SIZE];
//! TX buffer head pointer.
uint8_t UART_TxHead = 0;
//! TX buffer tail pointer.
volatile uint8_t UART_TxTail = 0;
#endif
#endif	// UART_ENABLE


#if NMEA_ENABLE
uint8_t		nmea_toggle;
uint8_t		nmea_grpCnt;
#endif

#if HYT_ENABLE
uint8_t		hyt_toggle;
#endif

#if GPS_DATA_LOG_ENABLE
struct gps_data_log		log_data;
#endif

/*---------------------------ISR-Deklarationen------------------------------*/

/*---------------------------Unterprogramm-Deklarationen---------------------*/
static void periph_init(void); // Peripherie initialisieren

static void lcd_wbyte(uint8_t rs, uint8_t value);
static void lcd_init(void);
static void lcd_clear(void);
static void lcd_contrast (uint8_t contrast);
static void lcd_text (uint8_t pos, const char *text);
static void lcd_char (uint8_t pos, char character);

void strins(char *dst, const char *src, uint8_t end);

#if EE24_ENABLE
int ee24xx_write_bytes(uint16_t eeaddr, int len, uint8_t *buf);
int ee24xx_write_page(uint16_t eeaddr, int len, uint8_t *buf);
int ee24xx_read_bytes(uint16_t eeaddr, int len, uint8_t *buf);
#endif

#if ADM_ENABLE
int8_t adm_read (uint8_t regAddr, uint8_t *value);
#endif


//void spi_write (const uint8_t *data, uint8_t len);
//uint8_t spi_read (void);

#if SD_CARD_ENABLE
void sd_command (uint8_t cmd, uint8_t arg0, uint8_t arg1, uint8_t arg2, uint8_t arg3, uint8_t crc);
uint8_t sd_receive (uint8_t timeout);
uint8_t sd_crc7 (const uint8_t *data, uint8_t len);
#endif


#if UART_ENABLE
uint8_t uart_SendByte (uint8_t data);
uint8_t uart_RecvByte (void);

void uart_SendString(uint8_t *Str);
//void uart_SendInt(int Tall);
void uart_FlushRxBuffer(void);

#endif

//uint8_t		valu8;
//uint16_t	valu16;
//uint32_t	valu32;
int32_t		vali32;

/*---------------------------Hauptprogramm-----------------------------------*/
int main(void)
{
	uint8_t		m, n;
	int8_t		err;

#if NMEA_ENABLE
//	uint8_t		nmea_toggle;
//	uint8_t		nmea_grpCnt;
#endif
#if HYT_ENABLE
//	uint8_t		hyt_toggle;
#endif
	cli ();

	// alle Peripherie initialisieren
	periph_init ();

	// 1s Pause
	_delay_ms (100);

	// LCD initialisieren
	lcd_init ();

	// Kontrast abhängig von der Betriebsspannung
	lcd_contrast (LCD_CONTRAST);

	// Begrüßung
	memcpy_P (text, text_welcome_0, LCD_LINE_LENGTH);
	lcd_text (LCD_LINE0, text);
	memcpy_P (text, text_welcome_1, LCD_LINE_LENGTH);
	lcd_text (LCD_LINE1, text);
	memcpy_P (text, text_welcome_2, LCD_LINE_LENGTH);
	lcd_text (LCD_LINE2, text);

	// 3s Pause
	for (m = 0; m < 200; m++)
		_delay_ms (15);

#if LCD_TEST_CONTRAST
	// testet alle möglichen Werte für den LCD-Kontrast
	while (1)
	{
		for (uint8_t i = 0; i <= 0b00111111; i++) {
			// Kontrast einstellen
			lcd_contrast (i);
			// Wert anzeigen
			utoa (i, vartext, 10);
			text[LCD_LINE_LENGTH - 2] = ' ';
			text[LCD_LINE_LENGTH - 1] = ' ';
			strins (text, vartext, LCD_LINE_LENGTH - 1);
			lcd_text (LCD_LINE2, text);
			// 2s Pause
			for (m = 0; m < 200; m++)
				_delay_ms (10);
		}
	}
#endif	// LCD_TEST_CONTRAST

	// Interrupte erlauben
#if UART_ENABLE
#if UART_RECV_ENABLE
	// UART Empfangsinterrupt ein
	SET_RXCIE;
#endif
#endif

	sei ();

#if NMEA_ENABLE
	nmea_init ();
	// lokaler Aktualisierungsanzeiger
	nmea_toggle = 0;
	nmea_grpCnt = 0;
#endif

#if ADM_ENABLE
	//i2c_write (TWI_SLA_ADM1032, ADM_WR_CONFIG, _BV(7));
	// Manufacturer ID
	err = adm_read (ADM_RD_MFG_ID, &valu8);
	if (err != 0)
		utoa (err, vartext, 10);
	else
		utoa (valu8, vartext, 2);

	memcpy_P (text, text_empty, LCD_LINE_LENGTH);
	strins (text, vartext, LCD_LINE_LENGTH - 1);
	lcd_text (LCD_LINE0, text);

//	valu8 = 0;
//	i2c_write (TWI_SLA_ADM1032, ADM_WR_OFFS_L, valu8);
//	valu8 = 0;
//	i2c_write (TWI_SLA_ADM1032, ADM_WR_OFFS_H, valu8);

#elif HYT_ENABLE
#if HYT_TIMER_ENABLE
	// Timer initialisieren und starten
	hyt_init ();
	// lokaler Aktualisierungsanzeiger
	hyt_toggle = 0;
#endif
#endif

#if SD_CARD_ENABLE
	sd_response = sd_raw_init();

	utoa (sd_response, vartext, 10);
	memcpy (text, "Init:           ", LCD_LINE_LENGTH);
	strins (text, vartext, LCD_LINE_LENGTH - 1);
	lcd_text (LCD_LINE2, text);

	if (sd_response == 0) {
		sd_response = sd_raw_get_info (&sd_info);
		if (sd_response != 0) {

			memcpy (text, "               ", LCD_LINE_LENGTH - 1);
			//ultoa (sd_info.serial, vartext, 16);
			ultoa (sd_info.capacity, vartext, 16);
			strins (text, vartext, LCD_LINE_LENGTH - 3);

			if (sd_raw_read (0, sd_data, sizeof(sd_data))) {
				utoa (sizeof(sd_data), vartext, 16);
				memcpy (&text[0], vartext, 2);

				utoa (sd_data[0], vartext, 16);
				memcpy (&text[2], vartext, 2);

				utoa (sd_data[2], vartext, 16);
				memcpy (&text[4], vartext, 2);
			}
			lcd_text (LCD_LINE2, text);
		}
	}
#endif	// SD_CARD_ENABLE

	// Texte löschen
	memset (text, ' ', LCD_LINE_LENGTH);
	lcd_text (LCD_LINE0, text);
	lcd_text (LCD_LINE1, text);
	lcd_text (LCD_LINE2, text);

	n = 0;
	m = 0;
	while (1)
	{
#if ADM_ENABLE
		// 1s Pause
		for (m = 0; m < 100; m++)
			_delay_ms (10);

		// Local Temp
		err = adm_read (ADM_RD_TEMP_L, &valu8);
		if (err != 0) {
			memcpy_P (text, text_error, LCD_LINE_LENGTH);
			itoa (err, vartext, 10);
		} else {
			memcpy_P (text, text_localTemp, LCD_LINE_LENGTH);
			utoa (valu8, vartext, 10);
		}
		strins (text, vartext, LCD_LINE_LENGTH - 1);
		lcd_text (LCD_LINE0, text);

		// Remote Temp High
		err = adm_read (ADM_RD_TEMP_RH, &valu8);
		if (err != 0) {
			memcpy_P (text, text_error, LCD_LINE_LENGTH);
			itoa (err, vartext, 10);
		} else {
			memcpy_P (text, text_remoteTemp, LCD_LINE_LENGTH);
			utoa (valu8, vartext, 10);
		}
		strins (text, vartext, LCD_LINE_LENGTH - 1);
		lcd_text (LCD_LINE1, text);

#elif HYT_ENABLE
#if HYT_TIMER_ENABLE
		// Automat
		if (hyt_process() == HYT_DATA_READY) {
			// Messung liegt vor
#else
		n++;

		// 1s Pause
		for (m = 0; m < 100; m++)
			_delay_ms (10);

		// Read Measurement
		err = hyt_read ((uint8_t *)&hyt_result, sizeof(hyt_result));
		if (err != 0) {
			// Fehler
			memcpy_P (text, text_error, LCD_LINE_LENGTH);
			// Zähler
			utoa (n, vartext, 10);
			strins (text, vartext, LCD_LINE_LENGTH - 1);
			// Fehler
			itoa (err, vartext, 10);
			strins (text, vartext, LCD_LINE_LENGTH - (1 + 4));
			lcd_text (LCD_LINE0, text);
		} else {
#endif	// HYT_TIMER_ENABLE

#if NMEA_ENABLE
			// alles kompakt in der ersten Zeile darstellen
			memcpy_P (text, text_nmea_status_hyt, LCD_LINE_LENGTH + 1);

#if 0
			// Zähler
			utoa (n, vartext, 10);
			strins (text, vartext, 3);
			n++;
#else
			// Stern toggeln
			if ((hyt_toggle & 1) != 0) {
				text[3] = '*';
			}
			hyt_toggle++;
#endif
			// Temperatur
#if HYT_AVERAGE_NO > 0
			itoa (hyt_avg.temp, vartext, 10);
#else
			itoa (hyt_result.temp, vartext, 10);
#endif
			// am Ende ein Zeichen frei lassen
			strins (text, vartext, 7);
			// die letzte Ziffer ans Ende kopieren
			text[8] = text[7];
			// und einen Dezimalpunkt setzen
			text[7] = '.';

			if ((hyt_toggle & 4) != 0) {
				// Feuchte
#if HYT_AVERAGE_NO > 0
				itoa (hyt_avg.humid, vartext, 10);
#else
				itoa (hyt_result.humid, vartext, 10);
#endif
			} else {
				// Taupunkt
				text[15] = 'C';
#if HYT_AVERAGE_NO > 0
				itoa (hyt_calcDewPoint (hyt_avg.humid, hyt_avg.temp), vartext, 10);
#else
				itoa (hyt_calcDewPoint (hyt_result.humid, hyt_result.temp), vartext, 10);
#endif
			}
			// am Ende zwei Zeichen frei lassen
			strins (text, vartext, 13);
			// die letzte Ziffer ans Ende kopieren
			text[14] = text[13];
			// und einen Dezimalpunkt setzen
			text[13] = '.';

			// Text anzeigen (ab 3tem Zeichen)
			lcd_text (LCD_LINE0 + 3, &text[3]);

#if GPS_DATA_LOG_ENABLE
			// Werte speichern
			log_data.temp[0] = (hyt_result.temp >> 8) & 0xFF;
			log_data.temp[1] = (hyt_result.temp >> 0) & 0xFF;
			log_data.humidity[0] = (hyt_result.humid >> 8) & 0xFF;
			log_data.humidity[1] = (hyt_result.humid >> 0) & 0xFF;
#endif

#else	// NMEA_ENABLE

			// Status
			memcpy_P (text, text_status, LCD_LINE_LENGTH);

			// Zähler
			utoa (n, vartext, 10);
			strins (text, vartext, LCD_LINE_LENGTH - 1);
			n++;

			// Status
			valu8 = HYT_STATUS_VALUE(hyt_result.capHigh);
			itoa (valu8, vartext, 10);
			strins (text, vartext, LCD_LINE_LENGTH - (1 + 4));
			lcd_text (LCD_LINE0, text);

			// Temperatur
			memcpy_P (text, text_localTemp, LCD_LINE_LENGTH);
			itoa (hyt_result.temp, vartext, 10);
			// am Ende ein Zeichen frei lassen
			strins (text, vartext, LCD_LINE_LENGTH - 2);
			// die letzte Ziffer ans Ende kopieren
			text[LCD_LINE_LENGTH - 1] = text[LCD_LINE_LENGTH - 2];
			// und einen Dezimalpunkt setzen
			text[LCD_LINE_LENGTH - 2] = '.';
			lcd_text (LCD_LINE1, text);

			// Feuchte
			memcpy_P (text, text_humidity, LCD_LINE_LENGTH);
			itoa (hyt_result.humid, vartext, 10);
			// am Ende ein Zeichen frei lassen
			strins (text, vartext, LCD_LINE_LENGTH - 2);
			// die letzte Ziffer ans Ende kopieren
			text[LCD_LINE_LENGTH - 1] = text[LCD_LINE_LENGTH - 2];
			// und einen Dezimalpunkt setzen
			text[LCD_LINE_LENGTH - 2] = '.';
			lcd_text (LCD_LINE2, text);

#endif	// NMEA_ENABLE

#if HYT_TIMER_ENABLE
			// Ergebnis abgeholt -> idle
			HYT_SET_STATE_IDLE();
#endif
		}
#endif	// HYT_ENABLE

#if UART_ENABLE
#if UART_RECV_ENABLE
		// Zeichen empfangen
		uint8_t val = uart_RecvByte ();
		if (val > 0) {
#if NMEA_ENABLE
			// dem Parser zuführen
			err = nmea_parser (val);
#if 0
			if (valu8 > ' ') {
				lcd_char (LCD_LINE0 + 0, valu8);

				if (nmea_param_id == 0)
					lcd_char (LCD_LINE0 + 1, nmea_param_grp + '0');
				if (nmea_param_id > 0)
					lcd_char (LCD_LINE0 + 2, nmea_param_id + 'A');
			}
#endif
#else
			if (valu8 >= ' ') {
				if (valu8 == '$') {
					//lcd_clear ();
					n = 0;
				}
				// Zeichen ausgeben
				lcd_char (n, valu8);
				// nächstes Zeichen, bei Überlauf wieder von vorn anfangen
				if (++n >= LCD_POS_MAX)
					n = 0;
			}
#endif
		} else {
#if NMEA_ENABLE
			// Anzeige muss nur aktualisiert werden, wenn eine Parametergruppe abgeschlossen wurde
#if NMEA_GGA_ENABLE
			if (nmea_group_complete[NMEA_GRP_GPGGA] > 0) {
				nmea_group_complete[NMEA_GRP_GPGGA] = 0;
				nmea_grpCnt += 1;
			}
#endif
#if NMEA_RMC_ENABLE
			if (nmea_group_complete[NMEA_GRP_GPRMC] > 0) {
				nmea_group_complete[NMEA_GRP_GPRMC] = 0;
				nmea_grpCnt += 1;
			}
#endif
			// Änderung in einer der Gruppen gefunden
			if (nmea_grpCnt > 0) {

				if (/*nmea_date.changed ||*/ nmea_time.changed) {
					//nmea_date.changed = 0;
					nmea_time.changed = 0;

#if GPS_DATA_LOG_ENABLE
					// Werte speichern
					log_data.day	= nmea_date.day;
					log_data.month	= nmea_date.month;
					log_data.year	= nmea_date.year;
					log_data.hour	= nmea_time.hour;
					log_data.minute	= nmea_time.minute;
					log_data.second	= nmea_time.second;
#endif

#if 0
					memset (text, ' ', LCD_LINE_LENGTH);

					// Datum
#if NMEA_DATE_AS_INT
					utoa (nmea_date.day, vartext, 10);
					strins (text, vartext, 1);
					utoa (nmea_date.month, vartext, 10);
					strins (text, vartext, 1 + 2);
					utoa (nmea_date.year, vartext, 10);
					strins (text, vartext, 1 + 2 + 4);
#else
					// DDMMYY
					strins (text, nmea_date.str, 6 - 1);
#endif

					// Uhrzeit
#if NMEA_TIME_AS_INT
					utoa (nmea_time.hour, vartext, 10);
					strins (text, vartext, 1 + 2 + 4 + 3);
					utoa (nmea_time.minute, vartext, 10);
					strins (text, vartext, 1 + 2 + 4 + 3 + 2);
					utoa (nmea_time.second, vartext, 10);
					strins (text, vartext, 1 + 2 + 4 + 3 + 2 + 2);
#else
					// HHMMSS
					strins (text, nmea_time.str, 6 + 1 + 6 - 1);
#endif
#if 0
					// Status (A=Active, V=Void)
					text[LCD_LINE_LENGTH - 2] = nmea_strings[NMEA_RESULT_STATUS - NMEA_RESULT_STRING_START][0];
					// Qualität (A=autonomous, D=differential, E=estimated, N=not valid)
					text[LCD_LINE_LENGTH - 1] = nmea_strings[NMEA_RESULT_QUAL - NMEA_RESULT_STRING_START][0];
#else
#if NMEA_GGA_ENABLE
					utoa (nmea_numbers[NMEA_RESULT_SAT_NO].integer, vartext, 10);
					strins (text, vartext, LCD_LINE_LENGTH - 1);
#endif
#endif
					// Text ausgeben
					lcd_text (LCD_LINE1, text);
#else
					// Stern toggeln
					if ((nmea_toggle & 1) != 0) {
						text[0] = '*';
					} else {
						text[0] = ' ';
					}
					nmea_toggle++;

					// Status (A=Active, V=Void)
					text[1] = nmea_strings[NMEA_RESULT_STATUS - NMEA_RESULT_STRING_START][0];
#if GPS_DATA_LOG_ENABLE
					if (nmea_strings[NMEA_RESULT_STATUS - NMEA_RESULT_STRING_START][0] == 'A')
						log_data.status |= GPS_STATUS_ACTIVE;
					else
						log_data.status &= ~GPS_STATUS_ACTIVE;
#endif

					// Qualität
					text[2] = nmea_strings[NMEA_RESULT_QUAL - NMEA_RESULT_STRING_START][0];

					// Text abschließen und anzeigen
					text[3] = '\0';
					lcd_text (LCD_LINE0 + 0, text);

#if GPS_DATA_LOG_ENABLE
					log_data.status &= ~GPS_STATUS_FIX;
#if NMEA_GGA_ENABLE
					// 0 invalid, 1 gps fix, 2 dgps fix, 6 estimated
					if (nmea_strings[NMEA_RESULT_QUAL - NMEA_RESULT_STRING_START][0] == '1')
						log_data.status |= GPS_STATUS_GPS_FIX;
					else if (nmea_strings[NMEA_RESULT_QUAL - NMEA_RESULT_STRING_START][0] == '2')
						log_data.status |= GPS_STATUS_DGPS_FIX;
					else if (nmea_strings[NMEA_RESULT_QUAL - NMEA_RESULT_STRING_START][0] == '6')
						log_data.status |= GPS_STATUS_ESTIMATED;
#else
					// A=autonomous, D=differential, E=estimated, N=not valid
					if (nmea_strings[NMEA_RESULT_QUAL - NMEA_RESULT_STRING_START][0] == 'A')
						log_data.status |= GPS_STATUS_GPS_FIX;
					else if (nmea_strings[NMEA_RESULT_QUAL - NMEA_RESULT_STRING_START][0] == 'D')
						log_data.status |= GPS_STATUS_DGPS_FIX;
					else if (nmea_strings[NMEA_RESULT_QUAL - NMEA_RESULT_STRING_START][0] == 'E')
						log_data.status |= GPS_STATUS_ESTIMATED;
#endif
#endif
#endif
				}
#if 0
				if (nmea_numbers[NMEA_RESULT_LAT].changed || nmea_numbers[NMEA_RESULT_LONG].changed) {
					nmea_numbers[NMEA_RESULT_LAT].changed = 0;
					nmea_numbers[NMEA_RESULT_LONG].changed = 0;

					memcpy_P (text, text_nmea_position, LCD_LINE_LENGTH);

					utoa (nmea_numbers[NMEA_RESULT_LAT].integer, vartext, 10);
					strins (text, vartext, 3);
					utoa (nmea_numbers[NMEA_RESULT_LAT].fraction, vartext, 10);
					strins (text, vartext, 3 + 3);
					text[3 + 3 + 1]  = nmea_strings[NMEA_RESULT_LAT_NS - NMEA_RESULT_STRING_START][0];

					utoa (nmea_numbers[NMEA_RESULT_LONG].integer, vartext, 10);
					strins (text, vartext, 7 + 4);
					utoa (nmea_numbers[NMEA_RESULT_LONG].fraction, vartext, 10);
					strins (text, vartext, 7 + 4 + 3);
					text[7 + 4 + 3 + 1]  = nmea_strings[NMEA_RESULT_LONG_EW - NMEA_RESULT_STRING_START][0];

					lcd_text (LCD_LINE2, text);
				}
#if 0
				if (nmea_numbers[NMEA_RESULT_SPEED].changed || nmea_numbers[NMEA_RESULT_TRACK].changed) {
					nmea_numbers[NMEA_RESULT_SPEED].changed = 0;
					nmea_numbers[NMEA_RESULT_TRACK].changed = 0;

					//memcpy_P (text, text_empty, LCD_LINE_LENGTH);
					memset (text, ' ', LCD_LINE_LENGTH);

					utoa (nmea_numbers[NMEA_RESULT_SPEED].integer, vartext, 10);
					strins (text, vartext, 3);
					utoa (nmea_numbers[NMEA_RESULT_SPEED].fraction, vartext, 10);
					strins (text, vartext, 3 + 3);
					//text[3 + 3 + 1]  = nmea_strings[NMEA_RESULT_LAT_NS - NMEA_RESULT_STRING_START][0];

					utoa (nmea_numbers[NMEA_RESULT_TRACK].integer, vartext, 10);
					strins (text, vartext, 7 + 4);
					utoa (nmea_numbers[NMEA_RESULT_TRACK].fraction, vartext, 10);
					strins (text, vartext, 7 + 4 + 3);
					//text[7 + 4 + 3 + 1]  = nmea_strings[NMEA_RESULT_LONG_EW - NMEA_RESULT_STRING_START][0];

					lcd_text (LCD_LINE2, text);
				}
#endif
#else
				// Text vorinitialisieren
				memcpy_P (text, text_nmea_lat_speed, LCD_LINE_LENGTH + 1);

				if (nmea_numbers[NMEA_RESULT_LAT].changed) {
					nmea_numbers[NMEA_RESULT_LAT].changed = 0;

					// Latitude Vorkomma 4 Ziffern
					itoa (nmea_numbers[NMEA_RESULT_LAT].integer, vartext, 10);
					strins (text, vartext, 3);
					// N / S
					text[3 + 1] = nmea_strings[NMEA_RESULT_LAT_NS - NMEA_RESULT_STRING_START][0];
					// Latitude Nachkomma 2 Ziffern
					itoa (nmea_numbers[NMEA_RESULT_LAT].fraction, vartext, 10);
					strins (text, vartext, 3 + 1 + 2);

					// Text abschließen und anzeigen
				//	text[7] = '\0';
					lcd_text (LCD_LINE1 + 0, text);

#if GPS_DATA_LOG_ENABLE
					log_data.lat[0] = (nmea_numbers[NMEA_RESULT_LAT].integer >> 8) & 0xFF;
					log_data.lat[1] = (nmea_numbers[NMEA_RESULT_LAT].integer >> 0) & 0xFF;
					log_data.lat[2] = (nmea_numbers[NMEA_RESULT_LAT].fraction);

					if (nmea_strings[NMEA_RESULT_LAT_NS - NMEA_RESULT_STRING_START][0] == 'S')
						log_data.status |= GPS_STATUS_SOUTH;
					else
						log_data.status &= ~GPS_STATUS_SOUTH;
#endif
				}

				if (nmea_numbers[NMEA_RESULT_SPEED].changed) {
					nmea_numbers[NMEA_RESULT_SPEED].changed = 0;
#if 0
					// anzeigen in Knoten
					uint8_t	tmp;

					// Speed 3 + 1 Ziffern
					itoa (nmea_numbers[NMEA_RESULT_SPEED].integer, vartext, 10);
					strins (text, vartext, 10);
					// aus den maximal 2 Ziffern eine machen
					tmp = nmea_numbers[NMEA_RESULT_SPEED].fraction;
					if (tmp > 9) {
						// aufrunden
						tmp += 9;
						tmp /= 10;
					}
					itoa (tmp, vartext, 10);
					strins (text, vartext, 10 + 2);
#else
					// umrechnen in km/h: Knoten * 1.852
					int16_t	input;
					int16_t	output;
					int16_t	tmp;

					// Vorkommaanteil und Nachkommaanteil vereinen
					input = nmea_numbers[NMEA_RESULT_SPEED].integer;
					// Nachkomma ist maximal 99
					input *= 100;
					input += nmea_numbers[NMEA_RESULT_SPEED].fraction;
					// max. 327.67 Knoten
					// (in * 8) / 10 vs (in / 10) * 8 :
					// 32767 / 8 = 4095 -> 40 Knoten wäre Max, daher erst teilen, dann multiplizieren
					// * 0.8
					tmp = (input + 9) / 10;
					tmp *= 8;
					output = input + tmp;
					// * 0.05
					tmp = (input + 99) / 100;
					tmp *= 5;
					output += tmp;
					// * 0.002
					tmp = (input + 999) / 1000;
					tmp *= 2;
					output += tmp;
					// nur eine Nachkommastelle anzeigen
					//output = (output + 9) / 10;		nur einmal aufrunden, bei *0.852 war der Fehler kleiner
					output = output / 10;
					// Speed 3 + 1 Ziffern
					itoa (output, vartext, 10);
					strins (text, vartext, 11);
					// die letzte Ziffer ans Ende kopieren
					text[12] = text[11];
					// und einen Dezimalpunkt setzen
					text[11] = '.';

#if GPS_DATA_LOG_ENABLE
					// nur Vorkomma speichern
					log_data.speed[0] = ((output / 10) >> 8) & 0xFF;
					log_data.speed[1] = ((output / 10) >> 0) & 0xFF;
#endif
#endif
					// Text abschließen und anzeigen
				//	text[8 + 5] = '\0';
					lcd_text (LCD_LINE1 + 8, &text[8]);
				}

#if NMEA_GGA_ENABLE
				if (nmea_numbers[NMEA_RESULT_SAT_NO].changed) {
					nmea_numbers[NMEA_RESULT_SAT_NO].changed = 0;

					// Satellite no
					itoa (nmea_numbers[NMEA_RESULT_SAT_NO].integer, vartext, 10);
					strins (text, vartext, 15);

					// Text abschließen und anzeigen
				//	text[14 + 2] = '\0';
					lcd_text (LCD_LINE1 + 14, &text[14]);
				}
#endif

				// Text vorinitialisieren
				memcpy_P (text, text_nmea_long_track, LCD_LINE_LENGTH + 1);

				if (nmea_numbers[NMEA_RESULT_LONG].changed) {
					nmea_numbers[NMEA_RESULT_LONG].changed = 0;

					// Longitude Vorkomma 4 Ziffern
					itoa (nmea_numbers[NMEA_RESULT_LONG].integer, vartext, 10);
					strins (text, vartext, 3);
					// E / W
					text[3 + 1] = nmea_strings[NMEA_RESULT_LONG_EW - NMEA_RESULT_STRING_START][0];
					// Longitude Nachkomme 2 Ziffern
					itoa (nmea_numbers[NMEA_RESULT_LONG].fraction, vartext, 10);
					strins (text, vartext, 3 + 1 + 2);

					// Text abschließen und anzeigen
				//	text[7] = '\0';
					lcd_text (LCD_LINE2 + 0, &text[0]);

#if GPS_DATA_LOG_ENABLE
					log_data.lon[0] = (nmea_numbers[NMEA_RESULT_LONG].integer >> 8) & 0xFF;
					log_data.lon[1] = (nmea_numbers[NMEA_RESULT_LONG].integer >> 0) & 0xFF;
					log_data.lon[2] = (nmea_numbers[NMEA_RESULT_LONG].fraction);

					if (nmea_strings[NMEA_RESULT_LONG_EW - NMEA_RESULT_STRING_START][0] == 'W')
						log_data.status |= GPS_STATUS_WEST;
					else
						log_data.status &= ~GPS_STATUS_WEST;
#endif
				}

				if (nmea_numbers[NMEA_RESULT_TRACK].changed) {
					nmea_numbers[NMEA_RESULT_TRACK].changed = 0;

					// Track 3 Ziffern
					itoa (nmea_numbers[NMEA_RESULT_TRACK].integer, vartext, 10);
					strins (text, vartext, 10);
					//utoa (nmea_numbers[NMEA_RESULT_TRACK].fraction, vartext, 10);
					//strins (text, vartext, 10 + 2);

					// Text abschließen und anzeigen
				//	text[8 + 3] = '\0';
					lcd_text (LCD_LINE2 + 8, &text[8]);
				}

#if NMEA_GGA_ENABLE
				if (nmea_numbers[NMEA_RESULT_ALT].changed) {
					nmea_numbers[NMEA_RESULT_ALT].changed = 0;
					// Altitude 4 Ziffern
					itoa (nmea_numbers[NMEA_RESULT_ALT].integer, vartext, 10);
					strins (text, vartext, 15);

					// Text abschließen und anzeigen
				//	text[12 + 4] = '\0';
					lcd_text (LCD_LINE2 + 12, &text[12]);

#if GPS_DATA_LOG_ENABLE
					log_data.altitude[0] = (nmea_numbers[NMEA_RESULT_ALT].integer >> 8) & 0xFF;
					log_data.altitude[1] = (nmea_numbers[NMEA_RESULT_ALT].integer >> 0) & 0xFF;
#endif
				}
#endif
#endif
				if (nmea_grpCnt >= NMEA_GRP_NO) {
					// alle Gruppen abgeschlossen
					nmea_grpCnt = 0;
				}
			}	// nmea_grpCnt > 0
#endif	// NMEA_ENABLE
		}	// val > 0
#endif	// UART_RECV_ENABLE
#endif	// UART_ENABLE
	}
	return 0;
}


/*---------------------------Unterprogramme----------------------------------*/
void periph_init(void) //Timer0 initialisieren
{
//	volatile char IOReg;

	/*	Timer läuft im CTC-Mode -> WGM12:0 = 010, die Output Compare Ausgänge
		werden nicht benutzt -> COM0A1:0 = 00, COM0B1:0 = 00, als Clock
		erstmal nix, der Timer soll noch nicht loslaufen -> CS02:0 = 000*/
//	TCCR0A = 0;
//	TCCR0B = 0;
//	TCNT0 = 0;
//	OCR0A = 0;
//	TIMSK0 = 0;

	/*	Timer läuft im CTC-Mode -> WGM13:0 = 0100, die Output Compare Ausgänge
		werden nicht benutzt -> COM1A1:0 = 00, COM1B1:0 = 00, als Clock
		erstmal nix, der Timer soll noch nicht loslaufen -> CS12:0 = 000*/
//	TCCR1A = 0;
//	TCCR1B = _BV(WGM12);
//	TCCR1C = 0;
//	TCNT1H = 0;
//	TCNT1L = 0;
//	OCR1AH = 0x7A;
//	OCR1AL = 0x12;
//	TIMSK1 |= _BV(OCIE1A);

#if UART_ENABLE
	/* Baudrate einstellen ( Normaler Modus ) */
	UBRR0H = (uint8_t)(UBRR_BAUD >> 8);
	UBRR0L = (uint8_t)(UBRR_BAUD);

	UCSR0A = 0;

	/* Einstellen des Datenformats: 8 Datenbits, 1 Stoppbit, no parity */
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);

	/* Sender und Empfänger ein */
	UCSR0B = 0
#if UART_SEND_ENABLE
		| _BV(TXEN0)
#endif
#if UART_RECV_ENABLE
		| _BV(RXEN0)
#endif
		;
#endif	// UART_ENABLE

	/* ADC */
	ADMUX = 0;
	ADCSRA = 0;	//_BV(ADEN) | _BV(ADIE) | _BV(ADPS2); //ADC ein und Prescaler auf 16

	/* Power Reduction Register */
	PRR = _BV(PRTIM2); // Timer2 aus

	/* Digital I/O Disable Register */
	DIDR0 = 0;	//_BV(ADC3D) | _BV(ADC2D);

#if I2C_ENABLE
	/* TWI */
	/* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
	#if defined TWPS0
		/* has prescaler (mega128 & newer) */
		TWSR = 0;
	#endif

	#if F_CPU < 3600000UL
		TWBR = 10;			/* smallest TWBR value, see note [5] */
	#else
		TWBR = (F_CPU / 100000UL - 16) / 2;
	#endif

	// TWI ein
	TWCR = _BV(TWEN);
#endif	// I2C_ENABLE

#if 0
	/* SPI */
	// enable SPI Interrupt and SPI in Master Mode with SCK = CK/128
#ifdef SPI_INTERRUPT_DRIVEN
	SPCR = _BV(SPIE) | _BV(SPE) | _BV(MSTR) | _BV(SPR1) | _BV(SPR0);
#else
	SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR1) | _BV(SPR0);
#endif
	// clear SPIF bit in SPSR
	IOReg = SPSR;
	IOReg = SPDR;
#endif

	/* Pin Change Mask Registers */
	PCMSK0 = 0;		//_BV(PCINT0);
	PCMSK1 = 0;
	PCMSK2 = 0;		//_BV(PCINT23) | _BV(PCINT22) | _BV(PCINT21) | _BV(PCINT18);

	/* I/O-Ports */
	// Out: PB2(/SS), PB3(MOSI), PB5(SCK)
	PORTB = _BV(PB2);
	DDRB  = _BV(PB2) | _BV(PB3) | _BV(PB5);

	//		PC0        PC1
	PORTC = 0;
	DDRC = _BV(PC2) | _BV(PC3);

	//		TXD
	PORTD = 0;
	DDRD = _BV(PD1);

	/* Schlafmodus einstellen: Powerdown */
	SMCR = _BV(SM1);
}

// ein Byte ans Display senden
void lcd_wbyte (uint8_t rs, uint8_t value)
{
	// Enable auf 0
//	LCD_EN_LO;

	// Register Select
	if (rs)
		LCD_RS_HI;
	else
		LCD_RS_LO;

	// Daten ausgeben
	LCD_DATA (value);

	// Enable Pulse High
	_delay_us (10);
	LCD_EN_HI;
	_delay_us (10);
	LCD_EN_LO;
	_delay_us (10);
}

// Display initialisieren
void lcd_init(void)
{
	// Pins zurücksetzen
	LCD_EN_LO;
	LCD_RS_LO;
	LCD_DATA(0);

	// Datenpins auf Ausgang
	LCD_DDR_OUT;
	// Steuerpins auf Ausgang
	LCD_EN_OUT;
	LCD_RS_OUT;

	// kurz warten
	_delay_ms(100);

	// Initialisierung DOGM163 für 3.3V
	lcd_wbyte(0, 0b00111001);	// 8Bit Daten, 2 Zeilen, Instruction Table 1
	lcd_wbyte(0, 0b00010101);	// BS 1/5, 3 Zeilen
	lcd_wbyte(0, 0b01010100);	// Booster ein, Kontrast C5, C4 setzen
	lcd_wbyte(0, 0b01101110);	// Spannungsfolger und Verstärkung setzen
	lcd_wbyte(0, 0b01111001);	// Kontrast C3 C2 C1 C0 setzen
	//lcd_wbyte(0, 0b00111000);	// Zurück zur Instruction Table 0
	_delay_ms(10);
	lcd_wbyte(0, 0b00001100);	// Display ein, Cursor aus, Cursor blinken aus
	lcd_wbyte(0, 0b00000001);	// Display löschen, Cursor Home
	lcd_wbyte(0, 0b00000110);	// Cursor Auto-Increment

	// Pause
	_delay_ms(100);
}

// Display löschen
void lcd_clear (void)
{
	lcd_wbyte(0, 0b00000001);	// Display löschen, Cursor Home
	// Pause
	_delay_ms(1);
}

// Display Kontrast
void lcd_contrast (uint8_t contrast)
{
	uint8_t	temp;

	temp = (contrast & 0b00001111);
	temp |= 0b01110000;
	lcd_wbyte (0, temp);	// Kontrast C3 C2 C1 setzen

	temp = (contrast & 0b00110000) >> 4;
	temp |= 0b01010100;
	lcd_wbyte (0, temp);	// Booster ein, Kontrast C5, C4 setzen
}

void lcd_text (uint8_t pos, const char *text)
{
	if (pos < LCD_POS_MAX) {
		// Adresse setzen
		lcd_wbyte (0, 0b10000000 | pos);

		while (text[0] != '\0') {
			// Zeichen schreiben
			lcd_wbyte (1, text[0]);
			text++;
		}
	}
}

void lcd_char (uint8_t pos, char character)
{
	if (pos < LCD_POS_MAX) {
		// Adresse setzen
		lcd_wbyte (0, 0b10000000 | pos);
		// Datum schreiben
		lcd_wbyte (1, character);
	}
}

void strins (char *dst, const char *src, uint8_t end)
{
	const char	*s;
	char		*d;
	// lokale Zeiger
	s = src;
	d = dst + end;
	// zum Ende der Quelle
	while (*s != '\0')
		s++;
	// aufs letzte Zeichen zurück
	s--;
	// Zeichen kopieren
	while (d >= dst && s >= src) {
		*d = *s;
		d--;
		s--;
	}
}

#if 0

#ifdef SPI_INTERRUPT_DRIVEN
// Interrupt Routine Master Mode (interrupt controlled)
SIGNAL(SIG_SPI)
{
	if (spi_slen > 0) {			// if end not reached
		if (spi_rdata != NULL) {
			*spi_rdata = SPDR;		// read Character
			spi_rdata++;
		}

		SPDR  = *spi_sdata;		// send next Character
		spi_sdata++;					// Point to next Char in String
		spi_slen--;
	} else {
		spi_cts = 1;			// if end reached enable transmission of next String
	}
}
#endif

// Sending Routine Master Mode (interrupt controlled)
void spi_write (const uint8_t *data, uint8_t len)
{
	uint8_t	reg, dummy;

	// keine Übertragung aktiv ?
	if (spi_cts == 1 && len > 0) {
		// neue Übertragungen blockieren
		spi_cts = 0;
		// Zeiger merken
//		spi_sdata  = data;
		// Länge merken
//		spi_slen = len;

		// Chipselect aktivieren
//		SPI_CS_ACTIVE;

		do {
			// erstes Zeichen senden
			SPDR = *data;
			// nächstes Zeichen
			data++;
			len--;

			// warten bis die Übertragung abgeschlossen ist
			//loop_until_bit_is_set(SPSR, SPIF);
			do {reg = SPSR;} while ((reg & _BV(SPIF)) == 0);
			// Datenregister lesen, damit das Flag gelöscht wird
			dummy = SPDR;
		// solange noch Daten zu senden sind
		} while (len > 0);

		// Freigabe für neue Daten
		spi_cts = 1;

		// Chipselect deaktivieren
//		SPI_CS_INACTIVE;
	}
}
#if 0
// Sending Routine Master Mode (interrupt controlled)
uint8_t spi_read (void)
{
	uint8_t	data = 0xFF;

	// keine Übertragung aktiv ?
	if (spi_cts == 1) {
		// neue Übertragungen blockieren
		spi_cts = 0;

		// Chipselect aktivieren
//		SPI_CS_ACTIVE;

		// Dummy Zeichen senden
		SPDR = 0xFF;
		// warten bis die Übertragung abgeschlossen ist
		loop_until_bit_is_set(SPSR, SPIF);
		// Daten Lesen
		data = SPDR;

		// Freigabe für neue Daten
		spi_cts = 1;

		// Chipselect deaktivieren
//		SPI_CS_INACTIVE;
	}
	// fertig
	return data;
}
#endif

void sd_command (uint8_t cmd, uint8_t arg0, uint8_t arg1, uint8_t arg2, uint8_t arg3, uint8_t crc)
{
	// wenn SPI frei ist
	if (spi_cts == 1) {
		// Parameter eintragen
		sd_cmd[0] = cmd;
		sd_cmd[1] = arg0;
		sd_cmd[2] = arg1;
		sd_cmd[3] = arg2;
		sd_cmd[4] = arg3;
		// CRC selbst ausrechnen
		crc = sd_crc7 (sd_cmd, 5);
		crc = (crc << 1) | 1;
		// und eintragen
		sd_cmd[5] = crc;
		// Kommando senden
		spi_write (sd_cmd, 6);
	}
}

uint8_t sd_receive (uint8_t timeout)
{
	uint8_t	reg;
	uint8_t	data = 0xFF;

	// keine Übertragung aktiv ?
	if (spi_cts == 1 && timeout > 0) {
		// neue Übertragungen blockieren
		spi_cts = 0;

		// Chipselect aktivieren
//		SPI_CS_ACTIVE;

		do {
			// Dummy Zeichen senden
			SPDR = 0xFF;
			// warten bis die Übertragung abgeschlossen ist
			//loop_until_bit_is_set(SPSR, SPIF);
			do {reg = SPSR;} while ((reg & _BV(SPIF)) == 0);
			// Daten Lesen
			data = SPDR;
			// irgendwann muss auch mal Schluss sein...
			timeout--;
		// weitermachen
		} while (data == 0xFF && timeout > 0);

		// Freigabe für neue Daten
		spi_cts = 1;

		// Chipselect deaktivieren
//		SPI_CS_INACTIVE;
	}
	// fertig
	return data;
}

uint8_t sd_crc7 (const uint8_t *data, uint8_t len)
{
	uint8_t		bit, crc, do_xor;

	crc = 0;

	// jedes Datenbyte einzeln
	while (len > 0) {
		// jedes Bit im Datenbyte einzeln
		for (bit = 0; bit < 8; bit++) {
			// MSB der CRC-7 sichern
			do_xor = (crc & 0x40);
			// Schieben
			crc = (crc << 1);
			// MSB der Daten testen
			if ((*data & 0x80) != 0)
				crc |= 1;
			// ver-x-odern ?
			if (do_xor != 0)
				crc = crc ^ 0x09;
		}
		// nächstes Byte
		data++;
		len--;
	}
	// am Ende noch 7 Nullen nachschieben
	for (bit = 0; bit < 7; bit++) {
		// MSB der CRC-7 sichern
		do_xor = (crc & 0x40);
		// Schieben
		crc = (crc << 1);
		// ver-x-odern ?
		if (do_xor != 0)
			crc = crc ^ 0x09;
	}
	// fertig
	return crc;
}
#endif

#if I2C_ENABLE
#if 0
void i2c_error(uint8_t fnr)
{

//  printf("error: TWI status %#x\n", twst);
//  exit(0);
//	lcd_text(0, "ERROR!", 1);
}
#endif

/* schickt eine Anzahl Bytes zu einem I2C-Slave */
int8_t i2c_write (uint8_t addr, const uint8_t *data, uint8_t len)
{
	uint8_t	n;
	int8_t	rv;
	uint8_t	twst;

	n = 0;
	rv = -1;

restart:
	if (n++ >= TWI_MAX_ITER) {
		return -1;
	}

begin:
	/* send start condition */
	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);	/* send start condition */
	while ((TWCR & _BV(TWINT)) == 0) ;			/* wait for transmission */
	switch ((twst = TW_STATUS))
	{
	case TW_REP_START:		/* OK, but should not happen */
	case TW_START:
		break;

	case TW_MT_ARB_LOST:
		goto begin;

	default:
		return -2;		/* error: not in start condition */
						/* NB: do /not/ send stop condition */
	}

	/* send SLA+W */
	TWDR = addr | TW_WRITE;
	TWCR = _BV(TWINT) | _BV(TWEN);		/* clear interrupt to start transmission */
	while ((TWCR & _BV(TWINT)) == 0) ;	/* wait for transmission */
	switch ((twst = TW_STATUS))
	{
	case TW_MT_SLA_ACK:
		break;

	case TW_MT_SLA_NACK:	/* nack during select: device busy */
		goto restart;

	case TW_MT_ARB_LOST:	/* re-arbitrate */
		goto begin;

	default:
		goto error;		/* must send stop condition */
	}

	rv = 0;

	if (data != NULL) {
		/* send data */
		while (len-- > 0) {
			TWDR = *data++;						/* data byte */
			TWCR = _BV(TWINT) | _BV(TWEN);		/* clear interrupt to start transmission */
			while ((TWCR & _BV(TWINT)) == 0) ;	/* wait for transmission */
			switch ((twst = TW_STATUS))
			{
			case TW_MT_DATA_ACK:
				break;

			case TW_MT_DATA_NACK:	/* data not successfully written */
				goto error;

			case TW_MT_ARB_LOST:
				goto begin;

			default:
				goto error;		/* must send stop condition */
			}
		}
	}

quit:
	TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN); /* send stop condition */
	return rv;

error:
	rv = -3;
	goto quit;
}

/* liest eine Anzahl Bytes vom I2C-Slave */
int8_t i2c_read (uint8_t addr, uint8_t *data, uint8_t len)
{
	uint8_t	n;
	int8_t	rv;
	uint8_t	twst;

	n = 0;
	rv = -1;

restart:
	if (n++ >= TWI_MAX_ITER) {
		return -4;
	}

begin:
	/* send start condition */
	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN); /* send start condition */
	while ((TWCR & _BV(TWINT)) == 0) ; /* wait for transmission */
	switch ((twst = TW_STATUS))
	{
	case TW_REP_START:		/* OK, but should not happen */
	case TW_START:
		break;

	case TW_MT_ARB_LOST:
		goto begin;

	default:
		return -5;		/* error: not in start condition */
						/* NB: do /not/ send stop condition */
	}

	/* send SLA+R */
	TWDR = addr | TW_READ;
	TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transmission */
	while ((TWCR & _BV(TWINT)) == 0) ; /* wait for transmission */
	switch ((twst = TW_STATUS))
	{
	case TW_MR_SLA_ACK:
		break;

	case TW_MR_SLA_NACK:
		rv = -6;
		goto error;

	case TW_MR_ARB_LOST:
		goto restart;

	default:
		goto error;
	}

	rv = 0;
	if (data != NULL && len > 0) {
		/* read data */
		while (len-- > 1) {
			/* read bytes and send ACK */
			TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) /* Note [13] */;
			while ((TWCR & _BV(TWINT)) == 0) ; /* wait for transmission */
			switch ((twst = TW_STATUS))
			{
			case TW_MR_DATA_NACK:
			/* FALLTHROUGH */
			case TW_MR_DATA_ACK:
				*data++ = TWDR;
				break;

			default:
				rv = -6;
				goto error;
			}
		}
		/* read last byte and send NACK */
		TWCR = _BV(TWINT) | _BV(TWEN) /* Note [13] */;
		while ((TWCR & _BV(TWINT)) == 0) ; /* wait for transmission */
		switch ((twst = TW_STATUS))
		{
		case TW_MR_DATA_NACK:
		/* FALLTHROUGH */
		case TW_MR_DATA_ACK:
			*data = TWDR;
			break;

		default:
			rv = -7;
			goto error;
		}
	}

quit:
	/* Note [14] */
	TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN); /* send stop condition */
	return rv;

error:
	//rv = -3;
	goto quit;
}
#endif

#if ADM_ENABLE
int8_t adm_read (uint8_t regAddr, uint8_t *value)
{
	int8_t	err;

	// Registeradresse senden
	err = i2c_write (TWI_SLA_ADM1032, &regAddr, 1);
	if (err == 0) {
		// Datum lesen
		err = i2c_read (TWI_SLA_ADM1032, value, 1);
	}
	return err;
}
#endif

#if UART_ENABLE
#if UART_SEND_ENABLE
/*! \brief send a byte.
 *
 *  Puts a byte in TX buffer and starts uart TX interrupt.
 *  If TX buffer is full it will hang until space.
 *
 *  \param data  Data to be sent.
 */
uint8_t uart_SendByte (uint8_t data)
{
	uint8_t tmphead;

	// Calculate buffer index
	tmphead = (UART_TxHead + 1) & UART_TX_BUFFER_MASK;
	// Test for free space in buffer
	if (tmphead != UART_TxTail) {
		// Save data
		UART_TxBuffer[tmphead] = data;
		// Store new index
		UART_TxHead = tmphead;
		// Enable UDRE interrupt
		SET_UDRIE;

		return 1;
	} else {
		// kein Platz mehr
	}
	return 0;
}
#endif	// UART_SEND_ENABLE

uint8_t uart_RecvByte (void)
{
	uint8_t data;

	// wenn Daten im Puffer stehen
	if (UART_RxTail != UART_RxHead) {
		// das letzte herausholen
		data = UART_RxBuffer[UART_RxTail];
		// Zeiger weiterschieben
		UART_RxTail = (UART_RxTail + 1) & UART_RX_BUFFER_MASK;
	} else {
		// nix empfangen
		data = 0;
	}

	return data;
}

#if UART_SEND_ENABLE
/*! \brief Sends a string.
 *
 *  Loops thru a string and send each byte with uart_SendByte.
 *  If TX buffer is full it will hang until space.
 *
 *  \param Str  String to be sent.
 */
void uart_SendString (uint8_t *Str)
{
	uint8_t n = 0;

	while (Str[n] != 0) {
		uart_SendByte (Str[n]);
		n++;
	}
}

#if 0
/*! \brief Sends a integer.
 *
 *  Converts a integer to ASCII and sends it using uart_SendByte.
 *  If TX buffer is full it will hang until space.
 *
 *  \param x  Integer to be sent.
 */
void uart_SendInt(int x)
{
	static const char dec[] = "0123456789";
	unsigned int div_val = 10000;

	if (x < 0){
		x = - x;
		uart_SendByte('-');
	}
	while (div_val > 1 && div_val > x)
		div_val /= 10;
	do {
		uart_SendByte (dec[x / div_val]);
		x %= div_val;
		div_val /= 10;
	} while(div_val);
}
#endif
#endif	// UART_SEND_ENABLE

/*! \brief Empties the uart RX buffer.
 *
 *  Empties the uart RX buffer.
 *
 *  \return x  Integer to be sent.
 */
void uart_FlushRxBuffer(void)
{
	UART_RxHead = 0;
	UART_RxTail = 0;
}

#if UART_RECV_ENABLE
/* - Interrupt-Behandlung - */
/*! \brief RX interrupt handler.
 *
 *  RX interrupt handler.
 *  RX interrupt always enabled.
 */
SIGNAL (SIG_USART_RECV)
{
	uint8_t data;
	uint8_t st;
	uint8_t tmpHead;

	st = UCSR0A;	// Status sichern
	data = UDR0;	// Datum speichern

	if (st & _BV(FE0)) {
		// Frame Error
		if (data == 0) {
			// BREAK, oder Kurzschluss nach Masse -> komplett deaktivieren
			UCSR0B &= ~_BV(RXEN0);
		}
		goto uart_error;
	}
	if (st & _BV(DOR0))	goto uart_error;	// Data Overrun
	if (st & _BV(UPE0))	goto uart_error;	// Parity Error, Paket wegwerfen

	// Schreibzeiger temporär eins weiterschieben
	tmpHead = (UART_RxHead + 1) & UART_RX_BUFFER_MASK;
	// und mit dem Lesezeiger vergleichen
	if (tmpHead != UART_RxTail) {
		// Datum passt noch rein
		UART_RxBuffer[UART_RxHead] = data;
		UART_RxHead = tmpHead;
	} else {
		// Puffer voll !!!
	}

uart_error:
	return;
}
#endif	// UART_RECV_ENABLE

#if UART_SEND_ENABLE
/*! \brief TX interrupt handler.
 *
 *  TX interrupt handler.
 *  TX interrupt turned on by uart_SendByte,
 *  turned off when TX buffer is empty.
 */
SIGNAL (SIG_USART_DATA)
{
	uint8_t tmpTail;

	tmpTail = UART_TxTail;

	// Check if all data is transmitted
	if (UART_TxHead != tmpTail) {
		// Calculate buffer index
		tmpTail = (UART_TxTail + 1) & UART_TX_BUFFER_MASK;
		// Store new index
		UART_TxTail = tmpTail;
		// Start transmition
		UDR0 = UART_TxBuffer[tmpTail];
	} else {
		// Disable UDRE interrupt
		CLR_UDRIE;
	}
}
#endif	// UART_SEND_ENABLE
#endif	// UART_ENABLE

#if EE24_ENABLE
/*
 * Note [7]
 *
 * Read "len" bytes from EEPROM starting at "eeaddr" into "buf".
 *
 * This requires two bus cycles: during the first cycle, the device
 * will be selected (master transmitter mode), and the address
 * transfered.  Address bits exceeding 256 are transfered in the
 * E2/E1/E0 bits (subaddress bits) of the device selector.
 *
 * The second bus cycle will reselect the device (repeated start
 * condition, going into master receiver mode), and transfer the data
 * from the device to the TWI master.  Multiple bytes can be
 * transfered by ACKing the client's transfer.  The last transfer will
 * be NACKed, which the client will take as an indication to not
 * initiate further transfers.
 */
int ee24xx_read_bytes(uint16_t eeaddr, int len, uint8_t *buf)
{

  uint8_t sla, twcr, n = 0;
  int rv = 0;

  /* patch high bits of EEPROM address into SLA */
  sla = TWI_SLA_24CXX | (((eeaddr >> 8) & 0x07) << 1);

  /*
   * Note [8]
   * First cycle: master transmitter mode
   */
  restart:
  if (n++ >= TWI_MAX_ITER)
  {
    return -1;
  }
  begin:

  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN); /* send start condition */
  while ((TWCR & _BV(TWINT)) == 0) ; /* wait for transmission */
  switch ((twst = TW_STATUS))
    {
    case TW_REP_START:		/* OK, but should not happen */
    case TW_START:
      break;

    case TW_MT_ARB_LOST:	/* Note [9] */
      goto begin;

    default:
      return -1;		/* error: not in start condition */
				/* NB: do /not/ send stop condition */
    }

  /* Note [10] */
  /* send SLA+W */
  TWDR = sla | TW_WRITE;
  TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transmission */
  while ((TWCR & _BV(TWINT)) == 0) ; /* wait for transmission */
  switch ((twst = TW_STATUS))
    {
    case TW_MT_SLA_ACK:
      break;

    case TW_MT_SLA_NACK:	/* nack during select: device busy writing */
				/* Note [11] */
      goto restart;

    case TW_MT_ARB_LOST:	/* re-arbitrate */
      goto begin;

    default:
      goto error;		/* must send stop condition */
    }

  TWDR = eeaddr;		/* low 8 bits of addr */
  TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transmission */
  while ((TWCR & _BV(TWINT)) == 0) ; /* wait for transmission */
  switch ((twst = TW_STATUS))
    {
    case TW_MT_DATA_ACK:
      break;

    case TW_MT_DATA_NACK:
      goto quit;

    case TW_MT_ARB_LOST:
      goto begin;

    default:
      goto error;		/* must send stop condition */
    }

  /*
   * Note [12]
   * Next cycle(s): master receiver mode
   */
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN); /* send (rep.) start condition */
  while ((TWCR & _BV(TWINT)) == 0) ; /* wait for transmission */
  switch ((twst = TW_STATUS))
    {
    case TW_START:		/* OK, but should not happen */
    case TW_REP_START:
      break;

    case TW_MT_ARB_LOST:
      goto begin;

    default:
      goto error;
    }

  /* send SLA+R */
  TWDR = sla | TW_READ;
  TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transmission */
  while ((TWCR & _BV(TWINT)) == 0) ; /* wait for transmission */
  switch ((twst = TW_STATUS))
    {
    case TW_MR_SLA_ACK:
      break;

    case TW_MR_SLA_NACK:
      goto quit;

    case TW_MR_ARB_LOST:
      goto begin;

    default:
      goto error;
    }

  for (twcr = _BV(TWINT) | _BV(TWEN) | _BV(TWEA) /* Note [13] */;
       len > 0;
       len--)
    {
      if (len == 1)
	twcr = _BV(TWINT) | _BV(TWEN); /* send NAK this time */
      TWCR = twcr;		/* clear int to start transmission */
      while ((TWCR & _BV(TWINT)) == 0) ; /* wait for transmission */
      switch ((twst = TW_STATUS))
	{
	case TW_MR_DATA_NACK:
	  len = 0;		/* force end of loop */
	  /* FALLTHROUGH */
	case TW_MR_DATA_ACK:
	  *buf++ = TWDR;
	  rv++;
	  break;

	default:
	  goto error;
	}
    }
  quit:
  /* Note [14] */
  TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN); /* send stop condition */

  return rv;

  error:
  rv = -1;
  goto quit;
}

/*
 * Write "len" bytes into EEPROM starting at "eeaddr" from "buf".
 *
 * This is a bit simpler than the previous function since both, the
 * address and the data bytes will be transfered in master transmitter
 * mode, thus no reselection of the device is necessary.  However, the
 * EEPROMs are only capable of writing one "page" simultaneously, so
 * care must be taken to not cross a page boundary within one write
 * cycle.  The amount of data one page consists of varies from
 * manufacturer to manufacturer: some vendors only use 8-byte pages
 * for the smaller devices, and 16-byte pages for the larger devices,
 * while other vendors generally use 16-byte pages.  We thus use the
 * smallest common denominator of 8 bytes per page, declared by the
 * macro PAGE_SIZE above.
 *
 * The function simply returns after writing one page, returning the
 * actual number of data byte written.  It is up to the caller to
 * re-invoke it in order to write further data.
 */
int ee24xx_write_page(uint16_t eeaddr, int len, uint8_t *buf)
{
  uint8_t sla, n = 0;
  int rv = 0;
  uint16_t endaddr;

  if (eeaddr + len < (eeaddr | (PAGE_SIZE - 1)))
    endaddr = eeaddr + len;
  else
    endaddr = (eeaddr | (PAGE_SIZE - 1)) + 1;
  len = endaddr - eeaddr;

  /* patch high bits of EEPROM address into SLA */
  sla = TWI_SLA_24CXX | (((eeaddr >> 8) & 0x07) << 1);

  restart:
  if (n++ >= TWI_MAX_ITER)
  {
    return -1;
  }
  begin:

  /* Note [15] */
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN); /* send start condition */
  while ((TWCR & _BV(TWINT)) == 0) ; /* wait for transmission */
  switch ((twst = TW_STATUS))
    {
    case TW_REP_START:		/* OK, but should not happen */
    case TW_START:
      break;

    case TW_MT_ARB_LOST:
      goto begin;

    default:
      return -1;		/* error: not in start condition */
				/* NB: do /not/ send stop condition */
    }

  /* send SLA+W */
  TWDR = sla | TW_WRITE;
  TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transmission */
  while ((TWCR & _BV(TWINT)) == 0) ; /* wait for transmission */
  switch ((twst = TW_STATUS))
    {
    case TW_MT_SLA_ACK:
      break;

    case TW_MT_SLA_NACK:	/* nack during select: device busy writing */
      goto restart;

    case TW_MT_ARB_LOST:	/* re-arbitrate */
      goto begin;

    default:
      goto error;		/* must send stop condition */
    }

  TWDR = eeaddr;		/* low 8 bits of addr */
  TWCR = _BV(TWINT) | _BV(TWEN); /* clear interrupt to start transmission */
  while ((TWCR & _BV(TWINT)) == 0) ; /* wait for transmission */
  switch ((twst = TW_STATUS))
    {
    case TW_MT_DATA_ACK:
      break;

    case TW_MT_DATA_NACK:
      goto quit;

    case TW_MT_ARB_LOST:
      goto begin;

    default:
      goto error;		/* must send stop condition */
    }

  for (; len > 0; len--)
    {
      TWDR = *buf++;
      TWCR = _BV(TWINT) | _BV(TWEN); /* start transmission */
      while ((TWCR & _BV(TWINT)) == 0) ; /* wait for transmission */
      switch ((twst = TW_STATUS))
	{
	case TW_MT_DATA_NACK:
	  goto error;		/* device write protected -- Note [16] */

	case TW_MT_DATA_ACK:
	  rv++;
	  break;

	default:
	  goto error;
	}
    }
  quit:
  TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN); /* send stop condition */

  return rv;

  error:
  rv = -1;
  goto quit;
}

/*
 * Wrapper around ee24xx_write_page() that repeats calling this
 * function until either an error has been returned, or all bytes
 * have been written.
 */
int ee24xx_write_bytes(uint16_t eeaddr, int len, uint8_t *buf)
{
  int rv, total;

  total = 0;
  do
    {
      rv = ee24xx_write_page(eeaddr, len, buf);
      if (rv == -1)
	    return -1;
      eeaddr += rv;
      len -= rv;
      buf += rv;
      total += rv;
    }
  while (len > 0);

  return total;
}
#endif
