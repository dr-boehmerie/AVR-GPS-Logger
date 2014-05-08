/*
 * Author: Michael Böhme
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */


#ifndef CONFIG_H
#define CONFIG_H

#define LCD_TEST_CONTRAST	0

//#define SPI_INTERRUPT_DRIVEN

#ifndef SD_CARD_ENABLE
#define SD_CARD_ENABLE		0
#endif

#define ADM_ENABLE			0

#define HYT_ENABLE			1
#define HYT_TIMER_ENABLE	1
#define HYT_AVERAGE_NO		8

#define HYT_T_STEP_5		0
#define HYT_RH_STEP_5		0

#define EE24_ENABLE			0

#define UART_ENABLE			1
#define UART_SEND_ENABLE	0
#define UART_RECV_ENABLE	1

// UART Buffer Defines
#define UART_RX_BUFFER_SIZE		128		// 2,4,8,16,32,64,128 or 256 bytes
#define UART_RX_BUFFER_MASK		(UART_RX_BUFFER_SIZE - 1)
#if (UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK)
	#error RX buffer size is not a power of 2
#endif

#define UART_TX_BUFFER_SIZE		32		// 2,4,8,16,32,64,128 or 256 bytes
#define UART_TX_BUFFER_MASK		(UART_TX_BUFFER_SIZE - 1)
#if (UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK)
	#error TX buffer size is not a power of 2
#endif

#define BAUD		38400UL


#define NMEA_ENABLE			1

#define GPS_DATA_LOG_ENABLE	1

#define FRAM_ENABLE			0

#define I2C_ENABLE			(EE24_ENABLE || ADM_ENABLE || HYT_ENABLE)
#endif
