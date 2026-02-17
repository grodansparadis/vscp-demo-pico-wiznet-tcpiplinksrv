// FILE: demo.h

/* ******************************************************************************
 * 	VSCP (Very Simple Control Protocol)
 * 	https://www.vscp.org
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2000-2026 Ake Hedman, Grodans Paradis AB <info@grodansparadis.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *	This file is part of VSCP - Very Simple Control Protocol
 *	https://www.vscp.org
 *
 * ******************************************************************************
 */

#include <stdio.h>
#include <time.h>

#include <vscp.h>
#include <vscp-fifo.h>
#include <vscp-firmware-helper.h>
#include <vscp-link-protocol.h>
#include <vscp-firmware-level2.h>

#ifndef __VSCP_DEMO_WIZNET_PICO__
#define __VSCP_DEMO_WIZNET_PICO__

// VSCP TCP link protocol server test debug message printout enable 
#define _DEMO_DEBUG_

// Max number of sockets (WS5100S == 4) 
#define MAX_CONNECTIONS   2

// Status LED 
#define LED_PIN 25

// Clock 
#define PLL_SYS_KHZ (133 * 1000)

// Buffer 
#define ETHERNET_BUF_MAX_SIZE (1024 * 2)

/** 
 * VSCP TCP link protocol character buffer size
 */
#ifndef DATA_BUF_SIZE
#define DATA_BUF_SIZE 512
#endif

/** 
 * Max number of events in the receive fifo 
 */
#define RECEIVE_FIFO_SIZE 16

/** 
 * Max number of events in each of the transmit fifos  
 */
#define TRANSMIT_FIFO_SIZE 16

#define DEMO_WELCOME_MSG "Welcome to the wiznet w5100s pico demo VSCP TCP link protocol node\r\n" \
                         "Copyright (C) 2000-2025 Grodans Paradis AB\r\n"                  \
                         "https://www.grodansparadis.com\r\n"                              \
                         "+OK\r\n"


/* 
  Socket context 
  This is the context for each open socket/channel.
*/
struct _ctx {
  uint8_t sn;                                   // Socket
  size_t size;                                  // Number of characters in buffer
  uint8_t buf[ETHERNET_BUF_MAX_SIZE];           // Command Buffer
  uint8_t user[VSCP_LINK_MAX_USER_NAME_LENGTH]; // Username storage
  vscp_fifo_t fifoEventsOut;                    // VSCP event send fifo
  int bValidated;                               // User is validated
  uint8_t privLevel;                            // User privilege level 0-15
  int bRcvLoop;                                 // Receive loop is enabled if non zero
  vscpEventFilter filter;                       // Filter for events
  VSCPStatistics statistics;                    // VSCP Statistics
  VSCPStatus status;                            // VSCP status
  uint32_t last_rcvloop_time;                   // Time of last received event
};

// Command to put core 2 into idle
static const uint32_t _GOTOSLEEP = 0xC0DED02E;

#define ENABLE_UDP_HEARTBEAT
#define ENABLE_MULTICAST_HEARTBEAT
#define ENABLE_NTP
//#define ENABLE_DHCP

// Defines for flash variable storage
#define FLASH_EEPROM_START  (uint8_t*)(0x1FF000) // Start of EEPROM simulated flash 4096 bytes, (2MB flash)
#define FLASH_CTRL_BYTE_MSB 0xFE                 // MSB of flash control byte
#define FLASH_CTRL_BYTE_LSB 0xFF                 // LSB of flash control byte

#define NTP_SERVER "pool.ntp.org" // https://www.pool.ntp.org/zone/@

// Register definitions

#define REG_DEVICE_ZONE         0x00000000		// (RW) Zone for device.
#define REG_DEVICE_SUBZONE      0x00000001		// (RW) Subzone for device.

/** 
 * bit 7 - Enable LED control
 * bit 6 - 
 * bit 5 -
 * bit 4 -
 * bit 3 -
 * bit 2 -
 * bit 1 - Enable LED blink
 */
#define REG_LED_CTRL            0x00000002		// (RW) LED Control register.

/**
 * bit 0 - 0 - LED on, 1 - LED off  
 */
#define REG_LED_STATUS          0x00000003    // (RW) LED Status register.

#define REG_LED_BLINK_INTERVAL  0x00000004		// (RW) LED blink interval register (100ms).

/**
 * bit 7 - GP9 direction, 0 - output, 1 - input
 * bit 6 - GP8 direction, 0 - output, 1 - input
 * bit 5 - GP7 direction, 0 - output, 1 - input
 * bit 4 - GP6 direction, 0 - output, 1 - input
 * bit 3 - GP5 direction, 0 - output, 1 - input
 * bit 2 - GP4 direction, 0 - output, 1 - input
 * bit 1 - GP3 direction, 0 - output, 1 - input
 * bit 0 - GP2 direction, 0 - output, 1 - input
 */
#define REG_IO_CTRL1             0x00000010    // (RW) I/O Control register.

/**
 * bit 7 - GP9 aĺarm enable, 0 - disable, 1 - enable
 * bit 6 - GP8 aĺarm enable, 0 - disable, 1 - enable
 * bit 5 - GP7 aĺarm enable, 0 - disable, 1 - enable
 * bit 4 - GP6 aĺarm enable, 0 - disable, 1 - enable
 * bit 3 - GP5 aĺarm enable, 0 - disable, 1 - enable
 * bit 2 - GP4 aĺarm enable, 0 - disable, 1 - enable
 * bit 1 - GP3 aĺarm enable, 0 - disable, 1 - enable
 * bit 0 - GP2 aĺarm enable, 0 - disable, 1 - enable
 */
#define REG_IO_CTRL2             0x00000011    // (RW) I/O Control register.

/**
 * Status for I/O bits
 * 
 * bit 7 - GP9 status, 0 - low, 1 - high
 * bit 6 - GP8 status, 0 - low, 1 - high
 * bit 5 - GP7 status, 0 - low, 1 - high
 * bit 4 - GP6 status, 0 - low, 1 - high
 * bit 3 - GP5 status, 0 - low, 1 - high
 * bit 2 - GP4 status, 0 - low, 1 - high
 * bit 1 - GP3 status, 0 - low, 1 - high
 * bit 0 - GP2 status, 0 - low, 1 - high
 *
 */
#define REG_IO_STATUS           0x00000012    // (RW) I/O Status register.

/**
 * bit 7 - Enable reporting of temperature.
 * bit 6 -
 * bit 5 -
 * bit 4 -
 * bit 3 -
 * bit 2 -
 * bit 1 - Unit of temperature. 0 = Kelvin, 1 = Celsius, 2 = Fahrenheit.
 * bit 0 - Unit LSB
 */
#define REG_TEMP_CTRL           0x00000020    // (RW) Temp sensor control register  ADC4
#define REG_TEMP_RAW_MSB        0x00000021    // (RO) Temperature raw MSB
#define REG_TEMP_RAW_LSB        0x00000022    // (RO) Temperature raw LSB
#define REG_TEMP_CORR_MSB       0x00000023    // (RW) Temperature correction factor MSB
#define REG_TEMP_CORR_LSB       0x00000024    // (RW) Temperature correction factor LSB
#define REG_TEMP_INTERVAL       0x00000025    // (RW) Temperature report interval

#define REG_ADC0_CTRL           0x00000030    // (RW) ADC0 control register 
#define REG_ADC0_MSB            0x00000031    // (RW) ADC0 MSB
#define REG_ADC0_LSB            0x00000032    // (RW) ADC0 LSB

#define REG_ADC1_CTRL           0x00000033    // (RW) ADC1 control register
#define REG_ADC1_MSB            0x00000034    // (RW) ADC1 MSB
#define REG_ADC1_LSB            0x00000035    // (RW) ADC1 LSB

#define REG_ADC2_CTRL           0x00000036    // (RW) ADC2 control register 
#define REG_ADC2_MSB            0x00000037    // (RW) ADC2 MSB
#define REG_ADC2_LSB            0x00000038    // (RW) ADC2 LSB

/* 
  The unique board id
  https://raspberrypi.github.io/pico-sdk-doxygen/group__pico__unique__id.html
*/
#define REG_BOARD_ID0           0x00000080    // (RO) Board ID 0
#define REG_BOARD_ID1           0x00000081    // (RO) Board ID 1
#define REG_BOARD_ID2           0x00000082    // (RO) Board ID 2
#define REG_BOARD_ID3           0x00000083    // (RO) Board ID 3
#define REG_BOARD_ID4           0x00000084    // (RO) Board ID 4
#define REG_BOARD_ID5           0x00000085    // (RO) Board ID 5
#define REG_BOARD_ID6           0x00000086    // (RO) Board ID 6
#define REG_BOARD_ID7           0x00000087    // (RO) Board ID 7
#define REG_BOARD_ID8           0x00000088    // (RO) Board ID 8


/* 
  *** Standard registers ***

  Some standard registers is stored in eeprom so they can be
  written during manufacturing
*/  

#define STDREG_USER_ID0               0x000000E1    // (RO) User ID 0 register.
#define STDREG_USER_ID1               0x000000E2    // (RO) User ID 1 register.
#define STDREG_USER_ID2               0x000000E3    // (RO) User ID 2 register.
#define STDREG_USER_ID3               0x000000E4    // (RO) User ID 3 register.
#define STDREG_USER_ID4               0x000000E5    // (RO) User ID 4 register.


/*
  Manufacturer and GUID is stored in eeprom on a device that should
  be possible to be configured with unique data in these locations before 
  it is shipped to a customer.
*/


#ifdef THIS_FIRMWARE_ENABLE_WRITE_2PROTECTED_LOCATIONS 

#define STDREG_MANUFACTURER_ID0       0x000000E6    // (RO) Manufacturer ID 0 register.
#define STDREG_MANUFACTURER_ID1       0x000000E7    // (RO) Manufacturer ID 1 register.
#define STDREG_MANUFACTURER_ID2       0x000000E8    // (RO) Manufacturer ID 2 register.
#define STDREG_MANUFACTURER_ID3       0x000000E9    // (RO) Manufacturer ID 3 register.

#define STDREG_MANUFACTURER_SUBID0    0x000000EA    // (RO) Manufacturer SUBID 0 register.
#define STDREG_MANUFACTURER_SUBID1    0x000000EB    // (RO) Manufacturer SUBID 1 register.
#define STDREG_MANUFACTURER_SUBID2    0x000000EC    // (RO) Manufacturer SUBID 2 register.
#define STDREG_MANUFACTURER_SUBID3    0x000000ED    // (RO) Manufacturer SUBID 3 register.

#define STDREG_GUID0                  0x000000EE    // (RO) GUID 0 register.
#define STDREG_GUID1                  0x000000EF    // (RO) GUID 1 register.
#define STDREG_GUID2                  0x000000F0    // (RO) GUID 2 register.
#define STDREG_GUID3                  0x000000F1    // (RO) GUID 3 register.
#define STDREG_GUID4                  0x000000F2    // (RO) GUID 4 register.
#define STDREG_GUID5                  0x000000F3    // (RO) GUID 5 register.
#define STDREG_GUID6                  0x000000F4    // (RO) GUID 6 register.
#define STDREG_GUID7                  0x000000F5    // (RO) GUID 7 register.
#define STDREG_GUID8                  0x000000F6    // (RO) GUID 8 register.
#define STDREG_GUID9                  0x000000F7    // (RO) GUID 9 register.
#define STDREG_GUID10                 0x000000F8    // (RO) GUID 10 register.
#define STDREG_GUID11                 0x000000F9    // (RO) GUID 11 register.
#define STDREG_GUID12                 0x000000FA    // (RO) GUID 12 register.
#define STDREG_GUID13                 0x000000FB    // (RO) GUID 13 register.
#define STDREG_GUID14                 0x000000FC    // (RO) GUID 14 register.
#define STDREG_GUID15                 0x000000FD    // (RO) GUID 15 register.

#endif

#define REG_DM_START            0xFFFE0000    // Start for decision matrix.
#define REG_STANDARD_START      0xFFFF0000		// Start for level II standard registers.


// Decision matrix action definitions.

#define DM_ACTION_NOOP          0x0000        // No operation.
#define DM_ACTION_LED_CTRL      0x0001			  // Arg '0': LED off, Arg '1': LED on.
#define DM_ACTION_IO_CTRL       0x0002			  // Control I/o pins.
#define DM_ACTION_REPORT_IO     0x0003        // Send I/O status events
#define DM_ACTION_REPORT_TEMP   0x0004        // Write action data to serial channel.
#define DM_ACTION_REPORT_ADC    0x0005        // Report ADC value. Arg is channel 0-3
#define DM_ACTION_NTP           0x0006        // Check time from NTP server


/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */


/**
 * @brief Set defaults for context structure
 * 
 */

void
setContextDefaults(struct _ctx* pctx);

/**
 * @brief Set the clock object
 */

void
set_clock_khz(void);

/**
 * @brief Write data to a connected socket
 * 
 * @param sn Socket number
 * @param buf Buffer with data to send
 * @param size Number of valid bytes in buffer
 * @return VSCP_ERROR_SUCCESS on success, error code on failure. 
 */

int
writeSocket(uint8_t sn, uint8_t* buf, uint16_t size);

/**
 * @brief Handle socket events
 * 
 * @param pctx Pointer to context structure.
 * @param port Port number.
 * @return 1 on success, wiznet error code on failure. 
 */

int32_t
vscp_handleSocketEvents(struct _ctx* pctx, uint16_t port);


/** 
 * @brief  Get millisecond timestamp
 */
time_t getMilliSeconds(void);


/**
 * @brief 
 * 
 */
void idle_other_core(void);

/**
 * @brief 
 * 
 */
void resume_other_core(void);

/**
 * @brief 
 * 
 */
void init_persistent_storage(void);

/*!
 * @brief Get time from NTP server
 */

void
getNtpTime(void);

/*!
  * @brief Get temperature from onboard sensor
  * @return Temperature in degrees celsius. 
  */

float
read_onboard_temperature(void);

/*!
  * @brief Get ADC value
  * @param channel ADC channel 0-2
  * @return Return voltage. 
  */

float 
read_adc(uint8_t channel);

#endif  // __VSCP_DEMO_WIZNET_PICO__