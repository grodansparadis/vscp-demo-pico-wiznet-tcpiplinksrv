// FILE: demo.c

/* ******************************************************************************
 * 	VSCP (Very Simple Control Protocol)
 * 	https://www.vscp.org
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2000-2022 Ake Hedman, Grodans Paradis AB <info@grodansparadis.com>
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

//#include "pico.h"

#include <hardware/gpio.h>
#include <hardware/adc.h>
#include <hardware/watchdog.h>
#include <hardware/sync.h>
#include <hardware/flash.h>
#include <pico/binary_info.h>
#include <pico/multicore.h>
#include <pico/mutex.h>
#include <hardware/structs/timer.h>
#include <pico/stdlib.h>
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "port_common.h"
#include "wizchip_conf.h"
#include "w5x00_spi.h"

#include "socket.h"
#include "wizchip_conf.h"
#include "sntp.h"
#include "timer.h"

#include "pico-eeprom.h"

#include "vscp-compiler.h"
#include "vscp-projdefs.h"

#include "demo.h"


/**
 * ----------------------------------------------------------------------------------------------------
 *                                        Global Variables
 * ----------------------------------------------------------------------------------------------------
 */

/*
  Core sync
*/
volatile bool __otherCoreState; // Use for syncronization between cores
mutex_t _idleMutex;               // Mutex for idle state

/* 
  Network 

  Note that the max address is used to construct the device GUID
*/
static wiz_NetInfo net_info = {
  .mac  = { 0x00, 0x08, 0xDC, 0x12, 0x34, 0x56 }, // MAC address (Also part of GUID)
  .ip   = { 192, 168, 1, 189 },                   // IP address
  .sn   = { 255, 255, 255, 0 },                   // Subnet Mask
  .gw   = { 192, 168, 1, 1 },                     // Gateway
  .dns  = { 8, 8, 8, 8 },                         // DNS server (Default Google public DNS 8.8.8.8)
  .dhcp = NETINFO_STATIC                          // DHCP enable/disable
};

#ifdef ENABLE_NTP

/* Timezone */
#define TIMEZONE 0 // GMT is use for VSCP

/* Socket used  for SNTP */
#define SOCKET_SNTP 3

static uint8_t ntp_buf[ETHERNET_BUF_MAX_SIZE] = {
    0,
};
static uint8_t sntp_server_ip[4] = {216, 239, 35, 0}; // time.google.com

#endif

// Flash memory simulated EEPROM
struct _eeprom_ eeprom;

// Milliseconds timer 
static volatile uint32_t milliseconds = 0;

/**
  GUID for device
  This is the GUID that is used to identify the device.
  Use Ethernet MAC address as base. Can also be set explicitly.
*/
uint8_t device_guid[16] = THIS_FIRMWARE_GUID;

/**
  Received event are written to this fifo 
  from all channels and events is consumed by the 
  VSCP protocol handler.
*/
vscp_fifo_t fifoEventsIn;

struct _ctx ctx[MAX_CONNECTIONS]; // Socket context



/* -------------------------------------------------------------------------- */



///////////////////////////////////////////////////////////////////////////////
// repeating_timer_callback
//

static void repeating_timer_callback(void)
{
  milliseconds++;
}

///////////////////////////////////////////////////////////////////////////////
// idle_other_core
//

void idle_other_core(void)
{
  mutex_enter_blocking(&_idleMutex);
  __otherCoreState = false;
  multicore_fifo_push_blocking(_GOTOSLEEP);
  while (!__otherCoreState) { /* noop */ }
}

///////////////////////////////////////////////////////////////////////////////
// resume_other_core
//

void resume_other_core(void)
{
  mutex_exit(&_idleMutex);
  __otherCoreState = false;
  // Other core will exit busy-loop and return to operation
  // once __otherCoreIdState == false.
}


#define FLAG_VALUE 123

///////////////////////////////////////////////////////////////////////////////
// core1_entry
//

void core1_entry(void) {

    multicore_fifo_push_blocking(FLAG_VALUE);

    uint32_t g = multicore_fifo_pop_blocking();

    if (g != FLAG_VALUE)
        printf("Hmm, that's not right on core 1!\n");
    else
        printf("Its all gone well on core 1!");

    while (1) {
        tight_loop_contents();

        multicore_fifo_clear_irq();
        uint32_t status = save_and_disable_interrupts();
        while (multicore_fifo_rvalid()) {
          if (_GOTOSLEEP == multicore_fifo_pop_blocking()) {
            __otherCoreState = true;
            while (__otherCoreState) { /* noop */ }
            break;
          }
        }
        restore_interrupts(status);
    }
}

///////////////////////////////////////////////////////////////////////////////
// main
//

int
main()
{
  // Initialize 
  int rv = 0;
  uint32_t start_ms = 0;
  datetime time;

  bi_decl(bi_program_description("This is a demo binary for the VSCP tcp/ip link protocol on pico with wiznet w5100s."));
  bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));

  //set_clock_khz();
  stdio_init_all();

  mutex_init(&_idleMutex);

  /** 
   * Enable the watchdog, requiring the watchdog to be updated every 2000ms or
   * the chip will reboot second arg is pause on debug which means the watchdog
   * will pause when stepping through code
   */ 
  //watchdog_enable(5000, 1);

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  // Use Ethernet mac address as GUID 
  memcpy(device_guid + 8, net_info.mac, 6);

  for (int i = 0; i < MAX_CONNECTIONS; i++) {
    ctx[i].sn          = i;
    vscp_fifo_init(&ctx[i].fifoEventsOut, TRANSMIT_FIFO_SIZE);
    setContextDefaults(&ctx[i]);
  }

  vscp_fifo_init(&fifoEventsIn, RECEIVE_FIFO_SIZE);

  if (watchdog_caused_reboot()) {
    printf("Rebooted by Watchdog!\n");
    //return 0;
  }
  else {
    printf("Clean boot\n");
  }  

#ifdef _DEMO_DEBUG_
  /* Demo to allow serial software to open port */
  sleep_ms(2000);
#endif  

  // Run code on core 1
  //multicore_launch_core1(core1_entry);

  printf("\n\n\n\n\n\n_________________________________________________\n");

  // Initialize the EEPROM storage
  if (0 != eeprom_init(&eeprom, FLASH_PAGE_SIZE, (uint8_t *)FLASH_EEPROM_START)) {
    printf("EEPROM init failed\n");
  }

  // If eeprom signature is not present initialize persistent storage
  if ((eeprom_read(&eeprom, FLASH_CTRL_BYTE_MSB) != 0x55) || 
      (eeprom_read(&eeprom, FLASH_CTRL_BYTE_LSB) != 0xaa)) {
    printf("Init persistent storage\n");
    init_persistent_storage();
    eeprom_write(&eeprom, FLASH_CTRL_BYTE_MSB, 0x55);
    eeprom_write(&eeprom, FLASH_CTRL_BYTE_LSB, 0xaa);
    if (eeprom_commit(&eeprom)) {
      printf("EEPROM init failed\n");
    }
  }
  else {
    printf("Persistent storage already initialized\n");
  }


  /* Initialize TCP/IP stack/chip */

  wizchip_spi_initialize();
  wizchip_cris_initialize();

  wizchip_reset();
  wizchip_initialize();
  wizchip_check();

  wizchip_1ms_timer_initialize(repeating_timer_callback);

  network_initialize(net_info);

#ifdef ENABLE_NTP
  // Initialize ntp functionality
  SNTP_init(SOCKET_SNTP, sntp_server_ip, TIMEZONE, ntp_buf);
#endif  

  // Get network information 
  print_network_information(net_info);

  gpio_put(LED_PIN, 0);

  // --------------------------------------------------------------------------
  //                                   Start NTP
  // --------------------------------------------------------------------------

#ifdef ENABLE_NTP
  start_ms = getMilliseconds();

  // Get time
  do {
    rv = SNTP_run(&time);

    if (rv == 1) {
      break;
    }
  } while ((getMilliseconds() - start_ms) < RECV_TIMEOUT);

  if (rv != 1) {
#ifdef _DEMO_DEBUG_    
    printf(" SNTP failed : %d\n", rv);
#endif    
    // Nill yo get defaults on receiving side
    memset(&time, 0, sizeof(time));
  }

#ifdef _DEMO_DEBUG_
  printf("* * * NTP: %d-%02d-%02d, %02d:%02d:%02d\n", time.yy, time.mo, time.dd, time.hh, time.mm, time.ss);
#endif

#endif  

  // -------------------------------------------------------------------------
  //                                 End NTP
  // -------------------------------------------------------------------------

  while (1) {

    watchdog_update();

    // VSCP TCP link server handler 
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
      if ((rv = vscp_handleSocketEvents(&ctx[i], VSCP_DEFAULT_TCP_PORT)) < 0) {
        printf(" Link error : %d\n", rv);
        while (1) {
          ;
        }
      }

      // If buf contains a carriage return, we have a command to handle 
      char* p = NULL;
      if (NULL != (p = strstr(ctx[i].buf, "\r\n"))) {

        char cmd[80];
        memset(cmd, 0, sizeof(cmd));

        *p             = '\0';
        size_t cmdSize = strlen(ctx[i].buf);
        strncpy(cmd, ctx[i].buf, sizeof(cmd) - 1);
        printf("Command: %s\n", cmd);

        p += 2; // Point beyond the \r\n
        if (*p) {
          strncpy(ctx[i].buf, p, ctx[i].size - cmdSize - 2);
          ctx[i].size -= (cmdSize + 2);
        }
        else {
          ctx[i].buf[0] = '\0';
          ctx[i].size   = 0;
        }
        printf("Buffer after: %s\n", ctx[i].buf);

        // Parse VSCP command
        vscp_link_parser(&ctx[i], cmd);

        // Feed VSCP machine


        // We have a command to handle
        // if (0 == strncmp(ctx[i].buf, "quit\r\n", 6)) {

        //   // Confirm quit
        //   writeSocket(ctx[i].sn, "+OK\r\n", 5);

        //   // Disconnect from client
        //   disconnect(ctx[i].sn);
        // }
        // else {
        //   vscp_link_parser(&ctx, ctx[i].buf);
        // }

        // memset(ctx[i].buf, 0, ETHERNET_BUF_MAX_SIZE);
      }

      // Do protocol work here
      vscp2_do_periodic_work(NULL);

      // Handle rcvloop etc
      vscp_link_idle_worker(&ctx[i]);
    }

    gpio_put(LED_PIN, 1);

    // gpio_put(LED_PIN, 0);
    // sleep_ms(250);
    // gpio_put(LED_PIN, 1);
    // puts("Hello World\n");
    // sleep_ms(1000);
  }
}


///////////////////////////////////////////////////////////////////////////////
// getMilliseconds
//

time_t getMilliseconds(void)
{
  return milliseconds;
}


/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */


///////////////////////////////////////////////////////////////////////////////
// setContextDefaults
// 

void
setContextDefaults(struct _ctx* pctx)
{
  pctx->bValidated        = false;
  pctx->privLevel         = 0;
  pctx->bRcvLoop          = 0;
  pctx->size              = 0;
  pctx->last_rcvloop_time = time_us_32();
  vscp_fifo_clear(&pctx->fifoEventsOut);
  memset(pctx->buf, 0, ETHERNET_BUF_MAX_SIZE);
  memset(pctx->user, 0, VSCP_LINK_MAX_USER_NAME_LENGTH);  
  // Filter: All events received
  memset(&pctx->filter, 0, sizeof(vscpEventFilter));
  memset(&pctx->statistics, 0, sizeof(VSCPStatistics));
  memset(&pctx->status, 0, sizeof(VSCPStatus));
}


/* Clock */
void
set_clock_khz(void)
{
  // set a system clock frequency in khz
  set_sys_clock_khz(PLL_SYS_KHZ, true);

  // configure the specified clock
  clock_configure(
    clk_peri,
    0,                                                // No glitchless mux
    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
    PLL_SYS_KHZ * 1000,                               // Input frequency
    PLL_SYS_KHZ * 1000                                // Output (must be same as no divider)
  );
}

///////////////////////////////////////////////////////////////////////////////
// writeSocket
//

int
writeSocket(uint8_t sn, uint8_t* buf, uint16_t size)
{
  int32_t ret;
  uint16_t sentsize = 0;

  while (size != sentsize) {
    ret = send(sn, buf + sentsize, size - sentsize);
    if (ret < 0) {
      close(sn);
      return ret;
    }
    // Don't care SOCKERR_BUSY, because it is zero.
    sentsize += ret;
  }

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_writeClient
//

int
vscp_link_callback_writeClient(const void* pdata, const char* msg)
{
  writeSocket(0, (uint8_t*)msg, strlen(msg));
}

///////////////////////////////////////////////////////////////////////////////
// vscp_handleSocketEvents
//

int32_t
vscp_handleSocketEvents(struct _ctx* pctx, uint16_t port)
{
  int32_t rv;
  uint16_t size = 0;
  uint8_t buf[DATA_BUF_SIZE];

#ifdef _DEMO_DEBUG_
  uint8_t destip[4];
  uint16_t destport;
#endif

  switch (getSn_SR(pctx->sn)) {

    // ------------------------------------------------------------------------
    // Socket is opened
    // ------------------------------------------------------------------------
    case SOCK_ESTABLISHED:

      if (getSn_IR(pctx->sn) & Sn_IR_CON) {
#ifdef _DEMO_DEBUG_
        getSn_DIPR(pctx->sn, destip);
        destport = getSn_DPORT(pctx->sn);
        printf("%d:Connected - %d.%d.%d.%d : %d\r\n", pctx->sn, destip[0], destip[1], destip[2], destip[3], destport);
#endif
        setSn_IR(pctx->sn, Sn_IR_CON);

        // Tell link stack we are connected
        vscp_link_connect(pctx);
      }

      // Don't need to check SOCKERR_BUSY because it does not occur.
      if ((size = getSn_RX_RSR(pctx->sn)) > 0) {

        if (size > DATA_BUF_SIZE) {
          size = DATA_BUF_SIZE;
        }
        memset(buf, 0, DATA_BUF_SIZE);
        rv = recv(pctx->sn, buf, size);

        // check SOCKERR_BUSY & SOCKERR_XXX.
        // For showing the occurrence of SOCKERR_BUSY.
        if (rv <= 0) {
          return rv;
        }

        size = (uint16_t)rv;

        // Check that the buffer can hold the new data
        if (pctx->size + size > sizeof(pctx->buf)) {
          size = sizeof(pctx->buf) - pctx->size;
        }
        strncat(pctx->buf, buf, size);
#ifdef _DEMO_DEBUG_
        printf("buf = %s\r\n", pctx->buf);
#endif
        // sentsize = 0;

        // while (size != sentsize) {
        //   ret = send(sn, buf + sentsize, size - sentsize);
        //   if (ret < 0) {
        // 	  close(sn);
        // 	  return ret;
        //   }
        //   // Don't care SOCKERR_BUSY, because it is zero.
        //   sentsize += ret;
        // }
        // strcat(buf, " - Nice to meet you!\r\n");
        // size = strlen(buf);
        // writeSocket(pctx->sn, buf, size);
      }
      break;

    // ------------------------------------------------------------------------
    // Socket is closing/disconnecting
    // ------------------------------------------------------------------------
    case SOCK_CLOSE_WAIT:
#ifdef _DEMO_DEBUG_
      printf("%d:CloseWait\r\n", pctx->sn);
#endif
      if ((rv = disconnect(pctx->sn)) != SOCK_OK) {
        return rv;
      }
#ifdef _DEMO_DEBUG_
      printf("%d:Socket Closed\r\n", pctx->sn);
#endif
      // Tell link stack we are disconnected
      vscp_link_disconnect(pctx);
      break;

    // ------------------------------------------------------------------------
    // Socket is listening for connections
    // ------------------------------------------------------------------------
    case SOCK_INIT:
#ifdef _DEMO_DEBUG_
      printf("%d:Listen, VSCP TCP link protocol server, port [%d]\r\n", pctx->sn, port);
#endif
      if ((rv = listen(pctx->sn)) != SOCK_OK) {
        return rv;
      }
      break;

    // ------------------------------------------------------------------------
    // Socket is accepted
    // ------------------------------------------------------------------------
    case SOCK_CLOSED:
#ifdef _DEMO_DEBUG_
      printf("%d:VSCP TCP link protocol server, start\r\n", pctx->sn);
#endif
      if ((rv = socket(pctx->sn, Sn_MR_TCP, port, 0x00)) != pctx->sn) {
        return rv;
      }
#ifdef _DEMO_DEBUG_
      printf("%d:Socket opened\r\n", pctx->sn);
#endif
      break;

    default:
      break;
  }

  return 1;
}

///////////////////////////////////////////////////////////////////////////////
// init_persistent_storage
//

void
init_persistent_storage(void)
{
  eeprom_write(&eeprom, REG_DEVICE_ZONE, 11);         // Default Zone = 11
  eeprom_write(&eeprom, REG_DEVICE_SUBZONE, 22);      // Default subzone = 22

  eeprom_write(&eeprom, REG_LED_CTRL, 0x81);          // LED enable. Blink enable.
  eeprom_write(&eeprom, REG_LED_BLINK_INTERVAL, 5);   // 500 ms ON, 500 ms OFF.

  eeprom_write(&eeprom, REG_IO_CTRL1, 0xff);          // All I/O lines are inputs.
  eeprom_write(&eeprom, REG_IO_CTRL2, 0x00);          // All I/O line alarms are off.

  eeprom_write(&eeprom, REG_TEMP_CTRL, 0x81);         // Enabled. Temp. in degrees C.
  eeprom_write(&eeprom, REG_TEMP_CORR_MSB, 0);        // No correction.
  eeprom_write(&eeprom, REG_TEMP_CORR_LSB, 0);        // No correction.
  eeprom_write(&eeprom, REG_TEMP_INTERVAL, 30);       // Report temperature half minute

  eeprom_write(&eeprom, REG_ADC0_CTRL, 0);            // No setting for ADC0.
  eeprom_write(&eeprom, REG_ADC1_CTRL, 0);            // No setting for ADC1.
  eeprom_write(&eeprom, REG_ADC2_CTRL, 0);            // No setting for ADC2.
}










/* References for this implementation:
 * raspberry-pi-pico-c-sdk.pdf, Section '4.1.1. hardware_adc'
 * pico-examples/adc/adc_console/adc_console.c */
float read_onboard_temperature(const char unit) 
{
    
    /* 12-bit conversion, assume max value == ADC_VREF == 3.3 V */
    const float conversionFactor = 3.3f / (1 << 12);

    float adc = (float)adc_read() * conversionFactor;
    float tempC = 27.0f - (adc - 0.706f) / 0.001721f;

    if (unit == 'C') {
        return tempC;
    } else if (unit == 'F') {
        return tempC * 9 / 5 + 32;
    }

    return -1.0f;
}

