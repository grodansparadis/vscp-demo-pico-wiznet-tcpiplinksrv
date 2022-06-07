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

#include <hardware/structs/timer.h>

#include <hardware/gpio.h>
#include <hardware/adc.h>
#include <hardware/watchdog.h>
#include <hardware/sync.h>
#include <hardware/flash.h>
#include <hardware/rtc.h>

#include <pico/binary_info.h>
#include <pico/multicore.h>
#include <pico/mutex.h>
#include <pico/stdlib.h>

#include "pico/util/datetime.h"

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
wiz_NetInfo net_info = {
  .mac  = { 0x00, 0x08, 0xDC, 0x12, 0x34, 0x56 }, // MAC address (Also part of GUID)
  .ip   = { 192, 168, 1, 189 },                   // IP address
  .sn   = { 255, 255, 255, 0 },                   // Subnet Mask
  .gw   = { 192, 168, 1, 1 },                     // Gateway
  .dns  = { 8, 8, 8, 8 },                         // DNS server (Default Google public DNS 8.8.8.8)
  .dhcp = NETINFO_STATIC                          // DHCP enable/disable
};

#ifdef ENABLE_NTP

// Socket used for SNTP 
#define SOCKET_SNTP             3

static uint8_t ntp_buf[ETHERNET_BUF_MAX_SIZE] = {
    0,
};
static uint8_t sntp_server_ip[4] = {216, 239, 35, 0}; // time.google.com

#endif

// Flash memory simulated EEPROM
struct _eeprom_ eeprom;

// Milliseconds timer 
static volatile uint32_t milliseconds = 0;

// ADC 12-bit conversion, assume max value == ADC_VREF == 3.3 V 
static const float conversionFactor = 3.3f / (1 << 12);

/**
  GUID for device
  This is the GUID that is used to identify the device.
  Can use Ethernet MAC address as base.

  If set to writeable the GUID must be stored in eeprom or other
  persistent storage.
*/

#ifdef THIS_FIRMWARE_ENABLE_WRITE_2PROTECTED_LOCATIONS
uint8_t device_guid[16];
#else
uint8_t device_guid[16] = THIS_FIRMWARE_GUID;
#endif

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

  bi_decl(bi_program_description("A demo binary for the VSCP tcp/ip link protocol on pico with wiznet w5100s."));
  bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));

  set_clock_khz();
  stdio_init_all();

  if (watchdog_caused_reboot()) {
    printf("Rebooted by Watchdog!\n");
    //return 0;
  }
  else {
    printf("Clean boot\n");
  }

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

/*
  GUID is set from EEPROM if changeable
*/
#ifdef THIS_FIRMWARE_ENABLE_WRITE_2PROTECTED_LOCATIONS
  for (int i=0; i<16; i++) {
    device_guid[i] = eeprom_read(&eeprom, STDREG_GUID0 + i);
  }
#endif

  mutex_init(&_idleMutex);

  /** 
   * Enable the watchdog, requiring the watchdog to be updated every 2000ms or
   * the chip will reboot second arg is pause on debug which means the watchdog
   * will pause when stepping through code
   */ 
  //watchdog_enable(8000, 1);

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  // Enable GPIO2 - GPIO9
  gpio_init_mask(0x3FC);

  adc_init();
  adc_set_temp_sensor_enabled(true);  

  // Use Ethernet mac address as GUID 
  memcpy(device_guid + 8, net_info.mac, 6);

  for (int i = 0; i < MAX_CONNECTIONS; i++) {
    ctx[i].sn          = i;
    vscp_fifo_init(&ctx[i].fifoEventsOut, TRANSMIT_FIFO_SIZE);
    setContextDefaults(&ctx[i]);
  }

  // Initialize the input fifo
  vscp_fifo_init(&fifoEventsIn, RECEIVE_FIFO_SIZE);    

#ifdef _DEMO_DEBUG_
  /* Demo to allow serial software to open port */
  sleep_ms(2000);
#endif  

  // Start execution of code on core 1
  //multicore_launch_core1(core1_entry);

  printf("\n\n\n\n\n\n_________________________________________________\n");

  float temp = read_onboard_temperature();
  printf("temp = %.02f\n", temp);  

  // Initialize TCP/IP stack/chip 

  wizchip_spi_initialize();
  wizchip_cris_initialize();

  wizchip_reset();
  wizchip_initialize();
  wizchip_check();

  wizchip_1ms_timer_initialize(repeating_timer_callback);

  network_initialize(net_info);

#ifdef ENABLE_NTP
  // Initialize ntp functionality
  SNTP_init(SOCKET_SNTP, sntp_server_ip, 0, ntp_buf);
#endif  

  // Get network information 
  print_network_information(net_info);

  gpio_put(LED_PIN, 0);

#ifdef ENABLE_NTP
  getNtpTime();
#endif  

  ///////////////////////////////////////////////////////////////////////////////
  //                                Main loop
  ///////////////////////////////////////////////////////////////////////////////

  // Timing holders (ms for last turn over)
  time_t time_seconds = getMilliSeconds();
  time_t time_minute = getMilliSeconds();
  time_t time_hour = getMilliSeconds();

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

      // Parse VSCP command
      vscp_link_parser(&ctx[i], ctx[i].buf, &ctx[i].size);

      // Get event from input fifo
      vscpEvent *pev = NULL;
      vscp_fifo_read(&fifoEventsIn, &pev);

      // pev is NULL if no event is available here
      // The worker is still called.
      // if pev != NULL the worker is responsible for 
      // freeing the event

      // Do protocol work here
      vscp2_do_work(pev);

      // Handle rcvloop etc
      vscp_link_idle_worker(&ctx[i]);
    }

    gpio_put(LED_PIN, 1);

    // One second work
    if ((getMilliSeconds()-time_seconds)  > 1000) {

      datetime_t t;
      char datetime_buf[256];
      char *p = &datetime_buf[0];

      rtc_get_datetime(&t);
      datetime_to_str(p, sizeof(datetime_buf), &t);
      printf("\r%s      ", p);
      time_seconds = getMilliSeconds();
    }

    // One minute work
    if ((getMilliSeconds() - time_minute) > 60000) {
      printf("One minute\n");
      time_minute = getMilliSeconds();

      vscp2_send_heartbeat();
      vscp2_send_caps();
    }

    // One hour work
    if ((getMilliSeconds() - time_hour) > 3600000) {
      printf("One hour\n");
      time_hour = getMilliSeconds();
    }

    // gpio_put(LED_PIN, 0);
    // sleep_ms(250);
    // gpio_put(LED_PIN, 1);
    // puts("Hello World\n");
    // sleep_ms(1000);
  }
}


///////////////////////////////////////////////////////////////////////////////
// getMilliSeconds
//

time_t getMilliSeconds(void)
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
  pctx->bValidated        = 0;
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

#ifdef THIS_FIRMWARE_ENABLE_WRITE_2PROTECTED_LOCATIONS

  eeprom_write(&eeprom, STDREG_MANUFACTURER_ID0, 0);  // Manufacturer id 0.
  eeprom_write(&eeprom, STDREG_MANUFACTURER_ID1, 1);  // Manufacturer id 1.
  eeprom_write(&eeprom, STDREG_MANUFACTURER_ID2, 2);  // Manufacturer id 2.
  eeprom_write(&eeprom, STDREG_MANUFACTURER_ID3, 3);  // Manufacturer id 3.

  eeprom_write(&eeprom, STDREG_MANUFACTURER_SUBID0, 0);  // Manufacturer sub id = 0.
  eeprom_write(&eeprom, STDREG_MANUFACTURER_SUBID1, 0);  // Manufacturer sub id = 0.
  eeprom_write(&eeprom, STDREG_MANUFACTURER_SUBID2, 0);  // Manufacturer sub id = 0.
  eeprom_write(&eeprom, STDREG_MANUFACTURER_SUBID3, 0);  // Manufacturer sub id = 0.

  uint8_t guid[16] = THIS_FIRMWARE_GUID;
  for (uint8_t i = 0; i < 16; i++) {
    eeprom_write(&eeprom, STDREG_GUID0 + i, guid[i]);
  }
  
#endif  
}

///////////////////////////////////////////////////////////////////////////////
// getNtpTime
//

#ifdef ENABLE_NTP

void
getNtpTime(void)
{
  int rv;
  uint32_t start_ms = 0;
  datetime time;


  start_ms = getMilliSeconds();

  // Get time
  do {
    rv = SNTP_run(&time);

    if (rv == 1) {
      break;
    }
  } while ((getMilliSeconds() - start_ms) < RECV_TIMEOUT);

  if (rv != 1) {
#ifdef _DEMO_DEBUG_    
    printf(" SNTP failed : %d\n", rv);
#endif    
    // Nill yo get defaults on receiving side
    memset(&time, 0, sizeof(time));
  }

  datetime_t t;
  t.year = time.yy;
  t.month = time.mo;
  t.day = time.dd;
  t.hour = time.hh;
  t.min = time.mm;
  t.sec = time.ss;
  //t.dotw = (time.dd + 4) % 7;
  //t.dotw = time.dotw;

  long day = EPOCH / 86400;
  t.dotw = (day+5) % 7;

  rtc_init();
  rtc_set_datetime(&t);

#ifdef _DEMO_DEBUG_
  printf("* * * NTP: %d-%02d-%02d, %02d:%02d:%02d\n", time.yy, time.mo, time.dd, time.hh, time.mm, time.ss);
#endif 
}

#endif // ENABLE_NTP

/*
 * References for this implementation:
 * raspberry-pi-pico-c-sdk.pdf, Section '4.1.1. hardware_adc'
 * pico-examples/adc/adc_console/adc_console.c
 */
float
read_onboard_temperature(void)
{
  adc_select_input(4);  // Select ADC input 4 for temperature sensor

  float adc   = (float)adc_read() * conversionFactor;
  float temp = 27.0f - (adc - 0.706f) / 0.001721f;

  return temp;
}

/*!
  * @brief Read ADC value
  * @param[in] channel ADC channel to read (0,1,2)
  * @return ADC voltage.
  */

float 
read_adc(uint8_t channel)
{
  adc_select_input(channel&0x03);  // Select ADC input
  float adc   = (float)adc_read() * conversionFactor;
  return adc;
}