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

#include <vscp.h>
#include <vscp-fifo.h>
#include <vscp-firmware-helper.h>
#include <vscp-link-protocol.h>
#include <vscp-firmware-level2.h>

#include "pico-eeprom.h"

#include "vscp-compiler.h"
#include "vscp-projdefs.h"

#include "demo.h"

/**
 * ----------------------------------------------------------------------------------------------------
 *                                        Macros & Defines
 * ----------------------------------------------------------------------------------------------------
 */

/* VSCP TCP link protocol server test debug message printout enable */
#define _DEMO_DEBUG_

/* Max number of sockets (WS5100S == 4) */
#define MAX_CONNECTIONS 2

/* Status LED */
const uint LED_PIN = 25;

/* Clock */
#define PLL_SYS_KHZ (133 * 1000)

/* Buffer */
#define ETHERNET_BUF_MAX_SIZE (1024 * 2)

/* Socket */
#define SOCKET_VSCP_LINK_PROTOCOL1    0
#define SOCKET_VSCP_LINK_PROTOCOL2    1

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
                         "Copyright (C) 2000-2022 Grodans Paradis AB\r\n"                  \
                         "https://www.grodansparadis.com\r\n"                              \
                         "+OK\r\n"


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
static wiz_NetInfo g_net_info = {
  .mac  = { 0x00, 0x08, 0xDC, 0x12, 0x34, 0x56 }, // MAC address (Also part of GUID)
  .ip   = { 192, 168, 1, 189 },                   // IP address
  .sn   = { 255, 255, 255, 0 },                   // Subnet Mask
  .gw   = { 192, 168, 1, 1 },                     // Gateway
  .dns  = { 8, 8, 8, 8 },                         // DNS server (Default Google public DNS 8.8.8.8)
  .dhcp = NETINFO_STATIC                          // DHCP enable/disable
};

#ifdef ENABLE_NTP

/* Timezone */
#define TIMEZONE 2 // Sweden Summertime

/* SNTP */
#define SOCKET_SNTP 3

static uint8_t g_sntp_buf[ETHERNET_BUF_MAX_SIZE] = {
    0,
};
static uint8_t g_sntp_server_ip[4] = {216, 239, 35, 0}; // time.google.com

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

/* 
  Socket context 
  This is the context for each open socket/channel.
*/
struct _ctx {
  uint8_t sn;                                   // Socket
  uint16_t size;                                // Number of characters in buffer
  uint8_t buf[ETHERNET_BUF_MAX_SIZE];           // Command Buffer
  uint8_t user[VSCP_LINK_MAX_USER_NAME_LENGTH]; // Username storage
  vscp_fifo_t fifoEventsOut;                    // VSCP event send fifo
  bool bValidated;                              // User is validated
  uint8_t privLevel;                            // User privilege level 0-15
  int bRcvLoop;                                 // Receive loop is enabled if non zero
  vscpEventFilter filter;                       // Filter for events
  VSCPStatistics statistics;                    // VSCP Statistics
  VSCPStatus status;                            // VSCP status
  uint32_t last_rcvloop_time;                   // Time of last received event
};

struct _ctx ctx[MAX_CONNECTIONS]; // Socket context

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/* Clock */
static void
set_clock_khz(void);

// Write data to a connected socket
int32_t
writeSocket(uint8_t sn, uint8_t* buf, uint16_t size);

/* VSCP Links Server */
int32_t
vscp_handleSocketEvents(struct _ctx* pctx, uint16_t port);

/* -------------------------------------------------------------------------- */

/**
 * @brief Set defaults for context structure
 */

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

///////////////////////////////////////////////////////////////////////////////
// repeating_timer_callback
//

static void repeating_timer_callback(void)
{
  milliseconds++;
}

///////////////////////////////////////////////////////////////////////////////
// getMilliseconds
//

time_t getMilliseconds(void)
{
  return milliseconds;
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
  memcpy(device_guid + 8, g_net_info.mac, 6);

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
  if (0 != eeprom_init(&eeprom, FLASH_PAGE_SIZE, (uint8_t *)FLASH_EEPROM_SIM_START)) {
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

  network_initialize(g_net_info);

#ifdef ENABLE_NTP
  // Initialize ntp functionality
  SNTP_init(SOCKET_SNTP, g_sntp_server_ip, TIMEZONE, g_sntp_buf);
#endif  

  /* Get network information */
  print_network_information(g_net_info);

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

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/* Clock */
static void
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

/******************************************************************************
                          Write data to client
********************************************************************************/
int32_t
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
  int32_t ret;
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
        ret = recv(pctx->sn, buf, size);

        // check SOCKERR_BUSY & SOCKERR_XXX.
        // For showing the occurrence of SOCKERR_BUSY.
        if (ret <= 0) {
          return ret;
        }

        size = (uint16_t)ret;

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
      if ((ret = disconnect(pctx->sn)) != SOCK_OK) {
        return ret;
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
      if ((ret = listen(pctx->sn)) != SOCK_OK) {
        return ret;
      }
      break;

    // ------------------------------------------------------------------------
    // Socket is accepted
    // ------------------------------------------------------------------------
    case SOCK_CLOSED:
#ifdef _DEMO_DEBUG_
      printf("%d:VSCP TCP link protocol server, start\r\n", pctx->sn);
#endif
      if ((ret = socket(pctx->sn, Sn_MR_TCP, port, 0x00)) != pctx->sn) {
        return ret;
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
  eeprom_write(&eeprom, REG_DEVICE_ZONE, 11);    // Default Zone = 11
  eeprom_write(&eeprom, REG_DEVICE_SUBZONE, 22); // Default subzone = 22
  eeprom_write(&eeprom, REG_LED_CTRL, 0);        // LED Control register.

  eeprom_write(&eeprom, REG_SERIAL_CTRL, 0); // Serial channel control register.

  eeprom_write(&eeprom, REG_IO_CTRL, 0); // I/O Control register.

  eeprom_write(&eeprom, REG_TEMP_CTRL, 1);     // Temp read in degrees C.
  eeprom_write(&eeprom, REG_TEMP_CORR_MSB, 0); // No correction.
  eeprom_write(&eeprom, REG_TMP_CORR_LSB, 0);  // No correction.
  eeprom_write(&eeprom, REG_TMP_INTERVAL, 60); // Report temperature every minute

  eeprom_write(&eeprom, REG_NTP_TIME_ZONE, 2); // NTP time zone = GMT +2
}

// ****************************************************************************
//                       VSCP Link protocol callbacks
// ****************************************************************************




///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_write_client
//

int
vscp_link_callback_welcome(const void* pdata)
{
  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;

  writeSocket(pctx->sn, DEMO_WELCOME_MSG, strlen(DEMO_WELCOME_MSG));
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_write_client
//

int
vscp_link_callback_write_client(const void* pdata, const char* msg)
{
  if ((NULL == pdata) && (NULL == msg)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;
  writeSocket(pctx->sn, (uint8_t*)msg, strlen(msg));
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_quit
//

int
vscp_link_callback_quit(const void* pdata)
{
  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;

  // Confirm quit
  writeSocket(pctx->sn, VSCP_LINK_MSG_GOODBY, strlen(VSCP_LINK_MSG_GOODBY));

  // Disconnect from client
  disconnect(pctx->sn);

  // Set context defaults
  setContextDefaults(pctx);

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_help
//

int
vscp_link_callback_help(const void* pdata, const char* arg)
{
  if ((NULL == pdata) && (NULL == arg)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;
  writeSocket(pctx->sn, VSCP_LINK_MSG_OK, strlen(VSCP_LINK_MSG_OK));
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_get_interface_count
//

uint16_t
vscp_link_callback_get_interface_count(const void* pdata)
{
  /* Return number of interfaces we support */
  return 1;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_get_interface
//

int
vscp_link_callback_get_interface(const void* pdata, uint16_t index, struct vscp_interface_info *pif)
{
  if ((NULL == pdata) && (NULL == pif)) {
    return VSCP_ERROR_UNKNOWN_ITEM;
  }

  if (index != 0) {
    return VSCP_ERROR_UNKNOWN_ITEM;
  }

  // interface-id-n, type, interface-GUID-n, interface_real-name-n
  // interface types in vscp.h

  pif->idx = index;
  pif->type = VSCP_INTERFACE_TYPE_INTERNAL;
  memcpy(pif->guid, device_guid, 16);
  strncpy(pif->description, "Interface for the device itself", sizeof(pif->description));

  // We have no interfaces
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_check_user
//

int
vscp_link_callback_check_user(const void* pdata, const char* arg)
{
  if ((NULL == pdata) && (NULL == arg)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // trim
  const char* p = arg;
  while (*p && isspace(*p)) {
    p++;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;
  strncpy(pctx->user, p, VSCP_LINK_MAX_USER_NAME_LENGTH);
  writeSocket(pctx->sn, VSCP_LINK_MSG_USENAME_OK, strlen(VSCP_LINK_MSG_USENAME_OK));
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_check_password
//

int
vscp_link_callback_check_password(const void* pdata, const char* arg)
{
  if ((NULL == pdata) && (NULL == arg)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;

  // Must have a username before a password
  if (*(pctx->user) == '\0') {
    writeSocket(pctx->sn, VSCP_LINK_MSG_NEED_USERNAME, strlen(VSCP_LINK_MSG_NEED_USERNAME));
    return VSCP_ERROR_SUCCESS;
  }

  const char* p = arg;
  while (*p && isspace(*p)) {
    p++;
  }

  // if (!pctx->bValidated) {

  // }
  if (0 == strcmp(pctx->user, "admin") && 0 == strcmp(p, "secret")) {
    pctx->bValidated = true;
    pctx->privLevel = 15;
  }
  else {
    pctx->user[0]    = '\0';
    pctx->bValidated = false;
    pctx->privLevel = 0;
    writeSocket(pctx->sn, VSCP_LINK_MSG_PASSWORD_ERROR, strlen(VSCP_LINK_MSG_PASSWORD_ERROR));
    return VSCP_ERROR_SUCCESS;
  }

  writeSocket(pctx->sn, VSCP_LINK_MSG_PASSWORD_OK, strlen(VSCP_LINK_MSG_PASSWORD_OK));
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_challenge
//

int
vscp_link_callback_challenge(const void* pdata, const char* arg)
{
  uint8_t buf[80];
  uint8_t random_data[32];
  if ((NULL == pdata) && (NULL == arg)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;

  const char* p = arg;
  while (*p && isspace(*p)) {
    p++;
  }

  strcpy(buf, "+OK - ");
  p = buf + strlen(buf);

  for (int i = 0; i < 32; i++) {
    random_data[i] = rand() >> 16;
    if (i < sizeof(p)) {
      random_data[i] += (uint8_t)p[i];
    }
    vscp_fwhlp_dec2hex(random_data[i], (char*)p, 2);
    p++;
  }

  strcat(buf, "\r\n");
  writeSocket(pctx->sn, buf, strlen(buf));
  return VSCP_ERROR_SUCCESS;
}

/*!
*/
int
vscp_link_callback_check_authenticated(const void* pdata)
{
  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;

  if (pctx->bValidated) {
    return VSCP_ERROR_SUCCESS;
  }

  return VSCP_ERROR_INVALID_PERMISSION;
}

/*!
*/
int
vscp_link_callback_check_privilege(const void* pdata, uint8_t priv)
{
  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;

  if (pctx->privLevel >= priv) {
    return VSCP_ERROR_SUCCESS;
  }

  return VSCP_ERROR_INVALID_PERMISSION;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_challenge
//

int
vscp_link_callback_test(const void* pdata, const char* arg)
{
  if ((NULL == pdata) && (NULL == arg)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;

  writeSocket(pctx->sn, VSCP_LINK_MSG_OK, strlen(VSCP_LINK_MSG_OK));
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_send
//

int
vscp_link_callback_send(const void* pdata, vscpEvent* pev)
{
  if ((NULL == pdata) && (NULL == pev)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;

  // Filter
  if (!vscp_fwhlp_doLevel2Filter(pev, &pctx->filter)) {
    return VSCP_ERROR_SUCCESS;  // Filter out == OK
  }

  // Update send statistics
  pctx->statistics.cntTransmitFrames++;
  pctx->statistics.cntTransmitData += pev->sizeData;

  // Write event to receive fifo
  pev->obid = pctx->sn;
  if (!vscp_fifo_write(&fifoEventsIn, pev)) {
    pctx->statistics.cntOverruns++;
    return VSCP_ERROR_TRM_FULL;
  }

  // Write to send buffer of other interfaces
  for (int i = 0; i < MAX_CONNECTIONS; i++) {
    if (pctx->sn != i) {
      if (!vscp_fifo_write(&ctx[i].fifoEventsOut, pev)) {
        ctx[i].statistics.cntOverruns++;
        return VSCP_ERROR_TRM_FULL;
      }  
    }  
  }

  // We own the event from now on and must
  // delete it and it's data when we are done
  // with it

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_retr
//

int
vscp_link_callback_retr(const void* pdata, vscpEvent** pev)
{
  if ((NULL == pdata) && (NULL == pev)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;

  if (!vscp_fifo_read(&pctx->fifoEventsOut, pev)) {
    return VSCP_ERROR_RCV_EMPTY;
  }

  // Update receive statistics
  pctx->statistics.cntReceiveFrames++;
  pctx->statistics.cntReceiveData += (*pev)->sizeData;

  return VSCP_ERROR_SUCCESS;
}

/*!
  \fn vscp_link_callback_send_vscp_class
  \brief Enable/disable rcvloop functionality.
*/
int
vscp_link_callback_enable_rcvloop(const void* pdata, int bEnable)
{
  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;

  pctx->bRcvLoop = bEnable;
  pctx->last_rcvloop_time = time_us_32();

  return VSCP_ERROR_SUCCESS;
}

/*!
  \fn vscp_link_callback_get_rcvloop_status
  \brief Get rcvloop status
*/
int
vscp_link_callback_get_rcvloop_status(const void* pdata)
{
  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;

  return pctx->bRcvLoop; 
}

/*!
  \fn vscp_link_callback_get_rcvloop_status
  \brief Get rcvloop status
*/

int
vscp_link_callback_chkData(const void* pdata, uint16_t* pcount)
{
  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;
  *pcount = TRANSMIT_FIFO_SIZE - vscp_fifo_getFree(&pctx->fifoEventsOut);
  
  return VSCP_ERROR_SUCCESS; 
}

/**
 * 
 * \fn vscp_link_callback_clrAll
 * \brief Clear transmit queue
 */

int
vscp_link_callback_clrAll(const void* pdata)
{
  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;
  vscp_fifo_clear(&pctx->fifoEventsOut);

  return VSCP_ERROR_SUCCESS;
}

/*!
  \fn vscp_link_callback_get_channel_id
  \brief Get channel id
*/

int
vscp_link_callback_get_channel_id(const void* pdata, uint16_t *pchid)
{
  if ((NULL == pdata) && (NULL == pchid)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;
  *pchid = pctx->sn;
  
  return VSCP_ERROR_SUCCESS; 
}

/*!
  \fn vscp_link_callback_get_guid
  \brief Get channel id
*/

int
vscp_link_callback_get_guid(const void* pdata, uint8_t *pguid)
{
  if ((NULL == pdata) || (NULL == pguid)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  memcpy(pguid, device_guid, 16);
  return VSCP_ERROR_SUCCESS; 
}

/*!
  \fn vscp_link_callback_set_guid
  \brief Get channel id
*/

int
vscp_link_callback_set_guid(const void* pdata, uint8_t *pguid)
{
  if ((NULL == pdata) || (NULL == pguid)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  memcpy(device_guid, pguid, 16);
  return VSCP_ERROR_SUCCESS; 
}

/*!
  \fn vscp_link_callback_get_version
  \brief Get device version
*/

int
vscp_link_callback_get_version(const void* pdata, uint8_t *pversion)
{
  if ((NULL == pdata) || (NULL == pversion)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  pversion[0] = THIS_FIRMWARE_MAJOR_VERSION;
  pversion[1] = THIS_FIRMWARE_MINOR_VERSION;
  pversion[2] = THIS_FIRMWARE_RELEASE_VERSION;
  pversion[3] = THIS_FIRMWARE_BUILD_VERSION;
  
  return VSCP_ERROR_SUCCESS; 
}


/*!
  \fn vscp_link_callback_setFilter
  \brief Get device version
*/

int
vscp_link_callback_setFilter(const void* pdata, vscpEventFilter *pfilter)
{
  if ((NULL == pdata) || (NULL == pfilter)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;
  pctx->filter.filter_class = pfilter->filter_class;
  pctx->filter.filter_type = pfilter->filter_type;
  pctx->filter.filter_priority = pfilter->filter_priority;
  memcpy(pctx->filter.filter_GUID, pfilter->filter_GUID, 16);

  return VSCP_ERROR_SUCCESS; 
}

/*!
  \fn vscp_link_callback_setFilter
  \brief Get device version
*/

int
vscp_link_callback_setMask(const void* pdata, vscpEventFilter *pfilter)
{
  if ((NULL == pdata) || (NULL == pfilter)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;
  pctx->filter.mask_class = pfilter->mask_class;
  pctx->filter.mask_type = pfilter->mask_type;
  pctx->filter.mask_priority = pfilter->mask_priority;
  memcpy(pctx->filter.mask_GUID, pfilter->mask_GUID, 16);

  return VSCP_ERROR_SUCCESS; 
}

/*!
  \fn vscp_link_callback_statistics
  \brief Get statistical information
*/

int
vscp_link_callback_statistics(const void* pdata, VSCPStatistics *pStatistics)
{
  if ((NULL == pdata) || (NULL == pStatistics)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;
  memcpy(pStatistics, &pctx->statistics, sizeof(VSCPStatistics));

  return VSCP_ERROR_SUCCESS;
}

/*!
  \fn vscp_link_callback_info
  \brief Get status info
*/

int
vscp_link_callback_info(const void* pdata, VSCPStatus *pstatus)
{
  if ((NULL == pdata) || (NULL == pstatus)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;
  memcpy(pstatus, &pctx->status, sizeof(VSCPStatus));  

  return VSCP_ERROR_SUCCESS;
}

/**
 * @brief Called when a channel has a rcvloop activated
 * @param pdata 
 * @return VS_SUCCESS on success, error code on failure
 */
int
vscp_link_callback_rcvloop(const void* pdata, vscpEvent *pev)
{
  // Check pointer
  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;

  // Every second output '+OK\r\n' in rcvloop mode
  if ((time_us_32() - pctx->last_rcvloop_time) > 1000000) {
    pctx->last_rcvloop_time = time_us_32();
    return VSCP_ERROR_TIMEOUT;
  }

  if (!vscp_fifo_read(&pctx->fifoEventsOut, &pev)) {
    return VSCP_ERROR_RCV_EMPTY;
  }

  // Update receive statistics
  pctx->statistics.cntReceiveFrames++;
  pctx->statistics.cntReceiveData += pev->sizeData;

  return VSCP_ERROR_SUCCESS;
}

/**
 * @brief Called when wcyd command is received
 * @param pdata 
 * @return VS_SUCCESS on success, error code on failure
 */
int
vscp_link_callback_wcyd(const void* pdata, uint64_t *pwcyd)
{
  // Check pointers
  if ((NULL == pdata) || (NULL == pwcyd)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;

  *pwcyd = VSCP_SERVER_CAPABILITY_TCPIP | 
              VSCP_SERVER_CAPABILITY_DECISION_MATRIX | 
              VSCP_SERVER_CAPABILITY_IP4 | 
              /*VSCP_SERVER_CAPABILITY_SSL |*/
              VSCP_SERVER_CAPABILITY_TWO_CONNECTIONS;

  return VSCP_ERROR_SUCCESS;            
}

/**
 * @brief Shutdown the system to a safe state
 * @param pdata Pointer to context
 * @return VS_SUCCESS on success, error code on failure
 */

int
vscp_link_callback_shutdown(const void* pdata)
{
  // Check pointers
  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;

  // At this point
  // Shutdown the system
  // Set everything in a safe and inactive state

  // Stay here until someone presses the reset button
  // or power cycles the board
  while(1) {
    watchdog_update();
  }

  return VSCP_ERROR_SUCCESS; 
}

/**
 * @brief Restart the system
 * @param pdata Pointer to context
 * @return VS_SUCCESS on success, error code on failure
 */

int
vscp_link_callback_restart(const void* pdata)
{
  // Check pointers
  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;

  while(1); // Restart

  return VSCP_ERROR_SUCCESS; 
}




// ****************************************************************************
//                        VSCP protocol callbacks
// ****************************************************************************


/**
 * @brief Get one event fdrom the input queue
 * @param pdata Pointer to context.
 * @param pev Pointer te event pointer that will get event (if any).
 * @return VS_SUCCESS on success, error code on failure
 */
int
vscp2_callback_get_event(const void* pdata, vscpEvent** pev)
{
  // Check pointers
  if ((NULL == pdata) || (NULL == *pev)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;

  if (!vscp_fifo_read(&fifoEventsIn, pev)) {
    return VSCP_ERROR_RCV_EMPTY;
  }

  return VSCP_ERROR_SUCCESS;
}

int
vscp2_callback_read_reg(const void* pdata, uint32_t reg, uint8_t* pval)
{
  // Check pointers
  if ((NULL == pdata) || (NULL == pval)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  return VSCP_ERROR_SUCCESS;
}

/**
 * @brief Enter bootloader
 * @param pdata Pointer to context.
 * @return VS_SUCCESS on success, error code on failure
 */

int
vscp2_callback_enter_bootloader(const void* pdata)
{
  return VSCP_ERROR_SUCCESS;
}

/**
 * @brief Respons to DM info request
 * @param pdata Pointer to context.
 * @return VS_SUCCESS on success, error code on failure
 */
int
vscp2_callback_report_dmatrix(const void* pdata)
{
  return VSCP_ERROR_SUCCESS;
}

/**
 * @brief Response on embedded MDF request.
 * @param pdata Pointer to context.
 * @return VS_SUCCESS on success, error code on failure
 */
int
vscp2_callback_report_mdf(const void* pdata)
{
  return VSCP_ERROR_SUCCESS;
}

/**
 * @brief Response on event interest request.
 * @param pdata Pointer to context.
 * @return VS_SUCCESS on success, error code on failure
 */

int
vscp2_callback_report_events_of_interest(const void* pdata)
{
  return VSCP_ERROR_SUCCESS;
}

/**
 * @brief Get timestamp in microseconds
 * @param pdata Pointer to context.
 * @return VS_SUCCESS on success, error code on failure
 */
uint32_t
vscp2_callback_get_timestamp(const void* pdata)
{
  return time_us_32();
}

/**
 * @brief  Set VSCP event time
 * @param pdata Pointer to context.
 * @param pev Pointer to event.
 * @return VS_SUCCESS on success, error code on failure
 */

int
vscp2_callback_get_time(const void* pdata, const vscpEvent *pev)
{
  return VSCP_ERROR_SUCCESS;
}

/**
 * @brief Get timestamp in milliseconds
 * @param pdata Pointer to context.
 * @param pev Event to send
 * @return VS_SUCCESS on success, error code on failure
 */

int
vscp2_callback_send_event(const void* pdata, vscpEvent* pev)
{
  return VSCP_ERROR_SUCCESS;
}


int
vscp2_callback_restore_defaults(const void *pdata)
{
  return VSCP_ERROR_SUCCESS;
}

int
vscp2_callback_write_user_id(const void *pdata, uint8_t pos, uint8_t val)
{
  return VSCP_ERROR_SUCCESS;
}

int
vscp2_callback_write_app_reg(const void* pdata, uint32_t reg, uint8_t val)
{
  return VSCP_ERROR_SUCCESS;
}

/* References for this implementation:
 * raspberry-pi-pico-c-sdk.pdf, Section '4.1.1. hardware_adc'
 * pico-examples/adc/adc_console/adc_console.c */
float read_onboard_temperature(const char unit) {
    
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