#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
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

#include <vscp-link-protocol.h>
#include <vscp.h>
#include <vscp-fifo.h>

/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
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

/* Port */
#define PORT_VSCP_LINK_PROTOCOL 9598

/* DATA_BUF_SIZE define for VSCP TCP link protocol server example */
#ifndef DATA_BUF_SIZE
#define DATA_BUF_SIZE 512
#endif

/* Max number of events in the receive fifo */
#define RECEIVE_FIFO_SIZE 16

/* Max number of events in each of the transmit fifos */
#define TRANSMIT_FIFO_SIZE 16

#define DEMO_WELCOME_MSG "Welcome to the wiznet pico demo VSCP TCP link protocol node\r\n" \
                         "Copyright (C) 2000-2022 Grodans Paradis AB\r\n"                  \
                         "https://www.grodansparadis.com\r\n"                              \
                         "+OK\r\n"

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */

/* Network */
static wiz_NetInfo g_net_info = {
  .mac  = { 0x00, 0x08, 0xDC, 0x12, 0x34, 0x56 }, // MAC address
  .ip   = { 192, 168, 1, 189 },                   // IP address
  .sn   = { 255, 255, 255, 0 },                   // Subnet Mask
  .gw   = { 192, 168, 1, 1 },                     // Gateway
  .dns  = { 8, 8, 8, 8 },                         // DNS server
  .dhcp = NETINFO_STATIC                          // DHCP enable/disable
};

/* 
  GUID for device 
  This is the GUID that is used to identify the device.
  Use Ethernet MAC address as base. Can also be set explicitly.
*/
static uint8_t device_guid[16] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                  0x00, 0x00};

// Device version 
// Major, minor, sub-minor, build/patch
static uint8_t device_version[4] = {0,0,1,0};                  

/*!
  @brief Received event are written to this fifo and
  is consumed by the VSCP protocol handler.
*/
vscp_fifo_t fifoEventsRcv;

/* Socket context */
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
};

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

int
main()
{
  struct _ctx ctx[MAX_CONNECTIONS]; // Socket context

  /* Initialize */

  int retval = 0;

  // Use Ethernet mac address as GUID 
  memcpy(device_guid + 8, g_net_info.mac, 6);

  for (int i = 0; i < MAX_CONNECTIONS; i++) {
    ctx[i].bValidated  = false;
    ctx[i].privLevel = 0;
    ctx[i].bRcvLoop    = 0;
    ctx[i].sn          = i;
    ctx[i].size        = 0;
    memset(ctx[i].buf, 0, ETHERNET_BUF_MAX_SIZE);
    memset(ctx[i].user, 0, VSCP_LINK_MAX_USER_NAME_LENGTH);
    vscp_fifo_init(&ctx[i].fifoEventsOut, TRANSMIT_FIFO_SIZE);
    // Filter: All events received
    memset(&ctx[i].filter, 0, sizeof(vscpEventFilter));
    memset(&ctx[i].statistics, 0, sizeof(VSCPStatistics));
    memset(&ctx[i].status, 0, sizeof(VSCPStatus));
  }

  vscp_fifo_init(&fifoEventsRcv, RECEIVE_FIFO_SIZE);

  bi_decl(bi_program_description("This is a demo binary for the VSCP tcp/ip link protocol."));
  bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));

  set_clock_khz();
  stdio_init_all();

  if (watchdog_caused_reboot()) {
    printf("Rebooted by Watchdog!\n");
    return 0;
  }
  else {
    printf("Clean boot\n");
  }

  // Enable the watchdog, requiring the watchdog to be updated every 2000ms or
  // the chip will reboot second arg is pause on debug which means the watchdog
  // will pause when stepping through code
  watchdog_enable(2000, 1);

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  wizchip_spi_initialize();
  wizchip_cris_initialize();

  wizchip_reset();
  wizchip_initialize();
  wizchip_check();

  network_initialize(g_net_info);

#ifdef _DEMO_DEBUG_
  /* Demo to allow serial software to open port */
  sleep_ms(1000);
#endif

  /* Get network information */
  print_network_information(g_net_info);

  gpio_put(LED_PIN, 0);

  while (1) {

    watchdog_update();

    /* VSCP TCP link server handler */
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
      if ((retval = vscp_handleSocketEvents(&ctx[i], PORT_VSCP_LINK_PROTOCOL)) < 0) {
        printf(" Link error : %d\n", retval);
        while (1) {
          ;
        }
      }

      /* If buf contains a carriage return, we have a command to handle */
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

        if (ctx[i].bRcvLoop) {
          // Send event to the client if some in the queue
          writeSocket(ctx[i].sn, (uint8_t*)cmd, strlen(cmd));
        } 
        else {
          vscp_link_parser(&ctx[i], cmd, ctx[i].bRcvLoop);
        }

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
    // Socket is closing/disconneting
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

// ****************************************************************************
//                       VSCP Link protocol callbacks
// ****************************************************************************

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_write_client
//

int
vscp_link_callback_welcome(const void* pdata)
{
  writeSocket(0, DEMO_WELCOME_MSG, strlen(DEMO_WELCOME_MSG));
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
  return 0;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_get_interface
//

int
vscp_link_callback_get_interface(const void* pdata, uint16_t index, const uint8_t* piface)
{
  if ((NULL == pdata) && (NULL == piface)) {
    return VSCP_ERROR_UNKNOWN_ITEM;
  }

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
    vscp_link_dec2hex(random_data[i], (char*)p, 2);
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
  if (vscp_link_doLevel2Filter(pev, &pctx->filter)) {
    return VSCP_ERROR_SUCCESS;  // Filter out == OK
  }

  // Update send statistics
  pctx->statistics.cntTransmitFrames++;
  pctx->statistics.cntTransmitData += pev->sizeData;

  // Write event to receive fifo
  if (!vscp_fifo_write(&fifoEventsRcv, pev)) {
    pctx->statistics.cntOverruns++;
    return VSCP_ERROR_TRM_FULL;
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
vscp_link_callback_retr(const void* pdata, vscpEvent* pev)
{
  if ((NULL == pdata) && (NULL == pev)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;

  if (!vscp_fifo_read(&pctx->fifoEventsOut, &pev)) {
    return VSCP_ERROR_RCV_EMPTY;
  }

  // Update receive statistics
  pctx->statistics.cntReceiveFrames++;
  pctx->statistics.cntReceiveData += pev->sizeData;

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
vscp_link_callback_chkData(const void* pdata)
{
  char buf[10];

  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;
  sprintf(buf, "%zu\r\n", TRANSMIT_FIFO_SIZE - vscp_fifo_getFree(&pctx->fifoEventsOut));
  return VSCP_ERROR_SUCCESS; 
}

/*!
  \fn vscp_link_callback_get_channel_id
  \brief Get channel id
*/

int
vscp_link_callback_get_channel_id(const void* pdata)
{
  char buf[10];

  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;

  sprintf(buf, "%d\r\n", pctx->sn);
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

  memcpy(pversion, device_version, 4);
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
vscp_link_callback_statistics(const void* pdata, const VSCPStatistics *pStatistics)
{
  if ((NULL == pdata) || (NULL == pStatistics)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;
  memcpy(&pctx->statistics, pStatistics, sizeof(VSCPStatistics));

  return VSCP_ERROR_SUCCESS;
}

/*!
  \fn vscp_link_callback_info
  \brief Get status info
*/

int
vscp_link_callback_info(const void* pdata, const VSCPStatus *pstatus)
{
  if ((NULL == pdata) || (NULL == pstatus)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  struct _ctx* pctx = (struct _ctx*)pdata;
  memcpy(&pctx->status, pstatus, sizeof(VSCPStatus));

  return VSCP_ERROR_SUCCESS;
}

/**
 * @brief Called when a channel has a rcvloop activated
 * 
 * @param pdata 
 */
int
vscp_link_callback_rcvloop(const void* pdata)
{
  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  return VSCP_ERROR_SUCCESS;
}