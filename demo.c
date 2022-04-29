#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"

#include "port_common.h"
#include "wizchip_conf.h"
#include "w5x00_spi.h"

#include "socket.h"
#include "wizchip_conf.h"

#include "fifo.h"

#include <vscp.h>

/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */

// Blinking LED
const uint LED_PIN = 25;

/* Clock */
#define PLL_SYS_KHZ (133 * 1000)

/* Buffer */
#define ETHERNET_BUF_MAX_SIZE (1024 * 2)

/* Socket */
#define SOCKET_VSCP_LINK_PROTOCOL1   0
#define SOCKET_VSCP_LINK_PROTOCOL2   1

/* Port */
#define PORT_VSCP_LINK_PROTOCOL 9598

/* Loopback test debug message printout enable */
#define	_LOOPBACK_DEBUG_

/* DATA_BUF_SIZE define for Loopback example */
#ifndef DATA_BUF_SIZE
	#define DATA_BUF_SIZE			512
#endif

/************************/
/* Select LOOPBACK_MODE */
/************************/
#define LOOPBACK_MAIN_NOBLOCK    0
#define LOOPBACK_MODE   LOOPBACK_MAIN_NOBLOCK

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
/* Network */
static wiz_NetInfo g_net_info =
    {
        .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56}, // MAC address
        .ip = {192, 168, 1, 189},                    // IP address
        .sn = {255, 255, 255, 0},                    // Subnet Mask
        .gw = {192, 168, 1, 1},                      // Gateway
        .dns = {8, 8, 8, 8},                         // DNS server
        .dhcp = NETINFO_STATIC                       // DHCP enable/disable
};

/* Receive buffers */
static uint8_t g_loopback_buf1[ETHERNET_BUF_MAX_SIZE] = {
    0,
};

static uint8_t g_loopback_buf2[ETHERNET_BUF_MAX_SIZE] = {
    0,
};

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/* Clock */
static void set_clock_khz(void);

// Write data to a connected socket
int32_t writeSocket(uint8_t sn, uint8_t* buf, uint16_t size);

/* VSCP Links Server */
int32_t vscp_tcplinksrv(uint8_t sn, uint8_t* buf, uint16_t port);

/* -------------------------------------------------------------------------- */

int main() 
{
  fifo_t *pfifo;  // Input fifo
  uint8_t fbuf[2048];

  bi_decl(bi_program_description("This is a test binary."));
  bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));

  /* Initialize */
  int retval = 0;

  // Init fifo
  fifo_init(pfifo, fbuf, 2048);

  set_clock_khz();

  stdio_init_all();

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  wizchip_spi_initialize();
  wizchip_cris_initialize();

  wizchip_reset();
  wizchip_initialize();
  wizchip_check();

  network_initialize(g_net_info);

  /* Get network information */
  print_network_information(g_net_info);

  gpio_put(LED_PIN, 0);

  while (1) {

    /* VSCP TCP link server handler */
    if ((retval = vscp_tcplinksrv(SOCKET_VSCP_LINK_PROTOCOL1, g_loopback_buf1, PORT_VSCP_LINK_PROTOCOL)) < 0) {
      printf(" Link error : %d\n", retval);
      while (1) {
          ;
      }
    }

    if ((retval = vscp_tcplinksrv(SOCKET_VSCP_LINK_PROTOCOL2, g_loopback_buf2, PORT_VSCP_LINK_PROTOCOL)) < 0) {
      printf(" Link error : %d\n", retval);
      while (1) {
          ;
      }
    }
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
static void set_clock_khz(void)
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
                          Send data to client
********************************************************************************/
int32_t 
writeSocket(uint8_t sn, uint8_t* buf, uint16_t size)
{
  int32_t ret;
  uint16_t sentsize=0;

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

int32_t 
vscp_tcplinksrv(uint8_t sn, uint8_t* buf, uint16_t port)
{
  int32_t ret;
  uint16_t size = 0;

#ifdef _LOOPBACK_DEBUG_
  uint8_t destip[4];
  uint16_t destport;
#endif

  switch (getSn_SR(sn)) {

    // ------------------------------------------------------------------------   
    case SOCK_ESTABLISHED:
      if (getSn_IR(sn) & Sn_IR_CON) {
#ifdef _LOOPBACK_DEBUG_
			  getSn_DIPR(sn, destip);
			  destport = getSn_DPORT(sn);
			  printf("%d:Connected - %d.%d.%d.%d : %d\r\n",sn, destip[0], destip[1], destip[2], destip[3], destport);
#endif
			  setSn_IR(sn,Sn_IR_CON);
      }

      // Don't need to check SOCKERR_BUSY because it does not occur.  
		  if ((size = getSn_RX_RSR(sn)) > 0) {
			  
        if (size > DATA_BUF_SIZE) size = DATA_BUF_SIZE;
        memset(buf, 0, DATA_BUF_SIZE);
			  ret = recv(sn, buf, size);

        // check SOCKERR_BUSY & SOCKERR_XXX. 
        // For showing the occurrence of SOCKERR_BUSY.
			  if (ret <= 0) return ret;      
			  size = (uint16_t)ret;
        //fifo_write(pfifo, buf, size);
			  //sentsize = 0;

			  // while (size != sentsize) {
				//   ret = send(sn, buf + sentsize, size - sentsize);
				//   if (ret < 0) {
				// 	  close(sn);
				// 	  return ret;
				//   }
        //   // Don't care SOCKERR_BUSY, because it is zero.
				//   sentsize += ret; 
			  // }
        strcat(buf, " - Nice to meet you!\r\n");
        size = strlen(buf);
        writeSocket(sn, buf, size);

      }
      break;

    // ------------------------------------------------------------------------
    case SOCK_CLOSE_WAIT:
#ifdef _LOOPBACK_DEBUG_
      printf("%d:CloseWait\r\n",sn);
#endif
      if ((ret = disconnect(sn)) != SOCK_OK) return ret;
#ifdef _LOOPBACK_DEBUG_
      printf("%d:Socket Closed\r\n", sn);
#endif
      break;

    // ------------------------------------------------------------------------
    case SOCK_INIT :
#ifdef _LOOPBACK_DEBUG_
    	printf("%d:Listen, TCP server loopback, port [%d]\r\n", sn, port);
#endif
      if ( (ret = listen(sn)) != SOCK_OK) return ret;
      break;

    // ------------------------------------------------------------------------
    case SOCK_CLOSED:
#ifdef _LOOPBACK_DEBUG_
      printf("%d:TCP server loopback start\r\n",sn);
#endif
      if ((ret = socket(sn, Sn_MR_TCP, port, 0x00)) != sn) return ret;
#ifdef _LOOPBACK_DEBUG_
      printf("%d:Socket opened\r\n",sn);
#endif
      break;

    default:
      break;
   }

   return 1;
}