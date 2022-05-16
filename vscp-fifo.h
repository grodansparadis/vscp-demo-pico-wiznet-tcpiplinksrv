/*!
  @brief Header for fifo implementation for VSCP events
  @file vscp_fifo.h

  Thread safety (tread/write threads) is achieved by having an extra pos and use lookahead method
  https://embeddedartistry.com/blog/2017/05/17/creating-a-circular-buffer-in-c-and-c/
*/

/*! 
  Define to prevent recursive inclusion -------------------------------------
*/
#ifndef __VSCP_FIFO_H
#define __VSCP_FIFO_H

#ifdef __cplusplus
 extern "C" {
#endif 

/*! 
  Includes ------------------------------------------------------------------
*/
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <vscp.h>

/*!
  Fifo structure 
*/
typedef struct {
	vscpEvent **buf;
	size_t head;
	size_t tail;
	size_t size;
} fifo_t;


/*! 
  Prototypes 
*/

/*!
  @brief Initialize the fifo
  @param f Pointer to fifo structure
  @param size Size of fifo    
*/
void fifo_init(fifo_t *f, uint16_t size);

/*!
  @brief Clear the fifo
  @param f Pointer to fifo structure
*/
void fifo_clear(fifo_t *f);

/*!
  @brief Read a VSCP event from the fifo
  @param f Pointer to fifo structure
  @param pev Pointer to pointer to event structure
  @return Non zero if successful
*/
size_t fifo_read(fifo_t *f, vscpEvent **pev);

/*!
  @brief Put a VSCP event in the fifo
  @param f Pointer to fifo structure
  @param pev Pointer to VSCP event
  @return Non zero if successful
*/
size_t fifo_write(fifo_t *f, vscpEvent *pev);

/*!
  @brief Get the number of events in the fifo
  @param f Pointer to fifo structure
  @return Number of events in the fifo
*/
size_t fifo_getFree(fifo_t *f);

#ifdef __cplusplus
}
#endif

#endif /* __VSCP_FIFO_H */
