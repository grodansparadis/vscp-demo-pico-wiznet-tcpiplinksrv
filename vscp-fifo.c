/*!
  @brief fifo implementation
  @file vscp_fifo.c
*/

#include "vscp-fifo.h"

///////////////////////////////////////////////////////////////////////////////
// fifo_init
//
// This initializes the FIFO structure with the given buffer and size
//

void
fifo_init(fifo_t* f, uint16_t size)
{
  f->head = 0;
  f->tail = 0;
  f->size = size + 1; // One pos used for full signaling
  f->buf  = malloc(f->size * sizeof(struct VscpEvent*));
  memset(f->buf, 0, f->size * sizeof(struct VscpEvent*));
}

///////////////////////////////////////////////////////////////////////////////
// fifo_clear
//

void
fifo_clear(fifo_t* f)
{
  f->head = 0;
  f->tail = 0;
  for (int i = 0; i < f->size; i++) {
    if (NULL != f->buf[i]) {
      vscpEvent* pev = f->buf[i];
      if (NULL == pev->pdata) {
        free(pev->pdata);
        pev->pdata = NULL;
      }
      f->buf[i] = NULL;
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
// fifo_read
//
// This reads one event from the FIFO
//

size_t
fifo_read(fifo_t* f, vscpEvent** pev)
{
  if (f->tail != f->head) {            // see if any data is available
    *pev            = f->buf[f->tail]; // grab the event from the fifo
    f->buf[f->tail] = NULL;            // clear the pointer
    f->tail++;                         // increment the tail
    if (f->tail == f->size) {          // check for wrap-around
      f->tail = 0;
    }
  }
  else {
    return 0; // number of events read
  }

  return 1; // number of events read
}

///////////////////////////////////////////////////////////////////////////////
// fifo_write
//
// This writes one event to the FIFO
// The number of bytes written is returned
//

size_t
fifo_write(fifo_t* f, vscpEvent* pev)
{
  uint32_t i;

  // first check to see if there is space in the fifo
  if (((f->head + 1) % f->size) == f->tail) {
    return 0; // no space levt in the fifo
  }
  else {
    f->buf[f->head] = pev;    // write the event to the fifo
    f->head++;                // increment the head
    if (f->head == f->size) { // check for wrap-around
      f->head = 0;
    }
  }

  return 1;
}

///////////////////////////////////////////////////////////////////////////////
// fifo_getFree
//
// Get the size of the FIFO
//
// The number of bytes that can be written to the fifo is returned
//

size_t
fifo_getFree(fifo_t* f)
{
  if (f->head < f->tail) {
    return (f->tail - f->head - 1);
  }
  else {
    return (f->size + (f->tail - f->head) - 1);
  }
}

// ------------------------------------------------------------------------------
