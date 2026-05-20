/*!
  gcc test-fifo.c vscp-fifo.c -Ivscp-firmware/common
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "vscp-firmware/common/vscp.h"
#include "vscp-fifo.h"

void
addItem(vscp_fifo_t* f, int i) 
{
  vscpEvent *pev = malloc(sizeof(vscpEvent));
  memset(pev, 0, sizeof(vscpEvent));
  pev->vscp_class = i;
  pev->vscp_type = i;
  pev->sizeData = 3;
  pev->pdata = malloc(3);
  memset(pev->pdata, 0, 3);
  if (vscp_fifo_write(f, pev)) {
    printf("Written %d\t - ",i);
  }
  else {
    printf("Failed %d\t - ",i);      
  }
  printf("   head=%zu ", f->head);
  printf("   tail=%zu ", f->tail);
  printf("   free=%zu\n", vscp_fifo_getFree(f));
}

void
readItem(vscp_fifo_t* f) 
{
  vscpEvent *pev;
  if (vscp_fifo_read(f, &pev)) {
    printf("Read \t - class = %d ", pev->vscp_class);
    printf("   head=%zu ", f->head);
    printf("   tail=%zu ", f->tail);
    printf("   size=%zu ", f->size);
    printf("   free=%zu\n",vscp_fifo_getFree(f));
    free(pev->pdata);
    pev->pdata = NULL;
    free(pev);
  }
  else {
    printf("Failed to read\t - ");
    printf("   head=%zu ", f->head);
    printf("   tail=%zu ", f->tail);
    printf("   size=%zu ", f->size);
    printf("   free=%zu\n",vscp_fifo_getFree(f));
  }
}

int main()
{
  vscp_fifo_t f;
  
  printf("Init fifo\n");
  vscp_fifo_init(&f, 16);

  for (int i=0; i<16; i++) {
    addItem(&f, i);    
  }

  for (int i=0; i<20; i++) {
    vscpEvent *pev;
    if (vscp_fifo_read(&f, &pev)) {
      printf("Read %d\t - class = %d ", i, pev->vscp_class);
      printf("   head=%zu ", f.head);
      printf("   tail=%zu ", f.tail);
      printf("   size=%zu ", f.size);
      printf("   free=%zu\n",vscp_fifo_getFree(&f));
      free(pev->pdata);
      pev->pdata = NULL;
      free(pev);
    }
  }

  addItem(&f, 99);
  readItem(&f);
  readItem(&f);

  printf("Clear fifo\n");
  vscp_fifo_clear(&f);
}