/* 
  projdefs.h

  This file contains project definitions for the VSCP TCP/IP link protocol code.
*/



#ifndef _VSCP_PROJDEFS_H_
#define _VSCP_PROJDEFS_H_


/*!
  Set to non zero to show custom help. The callback is called so you can respond 
  with your custom help text.  This can be used to save memory if you work 
  on a constraint environment.
  
  If zero standard help is shown.
*/
#define VSCP_LINK_CUSTOM_HELP_TEXT  0

/*!
  Size for inout buffer and outputbuffer.
  Must be at least one for each fifo
*/
#define VSCP_LINK_MAX_IN_FIFO_SIZE    10
#define VSCP_LINK_MAX_OUT_FIFO_SIZE   10

#endif // _VSCP_PROJDEFS_H_