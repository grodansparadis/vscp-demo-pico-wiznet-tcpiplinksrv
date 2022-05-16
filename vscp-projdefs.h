/* 
  projdefs.h

  This file contains project definitions for the VSCP TCP/IP link protocol code.
*/



#ifndef _VSCP_PROJDEFS_H_
#define _VSCP_PROJDEFS_H_


/*!
  Max buffer for level II events. The buffer size is needed to
  convert an event to string. To handle all level II events
  512*5 + 110 = 2670 bytes is needed. In reality this is
  seldom needed so the value can be set to a lower value. In this
  case one should check the max data size for events that are of
  interest and set the max size accordingly 
*/
#define VSCP_LINK_MAX_BUF         2670

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