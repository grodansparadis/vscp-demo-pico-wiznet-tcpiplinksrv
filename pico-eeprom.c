/*
    pico-eeprom.c - RP2040 EEPROM emulation
    Copyright (c) 2021 Earle F. Philhower III. All rights reserved.

    Based on ESP8266 EEPROM library, which is
    Copyright (c) 2014 Ivan Grokhotkov. All rights reserved.

    220525
    Converted from C++ to C by: Ake Hedman, Grodans Paradis AB

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <pico.h>
#include <stdio.h>
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <pico/stdlib.h>

#include "vscp-compiler.h"
#include "vscp-projdefs.h"

#include "demo.h"
#include "pico-eeprom.h"


///////////////////////////////////////////////////////////////////////////////
// eeprom_init
//

int eeprom_init(struct _eeprom_ * const peeprom, size_t size, uint8_t *start_in_flash)
{
  if (NULL == peeprom) {
    return -1;
  }

  if ((size <= 0) || (size > 4096)) {
    size = 4096;
  }

  peeprom->_sector = XIP_BASE + start_in_flash;
  peeprom->_data = NULL;
  peeprom->_size = 0;
  peeprom->_dirty = 0;

  peeprom->_size = (size + 0xff) & (~0xff);  // Flash writes limited to 256 byte boundaries

  // In case begin() is called a 2nd+ time, don't reallocate if size is the same
  if (peeprom->_data && size != peeprom->_size) {
    VSCP_FREE(peeprom->_data);
    peeprom->_data = (uint8_t *)VSCP_MALLOC(size);
  } 
  else if (!peeprom->_data) {
    peeprom->_data = (uint8_t *)VSCP_MALLOC(size);
  }

  memcpy(peeprom->_data, peeprom->_sector, peeprom->_size);

  peeprom->_dirty = false; //make sure dirty is cleared in case begin() is called 2nd+ time 
  return 0;
}


///////////////////////////////////////////////////////////////////////////////
// eeprom_cleanup
//

int eeprom_cleanup(struct _eeprom_ * const peeprom) 
{
  int rv;

  if (!peeprom->_size) {
    return -1;
  }

  rv = eeprom_commit(peeprom);
  if (peeprom->_data) {
    VSCP_FREE(peeprom->_data);
  }
  peeprom->_data = 0;
  peeprom->_size = 0;
  peeprom->_dirty = 1;

  return rv;
}

///////////////////////////////////////////////////////////////////////////////
// eeprom_read
//

uint8_t eeprom_read(struct _eeprom_ * const peeprom, int const address) 
{
  if (address < 0 || (size_t)address >= peeprom->_size) {
    return 0;
  }
  if (!peeprom->_data) {
    return 0;
  }

  return peeprom->_data[address];
}

///////////////////////////////////////////////////////////////////////////////
// eeprom_write
//

void eeprom_write(struct _eeprom_ * const peeprom, int const address, uint8_t const value) 
{
  if (address < 0 || (size_t)address >= peeprom->_size) {
    return;
  }
  if (!peeprom->_data) {
    return;
  }

  // Optimize _dirty. Only flagged if data written is different.
  uint8_t* pData = &peeprom->_data[address];
  if (*pData != value) {
    *pData = value;
    peeprom->_dirty = 1;
  }
}

///////////////////////////////////////////////////////////////////////////////
// eeprom_commit
//

int eeprom_commit(struct _eeprom_ *const peeprom) 
{
  if (!peeprom->_size) {
    return -1;
  }

  if (!peeprom->_dirty) {
    return 0;
  }

  if (!peeprom->_data) {
    return -1;
  }

  uint32_t status = save_and_disable_interrupts();
  //IDLE_OTHER_CORE(); // define to nothing if not used
  flash_range_erase((intptr_t)peeprom->_sector - (intptr_t)XIP_BASE, FLASH_SECTOR_SIZE);
  flash_range_program((intptr_t)peeprom->_sector - (intptr_t)XIP_BASE, peeprom->_data, peeprom->_size);
  //RESUME_OTHER_CORE(); // define to nothing if not used
  restore_interrupts(status);

  return 0;
}

///////////////////////////////////////////////////////////////////////////////
// eeprom_getDataPtr
//

uint8_t *eeprom_getDataPtr(struct _eeprom_ * const peeprom) 
{
  peeprom->_dirty = true;
  return &peeprom->_data[0];
}

///////////////////////////////////////////////////////////////////////////////
// eeprom_getDataPtr
//

uint8_t const *eeprom_getConstDataPtr(struct _eeprom_ * const peeprom) 
{
  return &peeprom->_data[0];
}

