
/*
    pico-eeprom.h - RP2040 EEPROM emulation
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

#ifndef PICO_EEPROM_h
#define PICO_EEPROM_h

#include <stddef.h>
#include <stdint.h>
#include <string.h>

struct _eeprom_ {
  uint8_t* _sector;
  uint8_t* _data;
  size_t _size;
  int _dirty;
};

/**
 * @brief Initialize EEPROM storage
 * 
 * @param peeprom Pointer to the EEPROM structure
 * @return int 
 */

int eeprom_init(struct _eeprom_ * const peeprom, size_t size, uint8_t *start_in_flash);

/**
 * @brief Read data from EEPROM
 * 
 * @param peeprom Pointer to the EEPROM structure
 * @param dst Destination buffer
 * @param src Source address
 * @param size Size of data to read
 * @return int 
 */
int eeprom_cleanup(struct _eeprom_ * const peeprom);

/**
 * @brief Read data from EEPROM
 * 
 * @param peeprom Pointer to the EEPROM structure
 * @param dst Destination buffer
 * @param src Source address
 * @param size Size of data to read
 * @return int 
 */
uint8_t eeprom_read(struct _eeprom_ * const peeprom, int const address);

/**
 * @brief Write data to EEPROM
 * 
 * @param peeprom Pointer to the EEPROM structure
 * @param dst Destination buffer
 * @param src Source address
 * @param size Size of data to write
 * @return int 
 */
void eeprom_write(struct _eeprom_ * const peeprom, int const address, uint8_t const value);

/**
 * @brief Read data from EEPROM
 * 
 * @param peeprom Pointer to the EEPROM structure
 * @param dst Destination buffer
 * @param src Source address
 * @param size Size of data to read
 * @return int 
 */
int eeprom_commit(struct _eeprom_ * const peeprom);

/**
 * @brief Read data from EEPROM
 * 
 * @param peeprom Pointer to the EEPROM structure
 * @param dst Destination buffer
 * @param src Source address
 * @param size Size of data to read
 * @return int 
 */
uint8_t *eeprom_getDataPtr(struct _eeprom_ * const peeprom);

/**
 * @brief Read data from EEPROM
 * 
 * @param peeprom Pointer to the EEPROM structure
 * @param dst Destination buffer
 * @param src Source address
 * @param size Size of data to read
 * @return int 
 */
uint8_t const *eeprom_getConstDataPtr(struct _eeprom_ * const peeprom) ;


#endif // PICO_EEPROM_h
