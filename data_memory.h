/********************************************************************************
* data_memory.h: Contains function declarations and macro definitions for
*                implementation of a 4 kB data memory.
********************************************************************************/
#ifndef DATA_MEMORY_H_
#define DATA_MEMORY_H_

/* Include directives: */
#include "cpu.h"

/* Macro definitions: */
#define DATA_MEMORY_ADDRESS_WIDTH 512 /* 512 unique addresses (0 - 511). */
#define DATA_MEMORY_DATA_WIDTH 8      /* 8 bit storage capacity per address. */

/********************************************************************************
* data_memory_reset: Clears data memory.
********************************************************************************/
void data_memory_reset(void);

/********************************************************************************
* data_memory_write: Writes a byte to specified address and returns 0 after
*                    successful write. If an invalid address is specified,
*                    no write is done and error code 1 is returned.
*
*                    - address: Address to write to in data memory.
*                    - value  : Value to write to specified address.
********************************************************************************/
int data_memory_write(const uint16_t address,
                      const uint8_t data);

/********************************************************************************
* data_memory_read: Returns a byte read from specified address. If an invalid
*                   address is specified, no read is done and 0 is returned.
*
*                   - address: Address to read from in data memory.
********************************************************************************/
uint8_t data_memory_read(const uint16_t address);

#endif /* DATA_MEMORY_H_ */