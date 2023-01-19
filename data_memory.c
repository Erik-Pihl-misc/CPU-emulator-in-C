/********************************************************************************
* data_memory.c: Contains function definitions for implementation of a 
*                4 kB data memory.
********************************************************************************/
#include "data_memory.h"

/********************************************************************************
* data: Data memory with capacity of storing 4096 bytes.
********************************************************************************/
static uint8_t data[DATA_MEMORY_ADDRESS_WIDTH];

/********************************************************************************
* data_memory_reset: Clears data memory.
********************************************************************************/
void data_memory_reset(void)
{
   for (uint16_t i = 0; i < DATA_MEMORY_ADDRESS_WIDTH; ++i)
   {
      data[i] = 0x00;
   }
   return;
}

/********************************************************************************
* data_memory_write: Writes a byte to specified address and returns 0 after
*                    successful write. If an invalid address is specified,
*                    no write is done and error code 1 is returned.
* 
*                    - address: Address to write to in data memory.
*                    - value  : Value to write to specified address.
********************************************************************************/
int data_memory_write(const uint16_t address,
                      const uint8_t value)
{
   if (address < DATA_MEMORY_ADDRESS_WIDTH)
   {
      data[address] = value;
      return 0;
   }
   else
   {
      return 1;
   }
}

/********************************************************************************
* data_memory_read: Returns a byte read from specified address. If an invalid
*                   address is specified, no read is done and 0 is returned.
* 
*                   - address: Address to read from in data memory.
********************************************************************************/
uint8_t data_memory_read(const uint16_t address)
{
   if (address < DATA_MEMORY_ADDRESS_WIDTH)
   {
      return data[address];
   }
   else
   {
      return 0;
   }
}