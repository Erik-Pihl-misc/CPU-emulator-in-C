/********************************************************************************
* stack.c: Contains function definitions for implementation of a 4 kB stack, 
*          which grows downward.
********************************************************************************/
#include "stack.h"

/********************************************************************************
* stack: Stack with capacity of storing 4096 bytes.
********************************************************************************/
static uint8_t stack[STACK_ADDRESS_WIDTH];

/********************************************************************************
* sp: Stack pointer, points to the last added element to the stack.
********************************************************************************/
static uint16_t sp = STACK_ADDRESS_WIDTH - 1;

/********************************************************************************
* stack_empty: Indicates if the stack is empty. 
********************************************************************************/
static bool stack_empty = true;

/********************************************************************************
* stack_reset: Clears the stack.
********************************************************************************/
void stack_reset(void)
{
   for (uint16_t i = 0; i < STACK_ADDRESS_WIDTH; ++i)
   {
      stack[i] = 0x00;
   }

   sp = STACK_ADDRESS_WIDTH - 1;
   stack_empty = true;
   return;
}

/********************************************************************************
* stack_push: Pushes a byte to the bottom of the stack and returns 0 after
*             successful execution. If the stack is full, no push is done
*             and error code 1 is returned.
*
*             - value: Value to push to the stack.
********************************************************************************/
int stack_push(const uint8_t value)
{
   if (sp > 0)
   {
      if (stack_empty)
      {
         stack[sp] = value;
         stack_empty = false;
      }
      else
      {
         stack[--sp] = value;
      }
      return 0;
   }
   else
   {
      return 1;
   }
}

/********************************************************************************
* stack_pop: Returns a byte popped from the bottom of the stack. If the stack
*            is empty, no read is done and 0 is returned.
********************************************************************************/
uint8_t stack_pop(void)
{
   if (stack_empty)
   {
      return 0;
   }
   else
   {
      if (sp < STACK_ADDRESS_WIDTH - 1)
      {
         return stack[sp++];
      }
      else
      {
         stack_empty = true;
         return stack[sp];
      }
   }
}