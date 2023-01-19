/********************************************************************************
* stack.h: Contains function declarations and macro definitions for 
*          implementation of a 4 kB stack, which grows downward.
********************************************************************************/
#ifndef STACK_H_
#define STACK_H_

/* Macro definitions: */
#include "cpu.h"

/* Static functions: */
#define STACK_ADDRESS_WIDTH 512 /* 512 unique addresses (0 - 511). */
#define STACK_DATA_WIDTH 8      /* 8 bit storage capcity per address. */

/********************************************************************************
* stack_reset: Clears the stack.
********************************************************************************/
void stack_reset(void);

/********************************************************************************
* stack_push: Pushes a byte to the bottom of the stack and returns 0 after
*             successful execution. If the stack is full, no push is done 
*             and error code 1 is returned.
*
*             - value: Value to push to the stack.
********************************************************************************/
int stack_push(const uint8_t value);

/********************************************************************************
* stack_pop: Returns a byte popped from the bottom of the stack. If the stack
*            is empty, no read is done and 0 is returned.
********************************************************************************/
uint8_t stack_pop(void);

#endif /* STACK_H_ */