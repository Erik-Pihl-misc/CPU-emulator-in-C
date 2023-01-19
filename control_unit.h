/********************************************************************************
* control_unit.h: Contains function declarations for implementation of an
*                 8-bit control unit.
********************************************************************************/
#ifndef CONTROL_UNIT_H_
#define CONTROL_UNIT_H_

/* Include directives: */
#include "cpu.h"
#include "program_memory.h"
#include "data_memory.h"
#include "stack.h"
#include "alu.h"

/********************************************************************************
* control_unit_reset: Resets control unit and corresponding program.
********************************************************************************/
void control_unit_reset(void);

/********************************************************************************
* control_unit_run_next_state: Runs next state in the CPU instruction cycle.
********************************************************************************/
void control_unit_run_next_state(void);

/********************************************************************************
* control_unit_run_next_state: Runs next CPU instruction cycle, i.e. fetches
*                              a new instruction from program memory, decodes
*                              and executes it.
********************************************************************************/
void control_unit_run_next_instruction_cycle(void);

/********************************************************************************
* control_unit_print: Prints information about the processor, for instance
*                     current subroutine, instruction, state, content in
*                     CPU-registers and I/O registers DDRB, PORTB and PINB.
********************************************************************************/
void control_unit_print(void);

#endif /* CONTROL_UNIT_H_ */