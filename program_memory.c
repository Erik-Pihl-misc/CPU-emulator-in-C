/********************************************************************************
* program_memory.c: Contains function definitions and macro definitions for
*                   implementation of a 6 kB program memory, capable of storing
*                   up to 256 24-bit instructions. Since C doesn't support
*                   unsigned 24-bit integers (without using structs or unions),
*                   the program memory is set to 32 bits data width, but only
*                   24 bits are used.
********************************************************************************/
#include "program_memory.h"

/* Macro definitions: */
#define main            8  /* Start address for subroutine main. */
#define main_loop       9  /* Start address for loop in subroutine main. */
#define led1_toggle     10 /* Start address for subroutine led1_toggle. */
#define led1_off        13 /* Start address for subroutine led1_off. */
#define led1_on         19 /* Start address for subroutine led1_on. */
#define setup           25 /* Start address for subroutine setup. */
#define ISR_PCINT0      33 /* Start address for PCINT0 interrupt handler. */
#define ISR_PCINT0_end  37 /* End address for PCINT0 interrupt handler.*/
#define end             38 /* End address for current program. */

#define LED1 PORTB0        /* Led 1 connected to pin 8 (PORTB0). */
#define BUTTON1 PORTB5     /* Button 1 connected to pin 13 (PORTB5). */
#define led1_enabled 100   /* Address for variable storing the state of led 1.  */

/* Static functions: */
static inline uint32_t assemble(const uint8_t op_code,
                                const uint8_t op1,
                                const uint8_t op2);

/********************************************************************************
* data: Program memory with capacity for storing 256 instructions.
********************************************************************************/
static uint32_t data[PROGRAM_MEMORY_ADDRESS_WIDTH];

/********************************************************************************
* program_memory_write: Writes machine code to the program memory. This function
*                       should be called once when the program starts.
********************************************************************************/
void program_memory_write(void)
{
   static bool program_memory_initialized = false;
   if (program_memory_initialized) return;

   /********************************************************************************
   * RESET_vect: Reset vector and start address for the program. A jump is made
   *             to the main subroutine in order to start the program.
   ********************************************************************************/
   data[0]  = assemble(JMP, main, 0x00);
   data[1]  = assemble(NOP, 0x00, 0x00);

   /********************************************************************************
   * PCINT0_vect: Interrupt vector for pin change interrupt on I/O-port B. A jump
   *              is made to the corresponding interrupt handler ISR_PCINT0 to
   *              handle the interrupt.
   ********************************************************************************/
   data[2]  = assemble(JMP, ISR_PCINT0, 0x00);
   data[3]  = assemble(NOP, 0x00, 0x00);
   data[4]  = assemble(NOP, 0x00, 0x00);
   data[5]  = assemble(NOP, 0x00, 0x00);
   data[6]  = assemble(NOP, 0x00, 0x00);
   data[7]  = assemble(NOP, 0x00, 0x00);

   /********************************************************************************
   * main: Initiates the system at start. The program is kept running as long
   *       as voltage is supplied. The led connected to PORTB0 is enabled when
   *       the button connected to PORTB5 is pressed, otherwise it's disabled.
   ********************************************************************************/
   data[8]  = assemble(CALL, setup, 0x00);
   data[9]  = assemble(JMP, main_loop, 0x00);

   /********************************************************************************
   * led1_toggle: Toggle the led connected to PORTB0.
   ********************************************************************************/
   data[10] = assemble(LDS, R16, led1_enabled);
   data[11] = assemble(CPI, R16, 0x00);
   data[12] = assemble(BREQ, led1_on, 0x00);

   /********************************************************************************
   * led1_off: Disables the led connected to PORTB0.
   ********************************************************************************/
   data[13] = assemble(IN, R16, PORTB);
   data[14] = assemble(ANDI, R16, ~(1 << LED1));
   data[15] = assemble(OUT, PORTB, R16);
   data[16] = assemble(LDI, R16, 0x00);
   data[17] = assemble(STS, led1_enabled, R16);
   data[18] = assemble(RET, 0x00, 0x00);

   /********************************************************************************
   * led1_on: Enables the led connected to PORTB0.
   ********************************************************************************/
   data[19] = assemble(IN, R16, PORTB);
   data[20] = assemble(ORI, R16, (1 << LED1));
   data[21] = assemble(OUT, PORTB, R16);
   data[22] = assemble(LDI, R16, 0x01);
   data[23] = assemble(STS, led1_enabled, R16);
   data[24] = assemble(RET, 0x00, 0x00);

   /********************************************************************************
   * setup: Sets the led pin to output and enables the internal pull-up resistor 
   *        for the button pin.
   ********************************************************************************/
   data[25] = assemble(LDI, R16, (1 << LED1));
   data[26] = assemble(OUT, DDRB, R16);
   data[27] = assemble(LDI, R17, (1 << BUTTON1));
   data[28] = assemble(OUT, PORTB, R17);
   data[29] = assemble(SEI, 0x00, 0x00);
   data[30] = assemble(STS, PCICR, R16);
   data[31] = assemble(STS, PCMSK0, R17);
   data[32] = assemble(RET, 0x00, 0x00);

   /********************************************************************************
   * ISR_PCINT0: Interrupt handler for pin change interrupt at I/O-port B, which
   *             is generated at pressdown and release of BUTTON1 connected to
   *             PORTB5. At pressdown, the led connected to PORTB0 is toggled.
   ********************************************************************************/
   data[33] = assemble(IN, R16, PINB);
   data[34] = assemble(ANDI, R16, (1 << BUTTON1));
   data[35] = assemble(BREQ, ISR_PCINT0_end, 0x00);
   data[36] = assemble(CALL, led1_toggle, 0x00);
   data[37] = assemble(RETI, 0x00, 0x00);

   program_memory_initialized = true;
   return;
}

/********************************************************************************
* program_memory_read: Returns the instruction at specified address. If an
*                      invalid address is specified (should be impossible as
*                      long as the program memory address width isn't increased)
*                      no operation (0x00) is returned.
*
*                      - address: Address to instruction in program memory.
********************************************************************************/
uint32_t program_memory_read(const uint8_t address)
{
   if (address < PROGRAM_MEMORY_ADDRESS_WIDTH)
   {
      return data[address];
   }
   else
   {
      return 0x00;
   }
}

/********************************************************************************
* program_memory_subroutine_name: Returns the name of the subroutine at
*                                 specified address.
*
*                                 - address: Address within the subroutine.
********************************************************************************/
const char* program_memory_subroutine_name(const uint8_t address)
{
   if (address >= RESET_vect && address < PCINT0_vect)    return "RESET_vect";
   else if (address >= PCINT0_vect && address < main)     return "PCINT0_vect";
   else if (address >= main && address < led1_toggle)     return "main";
   else if (address >= led1_toggle && address < led1_off) return "led1_toggle";
   else if (address >= led1_off && address < led1_on)     return "led1_off";
   else if (address >= led1_on && address < setup)        return "led1_on";
   else if (address >= setup && address < ISR_PCINT0)     return "setup";
   else if (address >= ISR_PCINT0 && address < end)       return "ISR_PCINT0";
   else                                                   return "Unknown";
}

/********************************************************************************
* assemble: Returns instruction assembled to machine code.
* 
*           - op_code: OP code of the instruction.
*           - op1    : First operand (destination).
*           - op2    : Second operand (constant or read location).
********************************************************************************/
static inline uint32_t assemble(const uint8_t op_code,
                                const uint8_t op1,
                                const uint8_t op2)
{
   const uint32_t instruction = (op_code << 16) | (op1 << 8) | op2;
   return instruction;
}