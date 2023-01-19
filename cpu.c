/********************************************************************************
* cpu.c: Contains function definitions for getting names of CPU instructions,
*        CPU registers and number of binary digits in unsigned numbers as text.
********************************************************************************/
#include "cpu.h"

/* Static functions: */
static inline size_t num_binary_digits(uint32_t num);
static inline char integer_to_char(const int num);

/********************************************************************************
* cpu_instruction_name: Returns the name of specified instruction.
*
*                       - instruction: The specified CPU instruction.
********************************************************************************/
const char* cpu_instruction_name(const uint8_t instruction)
{
   if (instruction == NOP)       return "NOP";
   else if (instruction == LDI)  return "LDI";
   else if (instruction == MOV)  return "MOV";
   else if (instruction == IN)   return "IN";
   else if (instruction == OUT)  return "OUT";
   else if (instruction == STS)  return "STS";
   else if (instruction == LDS)  return "LDS";
   else if (instruction == ORI)  return "ORI";
   else if (instruction == ANDI) return "ANDI";
   else if (instruction == XORI) return "XORI";
   else if (instruction == OR)   return "OR";
   else if (instruction == AND)  return "AND";
   else if (instruction == XOR)  return "XOR";
   else if (instruction == CLR)  return "CLR";
   else if (instruction == INC)  return "INC";
   else if (instruction == DEC)  return "DEC";
   else if (instruction == ADDI) return "ADDI";
   else if (instruction == SUBI) return "SUBI";
   else if (instruction == ADDI) return "ADD";
   else if (instruction == SUBI) return "SUB";
   else if (instruction == LSL)  return "LSL";
   else if (instruction == LSR)  return "LSR";
   else if (instruction == BREQ) return "BREQ";
   else if (instruction == BRNE) return "BRNE";
   else if (instruction == BRGE) return "BRGE";
   else if (instruction == BRGT) return "BRGT";
   else if (instruction == BRLE) return "BRLE";
   else if (instruction == BRLT) return "BRLT";
   else if (instruction == JMP)  return "JMP";
   else if (instruction == CPI)  return "CPI";
   else if (instruction == CP)   return "CP";
   else if (instruction == CALL) return "CALL";
   else if (instruction == RET)  return "RET";
   else if (instruction == PUSH) return "PUSH";
   else if (instruction == POP)  return "POP";
   else if (instruction == SEI)  return "SEI";
   else if (instruction == CLI)  return "CLI";
   else if (instruction == RETI) return "RETI";
   else return "Unknown";
}

/********************************************************************************
* cpu_state_name: Returns the name of specified CPU state.
*
*                 - state: The specified CPU state.
********************************************************************************/
const char* cpu_state_name(const enum cpu_state state)
{
   if (state == CPU_STATE_FETCH)        return "Fetch";
   else if (state == CPU_STATE_DECODE)  return "Decode";
   else if (state == CPU_STATE_EXECUTE) return "Execute";
   else return "Unknown";
}

/********************************************************************************
* get_binary: Returns specified number as a binary string with specified
*             minimum number of characters.
*
*             - num      : The specified number.
*             - min_chars: Minimum number of characters in the returned string.
********************************************************************************/
const char* get_binary(uint32_t num,
                       const uint8_t min_chars)
{
   static char s[33] = { '\0' };

   size_t size = num_binary_digits(num);
   if (size < min_chars) size = min_chars;
   s[size] = '\0';

   for (size_t i = 0; i < size; ++i)
   {
      s[size - 1 - i] = integer_to_char(num % 2);
      num /= 2;
   }

   s[size] = '\0';
   return s;
}

/********************************************************************************
* get_binary_digits: Returns the number of binary digits in specified number.
* 
*                    - num: The number in unsigned integer form.
********************************************************************************/
static inline size_t num_binary_digits(uint32_t num)
{
   size_t num_digits = 1;
   while (num /= 2) num_digits++;
   return num_digits;
}

/********************************************************************************
* integer_to_char: Converts specified number from integer to corresponding 
*                  character in the ASCII-table.
*
*                    - num: The number in integer form.
********************************************************************************/
static inline char integer_to_char(const int num)
{
   return num + 48;
}
