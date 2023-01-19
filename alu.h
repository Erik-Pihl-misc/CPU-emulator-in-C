/********************************************************************************
* alu.h: Contains function declarations for implementation of an ALU 
*        (Aritmetic Logic Unit) for performing calculations and updating 
*        status bits SNZVC as described below:
* 
*        S (Signed)  : Set if result is negative with overflow considered.
*        N (Negative): Set if result is negative, i.e. S = result[7].
*        Z (Zero)    : Set if result is zero, i.e. N = result == 0 ? 1 : 0.
*        V (Overflow): Set if overflow occurs**.
*        C (Carry)   : Set if result contains a carry bit, C = result[9].
* 
*        * Signed flag is set if result is negative (N = 1) while 
*          overflow hasn't occured (V = 0) or result is positive (N = 0)
*          while overflow has occured (V = 1), i.e. S = N ^ V.
* 
*          For instance, consider the subtracting two 8-bit numbers -100 and 50.
*          The result is -100 - 50 = -150, but since only 8 bits are used, the
*          result is implemented as -150 + 2^8 = -150 + 256 = 106, i.e.
*          0110 0110. Since most significant bit is cleared, the N-flag is
*          cleared and the number if positive. However, overflow occured, 
*          since the two numbers -100 and 50 have different signs and the
*          result has the same sign as the subtrahend 50. Therefore the 
*          V-flag. Since N = 0 && V == 1, the S-flag is set. Therefore 
*          the number is correctly intepreted as negative.         
* 
*        ** Overflow occurs:
*
*           a) During addition (+) if the operands A and B are of the 
*              same sign and the result is of the opposite sign, i.e.
* 
*              V = (A[7] == B[7]) && (A[7] != result[7]) : 1 : 0
*  
*           b) During subtraction (-) if the operands A and B are of the
*              opposite sign and the result has the same sign as B, i.e.
* 
*              V = (A[7] != B[7]) && (B[7] == result[7]) : 1 : 0
********************************************************************************/
#ifndef ALU_H_
#define ALU_H_

#include "cpu.h"

/********************************************************************************
* alu: Performs calculation with specified operands and returns the result.
*      The status flags SNZVC of the referenced status register are updated 
*      in accordance with the result.
* 
*      - operation: The operation to perform (OR, AND, XOR, ADD or SUB).
*      - a        : First operand.
*      - b        : Second operand.
*      - sr       : Reference to status register containing SNZVC flags.
********************************************************************************/
uint8_t alu(const uint8_t operation, 
            const uint8_t a, 
            const uint8_t b, 
            uint8_t* sr);

#endif /* ALU_H_ */