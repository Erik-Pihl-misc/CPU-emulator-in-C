/********************************************************************************
* control_unit.c: Contains static variables and function definitions for 
*                 implementation of an 8-bit control unit.
********************************************************************************/
#include "control_unit.h"

/* Static functions: */
static void monitor_interrupts(void);
static void pci_check_event(const uint8_t pin_reg,
                            const uint8_t mask_reg,
                            const uint8_t pcie_bit,
                            const uint8_t flag_bit,
                            uint8_t* pin_reg_last_value);
static void pci_check_for_interrupt_requests(void);
static void generate_interrupt(const uint8_t interrupt_vector);
static void return_from_interrupt(void);

/* Static variables: */
static uint32_t ir; /* Instruction register, stores next instruction to execute. */
static uint8_t pc;  /* Program counter, stores address to next instruction to fetch. */
static uint8_t mar; /* Memory address register, stores address for current instruction. */
static uint8_t sr;  /* Status register, stores status bits ISNZVC. */

static uint8_t op_code; /* Stores OP-code, for example LDI, OUT, JMP etc. */
static uint8_t op1;     /* Stores first operand, most often a destination. */
static uint8_t op2;     /* Stores second operand, most often a value or read address. */

static enum cpu_state state;                    /* Stores current state. */
static uint8_t pinb_last_value; /* Last input read from the PINB register. */
static uint8_t pinc_last_value; /* Last input read from the PINC register. */
static uint8_t pind_last_value; /* Last input read from the PIND register. */

static uint8_t reg[CPU_REGISTER_ADDRESS_WIDTH]; /* CPU-registers R0 - R31. */

/********************************************************************************
* control_unit_reset: Resets control unit registers and corresponding program.
********************************************************************************/
void control_unit_reset(void)
{
   ir = 0x00;
   pc = 0x00;
   mar = 0x00;
   sr = 0x00;

   op_code = 0x00;
   op1 = 0x00;
   op2 = 0x00;

   state = CPU_STATE_FETCH;

   for (uint8_t i = 0; i < CPU_REGISTER_ADDRESS_WIDTH; ++i)
   {
      reg[i] = 0x00;
   }

   pinb_last_value = 0x00;
   pinc_last_value = 0x00;
   pind_last_value = 0x00;

   data_memory_reset();
   stack_reset();
   program_memory_write();
   return;
}

/********************************************************************************
* control_unit_run_next_state: Runs next state in the CPU instruction cycle:
********************************************************************************/
void control_unit_run_next_state(void)
{
   switch (state)
   {
      case CPU_STATE_FETCH:
      {
         ir = program_memory_read(pc); /* Fetches next instruction. */
         mar = pc;                     /* Stores address of current instruction. */
         pc++;                         /* Program counter points to next instruction. */
         state = CPU_STATE_DECODE;     /* Decodes the instruction during next clock cycle. */
         break;
      }
      case CPU_STATE_DECODE:
      {        
         op_code = ir >> 16;           /* Bit 23 downto 16 consists of the OP code. */
         op1 = ir >> 8;                /* Bit 15 downto 8 consists of the first operand. */
         op2 = ir;                     /* Bit 7 downto 0 consists of the second operand. */
         state = CPU_STATE_EXECUTE;    /* Executes the instruction during next clock cycle. */
         break;
      }
      case CPU_STATE_EXECUTE:
      {
         switch (op_code) /* Checks the OP code.*/
         {
            case NOP: /* NOP => do nothing. */
            {
               break; 
            }
            case LDI: /* LDI R16, 0x01 => op_code = LDI, op1 = R16, op2 = 0x01 */
            {
               reg[op1] = op2; 
               break;
            }
            case MOV: /* MOV R17, R16 => op_code = MOV, op1 = R17, op2 = R16 */
            {
               reg[op1] = reg[op2]; 
               break;
            }
            case OUT: /* OUT DDRB, R16 => op_code = OUT, op1 = DDRB, op2 = R16 */
            {
               data_memory_write(op1, reg[op2]); 
               break;
            }
            case IN: /* IN R16, PINB => op_code = IN, op1 = R16, op2 = PINB */
            {
               reg[op1] = data_memory_read(op2); 
               break;
            }
            case STS: /* STS counter, R16 => op_code = STS, op1 = counter, op2 = R16 */
            {
               data_memory_write(op1 + 256, reg[op2]); 
               break;
            }
            case LDS: /* LDS R16, counter => op_code = LDS, op1 = R16, op2 = counter*/
            {
               reg[op1] = data_memory_read(op2 + 256); 
               break;
            }
            case ORI: /* ORI R16, 0x01 => op_code = ORI, op1 = R16, op2 = 0x01 */
            {
               reg[op1] = alu(OR, reg[op1], op2, &sr);
               break;
            }
            case ANDI: /* ANDI R17, 0x20 => op_code = ANDI, op1 = R17, op2 = 0x20 */
            {
               reg[op1] = alu(AND, reg[op1], op2, &sr);
               break;
            }
            case XORI: /* XORI R18, 0x05 => op_code = XORI, op1 = R18, op2 = 0x05 */
            {
               reg[op1] = alu(XOR, reg[op1], op2, &sr);
               break;
            }
            case OR: /* OR R16, R17 => op_code = OR, op1 = R16, op2 = R17 */
            {
               reg[op1] = alu(OR, reg[op1], reg[op2], &sr);
               break;
            }
            case AND: /* AND R16, R17 => op_code = AND, op1 = R16, op2 = R17 */
            {
               reg[op1] = alu(AND, reg[op1], reg[op2], &sr);
               break;
            }
            case XOR: /* XOR R16, R17 => op_code = XOR, op1 = R16, op2 = R17 */
            {
               reg[op1] = alu(XOR, reg[op1], reg[op2], &sr);
               break;
            }
            case ADDI: /* ADDI R16, 0x10 => op_code = ADDI, op1 = R16, op2 = 0x10 */
            {
               reg[op1] = alu(ADD, reg[op1], op2, &sr);
               break;
            }
            case SUBI: /* SUBI R17, 0x05 => op_code = SUBI, op1 = R17, op2 = 0x05 */
            {
               reg[op1] = alu(SUB, reg[op1], op2, &sr);
               break;
            }
            case ADD: /* ADD R16, R17 => op_code = ADD, op1 = R16, op2 = R17 */
            {
               reg[op1] = alu(ADD, reg[op1], reg[op2], &sr);
               break;
            }
            case SUB: /* SUB R17, R18 => op_code = SUB, op1 = R17, op2 = R18 */
            {
               reg[op1] = alu(SUB, reg[op1], reg[op2], &sr);
               break;
            }
            case INC: /* INC R16 => op_code = INC, op1 = R16 */
            {
               reg[op1] = alu(ADD, reg[op1], 0x01, &sr);
               break;
            }
            case DEC: /* DEC R17 => op_code = INC, op1 = R17 */
            {
               reg[op1] = alu(SUB, reg[op1], 0x01, &sr);
               break;
            }
            case CPI: /* CPI R16, 0x01 => op_code = CPI, op1 = R16, op2 = 0x01 */
            {
               (void)alu(SUB, reg[op1], op2, &sr);
               break;
            }
            case CP: /* CPI R16, R17 => op_code = CP, op1 = R16, op2 = R17 */
            {
               (void)alu(SUB, reg[op1], reg[op2], &sr);
               break;
            }
            case JMP: /* JMP 0x05 => op_code = JMP, op1 = 0x05 */
            {
               pc = op1; 
               break;
            }
            case BREQ: /* BREQ 0x10 => op_code = BREQ, op1 = 0x10 */
            {
               if (read(sr, Z)) pc = op1;
               break;
            }
            case BRNE: /* BRNE 0x20 => op_code = BRNE, op1 = 0x20 */
            {
               if (!read(sr, Z)) pc = op1;
               break;
            }
            case BRGE: /* BRGE 0x30 => op_code = BRGE, op1 = 0x30 */
            {
               if (!read(sr, S)) pc = op1;
               break;
            }
            case BRGT: /* BRGT 0x40 => op_code = BRGT, op1 = 0x40 */
            {
               if (!read(sr, S) && !read(sr, Z)) pc = op1;
               break;
            }
            case BRLE: /* BRLE 0x50 => op_code = BRLE, op1 = 0x50 */
            {
               if (read(sr, S) || (read(sr, Z))) pc = op1;
               break;
            }
            case BRLT: /* BRLT 0x60 => op_code = BRLT, op1 = 0x60 */
            {
               if (read(sr, S)) pc = op1;
               break;
            }
            case CALL: /* CALL 0x10 => op_code = CALL, op1 = 0x10 */
            {
               stack_push(pc); 
               pc = op1;
               break;
            }
            case RET: /* RET => op_code = RET */
            {
               pc = stack_pop(); 
               break;
            }
            case RETI: /* RETI => op_code = RETI */
            {
               return_from_interrupt();
               break;
            }
            case PUSH: /* PUSH R16 => op_code = PUSH, op1 = R16 */
            {
               stack_push(reg[R16]); 
               break;
            }
            case POP: /* POP R16 => op_code = POP, op1 = R16 */
            {
               reg[op1] = stack_pop(); 
               break;
            }
            case SEI: /* SEI => op_code = SEI */
            {
               set(sr, I);
               break;
            }
            case CLI: /* CLI => op_code = CLI */
            {
               clr(sr, I);
               break;
            }
            default:  /* System reset if error occurs. */
            {
               control_unit_reset();
               break;
            }
         }

         pci_check_for_interrupt_requests();
         state = CPU_STATE_FETCH; /* Fetches next instruction during next clock cycle. */
         break;
      }
      default: /* System reset if error occurs. */
      {
         control_unit_reset();
         break;
      }

   }

   monitor_interrupts();
   return;
}

/********************************************************************************
* control_unit_run_next_state: Runs next CPU instruction cycle, i.e. fetches
*                              a new instruction from program memory, decodes
*                              and executes it.
********************************************************************************/
void control_unit_run_next_instruction_cycle(void)
{
   do
   {
      control_unit_run_next_state();
   } while (state != CPU_STATE_EXECUTE);
   return;
}

/********************************************************************************
* control_unit_print: Prints information about the processor, for instance
*                     current subroutine, instruction, state, content in
*                     CPU-registers and I/O registers DDRB, PORTB and PINB.
********************************************************************************/
void control_unit_print(void)
{
   printf("--------------------------------------------------------------------------------\n");
   printf("Current subroutine:\t\t\t\t%s\n", program_memory_subroutine_name(mar));
   printf("Current instruction:\t\t\t\t%s\n", cpu_instruction_name(op_code));
   printf("Current state:\t\t\t\t\t%s\n", cpu_state_name(state));

   printf("Program counter:\t\t\t\t%hu\n", pc);

   printf("Instruction register:\t\t\t\t%s ", get_binary((ir >> 16) & 0xFF, 8));
   printf("%s ", get_binary((ir >> 8) & 0xFF, 8));
   printf("%s\n", get_binary(ir & 0xFF, 8));

   printf("Status register (ISNZVC):\t\t\t%s\n\n", get_binary(sr, 6));

   printf("Content in CPU register R16:\t\t\t%s\n", get_binary(reg[R16], 8));
   printf("Content in CPU register R17:\t\t\t%s\n", get_binary(reg[R17], 8));
   printf("Content in CPU register R18:\t\t\t%s\n", get_binary(reg[R18], 8));
   printf("Content in CPU register R24:\t\t\t%s\n\n", get_binary(reg[R24], 8));

   printf("Content in data direction register DDRB:\t%s\n", get_binary(data_memory_read(DDRB), 8));
   printf("Content in data register PORTB:\t\t\t%s\n", get_binary(data_memory_read(PORTB), 8));
   printf("Content in pin input register PINB:\t\t%s\n", get_binary(data_memory_read(PINB), 8));

   printf("--------------------------------------------------------------------------------\n\n");
   return;
}

/********************************************************************************
* monitor_interrupts: Monitors input pins for pin change interrupts, where
*                     an interrupt request is generated at an event on a pin
*                     where pin change interrupt_is_enabled.
********************************************************************************/
static void monitor_interrupts(void)
{
   pci_check_event(PINB, PCMSK0, PCIE0, PCIF0, &pinb_last_value);
   pci_check_event(PINC, PCMSK1, PCIE1, PCIF1, &pinc_last_value);
   pci_check_event(PIND, PCMSK2, PCIE2, PCIF2, &pind_last_value);
   return;
}

/********************************************************************************
* pci_check_event: Checks for pin change interrupts on specified I/O-port.
* 
*                  - pin_reg: Pin input register to read.
*                  
********************************************************************************/
static void pci_check_event(const uint8_t pin_reg, 
                            const uint8_t mask_reg,
                            const uint8_t interrupt_enable_bit,
                            const uint8_t flag_bit,
                            uint8_t* pin_reg_last_value)
{
   const uint8_t pin_reg_new_value = data_memory_read(pin_reg);
   const uint8_t pcicr = data_memory_read(PCICR + 256);
   const uint8_t pcmsk = data_memory_read(mask_reg + 256);

   for (uint8_t i = 0; i < IO_REGISTER_DATA_WIDTH; ++i)
   {
      if (read(pin_reg_new_value, i) != read(*pin_reg_last_value, i))
      {
         if (read(pcicr, interrupt_enable_bit) && read(pcmsk, i))
         {
            uint8_t pcifr = data_memory_read(PCIFR + 256);
            set(pcifr, flag_bit);
            data_memory_write(PCIFR + 256, pcifr);
            break;
         }
      }
   }

   *pin_reg_last_value = pin_reg_new_value;
   return;
}

/********************************************************************************
* pci_check_for_interrupt_requests: Checks for pin change interrupt requests 
*                                   by controlling the pin change interrupt
*                                   flags in the PCIFR register. If a flag is 
*                                   set and the I-flag in the status register
*                                   is set, an interrupt is generated.
********************************************************************************/
static void pci_check_for_interrupt_requests(void)
{
   uint8_t pcifr = data_memory_read(PCIFR + 256);

   for (uint8_t i = PCIF0; i < PCIF2; ++i)
   {
      if (read(pcifr, i) && read(sr, I))
      {
         if (i == PCIF0)
         {
            generate_interrupt(PCINT0_vect);
         }
         else if (i == PCIF1)
         {
            generate_interrupt(PCINT1_vect);
         }
         else if (i == PCIF2)
         {
            generate_interrupt(PCINT2_vect);
         }

         clr(pcifr, i);
         data_memory_write(PCIFR + 256, pcifr);
      }
   }
   return;
}

/********************************************************************************
* generate_interrupt: Pushes content of all registers in the control unit to
*                     the stack and generates and interrupt, where the program 
*                     counter is assigned the interrupt vector. 
********************************************************************************/
static void generate_interrupt(const uint8_t interrupt_vector)
{
   stack_push(ir >> 16);
   stack_push(ir >> 8);
   stack_push(ir);
   stack_push(pc);
   stack_push(mar);
   stack_push(sr);

   stack_push(op_code);
   stack_push(op1);
   stack_push(op2);

   stack_push(state);
   stack_push(pinb_last_value);
   stack_push(pinc_last_value);
   stack_push(pind_last_value);

   for (uint8_t i = 0; i < CPU_REGISTER_ADDRESS_WIDTH; ++i)
   {
      stack_push(reg[i]);
   }

   pc = interrupt_vector;
   state = CPU_STATE_FETCH;
   clr(sr, I);
   return;
}

/********************************************************************************
* return_from_interrupt: Restores program to last state before interrupt was
*                        generated. Content from all registers in the control 
*                        unit, which was pushed to the stack before the 
*                        interrupt was generated, is restored.
********************************************************************************/
static void return_from_interrupt(void)
{
   for (uint8_t i = 0; i < CPU_REGISTER_ADDRESS_WIDTH; ++i)
   {
      reg[CPU_REGISTER_ADDRESS_WIDTH - 1 - i] = stack_pop();
   }

   pind_last_value = stack_pop();
   pinc_last_value = stack_pop();
   pinb_last_value = stack_pop();
   state = stack_pop();

   op2 = stack_pop();
   op1 = stack_pop();
   op_code = stack_pop();

   sr = stack_pop();
   mar = stack_pop();
   pc = stack_pop();

   ir = stack_pop();
   ir |= stack_pop() << 8; 
   ir |= stack_pop() << 16;
   return;
}


