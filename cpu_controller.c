/********************************************************************************
* cpu_controller.c: Contains functionality for control of the program flow
*                   by input from the keyboard.
********************************************************************************/
#include "cpu_controller.h"

/* Static functions: */
static inline void print_information_at_start(void);
static inline void print_menu(void);
static int execute_selection(void);
static uint8_t get_selection(void);
static void readline(char* s,
                     const int size);
static inline uint8_t get_byte(void);

/********************************************************************************
* cpu_controller_run_by_input: Controls the program flow and input to the PINB
*                              register by input from the keyboard.
********************************************************************************/
void cpu_controller_run_by_input(void)
{
   control_unit_reset();
   /* print_information_at_start(); */

   while (1)
   {
      control_unit_print();
      print_menu();
      if (execute_selection()) return;
   }
}

/********************************************************************************
* print_information_at_start: Prints information about connected devices.
********************************************************************************/
static inline void print_information_at_start(void)
{
   printf("A led is connected to pin 8 (PORTB0).\n");
   printf("Press the button connected to pin 13 (PORTB5) to toggle the led.\n");
   printf("To press the button, set the fifth bit of the PINB register, ");
   printf("for instance by entering the value 32!\n\n");
   return;
}

/********************************************************************************
* print_menu: Prints menu for user control of program flow and input.
********************************************************************************/
static inline void print_menu(void)
{
   printf("Please select between the following alternatives:\n");
   printf("1. Execute next instruction cycle\n");
   printf("2. Run next clock cycle\n");
   printf("3. Reset system\n");
   printf("4. Enter new input for pin input register PINB\n");
   printf("5. Finish execution\n\n");
   return;
}

/********************************************************************************
* execute_selection: Reads and executes user selection entered from keyboard.
********************************************************************************/
static int execute_selection(void)
{
   const uint8_t selection = get_selection();

   if (selection == 1)
   {
      control_unit_run_next_instruction_cycle();
   }
   else if (selection == 2)
   {
      control_unit_run_next_state();
   }
   else if (selection == 3)
   {
      control_unit_reset();
      printf("System reset!\n");
   }
   else if (selection == 4)
   {
      printf("Enter new data for pin input register PINB:\n");
      const uint8_t input = get_byte();
      data_memory_write(PINB, input);
      printf("Wrote %s to pin input register PINB!\n\n", get_binary(input, 8));
   }
   else if (selection == 5)
   {
      printf("System exit!\n\n");
      return 1;
   }
   return 0;
}

/********************************************************************************
* get_selection: Retunrs user selection from keyboard after correct input.
********************************************************************************/
static uint8_t get_selection(void)
{
   while (1)
   {
      const uint8_t selection = get_byte();

      if (selection >= 0 && selection <= 5)
      {
         return selection;
      }
      else
      {
         printf("Invalid input, try again!\n\n");
      }
   }
}

/********************************************************************************
* readline: Reads text entered from keyboard into referenced string. 
* 
*           - s   : Reference to the string which stores entered content.
*           - size: The capacity of the string.
********************************************************************************/
static void readline(char* s,
                     const int size)
{
   fgets(s, size, stdin);
   printf("\n");

   for (char* i = s; *i; ++i)
   {
      if (*i == '\n')
      {
         *i = '\0';
      }
   }
   return;
}

/********************************************************************************
* get_byte: Returns an unsigned integer entered from the terminal.
********************************************************************************/
static inline uint8_t get_byte(void)
{
   char s[20] = { '\0' };
   readline(s, sizeof(s));
   return (uint8_t)atoi(s);
}

