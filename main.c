/********************************************************************************
* main.c: Demonstration of an 8-bit CPU in progress, based on AVR architecture.
********************************************************************************/
#include "cpu_controller.h"

/********************************************************************************
* main: Controls the program flow of an 8-bit processor by keyboard input.
********************************************************************************/
int main(void)
{
   cpu_controller_run_by_input();
   return 0;
}