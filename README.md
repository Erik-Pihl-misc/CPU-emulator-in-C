# Embedded computer system emulerator in C
Demonstrates how an 8-bit embedded computer system works in a terminal environment. 
Based on microcontroller ATmega328P.

The instruction set constitutes of a subset of the Atmel AVR instruction set. 
Includes data memory, program memory, stack, ALU, pin change interrupts etc.

In the program, a button is connected to PORTB5 and a led is connected to PORTB0.
Pin change interrupt is enabled on the button pin, so that events at the button pin
generates an interrupt. At pressdown, the led is toggled, otherwise nothing is done.
A static variable stored at address 356 (100 + offset 256) in data memory is used to 
store the led state.

Contains files to be opened as a complete project in Visual Studio 2022.