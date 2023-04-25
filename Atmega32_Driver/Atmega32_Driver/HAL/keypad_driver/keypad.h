/*
 * Kaypad.h
 *
 * Created: 4/22/2023 11:34:52 AM
 *  Author: Ahmed Adel Wafdy 
 */ 


#ifndef KAYPAD_H_
#define KAYPAD_H_

#include <avr/io.h>

#define F_CPU						8000000UL
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>

//keypad mapping
#define R0							0
#define R1							1
#define R2							2
#define R3							3
#define C0							4
#define C1							5
#define C2							6
#define C3							7

#define Keypad_PORT					PORTD
#define Keypad_data_dir_PORT		DDRD
#define Keypad_pin					PIND

//APIs
void keypad_init(void);
char keypad_get_char();



#endif /* KAYPAD_H_ */
