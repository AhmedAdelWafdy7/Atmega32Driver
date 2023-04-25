/*
 * GPIO.c
 *
 * Created: 4/19/2023 3:01:17 AM
 *  Author: Ahmed Adel Wafdy
 */ 

#include "GPIO.h"


void INTIAL_PORT_INPUT(PORT_t port,PULLUP_STATUS_t status)
{
	switch (status)
	{
		case Pullup_OFF:
		switch (port)
		{
			case 0: DDRA  =0x00;
			PORTA =0x00;
			break;
			
			case 1: DDRB  =0x00;
			PORTB =0x00;
			break;
			
			case 2: DDRC  =0x00;
			PORTC =0x00;
			break;
			
			case 3: DDRD  =0x00;
			PORTD =0x00;
			break;
		}
		break;
		
		
		case Pullup_ON:
		switch (port)
		{
			case 0: DDRA  =0x00;
			PORTA =0xFF;
			break;
			
			case 1: DDRB  =0x00;
			PORTB =0xFF;
			break;
			
			case 2: DDRC  =0x00;
			PORTC =0xFF;
			break;
			
			case 3: DDRD  =0x00;
			PORTD =0xFF;
			break;
		}
		break;
	}
}

void INTIAL_PORT_OUTPUT(PORT_t port,uint8_t Intial_Value)
{
	switch (port)
	{
		case 0: DDRA  = 0xFF;
		PORTA = Intial_Value;
		break;
		
		case 1: DDRB  = 0xFF;
		PORTB = Intial_Value;
		break;
		
		case 2: DDRC  = 0xFF;
		PORTC = Intial_Value;
		break;
		
		case 3: DDRD  = 0xFF;
		PORTD = Intial_Value;
		break;
	}
	
}

void WRITE_PORT(PORT_t port, uint8_t DATA)
{
	switch (port)
	{
		case 0: PORTA = DATA;
		break;
		
		case 1: PORTB = DATA;
		break;
		
		case 2: PORTC = DATA;
		break;
		
		case 3: PORTD = DATA;
		break;
	}
	
}

uint8_t READ_PORT(PORT_t port)
{
	uint8_t DATA_READ;
	
	switch (port)
	{
		case 0: DATA_READ = PINA;
		break;
		
		case 1: DATA_READ = PINB;
		break;
		
		case 2: DATA_READ = PINC;
		break;
		
		case 3: DATA_READ = PIND;
		break;
	}
	return DATA_READ;
	
}

void TOGGLE_PORT(PORT_t port)
{
	switch (port)
	{
		case 0: PORTA ^=PORTA;
		break;
		
		case 1: PORTB ^=PORTB ;
		break;
		
		case 2: PORTC ^=PORTC;
		break;
		
		case 3: PORTD ^=PORTD;
		break;
	}
	
}


/*PIN Functions Definition*/
void INTIAL_PIN_INPUT(PORT_t port,uint8_t PIN_NUM,PULLUP_STATUS_t status)
{
	switch (status)
	{
		case Pullup_OFF:
		switch (port)
		{
			case 0: DDRA  &=~ (PIN_NUM);
			PORTA &=~ (PIN_NUM);
			break;
			
			case 1: DDRB  &=~ (PIN_NUM);
			PORTB &=~ (PIN_NUM);
			break;
			
			case 2: DDRC  &=~ (PIN_NUM);
			PORTC &=~ (PIN_NUM);
			break;
			
			case 3: DDRD  &=~ (PIN_NUM);
			PORTD &=~ (PIN_NUM);
			break;
		}
		break;
		
		
		case Pullup_ON:
		switch (port)
		{
			case 0: DDRA  &=~ (PIN_NUM);
			PORTA |=  (PIN_NUM);
			break;
			
			case 1: DDRB  &=~ (PIN_NUM);
			PORTB |=  (PIN_NUM);
			break;
			
			case 2: DDRC  &=~ (PIN_NUM);
			PORTC |=  (PIN_NUM);
			break;
			
			case 3: DDRD  &=~ (PIN_NUM);
			PORTD |=  (PIN_NUM);
			break;
		}
		break;
	}
	
}

void INTIAL_PIN_OUTPUT(PORT_t port,uint8_t PIN_NUM,PIN_Status_t Intial_val)
{
	switch (Intial_val)
	{
		case OUT_OFF : switch (port)
		{
			case 0: DDRA  |= (PIN_NUM);
			PORTA &=~(PIN_NUM);
			break;
			
			case 1: DDRB  |= (PIN_NUM);
			PORTB &=~(PIN_NUM);
			break;
			
			case 2: DDRC  |= (PIN_NUM);
			PORTC &=~(PIN_NUM);
			break;
			
			case 3: DDRD  |= (PIN_NUM);
			PORTD &=~(PIN_NUM);
			break;
		}
		break;
		
		case OUT_ON : switch (port)
		{
			case 0: DDRA  |= (PIN_NUM);
			PORTA |= (PIN_NUM);
			break;
			
			case 1: DDRB  |= (PIN_NUM);
			PORTB |= (PIN_NUM);
			break;
			
			case 2: DDRC  |= (PIN_NUM);
			PORTC |= (PIN_NUM);
			break;
			
			case 3: DDRD  |= (PIN_NUM);
			PORTD |= (PIN_NUM);
			break;
		}
		break;
	}
}

void WRITE_PIN(PORT_t port,uint8_t PIN_NUM,uint8_t data)
{
	switch (data)
	{
		case 0:  switch (port)
		{
			case 0: PORTA  &=~(PIN_NUM);
			break;
			
			case 1: PORTB &=~(PIN_NUM);
			break;
			
			case 2: PORTC &=~(PIN_NUM);
			break;
			
			case 3: PORTD &=~(PIN_NUM);
			break;
		}
		
		break;
		
		case 1:  switch (port)
		{
			case 0: PORTA |= (PIN_NUM);
			break;
			
			case 1: PORTB |= (PIN_NUM);
			break;
			
			case 2: PORTC |= (PIN_NUM);
			break;
			
			case 3: PORTD |= (PIN_NUM);
			break;
		}
	}
}

uint8_t READ_PIN(PORT_t port,uint8_t PIN_NUM)
{
	
	switch (port)
	{
		case 0: return ((PINA&=(PIN_NUM))?1:0);
		break;
		
		case 1: return ((PINB&=(PIN_NUM))?1:0);
		break;
		
		case 2: return ((PINC&=(PIN_NUM))?1:0);
		break;
		
		case 3: return ((PIND&=(PIN_NUM))?1:0);
		break;
	}
	
	
}