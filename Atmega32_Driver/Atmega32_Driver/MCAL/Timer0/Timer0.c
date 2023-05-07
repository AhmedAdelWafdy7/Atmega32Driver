/*
 * Timer0.c
 *
 * Created: 5/4/2023 11:38:49 PM
 *  Author: Ahmed Adel Wafdy
 */ 

#include "Timer0.h"


void (*GP_IRQ_CallBack)(void) = NULL;

TIMER0_Config_t G_TIMER0_Config;




void MCAL_TIMER0_Init(TIMER0_Config_t *TIMER0_Config)
{
	
	G_TIMER0_Config = *TIMER0_Config;
	
	/* Select Timer Mode */
	TCCR0 |= TIMER0_Config->Timer_Mode;
	if(TIMER0_Config->Timer_Mode == TIMER0_MODE_FAST_PWM_INVERTING || TIMER0_Config->Timer_Mode == TIMER0_MODE_FAST_PWM_NONINVERTING)
	{
		DDRB |= (1<<3);
	}
	
	/* Select Clock Source */
	TCCR0 |= TIMER0_Config->Clock_Source;
	if(TIMER0_Config->Clock_Source == TIMER0_CLOCK_SOURCE_EXTERNAL_FALLING_EDGE || TIMER0_Config->Clock_Source == TIMER0_CLOCK_SOURCE_EXTERNAL_RISING_EDGE)
	{
		DDRB &= ~(1<<0);
	}
	
	TIMSK |= TIMER0_Config->IRQ_Enable;
	if(TIMER0_Config->IRQ_Enable != TIMER0_IRQ_ENABLE_NONE)
	{
		Enable_G_Interrupt();
	}
	else
	{
		Disable_G_Interrupt();
	}
	
	GP_IRQ_CallBack = TIMER0_Config->P_IRQ_CallBack;
}

void MCAL_TIMER0_DeInit(void)
{
	TCCR0 &= ~((1<<0)|(1<<1)|(1<<2));
}


void MCAL_TIMER0_SetCompareValue(uint8_t TicksNumber)
{
	OCR0 = TicksNumber;
}


void MCAL_TIMER0_GetCompareValue(uint8_t* TicksNumber)
{
	*TicksNumber = TCNT0;
}


void MCAL_PWM_DutyCycle(uint8_t Duty_Cycle)
{
	if(G_TIMER0_Config.Timer_Mode == TIMER0_MODE_FAST_PWM_NONINVERTING)
	{
		OCR0 = Duty_Cycle;
	}
	else if(G_TIMER0_Config.Timer_Mode == TIMER0_MODE_FAST_PWM_INVERTING)
	{
		OCR0 = (uint8_t)(255 - Duty_Cycle);
	}
}

ISR(TIMER0_OVF_vect)
{
	GP_IRQ_CallBack();
}

ISR(TIMER0_COMP_vect)
{
	GP_IRQ_CallBack();
}