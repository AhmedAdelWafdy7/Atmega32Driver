/*
 * Atmega32_Driver.c
 *
 * Created: 4/22/2023 11:34:52 AM
 * Author : Ahmed Adel Wafdy
 */ 

#include "Timer0.h"
#define F_CPU 8000000UL

uint32_t Timer_Counter = 0;


int main(void)
{

	SET_BIT(DDRB,3);
	
	TIMER0_Config_t TMR_CFG;
	
	TMR_CFG.Timer_Mode = TIMER0_MODE_FAST_PWM_INVERTING;
	TMR_CFG.Clock_Source = TIMER0_CLOCK_SOURCE_INTERNAL_PRESCALER_8;
	TMR_CFG.IRQ_Enable = TIMER0_IRQ_ENABLE_NONE;
	TMR_CFG.P_IRQ_CallBack = NULL;
	
	MCAL_TIMER0_Init(&TMR_CFG);
    /* Replace with your application code */
    while (1) 
    {
		MCAL_PWM_DutyCycle(100);
    }
}

