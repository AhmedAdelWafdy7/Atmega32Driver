/*
 * Timer0.h
 *
 * Created: 5/4/2023 11:38:32 PM
 *  Author: Ahmed Adel Wafdy
 */ 


#ifndef TIMER0_H_
#define TIMER0_H_

//-----------------------------
//INCLUDES
//-----------------------------
#include "MemoryMap.h"
#include "Utilities.h"
#include "GPIO.h"
#include "avr/interrupt.h"
//USER TYPE DEFINITIONs
//-----------------------------

typedef struct
{
	uint8_t		Timer_Mode;					// specifies Timer Mode 
	// This parameter must be set based on @ref TIMER0_MODE_DEFINE
	
	uint8_t		Clock_Source;				// specifies Timer Clock Source and PreScaler 
	// This parameter must be set based on @ref TIMER0_CLOCK_SOURCE_DEFINE
	
	uint8_t		IRQ_Enable;					// Enable or Disable IRQ
	// This parameter must be set based on @ref TIMER0_IRQ_ENABLE_DEFINE
	
	void (*P_IRQ_CallBack)(void);			//set the C Function which will be called once IRQ Happens
	
}TIMER0_Config_t;



//@ref TIMER0_MODE_DEFINE
#define TIMER0_MODE_NORMAL									((uint8_t)(0))
#define TIMER0_MODE_CTC										((uint8_t)(1<<3))
#define TIMER0_MODE_FAST_PWM_INVERTING						((uint8_t)(0x78))
#define TIMER0_MODE_FAST_PWM_NONINVERTING					((uint8_t)(0x68))

//@ref TIMER0_CLOCK_SOURCE_DEFINE
#define TIMER0_CLOCK_SOURCE_INTERNAL_NO_PRESCALER			((uint8_t)(1<<0))
#define TIMER0_CLOCK_SOURCE_INTERNAL_PRESCALER_8			((uint8_t)(1<<1))
#define TIMER0_CLOCK_SOURCE_INTERNAL_PRESCALER_64			((uint8_t)(0x03))
#define TIMER0_CLOCK_SOURCE_INTERNAL_PRESCALER_256			((uint8_t)(0x04))
#define TIMER0_CLOCK_SOURCE_INTERNAL_PRESCALER_1024			((uint8_t)(0x05))
#define TIMER0_CLOCK_SOURCE_EXTERNAL_FALLING_EDGE			((uint8_t)(0x06))
#define TIMER0_CLOCK_SOURCE_EXTERNAL_RISING_EDGE			((uint8_t)(0x07))


//@ref TIMER0_IRQ_ENABLE_DEFINE
#define TIMER0_IRQ_ENABLE_NONE								((uint8_t)(0))
#define TIMER0_IRQ_ENABLE_TOIE0								((uint8_t)(1<<0))
#define TIMER0_IRQ_ENABLE_OCIE0								((uint8_t)(1<<1))



void MCAL_TIMER0_Init(TIMER0_Config_t *TIMER0_Config);
void MCAL_TIMER0_DeInit(void);

void MCAL_TIMER0_GetCompareValue(uint8_t* TicksNumber);
void MCAL_TIMER0_SetCompareValue(uint8_t TicksNumber);

void MCAL_PWM_DutyCycle(uint8_t Duty_Cycle);

#endif /* TIMER0_H_ */