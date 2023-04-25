/*
 * GPIO.h
 *
 * Created: 4/19/2023 3:01:17 AM
 *  Author: Ahmed Adel Wafdy
 */ 


#ifndef GPIO_H_
#define GPIO_H_

//-----------------------------
//INCLUDES
//-----------------------------
#include "MemoryMap.h"
#include "Utilities.h"

//-----------------------------
//USER TYPE DEFINITIONs
//-----------------------------

typedef enum Port {
	A=0,
	 B,
	 C,
	 D
} PORT_t;
typedef enum PullupStatus{
	Pullup_OFF=0,
	 Pullup_ON
}PULLUP_STATUS_t;
typedef enum PINStatus{
	OUT_OFF=0,
	 OUT_ON
}PIN_Status_t;
//macros
#define PIN_0 0b00000001
#define PIN_1 0b00000010
#define PIN_2 0b00000100
#define PIN_3 0b00001000
#define PIN_4 0b00010000
#define PIN_5 0b00100000
#define PIN_6 0b01000000
#define PIN_7 0b10000000

/*
 *============================================
 *============================================
 * APIs Supported by "MCAL GPIO DRIVER"
 *============================================
 *============================================
 *
 */

void INTIAL_PORT_INPUT(PORT_t port,PULLUP_STATUS_t status);
void INTIAL_PORT_OUTPUT(PORT_t port,uint8_t Intial_Value);
void WRITE_PORT(PORT_t port, uint8_t DATA);              
uint8_t READ_PORT(PORT_t port);                            
void TOGGLE_PORT(PORT_t port);

/*PIN Functions Deceleration*/
void INTIAL_PIN_INPUT(PORT_t port,uint8_t PIN_NUM,PULLUP_STATUS_t);
void INTIAL_PIN_OUTPUT(PORT_t port,uint8_t PIN_NUM,PIN_Status_t Intial_val);
void WRITE_PIN(PORT_t port,uint8_t PIN_NUM,uint8_t data);
uint8_t READ_PIN(PORT_t port,uint8_t PIN_NUM);
void TOGGLE_PIN(PORT_t port, uint8_t PIN_NUM);



#endif /* GPIO_H_ */