/*
 * usart.h
 *
 * Created: 4/25/2023 2:29:40 PM
 *  Author: Ahmed Adel Wafdy
 */ 


#ifndef USART_H_
#define USART_H_

//-----------------------------
//INCLUDES
//-----------------------------
#include "MemoryMap.h"
#include "Utilities.h"

//-----------------------------
//USER TYPE DEFINITIONs
//-----------------------------

/*
 *============================================
 *============================================
 * APIs Supported by "MCAL USART DRIVER"
 *============================================
 *============================================
 *
 */

void MCAL_USART_init(void);
void MCAL_USART_Transmit(uint8_t data);
uint8_t MCAL_USART_Receive(void);
void MCAL_USART_Send_String(char* str);
char* MCAL_USART_Receive_String(void);
void MCAL_UART_Send_NumberU32(uint32_t num);
uint32_t MCAL_UART_Receive_NumberU32(void);





#endif /* USART_H_ */