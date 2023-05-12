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
#include "GPIO.h"

//-----------------------------
//USER TYPE DEFINITIONs
//-----------------------------
typedef struct
{
	uint16_t	USART_Mode;			//This bit enables the transmitter or This bit enables the receiver
	// this parameter must be based on @ref USART_Mode_define
	uint32_t	BaudRate;			//specifies Baud rate value
	// this parameter must be based on @ref USART_BaudRate_define

	uint16_t	Payload_Length;		// Specifies the number of data bits transmitted or received in a frame
	// This parameters must be set based on @Ref UART_Payload_Length_define

	uint16_t	Parity;				// Specifies the parity mode
	// This parameters must be set based on @Ref UART_Parity_define

	uint16_t	StopBits;			// Specifies the number of stop bits
	// This parameters must be set based on @Ref UART_StopBits_define

	uint16_t	IRQ_Enable;			// enable or disable the interrupt
	// This parameters must be set based on @Ref UART_IRQ_define

	void (*P_IRQ_CallBack)(void);	// Set the C Function() which will be called once the IRQ Happen
}USART_Config_t;

//@ref USART_Mode_define
#define USART_Mode_Async				(uint8_t)(0<<6)
#define USART_Mode_Sync					(uint8_t)(1<<6)

//@ref USART_BaudRate_define
#define USART_BaudRate_2400				207
#define USART_BaudRate_4800				103
#define USART_BaudRate_9600				51
#define USART_BaudRate_14400			34
#define USART_BaudRate_19200			25
#define USART_BaudRate_28800			16
#define USART_BaudRate_38400			12
#define USART_BaudRate_57600			8
#define USART_BaudRate_76800			6
#define USART_BaudRate_115200			3
#define USART_BaudRate_230400			1
#define USART_BaudRate_250000			1

//@Ref UART_Payload_Length_define
/* Bit 2:1 – UCSZ1:0: Character Size
The UCSZ1:0 bits 
combined with the UCSZ2 bit in UCSRB sets the number of data bits (Character Size)
 in a frame the Receiver and Transmitter use.*/
#define UART_Payload_Length_5			0
#define UART_Payload_Length_6			1
#define UART_Payload_Length_7			2
#define UART_Payload_Length_8			3
#define UART_Payload_Length_9			4

//@Ref UART_Parity_define
/*Bit 5:4 – UPM1:0: Parity Mode
These bits enable and set type of parity generation and check. If enabled, the transmitter will
automatically generate and send the parity of the transmitted data bits within each frame. The
Receiver will generate a parity value for the incoming data and compare it to the UPM0 setting.
If a mismatch is detected, the PE Flag in UCSRA will be set*/

#define UART_Parity_None				0
#define UART_Parity_Even				1
#define UART_Parity_Odd					2

//@Ref UART_StopBits_define
/*Bit 3 – USBS: Stop Bit Select
This bit selects the number of Stop Bits to be inserted by the Transmitter. The Receiver ignores
this setting*/
#define UART_StopBits_1					0
#define UART_StopBits_2					1

//@Ref UART_IRQ_define
/*Bit 7 – RXCIE: RX Complete Interrupt Enable
Writing this bit to one enables interrupt on the RXC Flag.
 A USART Receive Complete Interrupt
will be generated only if the RXCIE bit is written to one,
 the Global Interrupt Flag in SREG is written to one and the RXC bit in UCSRA is set.

• Bit 6 – TXCIE: TX Complete Interrupt Enable
Writing this bit to one enables interrupt on the TXC Flag. 
A USART Transmit Complete Interrupt
will be generated only if the TXCIE bit is written to one,
 the Global Interrupt Flag in SREG is written to one and the TXC bit in UCSRA is set.*/
#define UART_IRQ_None					0
#define UART_IRQ_Transmiter				1
#define UART_IRQ_Receiver				2
#define UART_IRQ_Transmiter_Receiver	3


enum Polling_mechanism
{
	USART_disable,
	USART_enable
};

/*
 *============================================
 *============================================
 * APIs Supported by "MCAL USART DRIVER"
 *============================================
 *============================================
 *
 */

void MCAL_USART_Init(USART_Config_t *config);
void MCAL_USART_DeInit(void);

void MCAL_USART_Transmit(uint8_t data,enum Polling_mechanism pollingEn);
uint16_t MCAL_USART_Receive(enum Polling_mechanism pollingEn);
void MCAL_USART_Send_String(char* str);
void MCAL_USART_Receive_String(char* str);






#endif /* USART_H_ */