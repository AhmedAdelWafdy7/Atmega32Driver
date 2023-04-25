/*
 * usart.c
 *
 * Created: 4/25/2023 2:29:57 PM
 *  Author: Ahmed Adel wafdy
 */ 
#include "usart.h"

void MCAL_USART_init(void){
	//setting the baud rate
	UBRRL = 51;
	UBRRH = 0 ;
	//This bit selects between Asynchronous and Synchronous mode of operation 0 ====>Async
	CLEAR_BIT(UCSRC,UMSEL);
	
	
	/*Bit 1 – U2X: Double the USART Transmission Speed
	This bit only has effect for the asynchronous operation. Write this bit to zero when using synchronous operation.*/
	CLEAR_BIT(UCSRA,U2X);
	
	// setting frame format

/*Bit 5:4 – UPM1:0: Parity Mode
These bits enable and set type of parity generation and check.*/
	CLEAR_BIT(UCSRC,UPM0);
	CLEAR_BIT(UCSRC,UPM1);
	
	
	//This bit selects between accessing the UCSRC or the UBRRH Register
	SET_BIT(UCSRC,URSEL);
	
	//This bit selects the number of Stop Bits to be inserted by the Transmitter 0===> 1stop bit
	CLEAR_BIT(UCSRC,USBS);
	
	//The UCSZ1:0 bits combined with the UCSZ2 bit in UCSRB sets the number of data bits (Character Size) in a frame the Receiver and Transmitter use.
	// i set it as 8bit data
	SET_BIT(UCSRC,UCSZ1);
	SET_BIT(UCSRC,UCSZ0);
	CLEAR_BIT(UCSRB,UCSZ2);
	
	// enabling the	Transmitter or the Receiver depending on the usage
	
	//Writing this bit to one enables the USART Transmitter. 
	SET_BIT(UCSRB,TXEN);
	//Writing this bit to one enables the USART Receiver
	SET_BIT(UCSRB,RXEN);
}
void MCAL_USART_Transmit(uint8_t data){
	while(!(READ_BIT(UCSRA,UDRE)) );
	/*send data*/
	UDR = data;
}
uint8_t MCAL_USART_Receive(void){
	//This flag bit is set when there are unread data in the receive buffer and cleared when the receive buffer is empty
	while(!(READ_BIT(UCSRA,RXC)));
	return UDR;
}

void MCAL_USART_Send_String(char* str){
	while(*str != '\0'){
		MCAL_USART_Transmit(*str);
		str++;
	}
}
char* MCAL_USART_Receive_String(void){
	char data = MCAL_USART_Receive();
	char str[50];
	char counter = 0;
	while(data != '#'){
		str[counter] = data;
		data = MCAL_USART_Receive();
		counter++;
	} 
	str[counter] = '\0';
	counter = 0;
	return str;
}


void MCAL_UART_Send_NumberU32(uint32_t num)
{


	uint8_t  *ptr = (uint8_t*)&num;
	MCAL_USART_Transmit(ptr[0]);
	MCAL_USART_Transmit(ptr[1]);
	MCAL_USART_Transmit(ptr[2]);
	MCAL_USART_Transmit(ptr[3]);
}

uint32_t MCAL_UART_Receive_NumberU32(void)
{


	uint32_t num;
	uint8_t *p = (uint8_t*)&num;
	p[0] = MCAL_USART_Receive();
	p[1] = MCAL_USART_Receive();
	p[2] = MCAL_USART_Receive();
	p[3] = MCAL_USART_Receive();
	return num;
}