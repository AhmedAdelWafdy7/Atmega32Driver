/*
 * usart.c
 *
 * Created: 4/25/2023 2:29:57 PM
 *  Author: Ahmed Adel wafdy
 */ 
#include "usart.h"
//#include "avr/interrupt.h"

void (*GP_IRQ_CallBack)(void);

/* ================================================================
 * @Fn				- MCAL_UART_Init
 * @brief			- Initialize UART Asynchronous  only
 * @param [in] 		- USARTx: where x can be (1..3 depending on device used)
 * @param [in] 		- UART_Config: All UART configuration
 * @retval 			- none
 * Note				- support for now Asynchronous mode & clock 8 MHz
 */
void MCAL_USART_Init(USART_Config_t *config){
	//setting the baud rate
	USART->UBRRL = config->BaudRate;
	//This bit selects between Asynchronous and Synchronous mode of operation 0 ====>Async
	if(config->USART_Mode == USART_Mode_Async){
		UCSRC |= (1<<URSEL);
		UCSRC &= ~(1<<UMSEL);
	}
	else if(config->USART_Mode == USART_Mode_Sync){
		UCSRC |= (1<<URSEL);
		UCSRC |= (1<<UMSEL);
	}
	/*Bit 1 – U2X: Double the USART Transmission Speed
	This bit only has effect for the asynchronous operation. Write this bit to zero when using synchronous operation.*/
	USART->UCSRA.bits.U2X = 0;
	
	// setting frame format

/*Bit 5:4 – UPM1:0: Parity Mode
These bits enable and set type of parity generation and check.*/
	if(config->Parity == UART_Parity_None){
		UCSRC |= (1<<URSEL);
		UCSRC &= ~(1<<UPM0);
		UCSRC &= ~(1<<UPM1);
	}
	else if(config->Parity == UART_Parity_Even){
		UCSRC |= (1<<URSEL);
		UCSRC &= ~(1<<UPM0);
		UCSRC |= (1<<UPM1);
	}
	else if(config->Parity == UART_Parity_Odd){
		UCSRC |= (1<<URSEL);
		UCSRC |= (1<<UPM0);
		UCSRC |= (1<<UPM1);
	}
	
	
	//This bit selects the number of Stop Bits to be inserted by the Transmitter 0===> 1stop bit
		if(config->StopBits == UART_StopBits_1){
			UCSRC |= (1<<URSEL);
			UCSRC &= ~(1<<USBS);
		}
		else if(config->StopBits == UART_StopBits_2){
			UCSRC |= (1<<URSEL);
			UCSRC |= (1<<USBS);
		}

	
	//The UCSZ1:0 bits combined with the UCSZ2 bit in UCSRB sets the number of data bits (Character Size) in a frame the Receiver and Transmitter use.
		switch(config->Payload_Length){
			case UART_Payload_Length_5:
				UCSRC |= (1<<URSEL);
				UCSRC &= ~(1<<UCSZ0);
				UCSRC &= ~(1<<UCSZ1);
				USART->UCSRB.bits.UCSZ2 = 0;
				break;
			case UART_Payload_Length_6:
				UCSRC |= (1<<URSEL);
				UCSRC |= (1<<UCSZ0);
				UCSRC &= ~(1<<UCSZ1);
				USART->UCSRB.bits.UCSZ2 = 0;
				break;	
			case UART_Payload_Length_7:
				UCSRC |= (1<<URSEL);
				UCSRC &= ~(1<<UCSZ0);
				UCSRC |= (1<<UCSZ1);
				USART->UCSRB.bits.UCSZ2 = 0;
				break;
			case UART_Payload_Length_8:
				UCSRC |= (1<<URSEL);
				UCSRC |= (1<<UCSZ0);
				UCSRC |= (1<<UCSZ1);
				USART->UCSRB.bits.UCSZ2 = 0;
				break;
			case UART_Payload_Length_9:
				UCSRC |= (1<<URSEL);
				UCSRC |= (1<<UCSZ0);
				UCSRC |= (1<<UCSZ1);
				USART->UCSRB.bits.UCSZ2 = 1;
				break;					
		
		}
	// enabling the	Transmitter or the Receiver depending on the usage
	if(config->IRQ_Enable != UART_IRQ_None){
		if(config->IRQ_Enable == UART_IRQ_Transmiter){
			USART->UCSRB.bits.RXCIE = 0;
			USART->UCSRB.bits.TXCIE = 1;
			GP_IRQ_CallBack = config->P_IRQ_CallBack;
			Enable_G_Interrupt(); 
		}
		if(config->IRQ_Enable == UART_IRQ_Receiver){
			USART->UCSRB.bits.RXCIE = 1;
			USART->UCSRB.bits.TXCIE = 0;
			GP_IRQ_CallBack = config->P_IRQ_CallBack;
			Enable_G_Interrupt();
		}
		if(config->IRQ_Enable == UART_IRQ_Transmiter_Receiver){
			USART->UCSRB.bits.RXCIE = 1;
			USART->UCSRB.bits.TXCIE = 1;
			GP_IRQ_CallBack = config->P_IRQ_CallBack;
			Enable_G_Interrupt();
		}				
	}
	else {
			USART->UCSRB.bits.RXCIE = 0;
			USART->UCSRB.bits.TXCIE = 0;
					GP_IRQ_CallBack = NULL;
					Disable_G_Interrupt();
	}
	//Writing this bit to one enables the USART Transmitter. 
	USART->UCSRB.bits.TXEN = 1;
	//Writing this bit to one enables the USART Receiver
	USART->UCSRB.bits.RXEN = 1;
}
/* ================================================================
 * @Fn				- MCAL_UART_DeInit
 * @brief			- DEInit UART Asynchronous only
 * @retval 			- none
 * Note				- Initialize UART first
 */
void MCAL_USART_DeInit(void){
	while (!(USART->UCSRA.bits.UDRE));
	USART->UBRRL= 0x00;
	UBRRH = 0x00;
	UCSRC = 0x86;
	USART->UCSRB.UCSRB &= ~(0x02);
	USART->UCSRA.bits.MPCM = 0b0;
	USART->UCSRA.bits.U2X = 0b0;
	USART->UCSRA.bits.TXC = 0b0; 
}
/**===============================================
 * @Fn			- MCAL_USART_Transmit
 * @brief		- The transmitter can send data words of either 8 or 9 bits depending on the M bit status.
					When the transmit enable bit (TE) is set
 * @param[in]	- USARTx: Where x can be (1...3) to select the USART peripheral
 * @param[in]	- pTxBuffer: data buffer which will store in registers and transmit
 * @param[in]	- PollingEn: Polling Enable / Disable
 * @retval		- void ====> none
 * Note			- (Supported feature ASYNCH. Only) & Clock 8 MHZ S
 * 	First :
 * 	1. Enable the USART by writing the UE bit in USART_CR1 register to 1.
	2. Program the M bit in USART_CR1 to define the word length.
	3. Program the number of stop bits in USART_CR2.
	4. Select DMA enable (DMAT) in USART_CR3 if Multi buffer Communication is to takeplace. Configure the DMA register as explained in multibuffer communication.
	5. Select the desired baud rate using the USART_BRR register.
	6. Set the TE bit in USART_CR1 to send an idle frame as first transmission.
 * */
void MCAL_USART_Transmit(uint8_t data,enum Polling_mechanism pollingEn){
	if(pollingEn == USART_enable){
		while(!(USART->UCSRA.bits.UDRE) );
	}
	if(USART->UCSRB.bits.UCSZ2){
		USART->UCSRB.bits.TXB8 = ((data & (1<<8))>>8);
		USART->UDR = (uint8_t)data;
	}
	/*send data*/
	else{
		USART->UDR = (uint8_t)data;
	}
}
/**===============================================
 * @Fn			- MCAL_USART_Receive
 * @brief		- The USART can receive data words of either 8 or 9 bits depending on the M bit in the
					USART_CR1 register
 * @param[in]	- USARTx: Where x can be (1...3) to select the USART peripheral
 * @param[in]	- pRxBuffer: data buffer which will store in registers and transmit
 * @param[in]	- PollingEn: Polling Enable / Disable
 * @retval		- void ====> none
 * Note			- (Supported feature ASYNCH. Only) & Clock 8 MHZ S
 *  First:
 *  1. Enable the USART by writing the UE bit in USART_CR1 register to 1.
	2. Program the M bit in USART_CR1 to define the word length.
	3. Program the number of stop bits in USART_CR2.
	4. Select DMA enable (DMAR) in USART_CR3 if multibuffer communication is to take
	place. Configure the DMA register as explained in multibuffer communication. STEP 3
	5. Select the desired baud rate using the baud rate register USART_BRR
	6. Set the RE bit USART_CR1. This enables the receiver which begins searching for a start bit
 * */
uint16_t MCAL_USART_Receive(enum Polling_mechanism pollingEn){
	uint16_t data;
	//This flag bit is set when there are unread data in the receive buffer and cleared when the receive buffer is empty
	if(pollingEn == USART_enable){
		while(!(USART->UCSRA.bits.RXC) );
	}
	if(USART->UCSRB.bits.UCSZ2){
		data = ((USART->UCSRB.bits.RXB8)<<8);
		data = USART->UDR;
	}
	/*send data*/
	else{
		data = USART->UDR;
	}
	return data;
}

void MCAL_USART_Send_String(char* str){
	while(*str != '\0'){
		while(!(USART->UCSRA.bits.UDRE) );
		if(USART->UCSRB.bits.UCSZ2){
			USART->UCSRB.bits.TXB8 = 0;
			USART->UDR = (uint8_t)*str;
		}
		else{
			USART->UDR = *str;
			str++;
		}
	}
	while(!(USART->UCSRA.bits.UDRE) );
	
	USART->UDR = '\0';
}
void MCAL_USART_Receive_String(char* str){
	while(1)
	{
		while(!(USART->UCSRA.bits.RXC));
		
		*str = USART->UDR;
		
		if(*str == '\0')
		{
			break;
		}
		else
		{
			str++;
		}
	}
}

ISR(USART_RXC_vect)
{
	GP_IRQ_CallBack();
}

ISR(USART_TXC_vect)
{
	GP_IRQ_CallBack();
}