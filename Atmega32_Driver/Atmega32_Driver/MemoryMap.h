/*
 * MemoryMap.h
 *
 * Created: 4/19/2023 3:01:17 AM
 *  Author: Ahmed Adel Wafdy 
 */ 


#ifndef MEMORYMAP_H_
#define MEMORYMAP_H_
#include "stdint.h"

#define IO_MAPPING_OFFSET	0x20
#define FLASH_MEMORY		0x00
#define SRAM				0x60

#define TIMER0_Base			0x23
#define TIFR_Base			0x36
#define TIMSK_Base			0x37

#define USART_Base			0x09	
#define UCSRC_Base			0x20	
#define UBRRH_Base			0x20	

#define SPI_Base			0x0D	

//I2C
#define TWBR (*(volatile unsigned char*)0x20)
#define TWSR (*(volatile unsigned char*)0x21)
#define TWAR (*(volatile unsigned char*)0x22)
#define TWDR (*(volatile unsigned char*)0x23)
#define TWCR     (*(volatile unsigned char*)0x56)

#define TWINT		7
#define TWEA		6
#define TWSTA		5
#define TWSTO		4
#define TWWC		3
#define TWEN		2


//GPIO Registers
#define DDRA	*((volatile unsigned char*)0x3A)
#define PORTA	*((volatile unsigned char*)0x3B)
#define PINA	*((volatile unsigned char*)0x39)

#define DDRB	*((volatile unsigned char*)0x37)
#define PORTB	*((volatile unsigned char*)0x38)
#define PINB	*((volatile unsigned char*)0x36)

#define DDRC	*((volatile unsigned char*)0x34)
#define PORTC	*((volatile unsigned char*)0x35)
#define PINC	*((volatile unsigned char*)0x33)

#define DDRD	*((volatile unsigned char*)0x31)
#define PORTD	*((volatile unsigned char*)0x32)
#define PIND	*((volatile unsigned char*)0x30)

//External Interrupt 
#define MCUCSR   (*(volatile unsigned char*)0x54)
#define ISC2 6

#define MCUCR   (*(volatile unsigned char*)0x55)
#define ISC00 0
#define ISC01 1
#define ISC10 2
#define ISC11 3
#define GICR    (*(volatile unsigned char*)0x5B)
#define GIFR	(*(volatile unsigned char*)0x5A)

//ADC

#define ADMUX	(*(volatile unsigned char*)0x27)
#define ADCSRA	(*(volatile unsigned char*)0x26)
#define SFIOR	(*(volatile unsigned char*)0x50)
#define ADCL	(*(volatile unsigned char*)0x24)
#define ADCH	(*(volatile unsigned char*)0x25)


	

typedef struct
{
	volatile uint8_t OCR0_;		
	
	
	volatile uint8_t TCNT0_;		
	
	
	volatile union
	{
		volatile uint8_t TCCR0_;	
		struct
		{
			volatile uint8_t CS0n_	    	    :3;		
			volatile uint8_t WGM01_			:1;		
			volatile uint8_t COM0n_				:2;		
			volatile uint8_t WGM00_				:1;		
			volatile uint8_t FOC0_				:1;		
		}bits;
	}TCCR0_;
	
}TIMER0_t;

#define TIMSK			(*(volatile uint8_t*)(TIMSK_Base + IO_MAPPING_OFFSET))

#define TOIE0			0	
#define OCIE0			1

#define TIFR			(*(volatile uint8_t*)(TIFR_Base + IO_MAPPING_OFFSET))

#define TOV0			0	/* Timer/Counter0 Overflow Flag */
#define OCF0			1	/* Output Compare Flag 0 */

#define TIMER0			((TIMER0_t*) (TIMER0_Base + IO_MAPPING_OFFSET))


//-----------------------------
//USART Registers
//-----------------------------

typedef struct{
	
	volatile uint8_t UBRRL;		
	
	volatile union {
		volatile uint8_t UCSRB;
		struct {
			volatile uint8_t TXB8	  :1;		
			volatile uint8_t RXB8 	  :1;		
			volatile uint8_t UCSZ2   :1;	
			volatile uint8_t TXEN   :1;		
			volatile uint8_t RXEN   :1;		
			volatile uint8_t UDRIE   :1;	
			volatile uint8_t TXCIE   :1;		
			volatile uint8_t RXCIE   :1;		
		}bits;
	}UCSRB;
		
	volatile union
	{
		volatile uint8_t UCSRA_;		
		struct
		{
			volatile uint8_t MPCM	    :1;		
			volatile uint8_t U2X		:1;		
			volatile uint8_t PE		:1;		
			volatile uint8_t DOR		:1;		
			volatile uint8_t FE	:1;		
			volatile uint8_t UDRE	    :1;		
			volatile uint8_t TXC		:1;		
			volatile uint8_t RXC		:1;	
		}bits;
	}UCSRA;
	
	
	volatile uint8_t UDR;			
	
}USART_t;
#define USART			((USART_t*) (USART_Base + IO_MAPPING_OFFSET))

#define UCSRC			(*(volatile uint8_t*)(UCSRC_Base + IO_MAPPING_OFFSET))

#define UCPOL			0	
#define UCSZ0			1	
#define UCSZ1			2	
#define USBS			3	
#define UPM0			4	
#define UPM1			5	
#define UMSEL			6	
#define URSEL			7	

#define UBRRH			(*(volatile uint8_t*)(UBRRH_Base + IO_MAPPING_OFFSET))

//SPI
typedef struct
{
	volatile union
	{
		volatile uint8_t SPCR;	
		struct
		{
			volatile uint8_t SPR0	    :1;		
			volatile uint8_t SPR1	:1;	
			volatile uint8_t CPHA		:1;		
			volatile uint8_t CPOL		:1;		
			volatile uint8_t MSTR		:1;		
			volatile uint8_t DORD	    :1;		
			volatile uint8_t SPE		:1;		
			volatile uint8_t SPIE		:1;		
		}bits;
	}SPCR;
		
	volatile union
	{
		volatile uint8_t SPSR;		
		struct
		{
			volatile uint8_t SPI2X   :1;	
			volatile uint8_t Reversed	:5;	
			volatile uint8_t WCOL	:1;	
			volatile uint8_t SPIF		:1;		
		}bits;
	}SPSR;
	volatile uint8_t SPDR;		
}SPI_t;

#define SPI				((SPI_t*) (SPI_Base + IO_MAPPING_OFFSET))

/*///////////////////////////////////////////////
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
///////////// interrupt functions \\\\\\\\\\\\\\\\
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
////////////////////////////////////////////////////*/
#define ISR(vector,...)            \
void vector (void) __attribute__ ((signal))__VA_ARGS__ ; \
void vector (void)

/* USART, Rx Complete */
#define USART_RXC_vect			__vector_13
/* USART Data Register Empty */
#define USART_UDRE_vect			__vector_14
/* USART, Tx Complete */
#define USART_TXC_vect			__vector_15
/* Serial Transfer Complete */
#define SPI_STC_vect			__vector_12
#define SREG_Base		0x3F
#define SREG			(*(uint8_t*)(SREG_Base + IO_MAPPING_OFFSET))
#define I_Bit			7

#define Enable_G_Interrupt()	SREG |= (1 << I_Bit)
#define Disable_G_Interrupt()	SREG &= ~(1 << I_Bit)

#endif /* MEMORYMAP_H_ */