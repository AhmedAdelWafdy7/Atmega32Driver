/*
 * MemoryMap.h
 *
 * Created: 4/19/2023 3:01:17 AM
 *  Author: Ahmed Adel Wafdy 
 */ 


#ifndef MEMORYMAP_H_
#define MEMORYMAP_H_

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

#define GICR     (*(volatile unsigned char*)0x5B)
#define INT1 7
#define INT0 6
#define INT2 5

#define GIFR    (*(volatile unsigned char*)0x5A)
#define INTF1 7
#define INTF0 6
#define INTF2 5

//-----------------------------
//USART Registers
//-----------------------------

//USART I/O Data Register – UDR
#define UDR		*((volatile unsigned char*)0x2C)

//USART Control and Status Register A –UCSRA
#define UCSRA	*((volatile unsigned char*)0x2B)
//USART Control and Status Register B – UCSRB
#define UCSRB	*((volatile unsigned char*)0x2A)
//USART Control and Status Register C – UCSRC
//The UBRRH Register shares the same I/O location as the UCSRC Register
#define UCSRC	*((volatile unsigned char*)0x40)
#define UBRRH	*((volatile unsigned char*)0x40)
//USART Baud Rate Registers – UBRRL and UBRRH
#define UBRRL	*((volatile unsigned char*)0x29)
//USART BITS MACROS

//UCSRA
// Bit 7 – RXC: USART Receive Complete
#define RXC			7
//Bit 6 – TXC: USART Transmit Complete
#define TXC			6
//Bit 5 – UDRE: USART Data Register Empty
#define UDRE		5
//Bit 4 – FE: Frame Error
#define FE			4
//Bit 3 – DOR: Data OverRun
#define DOR			3
// Bit 2 – PE: Parity Error
#define PE			2
//Bit 1 – U2X: Double the USART Transmission Speed
#define U2X			1
//Bit 0 – MPCM: Multi-processor Communication Mode
#define MPCM		0

//UCSRB
//Bit 7 – RXCIE: RX Complete Interrupt Enable
#define RXCIE			7
// Bit 6 – TXCIE: TX Complete Interrupt Enable
#define TXCIE			6
//Bit 5 – UDRIE: USART Data Register Empty Interrupt Enable
#define UDRIE			5
//Bit 4 – RXEN: Receiver Enable
#define RXEN			4
//Bit 3 – TXEN: Transmitter Enable
#define TXEN			3
//Bit 2 – UCSZ2: Character Size
#define UCSZ2			2
//Bit 1 – RXB8: Receive Data Bit 8
#define RXB8			1
// Bit 0 – TXB8: Transmit Data Bit 8
#define TXB8			0

//UCSRC
// Bit 7 – URSEL: Register Select
#define URSEL			7
//Bit 6 – UMSEL: USART Mode Select
#define UMSEL			6
//Bit 5:4 – UPM1:0: Parity Mode
#define UPM1			5
#define UPM0			4
//Bit 3 – USBS: Stop Bit Select
#define USBS			3
// Bit 2:1 – UCSZ1:0: Character Size
#define UCSZ1			2
#define UCSZ0			1
//Bit 0 – UCPOL: Clock Polarity
#define UCPOL			0

#endif /* MEMORYMAP_H_ */