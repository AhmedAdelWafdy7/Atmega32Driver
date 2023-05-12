/*
 * SPI.h
 *
 * Created: 5/12/2023 7:45:21 PM
 *  Author: Ahmed Adel Wafdy
 */ 


#ifndef SPI_H_
#define SPI_H_

//-----------------------------
//INCLUDES
//-----------------------------
#include "MemoryMap.h"
#include "GPIO.h"

typedef struct
{
	uint16_t 	SPI_Mode;					// specifies SPI MODE Master/Slave
	// This parameter must be set based on @ref SPI_MODE_DEFINE

	uint16_t	Communication_Mode;			// specifies SPI Communication mode
	// This parameter must be set based on @ref COMMUNICATION_MODE_DEFINE

	uint16_t	Frame_Format;				// specifies Data MSB transmit first or LSB transmit first
	// This parameter must be set based on @ref FRAME_FORMAT_DEFINE

	uint16_t	CLK_Polarity;				// specifies SPI clock Polarity idle on low or idle on high
	// This parameter must be set based on @ref CLK_POLARITY_DEFINE

	uint16_t	CLK_Phase;					// specifies SPI clock Phase,first clock capture or second clock capture
	// This parameter must be set based on @ref CLK_PHASE_DEFINE

	uint16_t	CLK_Speed;					// specifies SPI clock Speed,Low or high
	// This parameter must be set based on @ref CLK_SPEED_DEFINE
	
	uint16_t	BaudRate_Prescaler;		    // specifies SPI clock BaudRate Prescaler
	// This parameter must be set based on @ref BAUDERATE_PRESCALER_DEFINE

	uint16_t	IRQ_Enable;					// specifies SPI Interrupt Requests
	// This parameter must be set based on @ref IRQ_ENABLE_DEFINE

	void (* P_IRQ_CallBack)(void);	//set the C Function which will be called once IRQ Happens


}SPI_Config_t;

//-----------------------------
//MACROS CONFIGURATION REFERENCES
//-----------------------------

//@ref SPI_MODE_DEFINE
/*Bit 4 – MSTR: Master/Slave Select
This bit selects Master SPI mode when written to one, and Slave SPI mode when written logic zero. 
If SS is configured as an input and is driven low while MSTR is set, MSTR will be cleared,
and SPIF in SPSR will become set. The user will then have to set MSTR to re-enable SPI Master mode.*/
#define SPI_MODE_MASTER						(uint8_t)(1<<4)
#define SPI_MODE_SLAVE						(uint8_t)(0)


//@ref COMMUNICATION_MODE_DEFINE
#define COMMUNICATION_MODE_FULL_DUPLEX			(uint8_t) (0)
#define COMMUNICATION_MODE_3_WIRE				(uint8_t) (0)

//@ref FRAME_FORMAT_DEFINE
/*Bit 5 – DORD: Data Order
When the DORD bit is written to one, the LSB of the data word is transmitted first.
When the DORD bit is written to zero, the MSB of the data word is transmitted first.*/
#define FRAME_FORMAT_LSB				(uint8_t) (1<<5)
#define FRAME_FORMAT_MSB				(uint8_t) (0)

//@ref CLK_POLARITY_DEFINE
/*Bit 3 – CPOL: Clock Polarity
When this bit is written to one, SCK is high when idle.
 When CPOL is written to zero, SCK is low
when idle. The CPOL functionality is summarized below:*/
#define CLK_POLARITY_RISING				(uint8_t) (0)
#define CLK_POLARITY_FALLING			(uint8_t) (1<<3)

//@ref CLK_PHASE_DEFINE
/* Bit 2 – CPHA: Clock Phase
The settings of the Clock Phase bit (CPHA) determine if data is sampled on the leading (first) or
trailing (last) edge of SCK.*/
#define CLK_PHASE_SAMPLE_SETUP				(uint8_t) (0)
#define CLK_PHASE_SETUP_SAMPLE				(uint8_t) (1<<2)

//@ref CLK_SPEED_DEFINE
#define CLK_SPEED_LOW							(uint8_t) (0)
#define CLK_SPEED_HIGH							(uint8_t) (0)

//@ref BAUDERATE_PRESCALER_DEFINE
#define BAUDERATE_PRESCALER_4					(uint8_t) (0<<0)
#define BAUDERATE_PRESCALER_16					(uint8_t) (1<<0)
#define BAUDERATE_PRESCALER_64					(uint8_t) (2<<0)
#define BAUDERATE_PRESCALER_128					(uint8_t) (3<<0)

/* SPSR.Bit 0 – SPI2X:?Double SPI Speed Bit */
#define BAUDERATE_PRESCALER_DOUBLE_SPEED_2		(uint8_t) (0<<0)
#define BAUDERATE_PRESCALER_DOUBLE_SPEED_8		(uint8_t) (1<<0)
#define BAUDERATE_PRESCALER_DOUBLE_SPEED_32		(uint8_t) (2<<0)
#define BAUDERATE_PRESCALER_DOUBLE_SPEED_64		(uint8_t) (3<<0)


//@ref IRQ_ENABLE_DEFINE
#define IRQ_ENABLE_NONE							(uint8_t) (0)	
#define IRQ_ENABLE								(uint8_t) (1<<7)

enum Polling_mechanism
{
	SPI_disable,
	SPI_enable
};

/*
 *============================================
 *============================================
 * APIs Supported by "MCAL SPI DRIVER"
 *============================================
 *============================================
 *
 */

void MCAL_SPI_Init( SPI_Config_t *Config);
void MCAL_SPI_DeInit(void);

void MCAL_SPI_GPIO_Set_Pins(void);

void MCAL_SPI_Send_Data(uint16_t *data, enum Polling_mechanism Polling_En);
void MCAL_SPI_Recieve_Data(uint16_t *data, enum Polling_mechanism Polling_En);

void MCAL_SPI_TX_RX(uint16_t *data, enum Polling_mechanism Polling_En);

#endif /* SPI_H_ */