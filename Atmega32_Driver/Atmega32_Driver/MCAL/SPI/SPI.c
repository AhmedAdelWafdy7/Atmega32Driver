/*
 * SPI.c
 *
 * Created: 5/12/2023 7:47:22 PM
 *  Author: Ahmed Adel Wafdy
 */ 

#include "SPI.h"

SPI_Config_t *G_SPI_Config = {NULL};
/* ================================================================
 * @Fn 				- MCAL_SPI_Init
 * @brief 			- Initializes SPIx according to the specified parameters in SPI_Config_t
 * @param [in] 		- Config : a pointer to SPI_Config_t structure that contains the configuration information for the specified SPI Module
 * @retval 			- None
 * Note 			- Supported for SPI FULL DUPLEX Master/Slave only & NSS Hardware/Software
 * 					- You have to configure RCC to select clock for the selected SPI Module
 */	
void MCAL_SPI_Init( SPI_Config_t *Config){
	uint8_t Temp_Reg = 0;
	
	G_SPI_Config = Config;
	
	//Enable SPI
	Temp_Reg |= (1<<6); 
	//Set mode 
	Temp_Reg |= Config->SPI_Mode;
	//Set communication Mode
	Temp_Reg |= Config->Communication_Mode;
	//set clock polarity
	Temp_Reg |= Config->CLK_Polarity;
	//set Clock phase
	Temp_Reg |= Config->CLK_Phase;
	//set frame format
	Temp_Reg |= Config->Frame_Format;
	
	if(Config->CLK_Speed == CLK_SPEED_LOW){
		Temp_Reg |= Config->BaudRate_Prescaler;
	}
	else {
		Temp_Reg |= Config->BaudRate_Prescaler;
		SPI->SPSR.bits.SPI2X =1;
	}
//check if IRQ
	if(IRQ_ENABLE_NONE != Config->IRQ_Enable){
		Enable_G_Interrupt();
		Temp_Reg |= Config->IRQ_Enable;
	}
	else{
		Temp_Reg |= Config->IRQ_Enable;
	}
	SPI->SPCR.SPCR =Temp_Reg;
}
/* ================================================================
 * @Fn 				- MCAL_SPI_DeInit
 * @brief 			- Resets Selected SPI Module
 * @retval 			- None
 * Note 			- Reset The Module By RCC & Disable NVIC
 */
void MCAL_SPI_DeInit(void){
	SPI->SPCR.SPCR =0x00;
		SPI->SPSR.bits.SPI2X =0;
}
/* ================================================================
 * @Fn 				- MCAL_SPI_GPIO_Set_Pins
 * @brief 			- Initializes GPIO Pins to be connected with the selected SPI
 * @param [in] 		- SPIx : where x can be (1..2 depending on device used) to select SPI peripheral
 * @retval 			- None
 * Note 			- Must open clock for AFIO & GPIO After GPIO Initialization
 * 					- Supported for SPI FULL DUPLEX Master/Slave only & NSS Hardware/Software
 */
void MCAL_SPI_GPIO_Set_Pins(void){

	
	/* Master */
	if (G_SPI_Config->SPI_Mode == SPI_MODE_MASTER)
	{
		/* PB7      SCK: */
		INTIAL_PIN_OUTPUT(PORTB,PIN_7,OUT_OFF);
		
		/* PB6      MISO:  */
		INTIAL_PIN_INPUT(PORTB,PIN_6,Pullup_OFF);

		
		/* PB5      MOSI:  */
		INTIAL_PIN_OUTPUT(PORTB,PIN_5,OUT_OFF);

		/* PB4      SS:  */
		INTIAL_PIN_OUTPUT(PORTB,PIN_4,OUT_OFF);

	}
	/* Slave */
	else
	{
		/* PB7      SCK:  */
		INTIAL_PIN_INPUT(PORTB,PIN_7,Pullup_OFF);

		
		/* PB6      MISO: */
		INTIAL_PIN_OUTPUT(PORTB,PIN_6,OUT_OFF);
;
		
		/* PB5      MOSI:  */
		INTIAL_PIN_INPUT(PORTB,PIN_5,Pullup_OFF);

		
		/* PB4      SS: */
		INTIAL_PIN_INPUT(PORTB,PIN_4,Pullup_OFF);

	}
}
/* ================================================================
 * @Fn 				- MCAL_SPI_Send_Data
 * @brief 			- Send Buffer With SPI
 * @param [in] 		- data : Pointer to buffer Which holds the Tx data
 * @param [in] 		- Polling_En : Enable Polling or Disable it
 * @retval 			- None
 * Note 			- None
 *
 */
void MCAL_SPI_Send_Data(uint16_t *data, enum Polling_mechanism Polling_En){
	SPI->SPDR = *data;
	if(Polling_En == SPI_enable){
		while(!(SPI->SPSR.bits.SPIF));
	}
}
/* ================================================================
 * @Fn 				- MCAL_SPI_Recieve_Data
 * @brief 			- Receive Buffer With SPI
 * @param [in] 		- data : Pointer to buffer which holds Rx data
 * @param [in] 		- Polling_En : Enable Polling or Disable it
 * @retval 			- None
 * Note 			- None
 */
void MCAL_SPI_Recieve_Data(uint16_t *data, enum Polling_mechanism Polling_En){
		if(Polling_En == SPI_enable){
			while(!(SPI->SPSR.bits.SPIF));
		}
		
		*data = SPI->SPDR ;
}
/* ================================================================
 * @Fn 				- MCAL_SPI_TX_RX
 * @brief 			- Transmit and Receive Data
 * @param [in] 		- data : Pointer to buffer which holds Transmit and Received data
 * @param [in] 		- Polling_En : Enable Polling or Disable it
 * @retval 			- None
 * Note 			- None
 */
void MCAL_SPI_TX_RX(uint16_t *data, enum Polling_mechanism Polling_En){
		SPI->SPDR = *data;
		if(Polling_En == SPI_enable){
			while(!(SPI->SPSR.bits.SPIF));
		}
	if(Polling_En == SPI_enable){
		while(!(SPI->SPSR.bits.SPIF));
	}

	*data = SPI->SPDR ;
}

ISR(SPI_STC_vect)
{
	G_SPI_Config->P_IRQ_CallBack();
}