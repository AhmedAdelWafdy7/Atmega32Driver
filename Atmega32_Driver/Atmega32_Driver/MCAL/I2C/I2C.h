/*
 * I2C.h
 *
 * Created: 5/13/2023 1:23:26 AM
 *  Author: Ahmed Adel Wafdy
 */ 


#ifndef I2C_H_
#define I2C_H_

#include "MemoryMap.h"

void MCAL_I2C_Init(unsigned long SCL_Clock);
void MCAL_I2C_Start(void);
void MCAL_I2C_Write(uint8_t Byte);
void MCAL_I2C_Stop(void);



#endif /* I2C_H_ */