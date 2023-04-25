/*
 * Utilities.h
 *
 * Created: 4/19/2023 3:01:01 AM
 *  Author: Ahmed Adel Wafdy
 */ 


#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <stdio.h>
#include <stdint.h>

#define SET_BIT(reg,bit)		(reg|=(1<<bit))
#define TOGGLE_BIT(reg,bit)		(reg^=(1<<bit))
#define CLEAR_BIT(reg,bit)		(reg&=~(1<<bit))
#define READ_BIT(reg,bit)		((reg>>bit)&1)




#endif /* UTILITIES_H_ */