/*
 * Atmega32_Driver.c
 *
 * Created: 4/22/2023 11:34:52 AM
 * Author : Ahmed Adel Wafdy
 */ 

#include "ADC.h"
#include "lcd.h"
#define F_CPU 8000000UL

uint32_t ADC_Data;
uint32_t Temp;

int main(void)
{
	lcd_init();
	ADC_Config_t adc_cfg;
	adc_cfg.ADC_Mode =ADC_MODE_SINGLE_CONVERSION;
	adc_cfg.ADC_Prescaler = ADC_PRESCALER_64;
	adc_cfg.ADC_Res_presentation = ADC_RES_PRESENTATION_LEFT_ADJUSTED;
	adc_cfg.ADC_Volt_Ref = ADC_VOLT_REF_AVCC;
	adc_cfg.ADC_IRQ_Enable = ADC_IRQ_ENABLE_NONE;
	adc_cfg.P_CallBack = NULL;
	
	MCAL_ADC_Init(&adc_cfg);
	while(1){
	MCAL_ADC_Get_Result(ADC1,&ADC_Data,ADC_ENABLE);
	Temp  =(((ADC_Data *5000) / 1024) /10);
	lcd_send_string("Temp: ");
	lcd_send_number(Temp);
	lcd_send_string(" C");
	_delay_ms(1000);
	lcd_clear();
	}
}

