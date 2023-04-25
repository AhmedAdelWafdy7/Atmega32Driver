/*
 * Atmega32_Driver.c
 *
 * Created: 4/22/2023 11:34:52 AM
 * Author : Ahmed Adel Wafdy
 */ 

#include "lcd.h"
#include "usart.h"

int main(void)
{
	lcd_init();
	MCAL_USART_init();
	lcd_send_string("usart will send string Benwafdy");
	lcd_clear();
	MCAL_USART_Send_String("Benwafdy");
		lcd_clear();

    /* Replace with your application code */
    while (1) 
    {
	
		 lcd_send_char(MCAL_USART_Receive());
    }
}

