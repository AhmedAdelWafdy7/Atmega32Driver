/*
 * lcd.c
 *
 * Created: 4/22/2023 8:17:40 AM
 *  Author: Ahmed Adel Wafdy
 */ 

#include "lcd.h"

void lcd_is_busy(void)
{
	LCD_DATA_dir_PORT &= ~(0xff<<DATA_shift);
	LCD_CTRL |= (1<<RW); //read
	LCD_CTRL  &= ~(1<<RS);
	
	lcd_kick();
	LCD_DATA_dir_PORT = 0xff;
	LCD_CTRL &= ~(1<<RW); //write
}
void lcd_kick(void)
{
	LCD_CTRL &= ~(1<<EN); 
	asm volatile ("nop");
	asm volatile ("nop");
	_delay_ms(50);
	LCD_CTRL |= (1<<EN); 

}
void lcd_send_command(unsigned char command)
{
	#ifdef EIGHT_BIT_MODE 
		lcd_is_busy();
		LCD_PORT = command;
		LCD_CTRL &= ~(1<<RW) | (1<<RS);
		lcd_kick();
		
#endif
	#ifdef FOUR_BIT_MODE
		lcd_is_busy();
		LCD_PORT = (LCD_PORT & 0x0F) | (command &0xF0);	
		LCD_CTRL &= ~(1<<RW) | (1<<RS);
		lcd_kick();
		LCD_PORT = (LCD_PORT & 0x0F) | (command << 4);
		LCD_CTRL &= ~(1<<RW) | (1<<RS);
		lcd_kick();
#endif
	
}
void lcd_init(void)
{
	_delay_ms(20);
	LCD_CTRL_dir_PORT |= (1<<EN | 1<<RW |1<<RS );
	LCD_CTRL &= ~(1<<EN | 1<<RW |1<<RS );
	LCD_DATA_dir_PORT = 0xFF;
	_delay_ms(20);
	
	lcd_clear();
	#ifdef EIGHT_BIT_MODE
		lcd_send_command(LCD_8BIT_MODE_2_LINE);
#endif

	#ifdef FOUR_BIT_MODE
		lcd_send_command(0x02);
		lcd_send_command(LCD_4BIT_MODE_2_LINE);
	#endif
	lcd_send_command(LCD_ENTERY_MODE_INC_SHIFT_OFF);
	lcd_send_command(LCD_BEGIN_AT_FIRST_RAW);
	lcd_send_command(LCD_DISPLAY_ON_UNDERLINE_ON_CURSOR_ON);
}
void lcd_send_char(unsigned char The_char)
{
		#ifdef EIGHT_BIT_MODE
			lcd_is_busy();
			LCD_PORT = The_char;
			LCD_PORT =((The_char)<< DATA_shift);
			LCD_CTRL |= 1<<RS;
			LCD_CTRL &= ~(1<<RW);
			lcd_kick();
		#endif

		#ifdef FOUR_BIT_MODE
			LCD_PORT = (LCD_PORT & 0x0F) | (The_char &0xF0);
			LCD_CTRL |= 1<<RS;
			LCD_CTRL &= ~(1<<RW);
			lcd_kick();
			LCD_PORT = (LCD_PORT & 0x0F) | (The_char << 4);
			
			LCD_CTRL |= 1<<RS;
			LCD_CTRL &= ~(1<<RW);
			lcd_kick();
		#endif
}
void lcd_send_string(unsigned char* The_char)
{
	int count=0;
	while(*The_char){
		lcd_send_char(*The_char++);
		count++;
		if(count == 16){
			lcd_send_command(LCD_BEGIN_AT_SECOND_RAW);
		}
		else if( count == 32 || count == 33){
			lcd_clear();
			lcd_send_command(LCD_BEGIN_AT_FIRST_RAW);
			count = 0;
		}
	}
	
}
void lcd_set_position(unsigned char line ,unsigned char pos)
{
	if(line == 0){
		if(pos <16 && pos>=0){
			lcd_send_command(0x80+pos);
		}else if(line == 1){
			if(pos <16 && pos>=0){
				lcd_send_command(0x80+pos);
				}
		}
	}
	
}
void lcd_clear(void)
{
	lcd_send_command(LCD_CLEAR);
	}
void lcd_send_number(int number)
{
	char str[7];
	sprintf(str,"%d",number);
	lcd_send_string(str);	
	
}
void lcd_send_real_number(float number)
{
		char str[16];
		char *tmpSign = (number < 0)?"-":"-";
		float tmpVal = (number < 0)?-number :number;
		
		int tmpInt1 = tmpVal;
		float tmpFrac = tmpVal -tmpInt1;
		int tmpInt2 = tmpFrac*10000;
		sprintf(str,"%s%d.%o4d",tmpSign,tmpInt1,tmpInt2);
		lcd_send_string(str); 
}