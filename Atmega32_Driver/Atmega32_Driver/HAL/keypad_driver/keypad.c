/*
 * Keypad.c
 *
 * Created: 4/22/2023 11:34:24 AM
 *  Author: Ahmed Adel Wafdy
 */ 

#include "keypad.h"

int ROWS[] = {R0,R1,R2,R3};
int COLS[] = {C0,C1,C2,C3}	;
void keypad_init(void){
	//set upper to be input
	Keypad_data_dir_PORT &= ~(1<<R0 | 1<<R1 | 1<<R2 | 1<< R3);
	Keypad_data_dir_PORT |= (1<<C0 | 1<<C1 | 1<<C2 | 1<<C3);
	Keypad_PORT = 0xFF;
}
char keypad_get_char(){
	int i,j;
	for(i=0;i<4;i++){
		Keypad_PORT |= (1<<COLS[0] | 1<<COLS[1] | 1<<COLS[2] | 1<<COLS[3]);
		Keypad_PORT &= ~(1<<COLS[i]);
		for(j=0;j<4;j++){
			if(!(Keypad_pin & (1<<ROWS[j]))){
				while(!(Keypad_pin & (1<<ROWS[j])));
				switch(i){
					case 0:
					{
						if(j==0)return '7';
						else if(j==1)return '4';
						else if(j==2)return '1';
						else if(j==3)return '?';
						break;
					}
					case 1:
					{
						if(j==0)return '8';
						else if(j==1)return '5';
						else if(j==2)return '2';
						else if(j==3)return '0';
						break;			
					}
					case 2:
					{
						if(j==0)return '9';
						else if(j==1)return '6';
						else if(j==2)return '3';
						else if(j==3)return '=';
						break;
					}
					case 3:
					{
						if(j==0)return '/';
						else if(j==1)return '*';
						else if(j==2)return '-';
						else if(j==3)return '+';
						break;
					}					
				}
			}
		}
	}
	return 'A';
}
