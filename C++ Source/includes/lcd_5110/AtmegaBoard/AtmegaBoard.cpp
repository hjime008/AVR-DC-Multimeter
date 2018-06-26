/*
 * AtmegaBoard.cpp
 *
 * Created: 4/8/2013 4:31:13 PM
 *  Author: radhoo
 */ 


#include <avr/io.h>
#include "timeout.h"
#include "5110/5110.h"
#include "introscreen.h"

LCD_5110 lcd;

int main(void)
{
	_delay_ms(100);
	
	// define the 5 LCD Data pins: SCE, RST, DC, DATA, CLK
    lcd.lcd_init(&PORTB, PB0, &PORTB, PB1, &PORTB, PB2, &PORTB, PB3, &PORTB, PB4);
    
	while (1) {
		// image demo
		lcd.printPictureOnLCD(introScreen);
		_delay_ms(2000);
		lcd.lcd_clear();
	
		// goto and char demo
		for (int i=0;i<14;i++)
			for (int j=0;j<6;j++) {
				lcd.lcd_goto_xy(i,j);
				lcd.lcd_chr('0' + (i + j) % 10);
				_delay_ms(10);
			}
		_delay_ms(2000);
		lcd.lcd_clear();
	
		// formatted string demo
		for (int i=10;i>0;i--) {
			lcd.lcd_goto_xy(0,1);
			lcd.lcd_string_format("   00.00.%02d   \n  pocketmagic \n     .net     ", i);
			_delay_ms(200);
		}
		_delay_ms(2000);
		lcd.lcd_clear();
		
		for (int i=0;i<84;i++) {
			int f = (i * i) % 48;
			lcd.drawPixel(i,0, 1);
			lcd.drawPixel(i, f, 1);
		}
		_delay_ms(2000);
		lcd.lcd_clear();
	}		
	
}