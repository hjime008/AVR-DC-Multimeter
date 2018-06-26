/*
 * hjime008
 * AVR DC Multimeter
 *
 * Created: 6/4/2017 2:56:23 PM
 * Author : Hector
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "5110/5110.h"
#include "5110.cpp"
#include <string.h>
#include <avr/pgmspace.h>
using namespace std;

//Declare global Variables and objects
LCD_5110 lcd;
enum states{Init, pausedInit, paused, buttonPressed, buttonReleased, voltmeterInit, voltmeter, voltSave, ohmmeterInit ,ohmmeter, ohmSave, ammeterInit ,ammeter, ampSave, freqCntrInit, freqCntr} state;
float EEMEM eepromVolts;//change to float?
float EEMEM eepromAmps;
float EEMEM eepromOhms;
char genUseBuffer [16];

volatile unsigned char TimerFlag = 0;
unsigned long _avr_timer_M = 1; // Start count from here, down to 0. Default 1 ms.
unsigned long _avr_timer_cntcurr = 0; // Current internal count of 1ms ticks

void set_PWM(double frequency) {
	static double current_frequency; // Keeps track of the currently set frequency
	// Will only update the registers when the frequency changes, otherwise allows
	// music to play uninterrupted.
	if (frequency != current_frequency) {
		if (!frequency) { TCCR0B &= 0x08; } //stops timer/counter
		else { TCCR0B |= 0x03; } // resumes/continues timer/counter
		
		// prevents OCR3A from overflowing, using prescaler 64
		// 0.954 is smallest frequency that will not result in overflow
		if (frequency < 0.954) { OCR0A = 0xFFFF; }
		
		// prevents OCR0A from underflowing, using prescaler 64					// 31250 is largest frequency that will not result in underflow
		else if (frequency > 31250) { OCR0A = 0x0000; }
		
		// set OCR3A based on desired frequency
		else { OCR0A = (short)(8000000 / (128 * frequency)) - 1; }
		//else { OCR0A = (short)(16000000 / (128 * frequency)) - 1; }

		TCNT0 = 0; // resets counter
		current_frequency = frequency; // Updates the current frequency
	}
}

void PWM_on() {
	TCCR0A = (1 << COM0A0 | 1 << WGM00);
	// COM3A0: Toggle PB3 on compare match between counter and OCR0A
	TCCR0B = (1 << WGM02) | (1 << CS01) | (1 << CS00);
	// WGM02: When counter (TCNT0) matches OCR0A, reset counter
	// CS01 & CS30: Set a prescaler of 64
	set_PWM(0);
}

void PWM_off() {
	TCCR0A = 0x00;
	TCCR0B = 0x00;
}

void TimerOn() {
	// AVR timer/counter controller register TCCR1
	TCCR1B = 0x0B;// bit3 = 0: CTC mode (clear timer on compare)
	// bit2bit1bit0=011: pre-scaler /64
	// 00001011: 0x0B
	// SO, 8 MHz clock or 8,000,000 /64 = 125,000 ticks/s
	// Thus, TCNT1 register will count at 125,000 ticks/s

	// AVR output compare register OCR1A.
	OCR1A = 125;	// Timer interrupt will be generated when TCNT1==OCR1A
	TIMSK1 = 0x02; // bit1: OCIE1A -- enables compare match interrupt

	//Initialize avr counter
	TCNT1=0;

	_avr_timer_cntcurr = _avr_timer_M;
	// TimerISR will be called every _avr_timer_cntcurr milliseconds

	//Enable global interrupts
	SREG |= 0x80; // 0x80: 1000000
}

void TimerOff() {
	TCCR1B = 0x00; // bit3bit1bit0=000: timer off
}

void TimerISR() {
	TimerFlag = 1;
}


ISR(TIMER1_COMPA_vect) {
	// CPU automatically calls when TCNT1 == OCR1 (every 1 ms per TimerOn settings)
	_avr_timer_cntcurr--; // Count down to 0 rather than up to TOP
	if (_avr_timer_cntcurr == 0) { // results in a more efficient compare
		TimerISR(); // Call the ISR that the user uses
		_avr_timer_cntcurr = _avr_timer_M;
	}
}

// Set TimerISR() to tick every M ms
void TimerSet(unsigned long M) {
	_avr_timer_M = M;
	_avr_timer_cntcurr = _avr_timer_M;
}

void ADCInit()
{
	ADMUX = ADMUX|0x40;
	ADCSRA = 0x87; //Enable ADC in single conversion mode with clock prescaler of 128
}

void ADCReInit()
{
	ADMUX = ADMUX&0x40;
}

uint16_t ADCOutput(unsigned char channel)
{
	
	channel = (channel & 0x07); //Format into to fit in range 0-7
	ADMUX = (ADMUX|channel); //Select ADC channel

	ADCSRA = (ADCSRA|0x40); //Perform a single conversion

	while(!(ADCSRA & 0x10)); //Stay in loop until conversion finishes

	ADCSRA = ADCSRA|0x10; //Clear flag that indicates conversion has finished

	return(ADC);
}


void sm_tick()
{
	//static float rawVoltage; //not needed?
	static float currVoltage;
	static float savedVoltage;
	//static float rawResistance; not needed?
	static float currResistance;
	static float savedResistance;
	//static uint16_t rawCurrent;//not needed?
	static uint16_t currentDifference;
	static float currCurrent;
	static float savedCurrent;
	static unsigned char tempD;
	static unsigned char userInput;
	
	tempD = ~PIND;
	
	switch (state) //Transitions
	{
	    case Init://show splash
	    state = pausedInit;
	   	 break;
		
		case pausedInit:
		 break;
		   
	    case paused:
		
		if(tempD == 0x01 || tempD == 0x02 || tempD == 0x04) //|| tempD == 0x08)//0x08 == freqcntr
		{
			userInput = tempD;
			state = buttonPressed;
			break; //Valid input detected, state will change
		}
		else
		{
			break; //No valid input detected, staying in current state
		}
		 break; 
		 
		case buttonPressed:
		if(tempD == 0)
		{
			state = buttonReleased; //Buttons have been released, state will change
			break;
		}
		else
		{
			break; //A button is still held. stay in current state
		}
		 break;
		 
		case buttonReleased:
		if(userInput == 0x01)
		{
			state = voltmeterInit; //Button 1 was pressed, voltmeter will initalize
			break;
		}
		else if(userInput ==  0x02)
		{
			state = ohmmeterInit;
			break;
		}
		else if(userInput == 0x04)
		{
			state = ammeterInit;
			break; 
		}
		else
		{
			state = freqCntrInit;
			break;
		}
		 break;
		
		case voltmeterInit:
		 break;
		 
		case ohmmeterInit:
		 break;
		
		case voltmeter: // add a case like this for all 4 functions
		if(tempD == 0x80)
		{
		    state = voltSave;
			break;
		}
		else if(tempD != 0x00)
		{
			userInput = tempD;
			state = buttonPressed;
			break;
		}
		else
		{
			break;
		}
		 break;
		 
		case ohmmeter: // add a case like this for all 4 functions
		if(tempD == 0x80)
		{
			state = ohmSave;
			break;
		}
		else if(tempD != 0x00)
		{
			userInput = tempD;
			state = buttonPressed;
			break;
		}
		else
		{
			break;
		}
		break;
		
		case ammeter: // add a case like this for all 4 functions
		if(tempD == 0x80)
		{
			state = ampSave;
			break;
		}
		else if(tempD != 0x00)
		{
			userInput = tempD;
			state = buttonPressed;
			break;
		}
		else
		{
			break;
		}
		break;
		
		case freqCntr:
		 break;
		
		
		default:
		state = Init;
	}
	
	switch (state) //Actions
	{
	    case pausedInit:
		lcd.lcd_clear();
		lcd.lcd_goto_xy(0, 0);
	    lcd.lcd_str(" Function Sel.");
	    lcd.lcd_goto_xy(0, 2);
	    lcd.lcd_str("B1: Voltmeter\n");
	    lcd.lcd_str("B2: Ohmmeter\n");
	    lcd.lcd_str("B3: Ammeter\n");
		lcd.lcd_str("B4: Sel. Menu\n");	
		state = paused;
	     break;
		
		case paused: 
		 break;
		 
		case voltmeterInit:
		lcd.lcd_clear();
		savedVoltage = eeprom_read_float(&eepromVolts);
		
		lcd.lcd_goto_xy(0, 0);
	    lcd.lcd_str("Voltmeter Mode\n");
		lcd.lcd_str("Volts: ");
		lcd.lcd_str("\n\n");
		lcd.lcd_str("Saved: ");
		lcd.lcd_goto_xy(7, 4);
		lcd.lcd_str(dtostrf(savedVoltage, 5, 2, genUseBuffer));
		state = voltmeter;
		 break;
		 
		case voltmeter:
		currVoltage = 0; //Clear currVoltage value
		ADCOutput(0); //Disregard first value from ADC
		for(unsigned char i = 0; i < 30; i++)//collect and sum 30 readings for averaging
		{
			currVoltage = currVoltage + ADCOutput(0);
			//_delay_ms(5);
		}
		currVoltage = currVoltage/30;//take average of sum of readings//cause of inaccuracy
        
		currVoltage = ((currVoltage/1023)*5)*5.02; //Scale up voltage. *5 for the voltage division factor, and another *5 to scale to reference voltage
		
		if(currVoltage < 0.02)
		{
			currVoltage = 0;
		}
		
		lcd.lcd_goto_xy(7, 2);
		lcd.lcd_str(dtostrf(currVoltage, 5, 2, genUseBuffer));
	     break;
		 
		case voltSave:
		//show "SAVING..."
		lcd.lcd_goto_xy(0, 5);
		lcd.lcd_str("SAVING");
		//save to eeprom
		eeprom_update_float(&eepromVolts, currVoltage);
		//update saved value display
		savedVoltage = eeprom_read_float(&eepromVolts);
		//delay 2s
		_delay_ms(2500);
		//clear "Saving..." text
		lcd.lcd_goto_xy(0, 5);
		lcd.lcd_str("      ");
		lcd.lcd_goto_xy(7, 4);
		lcd.lcd_str(dtostrf(savedVoltage, 5, 2, genUseBuffer));
		lcd.lcd_str("     ");
		state = voltmeter;
		 break;
		 
		case ohmmeterInit:
		lcd.lcd_clear();
		savedResistance = eeprom_read_float(&eepromOhms);
		
		lcd.lcd_goto_xy(0, 0);
		lcd.lcd_str("Ohmmeter Mode\n\n");
		lcd.lcd_str("Ohms: ");
		lcd.lcd_str("\n\n");
		lcd.lcd_str("Saved: ");
		lcd.lcd_goto_xy(7, 4);
		lcd.lcd_str(dtostrf(savedResistance, 5, 2, genUseBuffer));
		lcd.lcd_str("     ");
		state = ohmmeter;
		 break;

        case ohmmeter:
		currResistance = 0; //Clear currVoltage value
		ADCOutput(1); //Disregard first value from ADC

		currResistance = currResistance + ADCOutput(1);

		if(currResistance > 1021)//sets max resistance that can be measured 
		{
		    currResistance = 0;
		}
		else
		{
		
		    currResistance = (468/((1023/currResistance)-1)); //Scale up voltage. *5 for the voltage division factor, and another *5 to scale to reference voltage
		}
		

		if(currResistance > 0 && currResistance < 50)
		{
			lcd.lcd_goto_xy(6, 2);
			lcd.lcd_str(dtostrf(currResistance, 5, 1, genUseBuffer));
			lcd.lcd_str("     ");
			lcd.lcd_goto_xy(0, 3);
			lcd.lcd_str("Continuity!");
			set_PWM(2500);
		}
		else
		{
			lcd.lcd_goto_xy(6, 2);
			lcd.lcd_str(dtostrf(currResistance, 5, 1, genUseBuffer));
			lcd.lcd_str("     ");
			lcd.lcd_goto_xy(0, 3);
			lcd.lcd_str("           ");
			set_PWM(0);
		}
		
		 break;
		
		case ohmSave:
		//show "SAVING..."
		lcd.lcd_goto_xy(0, 5);
		lcd.lcd_str("SAVING");
		//save to eeprom
		eeprom_update_float(&eepromOhms, currResistance);
		//update saved value display
	    savedResistance = eeprom_read_float(&eepromOhms);
		//delay 2s
		_delay_ms(2500);
		//clear "Saving..." text
		lcd.lcd_goto_xy(0, 5);
		lcd.lcd_str("      ");
		lcd.lcd_goto_xy(6, 4);
		lcd.lcd_str(dtostrf(savedResistance, 5, 1, genUseBuffer));
		lcd.lcd_str("     ");
		state = ohmmeter;
		break; 
		
		case ammeterInit:
		lcd.lcd_clear();
		savedCurrent = eeprom_read_float(&eepromAmps);	
		
		lcd.lcd_goto_xy(0, 0);
		lcd.lcd_str("Ammeter Mode\n\n");
		lcd.lcd_str("mA: ");
		lcd.lcd_str("\n\n");
		lcd.lcd_str("Saved: ");
		lcd.lcd_goto_xy(7, 4);
		lcd.lcd_str(dtostrf(savedCurrent, 5, 3, genUseBuffer));
		lcd.lcd_str("     ");
		state = ammeter;
		break;
		
		case ammeter:
		currCurrent = 0; //Clear currVoltage value

		currCurrent = (ADCOutput(2));

		currCurrent /= 1023;
		currCurrent *= 5020;
		currCurrent = (currCurrent-0.0057)/(21);	

		
		
		lcd.lcd_goto_xy(4, 2);
		lcd.lcd_str(dtostrf(currCurrent, 5, 3, genUseBuffer));
		lcd.lcd_str("        ");
		break;
		
		case ampSave:
		//show "SAVING..."
		lcd.lcd_goto_xy(0, 5);
		lcd.lcd_str("SAVING");
		//save to eeprom
		eeprom_update_float(&eepromAmps, currCurrent);
		//update saved value display
		savedCurrent = eeprom_read_float(&eepromAmps);
		//delay 2s
		_delay_ms(2500);
		//clear "Saving..." text
		lcd.lcd_goto_xy(0, 5);
		lcd.lcd_str("      ");
		lcd.lcd_goto_xy(6, 4);
		lcd.lcd_str(dtostrf(savedCurrent, 5, 3, genUseBuffer));
		lcd.lcd_str("     ");
		state = ammeter;
		break;
		
	    case freqCntrInit:
	    lcd.lcd_clear();
	
	    lcd.lcd_goto_xy(0, 0);
	    lcd.lcd_str("Freq Cnt Mode\n\n");
	    lcd.lcd_str("Hz: ");
	    lcd.lcd_str("\n\n");
	    state = freqCntr;
	    break;
	
	    case freqCntr:
	    lcd.lcd_goto_xy(4, 2);
	    lcd.lcd_str(dtostrf(currCurrent, 5, 3, genUseBuffer));
	    lcd.lcd_str("        ");
	    break;



	}
	

	
}

int main(void)
{
	//Port and Register Initializations
	DDRD = 0x00; PORTD = 0xFF;//Initalize button input pins
	//DDRA = 0x00; PORTA = 0xFF;
	//lcd.lcd_init(&PORTB, PB2, &PORTB, PB1, &PORTB, PB3, &PORTB, PB5, &PORTB, PB7);//Initialize LCD pins
	lcd.lcd_init(&PORTB, PB1, &PORTB, PB0, &PORTB, PB2, &PORTB, PB5, &PORTB, PB7);//Initialize LCD pins
	DDRB |= 0x08; PORTB &= 0xF7;//Initialize led output pin
	ADCInit();
	TimerSet(100);
	TimerOn();
	PWM_on();
	
	//Variable declarations and initializations
	
	while (1) 
    {

			sm_tick();
			while (!TimerFlag);	// Wait 10ms
			TimerFlag = 0;
    }
}

