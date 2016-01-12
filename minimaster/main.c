#define F_CPU 16000000UL

#include <inttypes.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>
//#include <stdio.h>


int main()
{
	DDRC = 0b00110100;
	DDRD = 0b00000100;
	DDRB = 0b00001000;
	while(1)
	{
		PORTB = 0b00001000;
		_delay_ms(250);
		PORTB = 0b00000000;
		_delay_ms(250);
	}
	return 0;
}
