#define F_CPU 8000000UL

#include <inttypes.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>
#include <stdio.h>

#include "../common/manchester.h"


// DATA IN       : pin 3 (PB4)
// LED           : pin 2 (PB3)
// PC data out   : pin 7 (PB2)
// Jumper in     : pin 6 (PB1)
// Debug pulse   : pin 5 (PB0)

#define UART_1() sbi(PORTB, 2)
#define UART_0() cbi(PORTB, 2)

#define LED_ON()  sbi(PORTB, 3)
#define LED_OFF() cbi(PORTB, 3)
#define PULSE() { sbi(PORTB, 0); cbi(PORTB, 0);}

volatile uint8_t receiving;

// Fixed 115200 baud/s on 8 MHz CPU
// 1 clk cycle = 0.125 us
// 8.68 us/symbol
// 69.4 clk cycles per symbol

void uart_write_byte_block(uint8_t byte)
{
	UART_0();
	_delay_us(8.68-0.750);
	for(uint8_t i = 8; i > 0; --i)
	{
		if(byte & 1)
			UART_1();
		else
			UART_0();
		_delay_us(8.68-0.875);
		byte >>= 1;
	}
	UART_1();
	_delay_us(8.68*8);
}

int main()
{
	PRR = 0b00001111; // Unnecessary peripherals off.

	DDRB  = 0b00001101; // Data out, debug pulse out, led out
	PORTB = 0b00010110; // Data input pull-up, uart out high, jumper pull-up

	LED_ON();
	_delay_ms(250);
	LED_OFF();

	while(1)
	{
		data_t packet;

		manchester_wait_data_block();
		LED_ON();
		_delay_us(12-0.125);
		if(manchester_receive(&packet))
		{
			_delay_ms(500);
		}
		else
		{
			uart_write_byte_block(packet.a);
			uart_write_byte_block(packet.b);
			uart_write_byte_block(packet.c);
			uart_write_byte_block(packet.d);
		}

		LED_OFF();
	}

	return 0;
}


