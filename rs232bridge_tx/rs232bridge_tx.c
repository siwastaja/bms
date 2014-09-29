#define F_CPU 8000000UL

#include <inttypes.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>
#include <stdio.h>

#include "../common/manchester.h"


// DATA OUT      : pin 3 (PB4)
// LED           : pin 5 (PB0)
// PC data in    : pin 7 (PB2, PCINT2)
// Jumper in     : pin 6 (PB1)
// Debug pulse   : pin 2 (PB3)

#define UART_HI() (PINB&0b100)
#define UART_HI_BIT() ((PINB&0b100)>>2)

#define LED_ON()  sbi(PORTB, 0)
#define LED_OFF() cbi(PORTB, 0)
#define PULSE() { sbi(PORTB, 3); cbi(PORTB, 3);}

volatile uint8_t receiving;

// Fixed 115200 baud/s on 8 MHz CPU
// 1 clk cycle = 0.125 us
// 8.68 us/symbol
// 69.4 clk cycles per symbol

uint8_t uart_read_byte_block(uint8_t* out)
{
	uint8_t byte = 0;
	while(1)
	{
		if(!UART_HI())
		{
			_delay_us(1.0);
			if(!UART_HI())
				break;
		}
	}

	LED_ON();
	_delay_us(3.7);

	for(uint8_t i = 8; i > 0; --i)
	{
		_delay_us(7.7);
		byte >>= 1;
		if(UART_HI())
			byte |= 0b10000000;
		else
			__asm__ __volatile__("nop");
	}

	_delay_us(7.8);
	LED_OFF();

	PULSE();
	if(!UART_HI())
		return 1;
	*out = byte;
	return 0;
}

int main()
{
	PRR = 0b00001111; // Unnecessary peripherals off.

	DDRB  = 0b00011001; // Data out, debug pulse out, led out
	PORTB = 0b00000010; // Jumper pull-up

	LED_ON();
	_delay_ms(250);
	LED_OFF();

//	GIMSK = 0b00100000; // pin change interrupt enable.
//	PCMSK = 0b00000100; // PCINT2 enable.

	while(1)
	{
		// Read uart here

		data_t packet;
		uint8_t framing_err = 0;
		for(uint8_t byte_cnt = 0; byte_cnt < 4; byte_cnt++)
		{
			if(uart_read_byte_block(&(packet.block[byte_cnt])))
			{
				framing_err = 1;
				break;
			}
		}

		LED_ON();

		if(framing_err || check_crc(&packet))
		{
			_delay_ms(500);
		}
		else
		{
			manchester_send_no_crc_calc(&packet);
		}
		LED_OFF();
	}

	return 0;
}


