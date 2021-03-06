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

#define ONE()  sbi(PORTB, 4);
#define ZERO() cbi(PORTB, 4);


// Fixed 115200 baud/s on 8 MHz CPU
// 1 clk cycle = 0.125 us
// 8.68 us/symbol
// 69.4 clk cycles per symbol
// timeout=0 -> no timeout

uint8_t uart_read_byte_block(uint8_t* out, uint16_t timeout)
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
		if(timeout)
			timeout--;
		_delay_us(0.3);
		if(timeout == 1)
		{
			return 1;
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
		return 2;
	*out = byte;
	return 0;
}

#define TIMEOUT 50000

int main()
{
	PRR = 0b00001111; // Unnecessary peripherals off.

	DDRB  = 0b00011001; // Data out, debug pulse out, led out
	PORTB = 0b00010010; // Data output high, jumper pull-up

	LED_ON();
	_delay_ms(250);
	LED_OFF();

	cli();
	while(1)
	{
		data_t packet;
		uint8_t err = 0;
		for(uint8_t byte_cnt = 4; byte_cnt > 0; byte_cnt--)
		{
			if((err = uart_read_byte_block(&(packet.block[byte_cnt-1]), (byte_cnt==4)?0:TIMEOUT)))
			{
				break;
			}
		}

		LED_ON();

		if(err == 1)
		{
			for(uint8_t i = 0; i < 4; i++)
			{
				_delay_ms(100);
				LED_OFF();
				_delay_ms(100);
				LED_ON();
			}
		}

		if(err || check_crc(&packet))
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


