#define F_CPU 8000000UL

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>
#include <stdio.h>

#include "multichan_manchester_tx.h"

// Create 40 kbaud/s at 8.000 MHz
// 25 us = 200 clock cycles per symbol


// Communication is started by pulling line LOW.
// Then follows a high transition after 25 us. This equals to a valid bit '1'.
// After this, the payload is transferred, 25 us per symbol, so 50 us per valid
// bit.

#define CRC_INITIAL_REMAINDER 0x00
#define CRC_POLYNOMIAL 0x07 // As per CRC-8-CCITT

// Sets the CRC byte in data.
void manchester_send_1chan(bms_packet_t* data, uint8_t chan)
{
	register uint8_t pin = MANCH_OUTPORT_OFFSET + chan;
	uint8_t sreg = SREG;
	cli();
	data->crc = CRC_INITIAL_REMAINDER;

	cbi(MANCH_OUTPORT, pin);
	_delay_us(25);
	sbi(MANCH_OUTPORT, pin);
	_delay_us(25);

	for(uint8_t byte = 0; byte < 4; byte++)
	{
		uint8_t tmp = data->bytes[byte];
		if(byte < 3)
		{
			data->crc ^= tmp;
		}
		else
		{
			__asm__ __volatile__ ("nop");
		}

		for(uint8_t bit = 8; bit > 0; --bit)
		{
			if(tmp & 0b10000000)
			{
				cbi(MANCH_OUTPORT, pin);
				_delay_us(25);
				sbi(MANCH_OUTPORT, pin);
			}
			else
			{
				sbi(MANCH_OUTPORT, pin);
				_delay_us(25);
				cbi(MANCH_OUTPORT, pin);
			}
			_delay_us(24);
			tmp <<= 1;

			if(byte < 3)
			{
				if((data->crc) & 0b10000000)
				{
					data->crc = ((data->crc) << 1) ^ CRC_POLYNOMIAL;
				}
				else
				{
					__asm__ __volatile__ ("nop");
					data->crc = ((data->crc) << 1);
					__asm__ __volatile__ ("nop");
				}
			}
			else
			{
				__asm__ __volatile__ ("nop");
				__asm__ __volatile__ ("nop");
				__asm__ __volatile__ ("nop");
				__asm__ __volatile__ ("nop");
				__asm__ __volatile__ ("nop");
			}

		}
	}

	sbi(MANCH_OUTPORT, pin);
	SREG = sreg;
}

// Optimized tx on 4 parallel channels simultaneously.
void manchester_send_4chan(bms_packet_t* datas)
{
	uint8_t sreg = SREG;
	cli();

	MANCH_OUTPORT &= ~(0b00001111 << MANCH_OUTPORT_OFFSET);
	_delay_us(25);
	MANCH_OUTPORT |= (0b00001111 << MANCH_OUTPORT_OFFSET);
	_delay_us(25);


	SREG = sreg;
}
