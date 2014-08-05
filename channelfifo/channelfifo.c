#define F_CPU 8000000UL

#include <inttypes.h>
#include <avr/io.h>
//#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>
#include <stdio.h>

#include "../common/manchester.h"

// DATA IN             : pin 2 (PB3 PCINT3)
// DATA OUT            : pin 6 (PB1)
// DATA READ IN        : pin 5 (PB0)
// INT / CLOCK         : pin 7 (PB2)

// Packet reveiced
// INT/CLOCK outputs high.
// Master sets data read = 1, disables all interrupts, is ready to read data,
//            keeps data read high until receives clock pulse 
// Clock out starts at 1. Every falling edge denotes a new bit.
// First 8 bits is channelfifo status.
// Then 32 bits of data.
// If a new packet is received during this, clocking of the data ceases temporarily. Master must wait.


// Status packet
// bit 0: RX error, do not read 32 further bytes
// bit 1: Data overrun (packet received when FIFO = full)
// bit 2: Fifo almost full (<=2 empty slots)

#define STAT_RX_ERR      1
#define STAT_OVERRUN     2
#define STAT_ALMOST_FULL 4

typedef struct
{
	uint8_t status;
	data_t data;
} element_t;

typedef union
{
	element_t element;
	uint8_t[5] block;
} element_union;

#define FIFO_SLOTS 8

uint8_t status;
element_union fifo[FIFO_SLOTS];

uint8_t fifo_wr_idx = 0;
uint8_t out_idx = 0; // 0..7 = status byte, 8 = data
uint8_t fifo_rd_idx = 0; // 0 .. FIFO_SLOTS-1
uint8_t fifo_rd_byte_idx = 0; // 0 .. 5
uint8_t fifo_rd_bit_idx = 0; // 0 .. 8


// Function takes some time; is used right after packet is received;
// No new packet is expected during the push.
void fifo_push(element_t* element)
{
	for(uint8_t i = 5; i >= 0; i--)
		fifo[fifo_wr_idx].block[i] = element->block[i];
	fifo_wr_idx++;
	if(fifo_wr_idx >= FIFO_SLOTS)
		fifo_wr_idx = 0;
}

#define DATA_1 sbi(PORTB, 1);
#define DATA_0 cbi(PORTB, 1);
#define CLK_1  sbi(PORTB, 2);
#define CLK_0  cbi(PORTB, 2);


// Fifo pull pulls one bit at a time, as it can be interrupted by
// a new packet being received.
void fifo_pull_bit()
{
	if(out_idx < 8)
	{
		if(status&(1<<out_idx))
			DATA_1;
		else
			DATA_0;
		out_idx++;
		CLK_0;
		__asm__ __volatile__("nop");
		__asm__ __volatile__("nop");
		__asm__ __volatile__("nop");
		__asm__ __volatile__("nop");
		__asm__ __volatile__("nop");
	}
	else
	{
		if(fifo[fifo_rd_idx].block[fifo_rd_byte_idx]&(1<<fifo_rd_bit_idx))
			DATA_1;
		else
			DATA_0;
		fifo_rd_bit_idx++;
		CLK_0;
		if(fifo_rd_bit_idx > 7)
		{
			fifo_rd_bit_idx = 0;
			fifo_rd_byte_idx++;
			if(fifo_rd_byte_idx > 5)
			{
				fifo_rd_byte_idx = 0;
				fifo_rd_idx++;
				out_idx = 0;
			}
			else
			{
				__asm__ __volatile__("nop");
				__asm__ __volatile__("nop");
				__asm__ __volatile__("nop");
			}
		}
		else
		{
			__asm__ __volatile__("nop");
			__asm__ __volatile__("nop");
		}
	}
	CLK_1;
}

int main()
{
	element_t rcv_data;

	PRR = 0b00001111;

	DDRB  = 0b00000110;
	PORTB = 0b00001000; // Manchester data input pull-up.

	GIMSK = 0b00100000; // pin change interrupt enable.
	PCMSK = 0b00001000; // PCINT3 enable.
  	sbi(GIFR,5); // Clear PCINT flag.
	sei();


	while(1)
	{
		uint8_t ret;
	NO_RX_BACK_TO_SLEEP:
	  	sbi(GIFR,5); // Clear PCINT flag.
	  	sei();
		MCUCR = 0b00110000;
		__asm__ __volatile__("sleep");
		// NORMALLY, the program stays exactly here, until there is a falling
		// edge on the input pin.
		cli(); // 1 clk
		MCUCR = 0b00000000;  // 1 clk

		if(!receiving) // We woke up for nothing :(.
			goto NO_RX_BACK_TO_SLEEP; 
		// 4 clk

		receiving = 0; // 2 clk

		// Grand total from the edge: 15 us (measured).

		ret = manchester_receive(&(rcv_data.data)); 

		if(ret) // Error, but got at least the start bit ok.
		{
			// TODO: Indicate end-of-chain comm error
		}
		else  // Got the packet OK.
		{
			// TODO: Indicate received packet.
		}

	}

	return 0;
}


ISR(PCINT0_vect)
{
	// It takes at least 6+4+12 clock cycles (2.75 us) to wake up
	// and get here, so the signal should be stabilized at
	// low level
	// 6.5 us measured!

	// ~6.5 us spent from the edge.

	// Require a low level for some time while filtering out
	// a few high spikes.

	uint8_t n_low_readings = 0;
	for(uint8_t i = 8; i>0; --i)  // 5 clk per round (regardless of pin value.)
	{
		if(!pinstat)
			n_low_readings++;
	}

	// ~10.5us spent from the edge.

	receiving = n_low_readings >= 6;
	// ~11us

	// In reality, 12.5 us (measured).
	// ISR end overhead 15 clk = 1.875 us.
	// We were either in the "sleep" instruction or in the shunting loop structure when
	// the interrupt came, so go back where we were.
} // Grand total from the edge 12 us.

ISR(ADC_vect)
{
	// Function stub needed here to get back from ADC Noise Reduction Sleep when
	// the conversion is done.
}

