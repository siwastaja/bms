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
// 32 bits of data.
// If a new packet is received during this, clocking of the data ceases temporarily. Master must wait.


#define STAT_RX_ERR      1
#define STAT_OVERRUN     2
#define STAT_ALMOST_FULL 4

#define FIFO_SLOTS 8

uint8_t status;
data_t fifo[FIFO_SLOTS];

uint8_t fifo_wr_idx = 0;
uint8_t fifo_rd_idx = 0; // 0 .. FIFO_SLOTS-1
uint8_t fifo_rd_bit_idx = 0; // 0 ..31

uint8_t overrun_cnt = 0;

// Function takes some time; is used right after packet is received;
// No new packet is expected during the push.
// New packets may be pushed during the pull, but nothing is pulled
// during the push.
void fifo_push(data_t* element)
{
	if(((fifo_wr_idx < FIFO_SLOTS-1) && (fifo_wr_idx+1 == fifo_rd_idx)) ||
	   ((fifo_wr_idx == FIFO_SLOTS-1) && (0 == fifo_rd_idx)))
	{
		overrun_cnt++;
		return;
	}

	fifo_wr_idx++;
	if(fifo_wr_idx >= FIFO_SLOTS)
		fifo_wr_idx = 0;

	fifo[fifo_wr_idx].abcd = element->abcd;
}

#define DATA_1 sbi(PORTB, 1);
#define DATA_0 cbi(PORTB, 1);
#define CLK_1  sbi(PORTB, 2);
#define CLK_0  cbi(PORTB, 2);

#define DATA_RD (PINB&1)

// Fifo pull pulls one bit at a time, as it can be interrupted by
// a new packet being received.
// return 1 when more bits available, 0 when packet is done
// Leaves CLK at 0 after the last bit.
uint8_t fifo_pull_bit()
{
	if(fifo[fifo_rd_idx].abcd&((uint32_t)(1<<fifo_rd_bit_idx)))
	{
		DATA_1;
	}
	else
	{
		DATA_0;
	}

	fifo_rd_bit_idx++;
	CLK_0;
	if(fifo_rd_bit_idx > 31)
	{
		fifo_rd_bit_idx = 0;
		fifo_rd_idx++;
		if(fifo_rd_idx >= FIFO_SLOTS)
			fifo_rd_idx = 0;
		return 0;
	}
	else
	{
		__asm__ __volatile__("nop");
		__asm__ __volatile__("nop");
		__asm__ __volatile__("nop");
	}

	CLK_1;
	return 1;
}

int main()
{
	PRR = 0b00001111;

	DDRB  = 0b00000110;
	PORTB = 0b00001000; // Manchester data input pull-up.

	GIMSK = 0b00100000; // pin change interrupt enable.
	PCMSK = 0b00001000; // PCINT3 enable.
  	sbi(GIFR,5); // Clear PCINT flag.
	sei();


	while(1)
	{
		while(fifo_rd_idx != fifo_wr_idx)
		{
			CLK_1; // Interrupt main CPU
			// Expect a few cycles of RD signal.
			while(!DATA_RD);
			while(!DATA_RD);
			// Pull bits until a complete packet is done.
			while(fifo_pull_bit());
			_delay_ms(1);
		}

	  	sbi(GIFR,5); // Clear PCINT flag.
		MCUCR = 0b00110000;
		__asm__ __volatile__("sleep");
		// NORMALLY, the program stays exactly here, until there is a falling
		// edge on the input pin.
		MCUCR = 0b00000000;  // 1 clk

	}

	return 0;
}


ISR(PCINT0_vect)
{
	data_t rcv_data;
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

	if(n_low_readings >= 6)
	{
		_delay_us(1.5);
		uint8_t ret;
		ret = manchester_receive(&rcv_data);
		if(ret)
		{
			rcv_data.a = 253;
			rcv_data.b = 0x03; // standard comm error
			rcv_data.c = ret;
			rcv_data.d = 0;
		}
		fifo_push(&rcv_data);
	}
  	sbi(GIFR,5); // Clear PCINT flag.

}


