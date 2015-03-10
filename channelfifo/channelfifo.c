#define F_CPU 8000000UL

#include <inttypes.h>
#include <avr/io.h>
//#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>
#include <stdio.h>

#include "../common/manchester.h"

// MANCHESTER DATA IN  : pin 2 (PB3 PCINT3)
// DATA OUT            : pin 6 (PB1)
// READ CMD IN         : pin 5 (PB0)
// INTERRUPT / CLOCK   : pin 7 (PB2)

// Packet reveiced ->
// INT/CLOCK outputs low, interrupting master cpu.
// When ready to read, master disables any interrupts, sets data read = 1
//            keeps data read high at least until receives the first clock pulse
// Clock out starts at 0. Every rising edge denotes a new bit.
// 32 bits of data.
// If a new packet is received during this, clocking of the data ceases temporarily. Master must wait.
// The BMS architecture guarantees that at least for every packet (2 ms), there is equal pause (2 ms),
// so the FIFO->master communication cannot be starved due to incoming traffic, only slowed down.


//#define STAT_RX_ERR      1
//#define STAT_OVERRUN     2
//#define STAT_ALMOST_FULL 4

#define FIFO_SLOTS 16

//uint8_t status;
data_t fifo[FIFO_SLOTS];

uint8_t fifo_wr_idx;
uint8_t fifo_rd_idx; // 0 .. FIFO_SLOTS-1
uint32_t fifo_rd_bit_mask = 1;

//uint8_t overrun_cnt;

// Function takes some time; is used right after packet is received;
// No new packet is expected during the push.
// New packets may be pushed during the pull, but nothing is pulled
// during the push.
void fifo_push(data_t* element)
{
	if(((fifo_wr_idx < FIFO_SLOTS-1) && (fifo_wr_idx+1 == fifo_rd_idx)) ||
	   ((fifo_wr_idx == FIFO_SLOTS-1) && (0 == fifo_rd_idx)))
	{
//		overrun_cnt++;
		return;
	}

	fifo[fifo_wr_idx].abcd = element->abcd;

	fifo_wr_idx++;
	if(fifo_wr_idx >= FIFO_SLOTS)
		fifo_wr_idx = 0;

}

#define DATA_1 sbi(PORTB, 1);
#define DATA_0 cbi(PORTB, 1);
#define CLK_1  sbi(PORTB, 2);
#define CLK_0  cbi(PORTB, 2);

#define DATA_RD (PINB&1)

// Fifo pull pulls one bit at a time, as it can be interrupted by
// a new packet being received.
// return 1 when more bits available, 0 when packet is done
// Leaves CLK at 1 after the last bit.
uint8_t fifo_pull_bit()
{
	if(fifo[fifo_rd_idx].abcd & fifo_rd_bit_mask)
	{
		DATA_1;
	}
	else
	{
		DATA_0;
	}

	CLK_1;

	// Slow down the clock enough that the simple master code can keep up with it.
	__asm__ __volatile__("nop");
	__asm__ __volatile__("nop");
	__asm__ __volatile__("nop");

	fifo_rd_bit_mask <<= 1;
	if(fifo_rd_bit_mask == 0) // '1' just shifted out from left - no more bits left in this packet.
	{
		fifo_rd_bit_mask = 1;
		fifo_rd_idx++;
		if(fifo_rd_idx >= FIFO_SLOTS)
			fifo_rd_idx = 0;
		return 0;
		// Leave clock at '1'.
	}

	CLK_0;
	return 1;
}

int main() __attribute__((noreturn));
int main()
{
	PORTB = 0b00111100; // Manchester data input pull-up. Unused pins pulled up. nINT/CLK high
	DDRB  = 0b00000110;
	PRR = 0b00001111;

	GIMSK = 0b00100000; // pin change interrupt enable.
	PCMSK = 0b00001000; // PCINT3 enable.
  	sbi(GIFR,5); // Clear PCINT flag.
	sei();

	while(1)
	{
		while(fifo_rd_idx != fifo_wr_idx)
		{
			CLK_0; // Interrupt main CPU
			// Expect a few cycles of RD signal.
			while(!DATA_RD);
			while(!DATA_RD);
			// Pull bits until a complete packet is done.
			while(fifo_pull_bit());
			// Now: if there are more packets in FIFO, master is interrupted very quickly again.
			// So at master, clear your interrupt flag RIGHT after the final bit,
			// so that you get a new interrupt bending even if you didn't exit the ISR yet.
		}

		MCUCR = 0b00110000;
		__asm__ __volatile__("sleep");
		// NORMALLY, the program stays exactly here, until there is a falling
		// edge on the input pin.
		MCUCR = 0b00000000;  // 1 clk
		// Woke up from sleep - we have received a packet, so fifo_wr_idx is now incremented,
		// unless it was a false alarm (spike).
	}

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


