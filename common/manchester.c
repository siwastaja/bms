#define F_CPU 8000000UL

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>
#include <stdio.h>

#include "manchester.h"

// Create 40 kbaud/s at 8.000 MHz
// 25 us = 200 clock cycles per symbol


// Communication is started by pulling line LOW.
// Then follows a high transition after 25 us. This equals to a valid bit '1'.
// After this, the payload is transferred, 25 us per symbol, so 50 us per valid
// bit.

// Communication error occurs if there is no valid data bit transition 
// within a window of +/- 10 us.
// The first '1' bit has a larger window.

// Function manchester_receive expects that the line transition to low has just
// been detected (i.e., as an pin interrupt). The function checks that the line
// indeed is low, otherwise a communication error occurs.

// Framing is checked at every bit. CRC8 is checked automatically and calculated
// during rx.

// After a communication error, it is recommended to wait until the line has been
// silent for some time to drop any partial data and then report the communication
// error. Manchester coding guarantees a line change every 50 us, so waiting for,
// for example, 200 us without line changes should be enough.


#define del_us(n) _delay_us((n)) // If you need to apply any fixed compensation, do it here.



#define COMM_SUCCESS 0
#define CRC_ERROR 255

// in us
#define WINDOW_BEFORE 9   // MAX 10
#define WINDOW_AFTER  9  


#define ONE()  sbi(MANCH_OUTPORT, MANCH_OUTPIN);
#define ZERO() cbi(MANCH_OUTPORT, MANCH_OUTPIN);
#define PULLUP_ON()  sbi(MANCH_INPORT_PULLUP, MANCH_INPIN);
#define PULLUP_OFF() cbi(MANCH_INPORT_PULLUP, MANCH_INPIN);

//#define BLINK() sbi(PORTB, 2); cbi(PORTB, 2);


void manchester_wait_data_block()
{
	while(pinstat)
		;
}

uint8_t manchester_wait_data_block_timeout(uint32_t cnt)
{
	while(cnt>0)
	{
		cnt--;
		if(!pinstat)
			return 1;
	}
	return 0;

}

#define CRC_INITIAL_REMAINDER 0x00
#define CRC_POLYNOMIAL 0x07 // As per CRC-8-CCITT

#define CALC_CRC() \
	for(uint8_t bit = 8; bit > 0; --bit)   \
	{			                           \
		if(remainder & 0b10000000)         \
		{                                  \
			remainder = (remainder << 1) ^ CRC_POLYNOMIAL; \
		}                                  \
		else                               \
		{                                  \
			__asm__ __volatile__ ("nop");  \
			remainder = (remainder << 1);  \
			__asm__ __volatile__ ("nop");  \
		}                                  \
	}



uint8_t manchester_receive(data_t* data)  // Function call 4 clk, function overhead at start 5 clk
{
	// Grand total from the edge 15 us + 1.125 us = 16.125 us.
	uint8_t time_tmp;
	uint8_t remainder = CRC_INITIAL_REMAINDER; // 0 clk

	PULLUP_OFF();

	(*data).abcd = 0;  // 8 clk = 1 us.

	//if(pinstat)
	//	return 1;

	// Wait for high transition
	// Already consumed 17.125 us. For +13 us time window, it needs to come within (25+13) - 17.125 us = 20.875 us = 167 clk

	// Require two successive high readings.
	// LOOP: SBIS 2 + SBIC 2 + SUBI 1 + BRCC 2 = 7 clk /round.
	time_tmp = 24;	//167/7 would be 23.85, round to 24.
	
	while(time_tmp--)
	{
		if(pinstat && pinstat)
			goto OK1;
		
	}
	
	PULLUP_ON();
	return 2;

	OK1:
	
	// Now we are exactly aligned at first '1', which is discarded.
	
	_delay_us(10.125 + 1.5); // Compensation for CRC delay, see below.
	                         // +1.5 = measured correction.

	for(uint8_t i = 0; i < 32; i++)
	{
		(*data).abcd <<= 1;  // 20 clk = 2.5 us

		// Align at 35.0 us from previous data bit. 37.5 us is
		// halfway between the optional edge and the next data bit.
		// Sample the value between 35.0 us and 40.0 us.
		// The expected edge is at 50 us, but allow it some window
		// due to clock differences.

		del_us(35.0 - 2.5 - 10.125 - 2.5); // CRC calculation uses 10.125 us - see
		                             // below.
									 // -2.5 = measured correction.


		uint8_t n_low_readings = 0;
		for(uint8_t i = 8; i>0; --i)  // 5 clk per round (regardless of pin value.)
		{
			if(!pinstat)
				n_low_readings++;
		} // 40 clk = 5 us.
		

		// Num of zeroes: 0  1  2   3  4   5  6  7
		//                H I G H | ?  ? | L  O  W

		if(n_low_readings < 3) 
		{	// High -- expect low. 
			del_us(10-WINDOW_BEFORE);
			// time windows of +/-10 us = 20 us = 160 clock cycles starts here.
			// LOOP: SBIS 2 + SBIC 2 + SUBI 1 + BRCC 2 = 7 clk /round.
			time_tmp = 	((WINDOW_BEFORE+WINDOW_AFTER)*8)/7;
			while(time_tmp--)
			{ // Require two successive low readings.
				if((!pinstat) && (!pinstat))
					goto OK2;
			}
			PULLUP_ON();
			return 10+i;

			OK2:

			del_us(1.125);

		}
		else if(n_low_readings > 4) // low -- expect high
		{
			del_us(10-WINDOW_BEFORE);
			// time windows of +/-10 us = 20 us = 160 clock cycles starts here.
			// LOOP: SBIS 2 + SBIC 2 + SUBI 1 + BRCC 2 = 7 clk /round.
			time_tmp = 	((WINDOW_BEFORE+WINDOW_AFTER)*8)/7;
			while(time_tmp--)
			{ // Require two successive high readings.
				if(pinstat && pinstat)
					goto OK3;
			}
			PULLUP_ON();
			return 50+i;

			OK3:

			(*data).abcd |= 1; // 9 clk = 1.125 us

		}
		else
		{
			PULLUP_ON();
			return 100+i;
		}

		// Here, we are aligned perfectly again.


		// At the same time, calculate CRC8. Calculate every time 8 bits have been received,
		// but of course skip the last octet which is the CRC.

		// Consume a constant amount of time here, which can be then subtracted from
		// the delay at the beginning of the loop.

		if(i==7 || i==15 || i==23)  // 6 cycles used when not true, 3...7 if true.
		{
			// We have our latest full byte in data.d
			remainder ^= (*data).d;        // 3 cycles
			CALC_CRC();
			// Total 3+48+24 = 75 cycles = 9.375 us.
		}
		else
		{
			_delay_us(9.375);
		}
		// In total, 10.125 us was spent for the if + CRC.

	}

	PULLUP_ON();

	if(remainder != (*data).d)
		return CRC_ERROR;

	return COMM_SUCCESS;
}



uint8_t manchester_send(data_t data)
{
	uint8_t remainder = CRC_INITIAL_REMAINDER;

	ZERO();

	// Calculate CRC here in parts. This should be well timed.

	remainder ^= data.a;                // 3 cycles
	CALC_CRC();
	// Grand total 48+24+3 = 75 cycles = 9.375 us.

	remainder ^= data.b;                // 3 cycles
	CALC_CRC();
	// Grand total 48+24+3 = 75 cycles = 9.375 us.

	del_us(25 - 9.375 - 9.375);

	ONE();

	remainder ^= data.c;                // 3 cycles
	CALC_CRC();
	// Grand total 48+24+3 = 75 cycles = 9.375 us.

	del_us(25 - 9.375);
		

	for(int i = 8; i > 0; --i)
	{
		if(data.a & 0b10000000)
		{
			ZERO();
			del_us(25);
			ONE();
			del_us(25);
		}
		else
		{
			ONE();
			del_us(25);
			ZERO();
			del_us(25);
		}
		data.a = data.a << 1;
	}

	for(int i = 8; i > 0; --i)
	{
		if(data.b & 0b10000000)
		{
			ZERO();
			del_us(25);
			ONE();
			del_us(25);
		}
		else
		{
			ONE();
			del_us(25);
			ZERO();
			del_us(25);
		}
		data.b = data.b << 1;
	}


	for(int i = 8; i > 0; --i)
	{
		if(data.c & 0b10000000)
		{
			ZERO();
			del_us(25);
			ONE();
			del_us(25);
		}
		else
		{
			ONE();
			del_us(25);
			ZERO();
			del_us(25);
		}
		data.c = data.c << 1;
	}


	for(int i = 8; i > 0; --i)
	{
		if(remainder & 0b10000000)
		{
			ZERO();
			del_us(25);
			ONE();
			del_us(25);
		}
		else
		{
			ONE();
			del_us(25);
			ZERO();
			del_us(25);
		}
		remainder = remainder << 1;
	}


	ONE(); // linja ylös lopuksi

	return COMM_SUCCESS;

}
