// EEPROM MAP
// 0x00-0x1f: Non-checksummed area
// 0x00-0x0f: Boot counters, saturated at 255, indexed with MCUSR.
// 0x10: Node SW version number
// 0x11-0x14: serial number (uint32)
// 0x15: Checksum for SW version and serial number (sum of 5 bytes 0x10-0x14)

// 0x20 - 0x3e: checksummed area
// 0x20: Node ID
// 0x21, 0x22: V offset (int16).
// 0x23, 0x24: Ext sens offset (int16)
// 0x25, 0x26: Int Temp offset (int16).
// 0x27, 0x28: V gain (uint16)
// 0x29, 0x2a: Ext sens gain (uint16)
// 0x2b, 0x2c: Int temp gain (uint16)
// 0x2d: V temp coeff (int8)
// 0x2e: Ext sens temp coeff (int8)
// 0x2f: Int temp temp coeff (int8). Should always be 0.
// 0x30: Checksum of all previous (LSbyte of sum of bytes)

// ...0x3e: Reserved for future use within node code
// 0x41..0x4f: copy of factory-calibrated values 0x21..0x2f (for
//             those users who mess up playing with calibration
//             and want to revert to factory values)

#define F_CPU 8000000UL

#include <inttypes.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>
#include <stdio.h>

#include "../common/manchester.h"


// DATA IN       : pin 7 (PB2 PCINT2)
// DATA OUT      : pin 5 (PB0)
// BALANCING LED : pin 6 (PB1)
// VOLT DIV ENABn: pin 2 (PB3)
// VOLT ADC IN   : pin 3 (PB4 ADC2)
// Ext temp sensor: pin 1 (PB5 (reset))

#define BROADCAST_ID 255

typedef union
{
	int16_t  sboth;
	uint16_t both;
	struct { int8_t slo; int8_t shi; };
	struct { uint8_t lo; uint8_t hi; };
} union16_t;

typedef union
{
	uint32_t both32;
	struct { uint16_t lo16; uint16_t hi16; };
	struct { int16_t slo16; int16_t shi16; };
} union32_t;

#define SHADOW_LEN 16

typedef union
{
	struct { uint8_t own_id; int16_t offsets[3]; union16_t gains[3]; int8_t t_coeffs[3];};
	uint8_t all[SHADOW_LEN];
} shadow_t;

shadow_t shadow;

int16_t last_temp_diff; // difference of latest chip temperature and temp coeff midpoint (+23 degC), in 1/4C

#define LED_ON()  cbi(PORTB, 1)
#define LED_OFF() sbi(PORTB, 1)

// Vcc - PB4(ADC2) - PB3
// normally: 1 - an.input - 1
// when measuring: 1 - an.input - 0
// when shunting:  1 - 0 - 1

#define MEAS_RESISTOR_SHUNT_ON()  sbi(DDRB, 4)
#define MEAS_RESISTOR_SHUNT_OFF() cbi(DDRB, 4)


void reload_variables()
{
//	eeprom_read_block((void*)(&(shadow.own_id)), (void*)0x20, SHADOW_LEN);
	// Own implementation is 24 bytes smaller than eeprom_read_block
	uint8_t* p_shadow = &(shadow.own_id);
	for(uint8_t a = 0x20; a < 0x30; a++)
	{
		*p_shadow = eeprom_read_byte((uint8_t*)(uint16_t)a);
		p_shadow++;
	}
}

uint8_t recalc_eeprom_checksum(uint8_t save)
{
	uint8_t checksum = 0;
	for(uint8_t a = 0x20; a < 0x30; a++)
	{
		checksum += eeprom_read_byte((uint8_t*)(uint16_t)a);
	}
	if(save)
		eeprom_write_byte((uint8_t*)0x30, checksum);
	else if(checksum != eeprom_read_byte((uint8_t*)0x30))
		return 1;
	return 0;
}

volatile uint8_t receiving = 0;

#define SHUNT_I_START 500000

#define VOLTAGE 0
#define EXT_SENS 1
#define INT_TEMP 2
#define EXT_SENS_PU 3

int main() __attribute__((noreturn));

int main()
{
	uint8_t shunting = 0;

	uint32_t shunt_i = 0;
	uint8_t shunt_devices = 0b11;
	uint8_t eeprom_change_address = 255;
	uint8_t debug = 0;
//	uint8_t bod_disable = 0;

	union16_t prev_long_meas;
	prev_long_meas.both = 0;

	data_t rcv_data;
	data_t reply_data;

	PRR = 0b00001111; // Unnecessary peripherals off.

	DDRB  = 0b00001011;
	PORTB = 0b00001101; // Manchester data input pull-up. Data output high. Volt meas disable.
	DIDR0 = 0b00110000; // disable digital input on ADC0 & ADC2.

	uint8_t tmp;
	uint8_t tmpsr = MCUSR; // MCUSR is guaranteed < 16
	MCUSR = 0;

	tmp = eeprom_read_byte((uint8_t*)(uint16_t)tmpsr);
	if(tmp != 255)
		eeprom_write_byte((uint8_t*)(uint16_t)tmpsr, tmp+1);


	LED_ON();
	_delay_ms(100);
	LED_OFF();

	reload_variables();

	GIMSK = 0b00100000; // pin change interrupt enable.
	PCMSK = 0b00000100; // PCINT2 enable.

	while(1)
	{
		uint8_t ret;

	  	sbi(GIFR,5); // Clear PCINT flag.
	  	sei();

		while(shunting)
		{
			if(shunt_devices&1)
				LED_ON();
			if(shunt_devices&2)
				MEAS_RESISTOR_SHUNT_ON();
			while(shunt_i) // one-second loop
			{
				if(receiving)
				{
					cli(); // 1 clk
					_delay_us(3);
					goto SHUNT_RECEIVE;
				}
				shunt_i--;
			}
			shunt_i = SHUNT_I_START;
			shunting--;
		}

		// Stop shunting:
		MEAS_RESISTOR_SHUNT_OFF();
		LED_OFF();

	  NO_RX_BACK_TO_SLEEP:
	  	sbi(GIFR,5); // Clear PCINT flag.
	  	sei();
//		if(bod_disable)
//		{
			// AVR team has come up with a hilarious
			// "timed sequence" which goes like this:
//			MCUCR = 0b10000100;
			// max 4 clock cycles between these two
//			MCUCR = 0b10110000;
			// Sleep must be entered in 3 clock cycles
//		}
//		else
//		{
		MCUCR = 0b00110000;
//		}
		__asm__ __volatile__("sleep");
		// NORMALLY, the program stays exactly here, until there is a falling
		// edge on the input pin.
		cli(); // 1 clk
		MCUCR = 0b00000000;  // 1 clk
//		bod_disable = 0;

		if(!receiving)
		{	// Interrupt came, but ISR did not set the flag
			// (pulse was filtered out by ISR)
			goto NO_RX_BACK_TO_SLEEP; 
		}

      SHUNT_RECEIVE:
		MEAS_RESISTOR_SHUNT_OFF();

		receiving = 0;

		// Grand total from the edge: 15 us (measured).
		// (Added MEAS_RESISTOR_SHUNT_OFF(), LED_OFF() and bod_disable = 0 since then,
                // but shouldn't make big difference.

		if(debug)
			LED_ON();
		ret = manchester_receive(&rcv_data); 
		LED_OFF();
		reply_data.a = shadow.own_id;

		if(ret) // Error, but got at least the start bit ok.
		{
			if(debug)
			{
				LED_ON();
				_delay_ms(250);
				LED_OFF();
			}

			// Send "communication error" packet

			reply_data.b = 0x03;
			reply_data.c = ret;
			manchester_send(&reply_data);
		}
		else  // Got the packet OK.
		{

			uint8_t allow_broadcast = 1;
			if(rcv_data.a == shadow.own_id || rcv_data.a == BROADCAST_ID)
			{
				if((rcv_data.b & 0b11100000) == 0b10000000)
				{
					uint8_t tmp_rcv_data_b = rcv_data.b;

					// Compensate for charge usage difference in the chain.
					// Later nodes use more charge to relay all the messages from
					// the chain. The measurement message data field indicates
					// how many nodes are after this node in the chain.
					// For every node, 240 us of shunting compensates
					// for this difference.
					// This would be 120 shunt loops, but approximate
					// with 128 to remove the multiplication op.
					// debug mode increses the time by 4x (because debug == 0 or 2)
					// 3x extra = 720 us of 40 mA shunt, which corresponds to
					// ~1.5 ms of LED only (which is about right).

					if(rcv_data.c)
					{
						shunting = 1;
						shunt_devices = 0b11;
						shunt_i = ((uint32_t)rcv_data.c) << 7 << debug;
						rcv_data.c -= 1;
					}

					uint8_t src = tmp_rcv_data_b & 0b00000011;
					MEAS_RESISTOR_SHUNT_OFF();
					cbi(PRR, 0);

					switch(src)
					{
					case VOLTAGE:
						ADMUX = 0b0010;
						cbi(PORTB, 3); // Enable resistor divider
						break;
					case EXT_SENS_PU:
						sbi(PORTB, 5); // Ext sens pull-up
						src = EXT_SENS;
					case EXT_SENS:
						ADMUX = 0;
						break;
					case INT_TEMP:
						ADMUX = 0b1111;
						break;
					default:
						break;
					}

					if(tmp_rcv_data_b & 0b100) // 1V1 reference, else Vcc
						sbi(ADMUX, 7);

					uint16_t num_meas = 4;
					if(tmp_rcv_data_b & 0b00010000)
					{	// Long measurement - send old result and relay broadcast
						reply_data.b = prev_long_meas.hi;
						reply_data.c = prev_long_meas.lo;
						manchester_send(&reply_data);

						if(rcv_data.a == BROADCAST_ID)
						{
							allow_broadcast = 0;
							_delay_ms(2.2);
							manchester_send(&rcv_data);
						}

						// First meas = 25 ADC clk = 200 us
						// Subsequent = 13 ADC clk = 104 us
						// 4 = 0.51 ms
						// 2048 = 213 ms
						num_meas = 2048;
					}

					// enable ADC, enable ADC interrupt,
					// clear int flag, prescaler 64 (125 kHz)
					ADCSRA = 0b10011110;
					// let the filter capacitor charge and ADC stabilize.
					// 2 ms = 20x R*C time
					_delay_ms(2.2);

					cbi(GIMSK, 5); // Disable pin change interrupt
					sbi(GIFR, 5); // Clear PCINT flag.

					union32_t val_accum;
					val_accum.both32 = 0;

					while(num_meas--)
					{
						sei();
						MCUCR = 0b00101000; // ADC NOISE REDUCTION MODE.
						__asm__ __volatile__("sleep");
						cli();
						MCUCR = 0b00000000;

						val_accum.both32 += ADC;
					}

					ADCSRA = 0b00001110;  // disable ADC, disable ADC interrupt, clear int flag, prescaler 64 (125 kHz)

					sbi(PRR, 0);
					sbi(PORTB, 3); // disable resistor divider.


					if(tmp_rcv_data_b & 0b00010000)
						val_accum.both32 >>= 9; // /2048 and << 2

					if(tmp_rcv_data_b & 0b00001000)
					{	// Calibrate
						// val_accum is 0..4092 (can be compared as int16_t)
						int16_t offset = shadow.offsets[src];
						int16_t temp_corr = (int16_t)shadow.t_coeffs[src] * (int16_t)last_temp_diff;
						temp_corr >>= 8;
						offset += temp_corr;

						if(val_accum.slo16 > offset)
							val_accum.slo16 -= offset;
						else
							val_accum.lo16 = 0;

						uint16_t gain = shadow.gains[src].both;

						val_accum.both32 *= gain;
						val_accum.both32 >>= 14;

						if(val_accum.lo16 > 4095)
							val_accum.lo16 = 4095;
					}

					union16_t val;

					val.both = val_accum.lo16;

					if(src == INT_TEMP)
					{	// Store chip temperature result for future corrections
						last_temp_diff = val_accum.slo16 - 273*4 - 23*4;
					}

					// Result is 0b01000000 | (src<<4) | val.hi&0x0f.
					// This optimization removed about 22 bytes.
					src |= 0b00000100;
					src <<= 4;
					src |= (val.hi & 0x0f);

					if(tmp_rcv_data_b & 0b00010000)
					{	// Long measurement
						prev_long_meas.lo = val.lo;
						prev_long_meas.hi = src; // | (val.hi & 0x0f);
					}
					else
					{	// Single
						reply_data.b = src; // | (val.hi & 0x0f);
						reply_data.c = val.lo;
						manchester_send(&reply_data);
					}


					cbi(PORTB, 5); // Ext sens pull-up off
					sbi(GIMSK, 5); // Re-enable pin change interrupt

				}
				else if((rcv_data.b & 0b11110000) == 0b10100000) // Shunt for X time.
				{
					if(rcv_data.c == 0)
					{
						shunting = 0;
					}
					else
					{
						shunting = rcv_data.c;
						shunt_devices = rcv_data.b;
						shunt_i = SHUNT_I_START;
					}
				}
				else if(rcv_data.b == 0b10110000) // Change ID.
				{
					shadow.own_id = rcv_data.c;
					eeprom_write_byte((uint8_t*)0x20, shadow.own_id);
					recalc_eeprom_checksum(1);

					rcv_data.c += 1; // Next node gets the next ID.

					reply_data.a = shadow.own_id;
					reply_data.b = 0x00;
					reply_data.c = 0b10110000;
					manchester_send(&reply_data); // Send Operation Success msg
				}
				else if(rcv_data.b == 0xb1) //  Enable EEPROM change for address
				{
					eeprom_change_address = rcv_data.c;
				}
				else if(rcv_data.b == 0xb2) // Write 8-bit value
				{
					reply_data.c = 0xb2;
					if(eeprom_change_address > 127)
					{
						reply_data.b = 0x01; // OP not allowed
					}
					else
					{
						eeprom_write_byte((uint8_t*)(uint16_t)eeprom_change_address, rcv_data.c);
						recalc_eeprom_checksum(1);
						reply_data.b = 0x00; // OP success
						eeprom_change_address = 255; // remove access.
					}

					manchester_send(&reply_data);

				}
				else if(rcv_data.b == 0xb3) // Set state
				{
//					bod_disable = rcv_data.c & 1;
					debug = rcv_data.c & 2;

					reply_data.b = 0x00; // OP OK
					reply_data.c = 0xb3;
					manchester_send(&reply_data);
				}
				else if(rcv_data.b == 0xb4) // Reload variables
				{
					reload_variables();
					reply_data.b = 0x00;
					reply_data.c = 0xb4;
					manchester_send(&reply_data);
				}
				else if(rcv_data.b == 0xb5) //  Read eeprom.
				{
					reply_data.b = 0x05;
					reply_data.c = eeprom_read_byte((uint8_t*)(uint16_t)rcv_data.c);
					manchester_send(&reply_data);
				}
				else if(rcv_data.b == 0xb6) // Do self-test
				{
					reply_data.b = 0x06;
					reply_data.c = 0;
					if(recalc_eeprom_checksum(0))
						reply_data.c |= 0b00000001;

					manchester_send(&reply_data);
				}
				else
				{
					// Send an "unknown command" packet.

					reply_data.b = 0x02;
					reply_data.c = rcv_data.b;
					manchester_send(&reply_data);
				}

				if(rcv_data.b != 0xb1)
					eeprom_change_address = 255;

				if(allow_broadcast && (rcv_data.a == BROADCAST_ID))
				{
					_delay_ms(2.2); // (about 1.7 ms would be enough with no clock difference)
					manchester_send(&rcv_data);
				}
			}
			else // Not ours, relay as is.
			{
				manchester_send(&rcv_data);
			}
		}

	}

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
	// We were either in the "sleep" instruction or in the shunting loop when
	// the interrupt came, so go back where we were.
} // Grand total from the edge 12 us.

EMPTY_INTERRUPT(ADC_vect)
// Function stub needed here to get back from ADC Noise Reduction Sleep when
// the conversion is done.

