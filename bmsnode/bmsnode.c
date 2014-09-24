// EEPROM MAP
// Addr+1 is always a "checksum" of the previous 8-bit val (inverse)
// All values are 8-bit, starting at even addresses.
// 0x00: Num of mysterious boots
// 0x02: Num of power-on resets
// 0x04: Num of external resets
// 0x06: Num of brown-out resets
// 0x08: Num of watchdog resets
// Counters are saturated at 255.
// 0x0a, 0x0c,0x0e: Reserved for future use within node code.
// 0x10: Node ID
// 0x12: V calibration (128 = +0).
// 0x14: Ext sens without pull-up calibration (128 = +0)
// 0x16: Int Temp calibration (128 = +0).
// 0x18: Ext sens with pull-up calibration.
// 0x1a...0x3f: Reserved for future use within node code
// 0x40...0x7f: For free use by master.

#define F_CPU 8000000UL
#define VERSION_NUMBER 3

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

uint8_t own_id = 254;
uint8_t calibs[4] = {128, 128, 128, 128};

#define LED_ON()  cbi(PORTB, 1)
#define LED_OFF() sbi(PORTB, 1)

// Vcc - PB4(ADC2) - PB3
// normally: 1 - an.input - 1
// when measuring: 1 - an.input - 0
// when shunting:  1 - 0 - 1

#define MEAS_RESISTOR_SHUNT_ON()  sbi(DDRB, 4) 
#define MEAS_RESISTOR_SHUNT_OFF() cbi(DDRB, 4)

typedef union
{
	uint16_t both;
	struct { uint8_t lo; uint8_t hi; };
} union16_t;

// Return 1 if success, 0 if failure.
uint8_t get_eeprom_uint8(uint8_t addr, uint8_t* p_val)
{
	uint16_t address = addr;
	uint8_t val = eeprom_read_byte((uint8_t*)address);
	if(eeprom_read_byte((uint8_t*)(address+1)) != (~val))
		return 0;

	*p_val = val;

	return 1;
}

void put_eeprom_uint8(uint8_t addr, uint8_t val)
{
	uint16_t address = addr;
	eeprom_write_byte((uint8_t*)address, val);
	eeprom_write_byte((uint8_t*)(address+1), (~val));
}

uint8_t reload_variables()
{
	uint8_t fail = 0;
	uint8_t i = 0;
	for(uint8_t a = 0x12; a < 0x1a; a+=2)
	{
		if(!get_eeprom_uint8(a, &(calibs[i])))
		{
			calibs[i] = 128;
			fail++;
		}
		i++;
	}
	if(!get_eeprom_uint8(0x10, &own_id) ||
		own_id < 1 || own_id > 254)
	{
		own_id = 254;
		fail++;
	}
	return fail;
}

volatile uint8_t receiving = 0;

#define SHUNT_I_START 500000

#define VOLTAGE 0
#define EXT_SENS 1
#define INT_TEMP 2
#define EXT_SENS_PU 3

int main()
{
	uint8_t shunting = 0;

	uint32_t shunt_i = 0;
	uint8_t shunt_devices = 0b11;
	uint8_t eeprom_change_address = 255;
	uint8_t debug = 0;
	uint8_t bod_disable = 0;

	union16_t prev_long_meas;
	prev_long_meas.both = 0;

	data_t rcv_data;
	data_t reply_data;

	PRR = 0b00001111; // Unnecessary peripherals off.

	DDRB  = 0b00001011;
	PORTB = 0b00001101; // Manchester data input pull-up. Data output high. Volt meas disable.
	DIDR0 = 0b00110000; // disable digital input on ADC0 & ADC2.

	uint8_t tmp;
	uint8_t tmpsr = MCUSR;
	MCUSR = 0;

	uint8_t log_eepaddr = 0;
	if(tmpsr & 0x01)  // Power-on reset
	{
		log_eepaddr = 2;
	}
	else if(tmpsr & 0x02) // External reset
	{
		log_eepaddr = 4;
	}
	else if(tmpsr & 0x04) // Brown-out reset
	{
		log_eepaddr = 6;
	}
	else if(tmpsr & 0x08) // Watchdog reset
	{
		log_eepaddr = 8;
	}

	if(get_eeprom_uint8(log_eepaddr, &tmp) && tmp != 255)
		put_eeprom_uint8(log_eepaddr, tmp+1);


	reload_variables();

	LED_ON();
	if(own_id == 254)
		_delay_ms(500);

	_delay_ms(100);
	LED_OFF();


	GIMSK = 0b00100000; // pin change interrupt enable.
	PCMSK = 0b00000100; // PCINT2 enable.
  	sbi(GIFR,5); // Clear PCINT flag.
	sei();

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
		if(bod_disable)
		{
			// AVR team has come up with a hilarious
			// "timed sequence" which goes like this:
			MCUCR = 0b10000100;
			// max 4 clock cycles between these two
			MCUCR = 0b10110000;
			// Sleep must be entered in 3 clock cycles
		}
		else
		{
			MCUCR = 0b00110000;
		}
		__asm__ __volatile__("sleep");
		// NORMALLY, the program stays exactly here, until there is a falling
		// edge on the input pin.
		cli(); // 1 clk
		MCUCR = 0b00000000;  // 1 clk

		if(!receiving)
		{	// Interrupt came, but ISR did not set the flag
			// (pulse was filtered out by ISR)
			goto NO_RX_BACK_TO_SLEEP; 
		}
		// 4 clk

      SHUNT_RECEIVE:
		MEAS_RESISTOR_SHUNT_OFF();
		LED_OFF();

		receiving = 0; // 2 clk

		// Grand total from the edge: 15 us (measured).
		// (Added MEAS_RESISTOR_SHUNT_OFF() and LED_OFF() since then,
                // but shouldn't make big difference.

		ret = manchester_receive(&rcv_data); 

		reply_data.a = own_id;

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
			if(rcv_data.a == own_id || rcv_data.a == BROADCAST_ID)
			{
				if((rcv_data.b & 0b11100000) == 0b10000000)
				{
					uint8_t src = rcv_data.b & 0b00000011;
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
					case EXT_SENS:
						ADMUX = 0;
						break;
					case INT_TEMP:
						ADMUX = 0b1111;
						break;
					default:
						break;
					}

					if(rcv_data.b & 0b100) // 1V1 reference, else Vcc
						sbi(ADMUX, 7);

					uint16_t num_meas = 4;
					if(rcv_data.b & 0b00010000)
					{	// Long measurement - send old result and relay broadcast
						reply_data.b = prev_long_meas.hi;
						reply_data.c = prev_long_meas.lo;
						if(debug) LED_ON();
						manchester_send(&reply_data);
						LED_OFF();

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

					uint32_t val_accum = 0;
					while(num_meas--)
					{
						sei();
						MCUCR = 0b00101000; // ADC NOISE REDUCTION MODE.
						__asm__ __volatile__("sleep");
						cli();
						MCUCR = 0b00000000;

						val_accum += ADC;
					}

					ADCSRA = 0b00001110;  // disable ADC, disable ADC interrupt, clear int flag, prescaler 64 (125 kHz)

					sbi(PRR, 0);
					sbi(PORTB, 3); // disable resistor divider.

					union16_t val;

					if(rcv_data.b & 0b00010000)
						val.both = val_accum >> 9; // /2048 and << 2
					else
						val.both = val_accum;  // Quick reading

					if(rcv_data.b & 0b00001000)
					{	// Calibrate
						val.both -= 128;
						val.both += calibs[src];
					}

					if(rcv_data.b & 0b00010000)
					{	// Long measurement
						prev_long_meas.lo = val.lo;
						prev_long_meas.hi = 0b01000000 | (src<<4) | (val.hi & 0x0f);
					}
					else
					{	// Single
						reply_data.b = 0b01000000 | (src<<4) | (val.hi & 0x0f);
						reply_data.c = val.lo;
						if(debug) LED_ON();
						manchester_send(&reply_data);
						LED_OFF();
					}


					cbi(PORTB, 5); // Ext sens pull-up off
					sbi(GIMSK, 5); // Re-enable pin change interrupt

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
						rcv_data.c -= 1;
						shunting = 1;
						shunt_devices = 0b11;
						shunt_i = ((uint32_t)rcv_data.c) << 7 << debug;
					}

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
					own_id = rcv_data.c;
					put_eeprom_uint8(0x10, own_id);

					rcv_data.c += 1; // Next node gets the next ID.

					reply_data.b = 0x00;
					reply_data.c = 0b10110000;
					manchester_send(&reply_data); // Send Operation Success msg
				}
				else if(rcv_data.b == 0xb1) //  Enable EEPROM change for address
				{
					eeprom_change_address = rcv_data.c;
					reply_data.b = 0x00;
					reply_data.c = 0xb1;
					manchester_send(&reply_data);
				}
				else if(rcv_data.b == 0xb2) // Write 8-bit value
				{
					reply_data.c = 0xb2;
					if(eeprom_change_address == 255)
					{
						reply_data.b = 0x01; // OP not allowed
					}
					else
					{
						put_eeprom_uint8(eeprom_change_address, rcv_data.c);
						reply_data.b = 0x00; // OP success
						eeprom_change_address = 255; // remove access.
					}

					manchester_send(&reply_data);

				}
				else if(rcv_data.b == 0xb3) // Set state
				{
					bod_disable = rcv_data.c & 1;
					debug = rcv_data.c & 2;

					reply_data.b = 0x00; // OP OK
					reply_data.c = 0xb3;
					manchester_send(&reply_data);
				}
				else if(rcv_data.b == 0xb4) // Reload variables
				{
					reply_data.b = reload_variables();
					reply_data.c = 0xb4;
					manchester_send(&reply_data);
				}
				else if(rcv_data.b == 0xb5) // Node SW version
				{
					reply_data.b = 0x05;
					reply_data.c = VERSION_NUMBER;
					manchester_send(&reply_data);
				}
				else if((rcv_data.b&0b11110000) == 0b11000000) //  Read eeprom.
				{
					uint8_t cnt = (rcv_data.b&0x0f)+1;
					reply_data.b = 0b00010000;
					while(cnt--)
					{
						if(get_eeprom_uint8(rcv_data.c, &(reply_data.c)))
						{
							manchester_send(&reply_data);
							if(cnt)
							{
								_delay_ms(2.2);
								reply_data.b++;
								rcv_data.c += 2;
							}
						}
						else
						{
							reply_data.b = 0x04; // eeprom read error
							reply_data.c = rcv_data.c;
							manchester_send(&reply_data);
						}
					}

				}
				else
				{
					// Send an "unknown command" packet.

					reply_data.b = 0b00100000;
					reply_data.c = rcv_data.b;
					manchester_send(&reply_data);
				}

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
	// We were either in the "sleep" instruction or in the shunting loop when
	// the interrupt came, so go back where we were.
} // Grand total from the edge 12 us.

ISR(ADC_vect)
{
	// Function stub needed here to get back from ADC Noise Reduction Sleep when
	// the conversion is done.
}
