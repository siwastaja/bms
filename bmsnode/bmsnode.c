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
// 0x12: V offset (128 = +0).
// 0x14: Ext sens offset (128 = +0)
// 0x16: Int Temp offset (128 = +0).
// 0x18: V gain adjust (128 = +0)
// 0x1a: Ext sens gain adjust (128 = +0)
// 0x1c: Int temp gain adjust (128 = +0)
// 0x1e: V temp coeff adjust (128 = +0)
// 0x20: Ext sens temp coeff adjust (128 = +0)
// 0x22: Int temp temp coeff adjust (128 = +0). Should always be 128.
// 0x24: Base gain & shift for V
// 0x26: Base gain & shift for ext T
// 0x28: Base gain & shift for int T
// ...0x3f: Reserved for future use within node code
// 0x40...0x7f: For free use by master.


// Two-point calibration.
/*

    |                    /
    |                  /
RAW2|- - - - - - - - x
    |              / |
    |            /
    |          /     |
RAW1|- - - - x
    |      / |       |
    |    /
    |  /     |       |
OFS x/
    |        |       |
    --------------------------
    0       V1      V2

V2 = 4.100V (general) or 3.500V (LFP special)
V1 = 2.900V (general)

################################
gain = (RAW2 - RAW1) / (V2 - V1)
gain := 1/gain
################################

gain = (RAW1 - OFS) / (V1 - 0)
gain = (RAW2 - OFS) / (V2 - 0)

(RAW1 - OFS1) / (V1) = (RAW2 - RAW1) / (V2 - V1)
RAW1 - OFS1 = V1(RAW2 - RAW1) / (V2 - V1)
-OFS1 = V1(RAW2 - RAW1) / (V2 - V1)  - RAW1
OFS1 = -V1(RAW2 - RAW1) / (V2 - V1) + RAW1
OFS1 = V1(RAW1 - RAW2) / (V2 - V1) + RAW1

(RAW2 - OFS)2 / V2 = (RAW2 - RAW1) / (V2 - V1)
RAW2 - OFS2 = V2(RAW2 - RAW1) / (V2 - V1)
-OFS2 = V2(RAW2 - RAW1) / (V2 - V1) - RAW2
OFS2 = -V2(RAW2 - RAW1) / (V2 - V1) + RAW2
OFS2 = V2(RAW1 - RAW2) / (V2 - V1) + RAW2

###################
OFS = (OFS1+OFS2)/2
###################

Applying correction

Vcalibrated = (Vraw - OFS)*(gain + lastT*t_coeff)

Basic gain 10-bit to millivolts/2: 9/4 = 2.25
Basic gain 12-bit to millivolts/2: 9/16 = 0.5625

T internal: 12-bit output = kelvins (9 LSbits used)
V: 12-bit output = millivolts/2 (2000 means 4.000V)


Ex.1
Calibration points 3V, 4V
Raw @ 3000mV = 320
Raw @ 4000mV = 410
gain = 1/ ((410 - 320) / 1000) = 11.1111
OFS1 = 3000(-90) / 1000 + 320 = 50
OFS2 = 4000(-90) / 1000 + 410 = 50
OFS = 50
Convert 320: 11.1111*(320 - 50) = 3000
Convert 410: 11.1111*(410 - 50) = 4000
Ex.2
Raw @ 3000mV = 310
Raw @ 4000mV = 420
gain = 1/((420-310) / 1000)) = 9.090909
OFS1 = 3000(-110) / 1000 + 310 = -20
OFS2 = 4000(-110) / 1000 + 420 = -20
OFS = -20
Convert 310: 9.090909*(310 + 20) = 3000
Convert 420: 9.090909*(420 + 20) = 4000

Implementation:

12-bit data in (0...4095)
-= offset: Directly an int8_t (-128..+127)

gain (uint16_t): base_gain(uint16_t) + gain_adjust(int8_t) + (temperature-temp_coeff_midpoint)(int8_t)*temp_coeff(int8_t)
result = gain(uint16_t) * data(uint16_t with 12 bits)




*/


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

uint8_t own_id;

#define BASEGAIN_BITS 0b00011111
#define SHIFT_BITS    0b11100000

typedef union
{
	struct { int8_t offsets[3]; int8_t gains[3]; int8_t t_coeffs[3]; uint8_t shifts_basegains[3];};
	uint8_t all[12];
} calibs_t;

calibs_t calibs;

int8_t last_temp_diff; // difference of latest chip temperature and temp coeff midpoint (+23 degC)

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
// about 7 bytes of stack
uint8_t get_eeprom_uint8(uint8_t addr, uint8_t* p_val)
{
	uint16_t address = addr;
	uint8_t val = eeprom_read_byte((uint8_t*)address);
	if(eeprom_read_byte((uint8_t*)(address+1)) != (~val))
		return 0;

	*p_val = val;

	return 1;
}

// about 5 bytes of stack
void put_eeprom_uint8(uint8_t addr, uint8_t val)
{
	uint16_t address = addr;
	eeprom_write_byte((uint8_t*)address, val);
	eeprom_write_byte((uint8_t*)(address+1), (~val));
}

// about 10 bytes of stack
uint8_t reload_variables()
{
	uint8_t fail = 0;
	uint8_t i = 0;
	for(uint8_t a = 0x12; a < 0x2a; a+=2)
	{
		if(!get_eeprom_uint8(a, &(calibs.all[i])))
		{
			calibs.all[i] = 128;
			put_eeprom_uint8(a, 128);
			fail++;
		}
		i++;
	}
	if(!get_eeprom_uint8(0x10, &own_id) ||
		own_id < 1 || own_id > 254)
	{
		own_id = 254;
		put_eeprom_uint8(0x10, 254);
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

// Stack: about 34 bytes + 10 from functions = 44
// Available sram for .data = about 80 bytes
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

					if(rcv_data.b & 0b100) // 1V1 reference, else Vcc
						sbi(ADMUX, 7);

					uint16_t num_meas = 4;
					if(rcv_data.b & 0b00010000)
					{	// Long measurement - send old result and relay broadcast
						reply_data.b = prev_long_meas.hi;
						reply_data.c = prev_long_meas.lo;
						if(debug) LED_ON();
						manchester_send(&reply_data);

						if(rcv_data.a == BROADCAST_ID)
						{
							allow_broadcast = 0;
							_delay_ms(2.2);
							manchester_send(&rcv_data);
						}
						LED_OFF();

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
						val_accum >>= 9; // /2048 and << 2

					if(rcv_data.b & 0b00001000)
					{	// Calibrate
						// V: 0..4095
						// Offsets adjustment range:
						// +/- 3.1% (0.024% step)
						val_accum -= calibs.offsets[src];
						// Basegain = 1..32 (V: 9 or 18)
						// << 8 -> 128..8192
						// Adjustment range: +/-50%(0.39% step) (basegain = 1)
						//               to: +/-1.6%(0.012% step) (basegain = 32)
						uint16_t gain = (calibs.shifts_basegains[src]&BASEGAIN_BITS)+1;
						gain <<= 8;
						gain += calibs.gains[src]; // gain finetune (-128..+127)

						int16_t temp_corr = (int16_t)calibs.t_coeffs[src] * (int16_t)last_temp_diff;
						temp_corr >>= 8;
						gain += temp_corr;

						val_accum *= gain;
						val_accum >>= 8 + ((calibs.shifts_basegains[src]&SHIFT_BITS)>>5);
					}
					else
					{
						val.both = val_accum;
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

					if(src == INT_TEMP)
					{	// Store chip temperature result for future corrections
						last_temp_diff = (int16_t)val.both-273-23;
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
