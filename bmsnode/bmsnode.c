// 2014-07-13: Changed pin mappings + LED to active low for new PCB layout.
// 2014-05-31: Doubled shunting current by utilizing measurement resistors.

// EEPROM MAP
// Addr+1 is always a "checksum" of the previous 8-bit val (inverse)
// All values are 8-bit, starting at even addresses.
// 0x00: Num of mysterious boots
// 0x02: Num of power-on resets
// 0x04: Num of external resets
// 0x06: Num of brown-out resets
// 0x08: Num of watchdog resets
// After counting to 255, 0 is skipped.
// 0x0a: Set to 123 for debug mode (blink LED more). Set to any other value to disable.
// 0x0c,0x0e: Reserved for future use within node code.
// 0x10: Node ID
// 0x12: V pre-calibration (128 = +0). Node needs to be reset for this to be applied.
// 0x14: T pre-calibration (128 = +0). ------------------ " " ----------------------
// 0x16...0x3f: Reserved for future use within node code
// 0x40...0x7f: For free use by master.

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

uint8_t OWN_ID = 254;

#define LED_ON()  cbi(PORTB, 1)
#define LED_OFF() sbi(PORTB, 1)

// Vcc - PB4(ADC2) - PB3
// normally: 1 - an.input - 1
// when measuring: 1 - an.input - 0
// when shunting:  1 - 0 - 1

#define MEAS_RESISTOR_SHUNT_ON()  sbi(DDRB, 4) 
#define MEAS_RESISTOR_SHUNT_OFF() cbi(DDRB, 4)

// Return 1 if success, 0 if failure.
uint8_t get_eeprom_uint8(uint8_t addr, uint8_t* p_val)
{
	uint16_t address = addr;
	uint8_t val = eeprom_read_byte((uint8_t*)address);
	if(eeprom_read_byte((uint8_t*)(address+1)) != val+1)
		return 0;

	*p_val = val;

	return 1;
}

void put_eeprom_uint8(uint8_t addr, uint8_t val)
{
	uint16_t address = addr;
	eeprom_write_byte((uint8_t*)address, val);
	eeprom_write_byte((uint8_t*)(address+1), val+1);
}

volatile uint8_t shunting = 0;
volatile uint8_t receiving = 0;

#define SHUNT_I_START 500000

int main()
{
	uint32_t shunt_i = 0;
	data_t rcv_data;
	data_t reply_data;
//	uint8_t read_store_byte = 0, write_store_byte = 0;
	uint8_t eeprom_change_address = 255;

//	OCR1C = 244;
//	TCCR1 = 0b00001111;

	PRR = 0b00001111;

	DDRB  = 0b00001011;
	PORTB = 0b00001101; // Manchester data input pull-up. Data output high. Volt meas disable.
	DIDR0 = 0b00010000; // disable digital input on ADC2.

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
	else
	{
		put_eeprom_uint8(log_eepaddr, 1);
	}

	uint8_t debug = 0;
	if(get_eeprom_uint8(0x0a, &tmp) && tmp == 123)
		debug = 1;

	uint8_t v_calib = 128;
	uint8_t t_calib = 128;
	if(get_eeprom_uint8(0x12, &tmp))
		v_calib = tmp;
	if(get_eeprom_uint8(0x14, &tmp))
		t_calib = tmp;


	if(!get_eeprom_uint8(0x10, &OWN_ID) ||
		OWN_ID < 1 || OWN_ID > 254)
	{
		// Say first-time hello (4 longer blinks)
		for(uint8_t i = 4; i>0; i--)
		{
			LED_ON();
			_delay_ms(250);
			LED_OFF();
			_delay_ms(250);
		}

		OWN_ID = 254;
		put_eeprom_uint8(0x10, OWN_ID);

	}
	else // if(!(tmpsr & 0x04))
	{       // If we have a brown-out reset, don't blink.
		// Say hello (3 short blinks)
		for(uint8_t i = 3; i>0; i--)
		{
			LED_ON();
			_delay_ms(100);
			LED_OFF();
			_delay_ms(100);
		}
	}


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
			LED_ON();
			MEAS_RESISTOR_SHUNT_ON();
			while(shunt_i)
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
		MCUCR = 0b00110000;
		__asm__ __volatile__("sleep");
		// NORMALLY, the program stays exactly here, until there is a falling
		// edge on the input pin.
		cli(); // 1 clk
		MCUCR = 0b00000000;  // 1 clk

		if(!receiving) // We woke up for nothing :(.
			goto NO_RX_BACK_TO_SLEEP; 
		// 4 clk

      SHUNT_RECEIVE:
		MEAS_RESISTOR_SHUNT_OFF();
		LED_OFF();

		receiving = 0; // 2 clk

		// Grand total from the edge: 15 us (measured).
		// (Added MEAS_RESISTOR_SHUNT_OFF() and LED_OFF() since then,
                // but shouldn't make big difference.

		ret = manchester_receive(&rcv_data); 

		if(ret) // Error, but got at least the start bit ok.
		{
			if(debug)
			{
				// Quickly blink leds to indicate error
				for(uint8_t i = 8; i>0; i--)
				{
					LED_ON();
					_delay_ms(50);
					LED_OFF();
					_delay_ms(50);
				}
			}

			// Send "communication error" packet

			reply_data.a = OWN_ID;
			reply_data.b = 0b00011100;
			reply_data.c = ret;
			manchester_send(reply_data);
		}
		else  // Got the packet OK.
		{

			if(rcv_data.a == OWN_ID || rcv_data.a == BROADCAST_ID)
			{
				if(rcv_data.b == 0b10000100 ||  // Measure & Report V   or
				   (rcv_data.b & 0b11111100) == 0b10001000)    // Measure & Report T
				{
					MEAS_RESISTOR_SHUNT_OFF();
					cbi(PRR, 0);
 
					if(rcv_data.b == 0b10000100) // Voltage
					{
						cbi(PORTB, 3); // enable resistor divider.
						ADMUX = 0b10000010; // 1V1 reference, ADC2.
					}
					else // Temperature
					{
						// 00 = internal sensor
						// 11 = external sensor w/ Vcc reference
						// 01 = external sensor w/ 1V1 reference
						// 10 = reserved, currently same as 01
						if((rcv_data.b&0b11) == 0b00)
							ADMUX = 0b10001111; // 1V1 reference, ADC4 (internal temp. sensor).
						else if((rcv_data.b&0b11) == 0b11)
							ADMUX = 0b00000000; // Vcc reference, ADC0 (reset pin)
						else
							ADMUX = 0b10000000; // 1V1 reference, ADC0 (reset pin)
					}


					ADCSRA = 0b10011110;  // enable ADC, enable ADC interrupt, clear int flag, prescaler 64 (125 kHz)
					_delay_ms(2); // let the filter capacitor charge and ADC stabilize. // 2 ms = 20x R*C time

					cbi(GIMSK, 5); // Disable pin change interrupt
					sbi(GIFR,5); // Clear PCINT flag.
					sei();
					MCUCR = 0b00101000; // ADC NOISE REDUCTION MODE.
					__asm__ __volatile__("sleep");
					cli();
					MCUCR = 0b00000000;
					sbi(GIMSK, 5); // Re-enable pin change interrupt

					union
					{
						uint16_t both;
						struct { uint8_t lo; uint8_t hi; };
					} val;

					val.both = ADC;

					ADCSRA = 0b00001110;  // disable ADC, disable ADC interrupt, clear int flag, prescaler 64 (125 kHz)

					sbi(PRR, 0);
					sbi(PORTB, 3); // disable resistor divider.

					val.both -= 128;

					if(rcv_data.b == 0b10000100) // Voltage
					{
						val.both += v_calib;
						reply_data.c = val.lo;
						reply_data.b = 0b00000100 | (val.hi & 0b00000011);
					}
					else  // Temperature
					{
						val.both += t_calib;
						reply_data.c = val.lo;
						reply_data.b = 0b00001000 | (val.hi & 0b00000011);
					}

					reply_data.a = OWN_ID;

					if(debug) LED_ON();
					manchester_send(reply_data);
					LED_OFF();

					// Compensate for charge usage difference in the chain.
					// Later nodes use more charge to relay all the messages from
					// the chain. The measurement command data field indicates
					// how many nodes are after this node in the chain.
					// For every node, 480 us of shunting compensates
					// for this difference.
					// This would be 240 shunt loops, but approximate
					// with 256 to remove the large multiplication.
					// -- after doubling the current, 120 -> 128.

					if(rcv_data.c)
					{
						rcv_data.c -= 1;
						shunting = 1;
						shunt_i = ((uint32_t)rcv_data.c) << 7;
					}

				}
				else if(rcv_data.b == 0b10101100) // Shunt for X time.
				{
					if(rcv_data.c == 0)
					{
						shunting = 0;
					}
					else
					{
						shunting = rcv_data.c;
						shunt_i = SHUNT_I_START;
					}
				}
				else if(rcv_data.b == 0b10001100) // Change ID.
				{
					OWN_ID = rcv_data.c;
					put_eeprom_uint8(0x10, OWN_ID);

					if(rcv_data.a == BROADCAST_ID)
							rcv_data.c += 1; // Next node gets the next ID.

					reply_data.a = OWN_ID;
					reply_data.b = 0b00001100;
					reply_data.c = 0x10;
					manchester_send(reply_data); // Send EEPROM MODIFICATION SUCCEEDED reply.
				}
				else if(rcv_data.b == 0b10010000) //  Retrieve 8-bit eeprom value.
				{
					reply_data.a = OWN_ID;
					uint8_t val;
					if(get_eeprom_uint8(rcv_data.c, &val))
					{
						reply_data.b = 0b00010000;
						reply_data.c = val;
					}
					else
					{
						reply_data.b = 0b00010100; // eeprom read error
						reply_data.c = rcv_data.c;
					}

					manchester_send(reply_data);

				}
				else if(rcv_data.b == 0b10101000) //  Enable EEPROM change for address
				{
					eeprom_change_address = rcv_data.c;
					reply_data.a = OWN_ID;
					reply_data.b = 0b00011000;
					manchester_send(reply_data);

				}
				else if(rcv_data.b == 0b10011100) // Write 8-bit value
				{
					reply_data.a = OWN_ID;
					reply_data.c = eeprom_change_address;
					if(eeprom_change_address == 255)
					{
						reply_data.b = 0b00100100; // access denied
						manchester_send(reply_data);
					}
					else
					{
						put_eeprom_uint8(eeprom_change_address, rcv_data.c);
						reply_data.b = 0b00001100; // eeprom write success.
						manchester_send(reply_data);
						eeprom_change_address = 255; // remove access.
					}
				}
				else
				{
					// Send a "unknown command" packet.

					reply_data.a = OWN_ID;
					reply_data.b = 0b00100000;
					reply_data.c = rcv_data.b;
					manchester_send(reply_data);
				}

				if(rcv_data.a == BROADCAST_ID)
				{
					_delay_ms(2.2); // (about 1.7 ms would be enough with no clock difference)
					manchester_send(rcv_data);
				}
			}
			else // Not ours, relay as is.
			{
				manchester_send(rcv_data);
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
	// We were either in the "sleep" instruction or in the shunting loop structure when
	// the interrupt came, so go back where we were.
} // Grand total from the edge 12 us.

ISR(ADC_vect)
{
	// Function stub needed here to get back from ADC Noise Reduction Sleep when
	// the conversion is done.
}

