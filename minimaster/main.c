#define F_CPU 16000000UL

#include <inttypes.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>

#define cbi(x,y) x &= ~(1<<(y))
#define sbi(x,y) x |= (1<<(y))

#define POUT1_OFF() cbi(PORTC, 2)
#define POUT1_ON()  sbi(PORTC, 2)
#define POUT2_OFF() cbi(PORTC, 4)
#define POUT2_ON()  sbi(PORTC, 4)
#define POUT3_OFF() cbi(PORTC, 5)
#define POUT3_ON()  sbi(PORTC, 5)
#define POUT4_OFF() cbi(PORTD, 2)
#define POUT4_ON()  sbi(PORTD, 2)

#define ADMUX_REF() {ADMUX = 0b01000111;}
#define ADMUX_SIG() {ADMUX = 0b01000110;}

#define POWER_PWM(val) {OCR0B = (val);}
#define LOGIC_PWM(val) {OCR0A = (val);}

volatile uint16_t tmp = 420;

ISR(ADC_vect)
{
	tmp = ADC;
//	int16_t val = ADC<<6;
//	raw_I_acc += val;
//	val -= adc_offset;
//	val /= adc_divider;

//	if(val > max_I_tmp)
//		max_I_tmp = val;
//	if(val < min_I_tmp)
//		min_I_tmp = val;

//	I_acc += val;
//	num_I_meas++;
}


#define print_char(byte) {while((UCSR0A & 0b00100000) == 0) ; UDR0 = (byte); }

void print_string(const char* str)
{
	while(str[0] != 0)
	{
		print_char(str[0]);
		str++;
	}
}


int main()
{
	DDRC = 0b00110100;
	DDRD = 0b01100100;
	DDRB = 0b00001000;

	UBRR0 = 16;
	UCSR0A = 0b10;
	UCSR0B = 0b00011000;
	UCSR0C = 0b00001110;

	ADMUX_REF();

	ADCSRB = 0b00000000;
	ADCSRA = 0b11101111; // ADC interrupt on, prescaler = 128 -> 125 kHz -> interrupt at 9.6 kHz

	TCCR0A = 0b10100001; // PWM mode 1.
	TCCR0B = 0b00000100; // Prescaler=256 -> f = 31.25 kHz.

	PRR = 0b10000000; // Shut down TWI, saves 0.3 mA.

	sei();
	int i = 0;

	POWER_PWM(50);
	LOGIC_PWM(100);
	while(1)
	{
		char buf[10];

		_delay_ms(1000);
		cli();
		utoa(tmp, buf, 10);
		sei();
		print_string(buf);

		if(i&1)
		{
			print_string(" REF\n\r");
			ADMUX_SIG();
		}
		else
		{
			print_string(" SIG\n\r");
			ADMUX_REF();
		}

		i++;
	}
	return 0;
}
