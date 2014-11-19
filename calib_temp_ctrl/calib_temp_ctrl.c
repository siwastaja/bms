#define F_CPU 7372000UL

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

#define COOL 0
#define HEAT 1
#define OFF  2

#define MAX_PWR 65535

#define K_AT_0C 2731

// Temperature unit = 0.1 K
// signed ints used because temperatures are subtracted all the time to get differences.

// Time unit = 1 second.

uint16_t max_peak_pwr[2] = {65535, 30000};
uint16_t max_avg_pwr[2] = {65535, 20000};

int16_t ambient_temp = 235 + K_AT_0C;

// steady-state power requirement (heat escape) per temp unit of difference from ambient temp
int16_t steady_pwr_per_unit[2] = {100, 50};

// P factor (pwr per unit of temp difference from setpoint)
// must be int16_t so that * goes ok.
int16_t pwr_per_unit[2] = {1600, 800};

// Cumulative average of power. Use 2^n-1 for optimum performance.
// Same cumulative average is used for both heating and cooling. This way,
// power stays limited when changing from heating to cooling, to avoid thermal stress.
#define CUMUL_AVG_LEN 63
uint16_t pwr_avg;

#define MODE_CHANGE_DELAY 60
#define OK_LED_THRESHOLD 3
#define OK_LED_BLINK_THRESHOLD 7

uint8_t mode = OFF;

uint8_t clock;

// Port definitions
#define HEAT_ENA_REG PORTD
#define HEAT_ENA_PIN 6
#define COOL_ENA_REG PORTD
#define COOL_ENA_PIN 7

// OC1A (= PB1 = pin13) = HEAT PWM
// OC1B (= PB2 = pin14) = COOL PWM
#define HEAT_PWM OCR1A
#define COOL_PWM OCR1B

// shadows for printing:
uint16_t cool_pwm_shadow;
uint16_t heat_pwm_shadow;

#define HEAT_PWM_DISABLE() {cbi(TCCR1A, 7); cbi(PORTB, 1);}
#define HEAT_PWM_ENABLE()  {sbi(TCCR1A, 7);}
#define COOL_PWM_DISABLE() {cbi(TCCR1A, 5); cbi(PORTB, 2);}
#define COOL_PWM_ENABLE()  {sbi(TCCR1A, 5);}

#define HEAT_LED_ON()  sbi(PORTD, 3)
#define HEAT_LED_OFF() cbi(PORTD, 3)
#define COOL_LED_ON()  sbi(PORTD, 4)
#define COOL_LED_OFF() cbi(PORTD, 4)
#define OK_LED_ON()    sbi(PORTD, 2)
#define OK_LED_OFF()   cbi(PORTD, 2)

#define SENS_PORT_REG PORTC
#define SENS_PIN_REG  PINC
#define SENS_DDR_REG  DDRC
#define SENS_1_PIN 4
#define SENS_2_PIN 3
#define SENS_3_PIN 5
#define SENS_4_PIN 2

/* Peltier is driven by a H bridge of four mosfets.

+12V----+-------------+
        |             |
      Qheat_hi      Qcool_hi
        |             |
        +---PELTIER---+
        |             |
      Qcool_lo      Qheat_lo
        |             |
GND-----+-------------+

Extreme caution must be taken so that (Qheat_hi and
Qcool_lo) or (Qcool_hi and Qcool_lo) are not on at the same time.

Hi side fets select COOL/HEAT and only change state when mode
changes (no PWM). Low side is PWM'd. Thus, high side uses slower
drivers.

*/

char tmpstr[30];

#define print_char(byte) {while((UCSRA & 0b00100000) == 0) ; UDR = (byte); }

void print_string(const char* str)
{
	while(str[0] != 0)
	{
		print_char(str[0]);
		str++;
	}
}


#define print_int(val) {itoa(val, tmpstr, 10); print_string(tmpstr);}

void print_t(int16_t val, uint8_t convert_c)
{
	char buf[10];
	itoa(val-(convert_c?K_AT_0C:0), buf, 10);
	uint8_t i;
	for(i = 0; i < 10; i++)
	{
		if(buf[i] == 0)
		{
			if(i < 1 || i > 8)
				break;
			buf[i+1] = 0;
			buf[i] = buf[i-1];
			buf[i-1] = '.';
			goto PRINT_T_OK;
		}
	}
	buf[0] = 'E'; buf[1] = 'R'; buf[2] = 'R'; buf[3] = 0;
	PRINT_T_OK:
	print_string(buf);
}

void onewire_wr_byte(uint8_t pin, uint8_t val)
{
	for(uint8_t i = 0; i < 8; i++)
	{
		cbi(SENS_PORT_REG, pin);
		sbi(SENS_DDR_REG, pin);
		if(val & 1)
		{
			_delay_us(9.0);
			cbi(SENS_DDR_REG, pin);
			_delay_us(71.0);
		}
		else
		{
			_delay_us(75.0);
			cbi(SENS_DDR_REG, pin);
			_delay_us(5.0);
		}
		val >>= 1;
	}
}

uint8_t onewire_rd_byte(uint8_t pin)
{
	uint8_t pinmask = 1<<pin;
	uint8_t val = 0;
	for(uint8_t i = 0; i < 8; i++)
	{
		val >>= 1;
		sbi(SENS_DDR_REG, pin);
		_delay_us(5.0);
		cbi(SENS_DDR_REG, pin);
		_delay_us(5.0);
		if(SENS_PIN_REG & pinmask)
		{
			val |= 0b10000000;
		}
		_delay_us(70.0);
	}

	return val;
}

uint8_t reset_n_presence(uint8_t pin)
{
	sbi(SENS_DDR_REG, pin); // Reset pulse
	_delay_us(600.0); // "480 us or longer"
	cbi(SENS_DDR_REG, pin);
	_delay_us(70.0); // This is a very critical timing and DS18B20 is flawed here.
	if(SENS_PIN_REG & (1<<pin))
	{ // No presence pulse
		_delay_us(600.0);
		return 1;
	}

	_delay_us(500.0); // "minimum" 480 us from cbi(SENS_DDR_REG)
	return 0;
}

// Always keep SENS_PORT_REG at value 0. '1' is never written.
// Returns temperature as 0.1Kelvins (2731 is 0 deg C)
// Returns 0 on error.
// Call this once per second max, because the conversion starts at the end of the function.
// The result of previous conversion is returned, so the first result is wrong.
int16_t read_temp(uint8_t pin)
{
	typedef union
	{
		uint16_t uboth;
		int16_t sboth;
		struct {uint8_t lo; uint8_t hi;};
	} union16_t;

	union16_t val;

	if(reset_n_presence(pin))
		return 0;

	onewire_wr_byte(pin, 0xcc); // SKIP ROM
	onewire_wr_byte(pin, 0xbe); // Read scratchpad.

	val.lo = onewire_rd_byte(pin);
	val.hi = onewire_rd_byte(pin);

	if(reset_n_presence(pin))
		return 0;
	onewire_wr_byte(pin, 0xcc); // SKIP ROM
	onewire_wr_byte(pin, 0x44); // Convert T.
	val.sboth += 4370; // To kelvin, still in 1/16 units.
	// * 10/16 = 5/8 -> to 1/10th kelvins.
	val.uboth *= 5;
	val.uboth >>= 3;
	return val.sboth;
}


// Sets new power level. Limits the value if necessary.
// Selects cooling or heating based on mode.
void update_pwr(uint16_t pwr)
{
	// This function also takes care of cool/heat turnover by
	// switcing hi-side FETs. Some deadtime is inserted by
	// turning hi-side FET off, then doing calculation, then turning another on.

	if(mode != HEAT)
	{
		cbi(HEAT_ENA_REG, HEAT_ENA_PIN);
		HEAT_PWM_DISABLE();
		HEAT_PWM = 0;
		heat_pwm_shadow = 0;
	}
	if(mode != COOL)
	{
		cbi(COOL_ENA_REG, COOL_ENA_PIN);
		COOL_PWM_DISABLE();
		COOL_PWM = 0;
		cool_pwm_shadow = 0;
	}

	if(mode == OFF)
		return;

	if(pwr > max_peak_pwr[mode])
		pwr = max_peak_pwr[mode];

	uint16_t pwr_avg_next = (pwr + CUMUL_AVG_LEN*(uint32_t)pwr_avg) / (CUMUL_AVG_LEN+1);

	if(pwr_avg_next > max_avg_pwr[mode])
	{
		pwr -= (pwr_avg_next-max_avg_pwr[mode])*CUMUL_AVG_LEN;
		pwr_avg_next = (pwr + CUMUL_AVG_LEN*(uint32_t)pwr_avg) / (CUMUL_AVG_LEN+1);
	}

	pwr_avg = pwr_avg_next;

	if(mode == COOL)
	{
		COOL_PWM = pwr;
		cool_pwm_shadow = pwr;
		sbi(COOL_ENA_REG, COOL_ENA_PIN);
	}
	else if(mode == HEAT)
	{
		HEAT_PWM = pwr;
		heat_pwm_shadow = pwr;
		sbi(HEAT_ENA_REG, HEAT_ENA_PIN);
	}

//	print_string("upd_p ");
//	if(mode == COOL) print_string("C ");
//		else print_string("H ");
//	itoa(pwr,tmpstr,10); print_string(tmpstr); print_char(' ');
//	itoa(pwr_avg,tmpstr,10); print_string(tmpstr); print_string("\r\n");
}

#define abs_own(x) (((x)<0)?(-1*(x)):(x))

void update_mode()
{
	COOL_PWM = 0;
	HEAT_PWM = 0;
	cool_pwm_shadow = 0;
	heat_pwm_shadow = 0;

	COOL_PWM_DISABLE();
	cbi(COOL_ENA_REG, COOL_ENA_PIN);
	HEAT_PWM_DISABLE();
	cbi(HEAT_ENA_REG, HEAT_ENA_PIN);
	COOL_LED_OFF();
	HEAT_LED_OFF();

	if(mode == HEAT)
	{
		HEAT_LED_ON();
		_delay_ms(1);
		sbi(HEAT_ENA_REG, HEAT_ENA_PIN);
		HEAT_PWM_ENABLE();
	}
	else if(mode == COOL)
	{
		COOL_LED_ON();
		_delay_ms(1);
		sbi(COOL_ENA_REG, COOL_ENA_PIN);
		COOL_PWM_ENABLE();
	}
}


void adjust(int16_t temp_actual, int16_t temp_setpoint)
{
	static uint8_t mode_change_cnt = 0;
	int32_t new_pwr_32;
	if(mode == OFF)
		return;
	READJ:
	// new_pwr: positive = heating needed, negative = cooling needed
	// Heat escape compensation:
	new_pwr_32 = (int32_t)steady_pwr_per_unit[mode]*(temp_setpoint-ambient_temp);

//	print_string("adj: new_p/100 pre ");
//	itoa(new_pwr_32/100,tmpstr,10);
//	print_string(tmpstr);

	// P term (power correction based on error term):
	new_pwr_32 += (int32_t)pwr_per_unit[mode]*(int32_t)(temp_setpoint-temp_actual);

//	print_string(" post ");
//	itoa(new_pwr_32/100,tmpstr,10);
//	print_string(tmpstr);
//	print_string("\r\n");

	// Change mode if new_pwr sign disagrees with current mode for
	// long enough.
	if((new_pwr_32 < 0 && mode == HEAT) || (new_pwr_32 > 0 && mode == COOL))
	{
		mode_change_cnt++;
//		print_string("adj: m_chg_cnt ");
//		itoa(mode_change_cnt,tmpstr,10);
//		print_string(tmpstr);
//		print_string("\r\n");

		if(mode_change_cnt > MODE_CHANGE_DELAY)
		{
			mode_change_cnt = 0;
			if(mode == COOL) mode = HEAT;
			else if(mode == HEAT) mode = COOL;
			update_mode();
			goto READJ;
		}
	}
	else
	{
		mode_change_cnt = 0;
	}

	if(mode==COOL)
	{
		COOL_LED_ON();
		HEAT_LED_OFF();
	}
	else // mode==HEAT
	{
		HEAT_LED_ON();
		COOL_LED_OFF();
	}

	// Blink the "wrong" LED every second if the mode is going to possibly change.
	if(mode_change_cnt&1)
		(mode==COOL)?(HEAT_LED_ON()):(COOL_LED_ON());
	else
		(mode==COOL)?(HEAT_LED_OFF()):(COOL_LED_OFF());



	// If new_pwr sign disagrees with current mode, just ask for
	// zero power. The mode will change if this situation goes on.
	uint16_t new_pwr_16;
	if(mode == COOL)
	{
		if(new_pwr_32 > 0)
			new_pwr_32 = 0;

		new_pwr_32 *= -1;
	}
	else // mode == HEAT
	{
		if(new_pwr_32 < 0)
			new_pwr_32 = 0;
	}

	if(new_pwr_32 > MAX_PWR)
		new_pwr_16 = MAX_PWR;
	else
		new_pwr_16 = new_pwr_32;

	update_pwr(new_pwr_16);

	if((abs_own(temp_setpoint-temp_actual) <= OK_LED_BLINK_THRESHOLD && clock&1) ||
	   (abs_own(temp_setpoint-temp_actual) <= OK_LED_THRESHOLD))
		OK_LED_ON();
	else
		OK_LED_OFF();

}

void stop_fatal()
{
	HEAT_PWM = 0;
	COOL_PWM = 0;
	COOL_PWM_DISABLE();
	HEAT_PWM_DISABLE();
	cbi(COOL_ENA_REG, COOL_ENA_PIN);
	cbi(HEAT_ENA_REG, HEAT_ENA_PIN);
	print_string("\r\n\r\nFATAL ERROR: STOPPED.\r\n");
	while(1)
	{
		OK_LED_ON();
		HEAT_LED_ON();
		COOL_LED_ON();
		_delay_ms(200);
		OK_LED_OFF();
		HEAT_LED_OFF();
		COOL_LED_OFF();
		_delay_ms(200);
	}
}

#define RX_BUF_SIZE 50
char rx_buf[RX_BUF_SIZE];
uint8_t rx_point;

char* comp_str(char* str1, char* str2)
{
	while(*str2 != 0)
	{
		if(*str1 != *str2)
			return 0;
		str1++;
		str2++;
	}
	return str1;
}

int16_t temp_setpoint = K_AT_0C+200;

ISR(USART_RXC_vect)
{
	char byte = UDR;
	if(byte == ';')
	{
		// rx_point holds the last position.
		rx_buf[rx_point] = 0;
//		print_string(rx_buf);
		char* p_val;
		if((p_val = comp_str(rx_buf, "SET")))
		{
			int16_t new_setpoint = atoi(p_val);
			if(new_setpoint < -100 || new_setpoint > 200)
			{
				print_string("\r\nSET CMD OUT OF RANGE\r\n");
			}
			else
			{
				temp_setpoint = K_AT_0C+new_setpoint*10;
			}
		}
		else if(comp_str(rx_buf, "COOL"))
		{
			if(mode == OFF)
				mode = COOL;
			update_mode();
		}
		else if(comp_str(rx_buf, "HEAT"))
		{
			if(mode == OFF)
				mode = HEAT;
			update_mode();
		}
		else if(comp_str(rx_buf, "OFF"))
		{
			mode = OFF;
			update_mode();
		}
		rx_point = 0;
	}
	else
	{
		if(byte >= 'a' && byte <= 'z')
			byte = byte - 'a' + 'A';
		rx_buf[rx_point] = byte;
		rx_point++;
		if(rx_point >= RX_BUF_SIZE)
			rx_point = 0;
//		print_char(byte);
	}
}

int main()
{
	DDRD = 0b11011100;
	PORTD = 0;
	DDRC = 0;
	PORTC = 0;

	UCSRB = 0b10011000;
	UCSRC = 0b10000110;
	UBRRL = 3; // 115200 bps


	_delay_ms(2000);
	OK_LED_ON();

	HEAT_PWM = 0;
	COOL_PWM = 0;


	DDRB = 0b00000110;
	TCCR1A = 0b00000010;
	TCCR1B = 0b00011001;
	ICR1 = MAX_PWR;

	cbi(COOL_ENA_REG, COOL_ENA_PIN);
	cbi(HEAT_ENA_REG, HEAT_ENA_PIN);

	int16_t temps[3];
	int16_t temp_last_avg = K_AT_0C+230;

	read_temp(SENS_1_PIN);
	read_temp(SENS_2_PIN);
	read_temp(SENS_3_PIN);
	_delay_ms(1000);
	mode = OFF;
	update_mode();

	uint8_t prev_prev_num_sens_err = 0;
	uint8_t prev_num_sens_err = 0;

	uint8_t t = 0;
	uint8_t stable = 0;
	uint8_t stable_cnt = 0;
	uint8_t stable_cnt_req = 120;
	int16_t stable_allowed_error = 5;
	sei();
	while(1)
	{
		int16_t tmp_temps[3];
		tmp_temps[0] = read_temp(SENS_1_PIN);
		tmp_temps[1] = read_temp(SENS_2_PIN);
		tmp_temps[2] = read_temp(SENS_3_PIN);

		uint8_t num_sens_err = 0;
		for(uint8_t i = 0; i < 3; i++)
		{
			if(tmp_temps[i] < 1000 || tmp_temps[i] > 5000)
			{
				num_sens_err++;
				temps[i] = temp_last_avg;
			}
			else
			{
				temps[i] = tmp_temps[i];
			}
		}

		if(num_sens_err && prev_num_sens_err && prev_prev_num_sens_err)
		{
			stop_fatal();
		}
		prev_prev_num_sens_err = prev_num_sens_err;
		prev_num_sens_err = num_sens_err;

		int16_t diff1 = abs_own(temps[0]-temps[1]);
		int16_t diff2 = abs_own(temps[0]-temps[2]);
		int16_t diff3 = abs_own(temps[1]-temps[2]);

		int16_t diff = (diff1>diff2)?diff1:diff2;
		if(diff3 > diff) diff = diff3;

		temp_last_avg = (temps[0]+temps[1]+temps[2])/3;

		int16_t err = abs_own(temp_last_avg-temp_setpoint);
		stable = 0;
		if(diff <= stable_allowed_error && err <= stable_allowed_error)
		{
			if(stable_cnt >= stable_cnt_req)
			{
				stable = 1;
			}
			else
			{
				stable_cnt++;
			}
			OK_LED_ON();
		}
		else
		{
			stable_cnt = 0;
			OK_LED_OFF();
		}


		adjust(temp_last_avg, temp_setpoint);

		print_string("T1=");
		print_t(temps[0], 1);
		print_string(" T2=");
		print_t(temps[1], 1);
		print_string(" T3=");
		print_t(temps[2], 1);
		print_string(" Tavg=");
		print_t(temp_last_avg, 1);
		print_string(" Tdiff=");
		print_t(diff, 0);
		print_string(" Tset=");
		print_t(temp_setpoint, 1);
		if(mode == COOL)
		{
			print_string(" COOL=");
			uint8_t perc = cool_pwm_shadow/655;
			print_int(perc);
		}
		else if(mode == HEAT)
		{
			print_string(" HEAT=");
			uint8_t perc = heat_pwm_shadow/655;
			print_int(perc);
		}
		else
		{
			print_string(" OFF");
		}

		if(stable)
			print_string(" STABLE");
		print_string("\r\n");

		_delay_ms(500);
		if(!stable)
			OK_LED_OFF();
		_delay_ms(500);

		t++;
	}

	return 0;
}


