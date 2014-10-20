#define F_CPU 7372000UL

#include <inttypes.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>
#include <stdio.h>

#define cbi(x,y) x &= ~(1<<(y))
#define sbi(x,y) x |= (1<<(y))

#define COOL 0
#define HEAT 1

#define MAX_PWR 65535

#define K_AT_0C 2731

// Temperature unit = 0.1 K
// signed ints used because temperatures are subtracted all the time to get differences.

// Time unit = 1 second.

uint16_t max_peak_pwr[2] = {65535, 40000};
uint16_t max_avg_pwr[2] = {65535, 20000};

int16_t ambient_temp = 235 + K_AT_0C;

// steady-state power requirement (heat escape) per temp unit of difference from ambient temp
uint16_t steady_pwr_per_unit[2] = {20, 6};

// P factor (pwr per unit of temp difference from setpoint)
uint16_t pwr_per_unit[2] = {450, 150};

// Cumulative average of power. Use 2^n-1 for optimum performance.
// Same cumulative average is used for both heating and cooling. This way,
// power stays limited when changing from heating to cooling, to avoid thermal stress.
#define CUMUL_AVG_LEN 63
uint16_t pwr_avg = 0;

#define MODE_CHANGE_DELAY 20
#define OK_LED_THRESHOLD 3
#define OK_LED_BLINK_THRESHOLD 7

uint8_t mode = COOL;

uint8_t clock;

// Port definitions
#define COOL_PWR_REG OCR1B
#define HEAT_PWR_REG OCR1A

#define HEAT_ENA_REG PORTD
#define HEAT_ENA_PIN 6
#define COOL_ENA_REG PORTD
#define COOL_ENA_PIN 7

#define HEAT_LED_ON()  sbi(PORTD, 3)
#define HEAT_LED_OFF() cbi(PORTD, 3)
#define COOL_LED_ON()  sbi(PORTD, 4)
#define COOL_LED_OFF() cbi(PORTD, 4)
#define OK_LED_ON()    sbi(PORTD, 2)
#define OK_LED_OFF()   cbi(PORTD, 2)

#define SENS_PORT_REG PORTC
#define SENS_PIN_REG  PINC
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

// Sets new power level. Limits the value if necessary.
// Selects cooling or heating based on mode.
void update_pwr(uint16_t pwr)
{
	// This function also takes care of cool/heat turnover by
	// switcing hi-side FETs. Some deadtime is inserted by
	// turning hi-side FET off, then doing calculation, then turning another on.

	if(mode == COOL)
	{
		cbi(HEAT_ENA_REG, HEAT_ENA_PIN);
		HEAT_PWR_REG = 0;
	}
	else // mode == HEAT
	{
		cbi(COOL_ENA_REG, COOL_ENA_PIN);
		COOL_PWR_REG = 0;
	}
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
		COOL_PWR_REG = pwr;
		sbi(COOL_ENA_REG, COOL_ENA_PIN);
	}
	else
	{
		HEAT_PWR_REG = pwr;
		sbi(HEAT_ENA_REG, HEAT_ENA_PIN);
	}
}

#define abs(x) (((x)<0)?(-1*(x)):(x))

void adjust(int16_t temp_actual, int16_t temp_setpoint)
{
	static uint8_t mode_change_cnt = 0;

	// new_pwr: positive = heating needed, negative = cooling needed
	// Heat escape compensation:
	int32_t new_pwr_32 = (int32_t)steady_pwr_per_unit[mode]*(temp_setpoint-ambient_temp);
	// P term (power correction based on error term):
	new_pwr_32 += pwr_per_unit[mode]*(temp_setpoint-temp_actual);

	// Change mode if new_pwr sign disagrees with current mode for
	// long enough.
	if((new_pwr_32 < 0 && mode == HEAT) || (new_pwr_32 > 0 && mode == COOL))
	{
		mode_change_cnt++;

		if(mode_change_cnt > MODE_CHANGE_DELAY)
		{
			mode_change_cnt = 0;
			mode = (mode==COOL)?HEAT:COOL;
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

	if((abs(temp_setpoint-temp_actual) <= OK_LED_BLINK_THRESHOLD && clock&1) ||
	   (abs(temp_setpoint-temp_actual) <= OK_LED_THRESHOLD))
		OK_LED_ON();
	else
		OK_LED_OFF();

}


int main()
{
	DDRD = 0b00011100;
	while(1)
	{
		OK_LED_ON();
		_delay_ms(100);
		COOL_LED_ON();
		_delay_ms(100);
		HEAT_LED_ON();
		_delay_ms(300);
		OK_LED_OFF();
		_delay_ms(100);
		COOL_LED_OFF();
		_delay_ms(100);
		HEAT_LED_OFF();
		_delay_ms(300);
	}
	return 0;
}


