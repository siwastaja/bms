// Uudet:
// tilapelleilyn poisto
// Uusi käli
// Balansointi eeprommiin talteen.
// lämpötilan näyttö
// virtamittarin offsetin asetus (autom.& manuaalinen), gainin asetus
// Virtaintegraattorin nollaus haluttuun kohtaan.
// LVC:n IR-kompensaatio.

#define F_CPU 8000000UL

#include <inttypes.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>

#include "../common/manchester.h"

// PJ0 (PCINT9) = manch in (from last node)
// PJ1          = manch out (to first node)


#define LCD_DATA    PORTC
#define LCD_EN_PORT PORTG
#define LCD_EN_BIT  1
//#define LCD_RW_PORT PORTG
//#define LCD_RW_BIT  0
#define LCD_RS_PORT PORTD
#define LCD_RS_BIT  7

//#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
//#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))


#define LCD_CLEAR 0x01
#define LCD_HOME  0x02
#define LCD_LEFT  0x04
#define LCD_RIGHT 0x05
#define LCD_OFF   0x08
#define LCD_ON    0x0C
#define LCD_CURS  0x0F
#define LCD_INIT  0x38
#define LCD_LINE1 0x80
#define LCD_LINE2 0xC0
#define LCD_LINE3 0x94
#define LCD_LINE4 0xD4

const uint8_t LCD_LINES[4] = {LCD_LINE1, LCD_LINE2, LCD_LINE3, LCD_LINE4};

#define LCD_DELAY 800 // was 400

char buf[150];

uint8_t start_comm_err_cnt = 0;
uint8_t middle_comm_err_cnt = 0;
uint8_t end_comm_err_cnt = 0;
uint8_t misc_comm_err_cnt = 0;

uint8_t drive_en    = 0;
uint8_t charge_en   = 0;
uint8_t heat_en     = 0;
uint8_t balancer_en = 0;
uint8_t batt_full   = 1;
uint8_t batt_empty  = 0;

uint8_t batt_full_reset_soc = 97;


int16_t last_max_I = 0;
int16_t last_min_I = 0;
int16_t last_avg_I = 0;
int16_t last_avg_T = 0;
int32_t As_count = 0;
int32_t last_As_count = -999;
int16_t Ah_count = 0;
uint16_t cur_voltage = 0;
int16_t range_left = 0;

uint16_t pack_capacity_Ah = 0;
int16_t SoC = 0;

uint8_t over_SoC_limit = 110;

uint8_t no_idle_mode = 0;
uint8_t invert_current = 0;

volatile int16_t max_I_tmp = -30000;
volatile int16_t min_I_tmp = 30000;
volatile int32_t I_acc = 0;
volatile uint16_t num_I_meas = 0;
volatile int32_t raw_I_acc = 0;


#define NUM_MESSAGES 5
const char* MESSAGES[NUM_MESSAGES] =
{
 "            ",
 "BAT OVERTEMP",
 "EMPTY. STOP!",
 "BMS COMM ERR",
 "OVER SOC ERR"   // over-SoC charge shutdown
};

uint8_t cur_message = 0;


void lcd_inst(uint8_t instr)
{
	cbi(LCD_RS_PORT, LCD_RS_BIT); // instr
//	cbi(LCD_RW_PORT, LCD_RW_BIT); // write
	LCD_DATA = instr;
	_delay_us(LCD_DELAY);
	cbi(LCD_EN_PORT, LCD_EN_BIT);
	_delay_us(LCD_DELAY);
	sbi(LCD_EN_PORT, LCD_EN_BIT);
}

void lcd_data(uint8_t data)
{
	sbi(LCD_RS_PORT, LCD_RS_BIT); // data
//	cbi(LCD_RW_PORT, LCD_RW_BIT); // write
	LCD_DATA = data;
	_delay_us(LCD_DELAY);
	cbi(LCD_EN_PORT, LCD_EN_BIT);
	_delay_us(LCD_DELAY);
	sbi(LCD_EN_PORT, LCD_EN_BIT);
}


void lcd_print(const char* teksti)
{
	uint8_t i = 0;
	while(teksti[i] != 0)
	{
		lcd_data(teksti[i]);
		i++;
	}

}

void lcd_print_n(const char* teksti, uint8_t row)
{
	uint8_t i = 0;

	lcd_inst(LCD_LINES[row]);

	while(teksti[i] != 0)
	{
		lcd_data(teksti[i]);
		i++;
	}
}

#define lcd_clear() {lcd_inst(LCD_CLEAR); _delay_ms(2);}

#define print_char(byte) {while((UCSR1A & 0b00100000) == 0) ; UDR1 = (byte); }

void cgram_symbols_common()
{
	lcd_inst(0b01000000); // cgram 0

	// CGRAM 0 - LVL1
	lcd_data(0b00000);
	lcd_data(0b00000);
	lcd_data(0b00000);
	lcd_data(0b00000);
	lcd_data(0b00000);
	lcd_data(0b00000);
	lcd_data(0b00000);
	lcd_data(0b11111);

	// CGRAM 1 - LVL2
	lcd_data(0b00000);
	lcd_data(0b00000);
	lcd_data(0b00000);
	lcd_data(0b00000);
	lcd_data(0b00000);
	lcd_data(0b00000);
	lcd_data(0b11111);
	lcd_data(0b11111);

	// CGRAM 2 - LVL3
	lcd_data(0b00000);
	lcd_data(0b00000);
	lcd_data(0b00000);
	lcd_data(0b00000);
	lcd_data(0b00000);
	lcd_data(0b11111);
	lcd_data(0b11111);
	lcd_data(0b11111);

	// CGRAM 3 - LVL4
	lcd_data(0b00000);
	lcd_data(0b00000);
	lcd_data(0b00000);
	lcd_data(0b00000);
	lcd_data(0b11111);
	lcd_data(0b11111);
	lcd_data(0b11111);
	lcd_data(0b11111);

	// CGRAM 4 - LVL5
	lcd_data(0b00000);
	lcd_data(0b00000);
	lcd_data(0b00000);
	lcd_data(0b11111);
	lcd_data(0b11111);
	lcd_data(0b11111);
	lcd_data(0b11111);
	lcd_data(0b11111);

	// CGRAM 5 - LVL6
	lcd_data(0b00000);
	lcd_data(0b00000);
 	lcd_data(0b11111);
	lcd_data(0b11111);
	lcd_data(0b11111);
	lcd_data(0b11111);
	lcd_data(0b11111);
	lcd_data(0b11111);

	// CGRAM 6 - LVL7
	lcd_data(0b00000);
	lcd_data(0b11111);
	lcd_data(0b11111);
	lcd_data(0b11111);
	lcd_data(0b11111);
	lcd_data(0b11111);
	lcd_data(0b11111);
	lcd_data(0b11111);

	// CGRAM 7 - reserved for future use
	lcd_data(0b00000);
	lcd_data(0b00100);
	lcd_data(0b01010);
	lcd_data(0b01010);
	lcd_data(0b10001);
	lcd_data(0b11111);
	lcd_data(0b00000);
	lcd_data(0b00000);

}

const char BLOCK_LEVELS[8] = {0, 1, 2, 3, 4, 5, 6, 255};

void init_lcd()
{
        sbi(LCD_EN_PORT, LCD_EN_BIT);
        lcd_inst(LCD_INIT);
        lcd_inst(LCD_ON);
        lcd_inst(LCD_CLEAR);
        _delay_ms(2);

        lcd_clear();
        cgram_symbols_common();
}



uint8_t UART_receive()
{
//	while((UCSR1A & 0b10000000) == 0) ;
// Changed to non-blocking; it was used that way (together with
// UART_byte_received()) anyway. In normal cases, the while loop
// never executed at all, but there was a very slight chance that it
// could hang up the program (never happened though).

	return UDR1;
}

uint8_t UART_byte_received()
{ 
// Returns true only when both Frame Error and Data Overrun bits are off.
// This is to reduce the chance of electrical noise causing entering the
// menu or switching the mode.
	return (UCSR1A & 0b10000000);
}

#define UART_byte UDR1


void UART_init()
{
	UBRR1 = 8; // 8 MHz 115200 bps, remember to use 2x

	UCSR1A = 0b00000010; // 2x mode.

	// 8 data bits, 1 stop bit, no parity bit
	UCSR1C = 0b00000110;

	// Enable RX and TX, but no interrupts.
	UCSR1B = 0b00011000;
}




void print_hex(uint8_t num)
{
	uint8_t hi = (num & 0xF0) >> 4;
	uint8_t lo = num & 0x0F;

  	// Wait for empty transmit buffer 
  	while((UCSR1A & 0b00100000) == 0) ;

  	// Start transmission 
	if(hi>9)
  		UDR1 = hi+55;
	else
		UDR1 = hi+48;

  	// Wait for empty transmit buffer 
  	while((UCSR1A & 0b00100000) == 0) ;


  	// Start transmission 
	if(lo>9)
  		UDR1 = lo+55;
	else
		UDR1 = lo+48;

}

void print_string(const char* str)
{
	while(str[0] != 0)
	{
		print_char(str[0]);
		str++;
	}
}

//200000 = 374 ms
//2000000 = 3.74 s
//534 = 1 ms

uint32_t timeout_per_node = 20000;

#define NODE_OK  0
#define NODE_ERR 1

#define GLOBAL_COMM_OK  0
#define GLOBAL_MASTER_COMM_ERR 1
#define GLOBAL_NODE_COMM_ERR 2
#define GLOBAL_COMM_MISSING_NODES 4
#define GLOBAL_COMM_UNKNOWN_MESSAGES 8
#define GLOBAL_COMM_UNKNOWN_NODE_ID 16
#define GLOBAL_COMM_EXTRA_PACKETS 32

uint8_t last_global_status;
data_t latest_problem_packet;

uint8_t clock = 0;

// Unclear is handled as both LVC+HVC.

#define NODE_LVC                        1
#define NODE_HVC                        2
#define NODE_UNCLEAR                    4
#define NODE_OVERTEMP_SHUTDOWN          8
#define NODE_UNDERTEMP_CHARGE_SHUTDOWN  16
#define NODE_SHUNTING                   32
#define NODE_LVC_UNFILTERED             64
#define NODE_UNCLEAR_UNFILTERED         128
#define NODE_BAT_HEATER_ON              256

uint16_t hvc_limit       = 3600;
uint16_t lvc_limit_ocv   = 2600;
uint16_t shunt_limit     = 3700;
uint16_t overtemp_limit  = 328;
uint16_t charge_temp_limit = 273;
uint16_t bat_heater_temp_limit = 276; 
uint16_t wh_per_km = 150;

uint8_t unclear_time_limit = 4;
uint8_t lvc_filter_limit   = 4;

uint8_t  temperature_poll_rate = 10;

int16_t adc_offset = 192;
int16_t adc_divider = 97;


#define K2C(x) ((int16_t)(x)-273)
#define C2K(x) ((int16_t)(x)+273)

typedef struct
{
	uint16_t t_raw;
	uint16_t v_raw;

    /////////////////////////////////////////////////////////
	// Don't change the order or sizes. Block write/read used.
	uint16_t v_gain;
	int16_t v_offset;
	int8_t v_temp_coeff;
	uint16_t t_gain;
	int16_t t_offset;
	/////////////////////////////////////////////////////////

	uint16_t v; // in mV
	uint16_t t; // in K
	uint8_t v_last_sample_time;
	uint8_t t_last_sample_time;
	uint8_t last_status;

	uint16_t flags;
	uint8_t lvc_filter_num_events;
	uint8_t balance_time;
} node;

#define MAX_CALIB_NODES 30

typedef struct
{
	uint16_t v_raw[9];
	uint16_t t_raw[9];


	// Don't change the order or sizes. Block write/read used.
	uint16_t v_gain;
	int16_t v_offset;
	int8_t v_temp_coeff;
	uint16_t t_gain;
	int16_t t_offset;

	// Only here for logging.
	uint16_t v_gain_cold;
	uint16_t v_gain_hot;
	uint16_t v_offset_cold;
	uint16_t v_offset_hot;
	int8_t v_temp_coeff1;
	int8_t v_temp_coeff2;
	int8_t v_temp_coeff3;


} calib_node;

calib_node calib_nodes[MAX_CALIB_NODES] =
{
	{
		{5000, 6531, 8120, 5151, 6660, 8242, 5260, 6762, 8344},
		{2620, 2620, 2620, 3080, 3080, 3081, 3400, 3410, 3419}
	},
	{
		{4980, 6512, 8100, 5120, 6639, 8220, 5240, 6746, 8321},
		{2604, 2600, 2600, 3062, 3072, 3075, 3400, 3414, 3406}
	},
	{
		{4941, 6460, 8028, 5060, 6563, 8129, 5183, 6669, 8226},
		{2480, 2480, 2481, 2940, 2940, 2940, 3260, 3260, 3260}
	},
	{
		{4998, 6520, 8100, 5140, 6640, 8220, 5260, 6760, 8322},
		{2620, 2610, 2620, 3080, 3080, 3080, 3420, 3420, 3420}
	},
	{
		{4940, 6450, 8007, 5080, 6581, 8140, 5220, 6708, 8266},
		{2640, 2640, 2641, 3100, 3100, 3116, 3420, 3440, 3440}
	},
	{
		{4980, 6500, 8080, 5140, 6640, 8213, 5260, 6760, 8335},
		{2609, 2600, 2610, 3080, 3080, 3080, 3409, 3420, 3420}
	},
	{
		{4980, 6500, 8078, 5139, 6640, 8202, 5260, 6760, 8336},
		{2620, 2620, 2625, 3080, 3094, 3100, 3402, 3420, 3420}
	},
	{
		{4980, 6504, 8080, 5120, 6620, 8186, 5240, 6740, 8301},
		{2601, 2602, 2620, 3080, 3083, 3100, 3420, 3440, 3440}
	},
	{
		{5040, 6583, 8188, 5180, 6700, 8300, 5300, 6820, 8407},
		{2560, 2560, 2560, 3040, 3040, 3044, 3380, 3380, 3380}
	},
	{
		{4960, 6469, 8038, 5060, 6560, 8113, 5177, 6659, 8200},
		{2500, 2499, 2500, 2960, 2960, 2963, 3285, 3300, 3300}
	},
	{
		{4980, 6500, 8080, 5044, 6523, 8080, 5250, 6749, 8320},
		{2600, 2597, 2600, 3069, 3080, 3080, 3403, 3420, 3420}
	},
	{
		{4940, 6440, 8011, 5063, 6568, 8125, 5200, 6692, 8257},
		{2640, 2640, 2640, 3106, 3120, 3120, 3443, 3460, 3460}
	},
	{
		{5020, 6560, 8141, 5160, 6678, 8260, 5280, 6795, 8363},
		{2564, 2560, 2560, 3040, 3040, 3042, 3380, 3380, 3380}
	},
	{
		{4911, 6400, 7940, 5039, 6520, 8042, 5141, 6611, 8140},
		{2480, 2480, 2493, 2926, 2940, 2940, 3260, 3260, 3260}
	},
	{
		{5001, 6540, 8120, 5160, 6660, 8240, 5271, 6780, 8349},
		{2580, 2580, 2580, 3040, 3050, 3060, 3380, 3380, 3380}
	},
	{
		{5020, 6540, 8120, 5160, 6661, 8240, 5280, 6782, 8360},
		{2619, 2609, 2620, 3077, 3080, 3081, 3420, 3420, 3420}
	},
	{
		{4940, 6440, 8000, 5080, 6580, 8134, 5203, 6700, 8260},
		{2680, 2680, 2680, 3140, 3144, 3154, 3472, 3480, 3480}
	},
	{
		{5017, 6540, 8120, 5160, 6662, 8240, 5280, 6794, 8360},
		{2580, 2580, 2580, 3040, 3040, 3047, 3380, 3380, 3380}
	},
	{
		{4980, 6500, 8080, 5120, 6620, 8200, 5220, 6728, 8300},
		{2580, 2580, 2580, 3042, 3053, 3060, 3380, 3389, 3381}
	},
	{
		{4980, 6500, 8060, 5080, 6582, 8140, 5200, 6680, 8220},
		{2480, 2480, 2485, 2940, 2941, 2951, 3260, 3280, 3280}
	},
	{
		{5000, 6520, 8100, 5144, 6641, 8220, 5269, 6761, 8339},
		{2620, 2609, 2620, 3060, 3078, 3080, 3400, 3401, 3400}
	},
	{
		{4940, 6440, 8000, 5040, 6539, 8080, 5160, 6639, 8179},
		{2480, 2480, 2486, 2939, 2940, 2940, 3260, 3260, 3260}
	},
	{
		{5000, 6518, 8099, 5140, 6640, 8210, 5260, 6760, 8323},
		{2620, 2620, 2620, 3080, 3081, 3093, 3402, 3420, 3420}
	},
	{
		{5043, 6572, 8160, 5180, 6692, 8265, 5300, 6810, 8380},
		{2580, 2580, 2580, 3040, 3040, 3058, 3380, 3383, 3381}
	},
	{
		{4978, 6499, 8080, 5120, 6620, 8200, 5240, 6740, 8320},
		{2620, 2620, 2620, 3080, 3089, 3092, 3420, 3420, 3420}
	},
	{
		{4940, 6439, 7994, 5040, 6525, 8074, 5160, 6630, 8161},
		{2500, 2500, 2502, 2940, 2956, 2960, 3262, 3280, 3280}
	},
	{
		{5022, 6550, 8129, 5160, 6660, 8240, 5280, 6780, 8340},
		{2615, 2610, 2620, 3080, 3081, 3093, 3407, 3420, 3420}
	},
	{
		{4900, 5700, 7893, 5120, 6620, 8183, 5240, 6740, 8300},
		{2590, 2580, 2600, 3040, 3050, 3060, 3380, 3381, 3380}
	},
	{
		{5000, 6529, 8120, 5140, 6640, 8220, 5258, 6760, 8340},
		{2638, 2630, 2640, 3080, 3086, 3099, 3409, 3420, 3420}
	}

};

uint8_t use_calibration = 0;


#define MAX_NODES 60

// index is ID.
node nodes[MAX_NODES];

int num_nodes = 8;


#define POLL_V 0
#define POLL_T 1

#define POLL_OK 0
#define POLL_ERR_LINE_NOT_FREE 1 

#define TEMP_COEFF_MIDDLE 29

void conv_t(uint8_t node)
{
	if(use_calibration)
	{
		nodes[node].t = 32768*((int32_t)nodes[node].t_raw + (int32_t)nodes[node].t_offset) / (int32_t)nodes[node].t_gain;
	}
	else
	{
		nodes[node].t = nodes[node].t_raw + nodes[node].t_offset;
	}
}

void conv_v(uint8_t node)
{
	if(use_calibration)
	{
		nodes[node].v = ((int32_t)10*(int32_t)16384*((int32_t)nodes[node].v_raw+(int32_t)nodes[node].v_offset)/(int32_t)nodes[node].v_gain);
    	nodes[node].v -= (((int16_t)nodes[node].t-(int16_t)TEMP_COEFF_MIDDLE) * calib_nodes[node].v_temp_coeff)/(int16_t)20;
	}
	else
	{
		nodes[node].v = (nodes[node].v_raw*9)/2;
	}
}


int16_t measure_single_node_raw(uint8_t node_id, uint8_t poll_t)
{
	data_t data;
	data_t reply;
	data.a = node_id;
	data.b = poll_t?0b10001000:0b10000100;
	data.c = 0;

	cli();

	manchester_send(data);
	_delay_ms(1);

	manchester_wait_data_block_timeout(timeout_per_node*num_nodes);
	_delay_us(12);

	if(manchester_receive(&reply))
	{
		return -1;
	}
	else  // No errors at master data input.
	{
		if((((reply.b&0b11111100) == 0b00000100) && !poll_t) || (((reply.b&0b11111100) == 0b00001000) && poll_t))
		{
			// We got what we asked for.

			uint16_t val = reply.b&0b00000011;
			val <<= 8;
			val |= reply.c;

			return val;
		}
		else
			return -1;
	}

	sei();
}

uint8_t broadcast_poll(uint8_t poll_t)
{
	data_t data;
	data_t reply;
	data.a = 255;
	data.b = poll_t?0b10001000:0b10000100;
	data.c = 0;

	cli();
/*	// Make sure there is no data coming in.
	if(manchester_wait_data_block_timeout(timeout_per_node))
	{
		sei();
		return POLL_ERR_LINE_NOT_FREE;
	}
*/
	manchester_send(data);

	uint8_t got_broadcast = 0;

	last_global_status = GLOBAL_COMM_OK;

	// We also want to see our broadcast message coming back.
	// We also want to have one extra manchester_wait_data_block_timeout, to see if any
	// extra data is coming. We want it to timeout.

	for(uint8_t node = 0; node < num_nodes+2; node++)
	{
		// Timeout: allow a longer time for the first message to get through.
		// Then allow shorter time between the subsequent messages.
		if(manchester_wait_data_block_timeout(node==0?(timeout_per_node*num_nodes):timeout_per_node))
		{
			_delay_us(12); // to approx. match the delay expected by manchester_receive()

			if(manchester_receive(&reply))
			{
				last_global_status |= GLOBAL_MASTER_COMM_ERR;
				end_comm_err_cnt++;
				_delay_us(100);
			}
			else  // No errors at master data input.
			{
				// Use at least 50 us, at most 1 ms here.

				if((reply.a == data.a) && (reply.b == data.b) && (reply.c == data.c))
				{
					// Got our own broadcast.
					got_broadcast = 1;
					break;
				}
				else if(reply.a > num_nodes || reply.a == 0)
				{
					last_global_status |= GLOBAL_COMM_UNKNOWN_NODE_ID;
					latest_problem_packet.abcd = reply.abcd;
					misc_comm_err_cnt++;
				}
				else if(((reply.b&0b11111100) == 0b00011100) || ((reply.b&0b11111100) == 0b00100000))
				{
					// Node "communication error" or "unknown command" packet.
					last_global_status |= GLOBAL_NODE_COMM_ERR;
					latest_problem_packet.abcd = reply.abcd;
					nodes[reply.a - 1].last_status = NODE_ERR;
					if(reply.a == 1)
						start_comm_err_cnt++;
					else
						middle_comm_err_cnt++;
				}
				else if((((reply.b&0b11111100) == 0b00000100) && !poll_t) || (((reply.b&0b11111100) == 0b00001000) && poll_t))
				{
					// We got what we asked for.
					if((node+1) != reply.a)
					{
						last_global_status |= GLOBAL_COMM_MISSING_NODES;
						misc_comm_err_cnt++;
					}

					nodes[reply.a - 1].last_status = NODE_OK;

					uint16_t val = reply.b&0b00000011;
					val <<= 8;
					val |= reply.c;

					if(poll_t)
					{
						nodes[reply.a - 1].t_raw = val;
						conv_t(reply.a - 1);
						nodes[reply.a - 1].t_last_sample_time = clock;

					}
					else
					{
						nodes[reply.a - 1].v_raw = val;
						conv_v(reply.a - 1);
						nodes[reply.a - 1].v_last_sample_time = clock;
					}
				}
			}
		}
		else  // blocking read timeouted.
		{
			if(node <= num_nodes)
			{
				last_global_status |= GLOBAL_COMM_MISSING_NODES;
				misc_comm_err_cnt++;
			}
			else if(node == num_nodes+2)
			{
				last_global_status |= GLOBAL_COMM_EXTRA_PACKETS;
				misc_comm_err_cnt++;
				// If we see more than one extra packet, we just ignore them...
				// (While printing / doing other things)
				// One is enough to tell that things are going wrong.
			}
			break;
		}
	}

	sei();
	return 0;
}

// You absolutely need to call this function every time new samples are taken.
// Call this after updating the node struct values.
uint16_t check_limits(uint8_t node)
{
	nodes[node].flags = 0;

	if(((uint8_t)(clock - nodes[node].v_last_sample_time)) != 0)
		nodes[node].flags |= NODE_UNCLEAR_UNFILTERED;

	if((uint8_t)(clock - nodes[node].v_last_sample_time) > unclear_time_limit)
		nodes[node].flags |= NODE_UNCLEAR;

	// LVC IR compensation

	int16_t ir_comp_mV = last_avg_I;
	if(ir_comp_mV < -250)
		ir_comp_mV = -250;
	if(ir_comp_mV > 0)
		ir_comp_mV = 0;

	if(nodes[node].v <= (uint16_t)((int16_t)lvc_limit_ocv + ir_comp_mV))
	{
		// If the node is in "unknown" state, the old voltage value stays -- 
		// if it was below LVC, the LVC filter will count those. This is good.

		nodes[node].flags |= NODE_LVC_UNFILTERED;
		nodes[node].lvc_filter_num_events++;
		if(nodes[node].lvc_filter_num_events > lvc_filter_limit)
			nodes[node].flags |= NODE_LVC;
	}
	else
		nodes[node].lvc_filter_num_events = 0;

	if(nodes[node].v >= hvc_limit)
	{
		nodes[node].flags |= NODE_HVC;
	}

	if(nodes[node].v >= shunt_limit)
	{
		nodes[node].flags |= NODE_SHUNTING;
	}

	if(nodes[node].t >= overtemp_limit)
	{
		nodes[node].flags |= NODE_OVERTEMP_SHUTDOWN;
	}

	if(nodes[node].t <= charge_temp_limit)
	{
		nodes[node].flags |= NODE_UNDERTEMP_CHARGE_SHUTDOWN;
	}

	if(nodes[node].t <= bat_heater_temp_limit)
	{
		nodes[node].flags |= NODE_BAT_HEATER_ON;
	}


	return nodes[node].flags;
}



#define HEATER_OFF()  cbi(PORTJ, 2);
#define LOAD_OFF()    ; // cbi(PORTJ, 2);
#define CHARGER_OFF() cbi(PORTJ, 3);

#define HEATER_ON()  sbi(PORTJ, 2);
#define LOAD_ON()    ; // sbi(PORTJ, 2);
#define CHARGER_ON() sbi(PORTJ, 3);

void eep_save_temp_offsets()
{
	for(uint8_t node = 0; node < num_nodes; node++)
	{
		int8_t val = nodes[node].t_offset;
		eeprom_write_byte((uint8_t*)(500+node), val);
	}
}

void eep_load_temp_offsets()
{
	for(uint8_t node = 0; node < num_nodes; node++)
	{
		int8_t val = eeprom_read_byte((uint8_t*)(500+node));
		if(val < -30 || val > 30)
			val = 0;
		nodes[node].t_offset = val;
	}
}


void save_bal_eeprom()
{
	eeprom_write_byte((uint8_t*)200, balancer_en);
	for(uint8_t node = 0; node < num_nodes; node++)
	{
		eeprom_write_byte((uint8_t*)(201+node), nodes[node].balance_time);
	}
}

void load_bal_eeprom()
{
	balancer_en = eeprom_read_byte((uint8_t*)200);
	for(uint8_t node = 0; node < num_nodes; node++)
	{
		nodes[node].balance_time = eeprom_read_byte((uint8_t*)(201+node));
		if(nodes[node].balance_time == 255)
		{
			balancer_en = 0;
			break;
		}
	}
}



uint16_t balancer_low_v_point = 3450;
uint16_t balancer_high_v_point = 3570;
uint16_t balancer_max_time = 250;
uint8_t  balancer_unit_seconds = 20;

//
// high v point = 360
// low  v point = 345
// difference = 15
// balancer max time = 100
// time @ 360 = 100
// time @ 345 = 0
// time step = 100/15 = 6
//
// subtract smallest common.

uint8_t last_clock_when_updated_balancer = 0;

void start_balancer()
{
	As_count = 0;

	last_clock_when_updated_balancer = clock - balancer_unit_seconds;
	uint8_t min_balance_time = balancer_max_time;
	for(uint8_t node = 0; node < num_nodes; node++)
	{
		if(nodes[node].v < balancer_low_v_point)
		{
			nodes[node].balance_time = 0;
		}
		else if(nodes[node].v >= balancer_high_v_point)
		{
			nodes[node].balance_time = balancer_max_time;
		}
		else
		{
			nodes[node].balance_time = 
			(nodes[node].v - balancer_low_v_point) *  (balancer_max_time/(balancer_high_v_point-balancer_low_v_point));
		}

		if(nodes[node].balance_time < min_balance_time)
			min_balance_time = nodes[node].balance_time;

	}

	if(min_balance_time > 0)
	{
		for(uint8_t node = 0; node < num_nodes; node++)
		{
			nodes[node].balance_time -= min_balance_time;
		}
	}

	save_bal_eeprom();
}


// This function NEEDS to be called regularly so that load/charger are switched off.
uint16_t check_limits_all_nodes()
{
	// Combine all flags.
	uint16_t flags = 0;
	for(uint8_t node = 0; node < num_nodes; node++)
	{
		flags |= check_limits(node);
	}

	drive_en = 1;
	charge_en = 1;
	heat_en = 0;

	if(batt_full)
		charge_en = 0;

	if(SoC >= over_SoC_limit)
	{
		charge_en = 0;
		cur_message = 4;
	}

	if(flags & NODE_BAT_HEATER_ON)
	{
		heat_en = 1;
	}

	if(flags & NODE_UNCLEAR)
	{
		cur_message = 3;
		drive_en = 0;
		charge_en = 0;
		heat_en = 0;
		balancer_en = 0;
	}

	if(flags & NODE_LVC)
	{
		cur_message = 2;
		drive_en = 0;
		balancer_en = 0;
		batt_empty = 1;
	}

	if(flags & NODE_OVERTEMP_SHUTDOWN)
	{
		cur_message = 1;
		drive_en = 0;
		charge_en = 0;
		heat_en = 0;
	}

	if(flags & NODE_HVC)
	{
		charge_en = 0;
		batt_full = 1;
		eeprom_write_byte((uint8_t*)999, batt_full);
		start_balancer();
		balancer_en = 1;
	}

	if(flags & NODE_UNDERTEMP_CHARGE_SHUTDOWN)
	{
		charge_en = 0;
	}

	if(charge_en)
	{
		CHARGER_ON();
	}
	else
	{
		CHARGER_OFF();
	}

	if(drive_en)
	{
		LOAD_ON();
	}
	else
	{
		LOAD_OFF();
	}

	if(heat_en)
	{
		HEATER_ON();
	}
	else
	{
		HEATER_OFF();
	}

	return flags;
}


int16_t uart_read_int16()
{
	uint8_t pos = 0;
	while(1)
	{
		if(UART_byte_received())
		{
			char key = UDR1;
			if(key == '\r' || key == '\n')
			{
				break;
			}
			else if((key >= '0' && key <= '9') || key == '-')
			{
				print_char(key);
				buf[pos] = key;
				pos++;
				if(pos == 7)
					break;
			}
			else if(key == 8)
			{
				print_char(key);
				if(pos)
					pos--;
			}

		}
	}
	buf[pos] = 0;

	return atoi(buf);
}

#define NUM_CONF_PARAMS 20

const char* config_texts[NUM_CONF_PARAMS] =
{"num_cells",
"hvc_volt",
"lvc_volt",
"overtemp_limit",
"charge_temp_low_limit",
"unclear_action_delay",
"lvc_action_delay",
"balancer_unit_seconds",
"temperature_poll_rate",
"pack_capacity_Ah",
"over_soc_shutdown",
"timeout_per_node_ms",
"shunt_limit",
"invert_current",
"bat_heater_low_limit",
"wh_per_km",
"balancer_low_v",
"balancer_high_v",
"adc_offset",
"adc_divider"
};


const int16_t config_limit_mins[NUM_CONF_PARAMS] =
{2,
3300,
1800,
20,
-10,
1,
1,
1,
1,
1,
101,
2,
3200,
0,
-10,
10,
2000,
2000,
-20000,
10};

const int16_t config_limit_maxs[NUM_CONF_PARAMS] =
{MAX_NODES,
4200,
3500,
60,
40,
20,
20,
200,
20,
2000,
200, // over-SoC
250,
5000,
1,
30,
2000,
5000,
5000,
20000,
2000};

void assign_node_ids()
{
	data_t data;
	data.a = 255;
	data.b = 0b10001100;
	data.c = 1;
	cli();
	manchester_send(data);
	_delay_ms(1);
	sei();
}

void shunt_node(uint8_t node, uint8_t time)
{
	data_t data;
	data.a = node;
	data.b = 0b10101100;
	data.c = time;
	cli();
	manchester_send(data);
	sei();
}


void save_config()
{
	// TODO: checksumming the EEPROM.

	eeprom_write_byte((uint8_t*)0, num_nodes);
	eeprom_write_word((uint16_t*)2, hvc_limit);
	eeprom_write_word((uint16_t*)4, lvc_limit_ocv);
	eeprom_write_word((uint16_t*)6, overtemp_limit);
	eeprom_write_word((uint16_t*)8, charge_temp_limit);
	eeprom_write_word((uint16_t*)10, unclear_time_limit);
	eeprom_write_word((uint16_t*)12, lvc_filter_limit);
	eeprom_write_byte((uint8_t*)14, balancer_unit_seconds);
	eeprom_write_byte((uint8_t*)16, temperature_poll_rate);
	eeprom_write_word((uint16_t*)18, pack_capacity_Ah);
	eeprom_write_byte((uint8_t*)20, over_SoC_limit);
	eeprom_write_dword((uint32_t*)22, timeout_per_node);
	eeprom_write_word((uint16_t*)26, shunt_limit);
	eeprom_write_byte((uint8_t*)28, invert_current);
	eeprom_write_word((uint16_t*)30, bat_heater_temp_limit);
	eeprom_write_word((uint16_t*)32, wh_per_km);
	eeprom_write_word((uint16_t*)34, balancer_low_v_point);
	eeprom_write_word((uint16_t*)36, balancer_high_v_point);
	eeprom_write_word((uint16_t*)38, adc_offset);
	eeprom_write_word((uint16_t*)40, adc_divider);
}

void load_config()
{
	// TODO: checksumming the EEPROM.

	num_nodes = eeprom_read_byte((uint8_t*)0);
	hvc_limit = eeprom_read_word((uint16_t*)2);
	lvc_limit_ocv = eeprom_read_word((uint16_t*)4);
	overtemp_limit = eeprom_read_word((uint16_t*)6);
	charge_temp_limit = eeprom_read_word((uint16_t*)8);
	unclear_time_limit = eeprom_read_word((uint16_t*)10);
	lvc_filter_limit = eeprom_read_word((uint16_t*)12);
	balancer_unit_seconds = eeprom_read_byte((uint8_t*)14);
	temperature_poll_rate = eeprom_read_byte((uint8_t*)16);
	pack_capacity_Ah = eeprom_read_word((uint16_t*)18);
	over_SoC_limit = eeprom_read_byte((uint8_t*)20);
	timeout_per_node = eeprom_read_dword((uint32_t*)22);
	shunt_limit = eeprom_read_word((uint16_t*)26);
	invert_current = eeprom_read_byte((uint8_t*)28);
	bat_heater_temp_limit = eeprom_read_word((uint16_t*)30);
	wh_per_km = eeprom_read_word((uint16_t*)32);
	balancer_low_v_point = eeprom_read_word((uint16_t*)34);
	balancer_high_v_point = eeprom_read_word((uint16_t*)36);
	adc_offset = eeprom_read_word((uint16_t*)38);
	adc_divider = eeprom_read_word((uint16_t*)40);

	batt_full = eeprom_read_byte((uint8_t*)999);
	As_count = eeprom_read_dword((uint32_t*)1000);
}

uint16_t calib_actual_volts[3] = {2200, 3000, 3800};
int16_t calib_actual_temps[3] = {-20, 22, 50};


#define CALIB_OVERSAMPLE_SAMPLES 10  // Do not change.


//      2.5V  3.3V  4.1V
// 
// -20   0     1     2
// +20   3     4     5
// +50   6     7     8
//

// Temperature gain should be near to 1.0 LSB/K
// -> * 32768 to fit uint16
// Voltage gain should be near to 1.95 LSB/mV 
// -> * 16384 to fit uint16

// Temperature coeff: Typically around +0.001 (V/K)
// Middle point (no change) is at TEMP_COEFF_MIDDLE
// Positive temp coeff means that the raw values increase with temperature.


// v_temp_offset = difference in voltage offset (correction to the raw value, at v_offset_at) per 64 deg C.
// v_temp_coeff_gain = difference in v_gain (correction to v_gain value, 0 at v_offset_at) per 64 deg C.

void conv_v_t_calib(uint8_t node, uint16_t v_raw, uint16_t t_raw, uint16_t* v_calib, uint16_t* t_calib, uint8_t use_temp_comp)
{
	*t_calib = ((int32_t)32768*((int32_t)t_raw + (int32_t)calib_nodes[node].t_offset)) / ((int32_t)calib_nodes[node].t_gain);
	*v_calib = ((int32_t)10*(int32_t)16384*((int32_t)v_raw+(int32_t)calib_nodes[node].v_offset)/(int32_t)calib_nodes[node].v_gain);
	if(use_temp_comp)
	    *v_calib -= (((int16_t)(*t_calib)-(int16_t)TEMP_COEFF_MIDDLE) * calib_nodes[node].v_temp_coeff)/(int16_t)20;
}


void calibrate_node(uint8_t node)
{
	//uint32_t tmp32u;

	calib_nodes[node].t_gain =   (32768*(uint32_t)(calib_nodes[node].t_raw[7] - calib_nodes[node].t_raw[1]))
	                      /  //-----------------------------------------------------------------------------------
	                            (CALIB_OVERSAMPLE_SAMPLES*(uint32_t)(C2K(calib_actual_temps[2])-C2K(calib_actual_temps[0])));
	

	calib_nodes[node].t_offset = (((int32_t)calib_nodes[node].t_gain * ((int32_t)C2K(calib_actual_temps[0])))/32768 - (calib_nodes[node].t_raw[1]/10));



	// V_gain at the middle temperature
	calib_nodes[node].v_gain =   (16384*(uint32_t)(calib_nodes[node].v_raw[5] - calib_nodes[node].v_raw[3]))
	                       / //----------------------------------------------------------
						          ((uint32_t)(calib_actual_volts[2]-calib_actual_volts[0]));

	calib_nodes[node].v_gain_cold =   (16384*(uint32_t)(calib_nodes[node].v_raw[2] - calib_nodes[node].v_raw[0]))
	                       / //----------------------------------------------------------
						        ((uint32_t)(calib_actual_volts[2]-calib_actual_volts[0]));

	calib_nodes[node].v_gain_hot  =  (16384*(uint32_t)(calib_nodes[node].v_raw[8] - calib_nodes[node].v_raw[6]))
	                       / //----------------------------------------------------------
						        ((uint32_t)(calib_actual_volts[2]-calib_actual_volts[0]));



	// V_offset at the middle temperature
	calib_nodes[node].v_offset      = (((int32_t)calib_nodes[node].v_gain      * (int32_t)calib_actual_volts[0])/16384 - calib_nodes[node].v_raw[3])/10;
	calib_nodes[node].v_offset_cold = (((int32_t)calib_nodes[node].v_gain_cold * (int32_t)calib_actual_volts[0])/16384 - calib_nodes[node].v_raw[0])/10;
	calib_nodes[node].v_offset_hot  = (((int32_t)calib_nodes[node].v_gain_hot  * (int32_t)calib_actual_volts[0])/16384 - calib_nodes[node].v_raw[6])/10;

	int32_t tmp32;

	tmp32 = ((int32_t)16384*(int32_t)16*((int32_t)calib_nodes[node].v_raw[6] - (int32_t)calib_nodes[node].v_raw[0]))/((int32_t)calib_nodes[node].v_gain*(int32_t)(calib_actual_temps[2]-calib_actual_temps[0]));
	if(tmp32 > 127) tmp32 = 127;
	if(tmp32 < -128) tmp32 = -128;
	calib_nodes[node].v_temp_coeff1 = tmp32;
	tmp32 = ((int32_t)16384*(int32_t)16*((int32_t)calib_nodes[node].v_raw[7] - (int32_t)calib_nodes[node].v_raw[1]))/((int32_t)calib_nodes[node].v_gain*(int32_t)(calib_actual_temps[2]-calib_actual_temps[0]));
	if(tmp32 > 127) tmp32 = 127;
	if(tmp32 < -128) tmp32 = -128;
	calib_nodes[node].v_temp_coeff2 = tmp32;
	tmp32 = ((int32_t)16384*(int32_t)16*((int32_t)calib_nodes[node].v_raw[8] - (int32_t)calib_nodes[node].v_raw[2]))/((int32_t)calib_nodes[node].v_gain*(int32_t)(calib_actual_temps[2]-calib_actual_temps[0]));
	if(tmp32 > 127) tmp32 = 127;
	if(tmp32 < -128) tmp32 = -128;
	calib_nodes[node].v_temp_coeff3 = tmp32;

	int16_t tmp16;

	tmp16 = calib_nodes[node].v_temp_coeff1 + calib_nodes[node].v_temp_coeff2 + calib_nodes[node].v_temp_coeff3;
	tmp16 /= 3;
	calib_nodes[node].v_temp_coeff = tmp16;

}



uint8_t node_read_eeprom_8b(uint8_t node_id, uint8_t address, uint8_t* data)
{
	data_t msg, reply;
	msg.a = node_id;
	msg.b = 0b10010000; // Retrieve 8-bit eeprom value
	msg.c = address;

	manchester_send(msg);
	_delay_ms(1);

	if(manchester_wait_data_block_timeout(timeout_per_node*num_nodes))
	{
		_delay_us(12); // to approx. match the delay expected.
		if(manchester_receive(&reply) || reply.a != node_id || reply.b != 0b00010000)
			return 2;

		*data = reply.c;
	}
	else
		return 1;

	return 0;

}

uint8_t node_write_eeprom_8b(uint8_t node_id, uint8_t address, uint8_t data)
{
	data_t msg, reply;
	msg.a = node_id;
	msg.b = 0b10101000;  // Enable eeprom modification for address
	msg.c = address;
	manchester_send(msg);
	_delay_ms(1);

	if(manchester_wait_data_block_timeout(timeout_per_node*num_nodes))
	{
		_delay_us(12); // to approx. match the delay expected.
		if(manchester_receive(&reply) || reply.a != node_id || reply.b != 0b00011000)
			return 2;
	}
	else
		return 1;

	msg.b = 0b10011100; // Write 8-bit data
	msg.c = data;
	manchester_send(msg);
	_delay_ms(1);

	if(manchester_wait_data_block_timeout(timeout_per_node*num_nodes))
	{
		_delay_us(12); // to approx. match the delay expected.
		if(manchester_receive(&reply) || reply.a != node_id || reply.b != 0b00001100 || reply.c != address)
			return 4;
	}
	else
		return 3;

	return 0;
}


uint8_t node_write_eeprom_16b(uint8_t node_id, uint8_t address, uint16_t data)
{
	data_t msg, reply;
	msg.a = node_id;
	msg.b = 0b10101000;  // Enable eeprom modification for address
	msg.c = address;
	manchester_send(msg);
	_delay_ms(1);

	if(manchester_wait_data_block_timeout(timeout_per_node*num_nodes))
	{
		_delay_us(12); // to approx. match the delay expected.
		if(manchester_receive(&reply) || reply.a != node_id || reply.b != 0b00011000)
			return 2;
	}
	else
		return 1;

	msg.b = 0b10100000; // Write 16-bit MSB
	msg.c = (data&0xff00) >> 8;
	manchester_send(msg);
	_delay_ms(1);

	if(manchester_wait_data_block_timeout(timeout_per_node*num_nodes))
	{
		_delay_us(12); // to approx. match the delay expected.
		if(manchester_receive(&reply) || reply.a != node_id || reply.b != 0b00001100 || reply.c != address)
			return 4;
	}
	else
		return 3;

	msg.b = 0b10100100; // Write 16-bit LSB
	msg.c = data&0x00ff;
	manchester_send(msg);
	_delay_ms(1);

	if(manchester_wait_data_block_timeout(timeout_per_node*num_nodes))
	{
		_delay_us(12); // to approx. match the delay expected.
		if(manchester_receive(&reply) || reply.a != node_id || reply.b != 0b00001100 || reply.c != address)
			return 6;
	}
	else
		return 5;	

	return 0;
}

void init_calibration()
{
	print_string("Fetching calibration data from nodes...\r\n");
	cli();
	for(uint8_t node = 0; node < num_nodes; node++)
	{
		for(uint8_t byte = 0; byte < 9; byte++)
		{
			uint8_t ret = node_read_eeprom_8b(node+1, 0x29+byte*2, ((uint8_t*)((void*)&(nodes[node].v_gain)+(byte))));
			if(ret)
			{
				sprintf(buf, "Calibration data fetch error at node %u, error %u, calibration disabled.\r\n", node+1, ret);
				print_string(buf);
				use_calibration = 0;
				goto INIT_CALIBRATION_BREAK_BREAK;
			}
	/*		else
			{
			sprintf(buf, "Node %u: Tg = %u, To = %d, Vg = %u, Vo = %d, Vtc = %d\r\n",  node+1, nodes[node].t_gain, nodes[node].t_offset,
				nodes[node].v_gain, nodes[node].v_offset, nodes[node].v_temp_coeff);
			print_string(buf);
			}*/

		}

			sprintf(buf, "Node %u: Tg = %u, To = %d, Vg = %u, Vo = %d, Vtc = %d\r\n",  node+1, nodes[node].t_gain, nodes[node].t_offset,
				nodes[node].v_gain, nodes[node].v_offset, nodes[node].v_temp_coeff);
			print_string(buf);


		if(nodes[node].v_gain < 20000 || nodes[node].v_gain > 50000 ||
		   nodes[node].v_offset < -500 || nodes[node].v_offset > 500 ||
		   nodes[node].t_gain < 20000 || nodes[node].t_gain > 50000 ||
		   nodes[node].t_offset < -100 || nodes[node].t_offset > 100)
		{


			sprintf(buf, "Calibration data fetch error at node %u, error 47, calibration disabled.\r\n", node+1);
			print_string(buf);
			use_calibration = 0;
			goto INIT_CALIBRATION_BREAK_BREAK;

		}

	}
	sei();

	use_calibration = 1;
	print_string("Calibration in use.\r\n");

  INIT_CALIBRATION_BREAK_BREAK:
    return;

}

void zero_I_offset()
{
	cli();
	raw_I_acc = 0;
	num_I_meas = 0;
	sei();
	_delay_ms(1000);
	cli();
	adc_offset = raw_I_acc/num_I_meas;
	sei();
}



void calibration()
{
	lcd_inst(LCD_LINE1); for(uint8_t i = 0; i < 20; i++) lcd_data(255);
	lcd_inst(LCD_LINE2); lcd_data(255); lcd_print(" CALIBRATION MODE "); lcd_data(255);
	lcd_inst(LCD_LINE3); lcd_data(255); lcd_print(" ENTERED ON RS232 "); lcd_data(255);
	lcd_inst(LCD_LINE4); for(uint8_t i = 0; i < 20; i++) lcd_data(255);

	uint16_t measurement_done = 0; // lsb = meas 1.

	measurement_done = 511;

	calib_actual_temps[0] = -21;
	calib_actual_temps[1] = 24;
	calib_actual_temps[2] = 55;


	for(uint8_t node = 0; node < num_nodes; node++)
	{
		calibrate_node(node);
	}

	while(1)
	{
		print_string("Calibration mode.\r\n\r\n");

		sprintf(buf, "    actual V %4u mV [a] | %4u mV [b] | %4u mV [c]\r\n", calib_actual_volts[0], calib_actual_volts[1], calib_actual_volts[2]);
		print_string(buf);
		print_string("actual T    ---------------------------------------\r\n");
		for(uint8_t i = 0; i<3; i++)
		{
			sprintf(buf, "%3d C [%c]  |   %c  [%c]   |   %c  [%c]    |   %c  [%c]   |", 
				calib_actual_temps[i], 'd'+i,
				(((measurement_done>>(0+i*3))&1)?'X':'-'), '1'+3*i,
				(((measurement_done>>(1+i*3))&1)?'X':'-'), '2'+3*i,
				(((measurement_done>>(2+i*3))&1)?'X':'-'), '3'+3*i);
			print_string(buf);
			print_string("\r\n---------------------------------------------------\r\n");

		}

	CALIB_NEW_PROMPT:
		print_string("\r\n\r\n [a - f] Change actual voltages/temperatures\r\n");
		print_string("   [1 - 9] Measure at this point\r\n");
		if(measurement_done == 511)
		{
			print_string("\r\n Calibration values have been calculated!\r\n");
			print_string("             [p] Print calibration data\r\n");
			print_string("             [s] Send calibration data to nodes\r\n");
		}
		print_string(" [shift-q] Abort and quit calibration\r\n");


		while(!UART_byte_received());

		char key = UDR1;
		if(key == 'Q')
		{
			print_string("CALIBRATION ABORTED\r\n");
			return;
		}
		else if(key >= 'a' && key <= 'c')
		{
			print_string("Enter actual voltage ");
			print_char(key);
			print_char(':');
			int16_t val = uart_read_int16();
			if(val < 1000 || val > 5000)
			{
				print_string(" OUT OF RANGE\r\n");
				goto CALIB_NEW_PROMPT;
			}
			calib_actual_volts[key-'a'] = val;
		}
		else if(key >= 'd' && key <= 'f')
		{
			print_string("Enter actual temperature ");
			print_char(key);
			print_char(':');
			int16_t val = uart_read_int16();
			if(val < -50 || val > 120)
			{
				print_string(" OUT OF RANGE\r\n");
				goto CALIB_NEW_PROMPT;
			}
			calib_actual_temps[key-'d'] = val;
		}
		else if(key >= '1' && key <= '9')
		{
			for(uint8_t node = 1; node <= num_nodes; node++)
			{
				uint16_t v_acc = 0;
				uint16_t t_acc = 0;

				sprintf(buf, "\r\nMeasuring node %u: ", node);
				print_string(buf);
				uint8_t s;
				for(s = 0; s < CALIB_OVERSAMPLE_SAMPLES; s++)
				{
					int16_t val = measure_single_node_raw(node, POLL_V);
					if(val == -1)
					{
						// Continuous error will hang up the program here. This is OK. Reset and fix the communication.
						print_string("Warning: communication error.\n\r");
						s--;
						continue;
					}
					else
					{
						v_acc += val;
					}
					_delay_ms(10);
				}

				for(s = 0; s < CALIB_OVERSAMPLE_SAMPLES; s++)
				{
					int16_t val = measure_single_node_raw(node, POLL_T);
					if(val == -1)
					{
						// Continuous error will hang up the program here. This is OK. Reset and fix the communication.
						print_string("Warning: communication error.\n\r");
						s--;
						continue;
					}
					else
					{
						t_acc += val;
					}
					_delay_ms(10);
				}

				sprintf(buf," v_raw=%u, t_raw=%u\r\n", v_acc, t_acc);
				print_string(buf);

				calib_nodes[node-1].v_raw[key-'1'] = v_acc;
				calib_nodes[node-1].t_raw[key-'1'] = t_acc;
				sbi(measurement_done, key-'1');
			}

			if(measurement_done == 511)
			{
				print_string("\r\nAll calibration points done. Calulating calibration values...");
				for(uint8_t node = 0; node < num_nodes; node++)
				{
					calibrate_node(node);
				}
			}
		}
		else if(key == 'p' && measurement_done == 511)
		{
			print_string("\r\n\r\n\r\nCalibration data:\r\n");

			sprintf(buf, "Actual V:, %u, %u, %u, Actual T:, %d, %d, %d", 
				calib_actual_volts[0], calib_actual_volts[1], calib_actual_volts[2],
				calib_actual_temps[0], calib_actual_temps[1], calib_actual_temps[2]);

			print_string(buf);

			for(uint8_t node = 0; node < num_nodes; node++)
			{
				sprintf(buf, "\r\nNode %2u raw V:,", node+1);
				print_string(buf);
				for(uint8_t i = 0; i < 9; i++)
				{
					sprintf(buf, "%4u, ", calib_nodes[node].v_raw[i]);
					print_string(buf);
				}

				sprintf(buf, "\r\nNode %2u raw T:,", node+1);
				print_string(buf);
				for(uint8_t i = 0; i < 9; i++)
				{
					sprintf(buf, "%4u, ", calib_nodes[node].t_raw[i]);
					print_string(buf);
				}

				sprintf(buf, "\n\rNode %2u:\n\r"
							 "         T offset: %5d, T gain: %5u\n\r"
				             "         V offset: %5d, V gain: %5u\n\r"
							 "        (V gain cold: %5u, V gain hot: %5u)\n\r"
							 "        (V offset cold: %5d, V offset hot: %5d)\n\r"
							 "         V temp coeff: %4d\n\r"
							 "        (coeff @ V1: %4d, coeff @ V2: %4d, coeff @ V3: %4d)",
					node+1, calib_nodes[node].t_offset, calib_nodes[node].t_gain, 
					calib_nodes[node].v_offset, calib_nodes[node].v_gain, calib_nodes[node].v_gain_cold, calib_nodes[node].v_gain_hot,
					calib_nodes[node].v_offset_cold, calib_nodes[node].v_offset_hot,
					calib_nodes[node].v_temp_coeff,  calib_nodes[node].v_temp_coeff1, calib_nodes[node].v_temp_coeff2, calib_nodes[node].v_temp_coeff3);

				print_string(buf);

				print_string("\r\nCalibrated V:");
				for(uint8_t i = 0; i < 9; i++)
				{
					uint16_t v, t;
					conv_v_t_calib(node, calib_nodes[node].v_raw[i]/10, calib_nodes[node].t_raw[i]/10, &v, &t,0);
					sprintf(buf, "%5u, ", v);
					print_string(buf);
				}

				print_string("\r\nCalibrated V (+ temp compensation):");
				for(uint8_t i = 0; i < 9; i++)
				{
					uint16_t v, t;
					conv_v_t_calib(node, calib_nodes[node].v_raw[i]/10, calib_nodes[node].t_raw[i]/10, &v, &t,1);
					sprintf(buf, "%5u, ", v);
					print_string(buf);
				}


				print_string("\r\nCalibrated T:");
				for(uint8_t i = 0; i < 9; i++)
				{
					uint16_t v, t;
					conv_v_t_calib(node, calib_nodes[node].v_raw[i]/10, calib_nodes[node].t_raw[i]/10, &v, &t, 0);
					sprintf(buf, "%3d, ", K2C(t));
					print_string(buf);
				}

			}

			print_string("\r\n\r\n\r\nEND OF CALIBRATION DATA\r\n\r\n\r\n");
		}
		else if(key == 's' && measurement_done == 511)
		{
			print_string("Sending calibration to nodes...\r\n");

			cli();
			for(uint8_t node = 0; node < num_nodes; node++)
			{
				for(uint8_t byte = 0; byte < 9; byte++)
				{
					uint8_t ret = node_write_eeprom_8b(node+1, 0x29+byte*2, *((uint8_t*)((void*)&(calib_nodes[node].v_gain)+(byte))));
					if(ret)
					{
						sprintf(buf, "\r\nERROR %u sending to node %u, retrying...    ", ret, node+1);
						print_string(buf);

						if(node_write_eeprom_8b(node+1, 0x29+byte*2, *((uint8_t*)((void*)&(calib_nodes[node].v_gain)+(byte)))))
							print_string("Retry failed!");
						else
							print_string("Retry OK!");
					}

				}
			}
			sei();

			print_string("\r\nAll calibration data sent to dones.\r\n");
		}

	}

}

void config()
{
	data_t msg;

	lcd_inst(LCD_LINE1); for(uint8_t i = 0; i < 20; i++) lcd_data(255);
	lcd_inst(LCD_LINE2); lcd_data(255); lcd_print("CONFIGURATION MODE"); lcd_data(255);
	lcd_inst(LCD_LINE3); lcd_data(255); lcd_print(" ENTERED ON RS232 "); lcd_data(255);
	lcd_inst(LCD_LINE4); for(uint8_t i = 0; i < 20; i++) lcd_data(255);


	while(1)
	{
		print_string("\r\n\r\nGenDenBMS Configuration.");

		for(uint8_t i = 0; i < NUM_CONF_PARAMS; i++)
		{
			print_string("\r\n[");
			print_char('a'+i);
			print_string("] ");
			print_string(config_texts[i]);
			print_string(" = ");
			switch(i)
			{
				case 0: utoa(num_nodes, buf, 10); break;
				case 1: utoa(hvc_limit, buf, 10); break;
				case 2: utoa(lvc_limit_ocv, buf, 10); break;
				case 3: itoa(K2C(overtemp_limit), buf, 10); break;
				case 4: itoa(K2C(charge_temp_limit), buf, 10); break;
				case 5: utoa(unclear_time_limit, buf, 10); break;
				case 6: utoa(lvc_filter_limit, buf, 10); break;
				case 7: utoa(balancer_unit_seconds, buf, 10); break;
				case 8: utoa(temperature_poll_rate, buf, 10); break;
				case 9: utoa(pack_capacity_Ah, buf, 10); break;
				case 10: utoa(over_SoC_limit, buf, 10); break;
				case 11: utoa(timeout_per_node/534, buf, 10); break;
				case 12: utoa(shunt_limit, buf, 10); break;
				case 13: utoa(invert_current, buf, 10); break;
				case 14: itoa(K2C(bat_heater_temp_limit), buf, 10); break;
				case 15: utoa(wh_per_km, buf, 10); break;
				case 16: utoa(balancer_low_v_point, buf, 10); break;
				case 17: utoa(balancer_high_v_point, buf, 10); break;
				case 18: itoa(adc_offset, buf, 10); break;
				case 19: itoa(adc_divider, buf, 10); break;
				default: break;
			}
			print_string(buf);

		}


		print_string("\r\n\r\n[1] Assign new node IDs starting from 1");
		print_string("\r\n[2] Send custom message");
		print_string("\r\n[3] Locate node");
		print_string("\r\n[4] Node Calibration Program");
		print_string("\r\n[5] Re-zero current sensor offset");
		print_string("\r\n[6] Set current integrator value");
		print_string("\r\n[7] Calibrate temperatures");

		print_string("\r\n[x] Save configuration (don't forget!)");
		print_string("\r\n[y] Reload configuration (undo changes)");
		print_string("\r\n[z] Quit configuration menu");

	  NEW_PROMPT:

		print_string("\r\nEnter your command: ");


		while(!UART_byte_received());

		char key = UDR1;
		if(key == 'z')
		{
			goto QUIT_CONFIG;
		}
		else if(key == 'x')
		{
			save_config();
			print_string("CONFIGURATION SAVED\r\n");
		}
		else if(key == 'y')
		{
			load_config();
			print_string("CONFIGURATION RECALLED\r\n");

		}
		else if(key >= 'a' && key < 'a'+NUM_CONF_PARAMS)
		{
			print_string("Enter ");
			print_string(config_texts[key-'a']);
			print_char(':');
			int16_t val = uart_read_int16();
			if(val < config_limit_mins[key-'a'] || val > config_limit_maxs[key-'a'])
			{
				print_string(" OUT OF RANGE\r\n");
				goto NEW_PROMPT;
			}
			switch(key-'a')
			{
				case 0: num_nodes = val; break;
				case 1: hvc_limit = val; break;
				case 2: lvc_limit_ocv = val; break;
				case 3: overtemp_limit = C2K(val); break;
				case 4: charge_temp_limit = C2K(val); break;
				case 5: unclear_time_limit = val; break;
				case 6: lvc_filter_limit = val; break;
				case 7: balancer_unit_seconds = val; break;
				case 8: temperature_poll_rate = val; break;
				case 9: pack_capacity_Ah = val; break;
				case 10: over_SoC_limit = val; break;
				case 11: timeout_per_node = val*534; break;
				case 12: shunt_limit = val; break;
				case 13: invert_current = val; break;
				case 14: bat_heater_temp_limit = C2K(val); break;
				case 15: wh_per_km = val; break;
				case 16: balancer_low_v_point = val; break;
				case 17: balancer_high_v_point = val; break;
				case 18: adc_offset = val; break;
				case 19: adc_divider = val; break;
				default: break;
			}
		}
		else if(key == '1')
		{
			assign_node_ids();
		}
		else if(key == '2')
		{
			print_string("Enter msg in hex (eg. 05ac01): ");
			char keys[6];
			for(uint8_t i = 0; i < 6; i++)
			{
				while(1)
				{
					if(UART_byte_received())
					{
						keys[i] = UDR1;
						print_char(keys[i]);
						break;
					}
				}
			}

			msg.abcd = 0;
			for(uint8_t i = 0; i < 6; i++)
			{
				if(keys[i] >= '0' && keys[i] <= '9')
					msg.abcd |= keys[i]-'0';
				else
					msg.abcd |= 10+keys[i]-'a';
				msg.abcd <<= 4;
			}
			msg.abcd <<= 4;
			cli();
			manchester_send(msg);
			_delay_ms(1);

			uint8_t repl;
			data_t replies[8];
			uint8_t rets[8];
			for(repl = 0; repl < 8; repl++)
			{
				if(manchester_wait_data_block_timeout(repl==0?(timeout_per_node*num_nodes):timeout_per_node))
				{
					_delay_us(12); // to approx. match the delay expected.
					rets[repl] = manchester_receive(&(replies[repl]));
					_delay_us(50);
				}
				else
					break;

			}

			sei();
			print_string("\r\n8 first replies:\r\n");
			for(uint8_t o = 0; o < repl; o++)
			{
				print_hex(rets[o]);
				print_string("  ");
				print_hex(replies[o].a);
				print_char(' ');
				print_hex(replies[o].b);
				print_hex(replies[o].c);
				print_char(' ');
				print_hex(replies[o].d);
				print_string("\r\n");
			}
		}
		else if(key == '3')
		{
			print_string("Cell ID? :");
			int16_t val = uart_read_int16();
			if(val < 1 || val > MAX_NODES)
			{
				print_string("OUT OF RANGE\r\n");
				goto NEW_PROMPT;
			}
			print_string("Press any key to stop blinking.");
			while(1)
			{
				if(UART_byte_received())
					break;

				shunt_node(val, 1);
				_delay_ms(1000);
				if(UART_byte_received())
					break;
				_delay_ms(1000);
			}
			char turha;
			turha = UDR1;

		}
		else if(key == '4')
		{
			calibration();
		}
		else if(key == '5')
		{
			print_string("\r\nMeasuring...");
			zero_I_offset();
			print_string(" adc_offset set. Check and save if happy.\r\n");
		}
		else if(key == '6')
		{
			print_string("Enter Ah (-999 to 999): ");
			int16_t val = uart_read_int16();
			if(val < -999 || val > 999)
			{
				print_string("OUT OF RANGE\r\n");
				goto NEW_PROMPT;
			}
			As_count = (int32_t)val * 60 * 60;
			eeprom_write_dword((uint32_t*)1000, As_count);

		}
		else if(key == '7')
		{
			print_string("Enter current temp (5 to 40): ");
			int16_t val = uart_read_int16();
			if(val < 5 || val > 40)
			{
				print_string("OUT OF RANGE\r\n");
				goto NEW_PROMPT;
			}
			val += 273;

			while(1)
			{
				_delay_ms(500);
                        	if(broadcast_poll(POLL_T))
				{
                                	print_string("T_POLL_ERROR ");
					continue;
				}
				for(uint8_t n = 0; n < num_nodes; n++)
				{
					if(nodes[n].last_status != NODE_OK)
					{
						continue;
					}
				}
				break;
			}

			for(uint8_t n = 0; n < num_nodes; n++)
			{
				nodes[n].t_offset = val - nodes[n].t_raw;
			}
			eep_save_temp_offsets();
		}
	}

  QUIT_CONFIG:

	lcd_clear();
	return;

}

void print_global_error_flags()
{
	if(last_global_status & GLOBAL_MASTER_COMM_ERR)
		print_string("GLOBAL_MASTER_COMM_ERR ");
	if(last_global_status & GLOBAL_NODE_COMM_ERR)
		print_string("GLOBAL_NODE_COMM_ERR ");
	if(last_global_status & GLOBAL_COMM_MISSING_NODES)
		print_string("GLOBAL_COMM_MISSING_NODES ");
	if(last_global_status & GLOBAL_COMM_UNKNOWN_MESSAGES)
		print_string("GLOBAL_COMM_UNKNOWN_MESSAGES ");
	if(last_global_status & GLOBAL_COMM_UNKNOWN_NODE_ID)
		print_string("GLOBAL_COMM_UNKNOWN_NODE_ID ");
	if(last_global_status & GLOBAL_COMM_EXTRA_PACKETS)
		print_string("GLOBAL_COMM_EXTRA_PACKETS ");
}

void print_flags(uint16_t flags)
{
	if(flags & NODE_LVC)
		print_string("LVC ");

	if(flags & NODE_HVC)
		print_string("HVC ");

	if(flags & NODE_UNCLEAR)
		print_string("UNCLEAR ");

	if(flags & NODE_OVERTEMP_SHUTDOWN)
		print_string("OVERTEMP_SHUTDOWN ");

	if(flags & NODE_UNDERTEMP_CHARGE_SHUTDOWN)
		print_string("UNDERTEMP_CHARGE_SHUTDOWN ");

	if(flags & NODE_SHUNTING)
		print_string("NODE_SHUNTING ");

	if(flags & NODE_LVC_UNFILTERED)
		print_string("LVC_UNFILTERED ");

	if(flags & NODE_UNCLEAR_UNFILTERED)
		print_string("UNCLEAR_UNFILTERED ");

	if(flags & NODE_BAT_HEATER_ON)
		print_string("BAT_HEATER_ON ");

}

ISR(ADC_vect)
{
	int16_t val = ADC<<6;
	raw_I_acc += val;
	val -= adc_offset;
	val /= adc_divider;

	if(invert_current)
		val *= -1;

	if(val > max_I_tmp)
		max_I_tmp = val;
	if(val < min_I_tmp)
		min_I_tmp = val;

	I_acc += val;
	num_I_meas++;
}

#define AS_SAVE_EVERY  10

void accumulate_As(uint8_t time)
{
	static uint8_t As_save = 0;

	int32_t I_acc_tmp;
	uint16_t num_I_meas_tmp;

	cli();

	last_max_I = max_I_tmp;
	last_min_I = min_I_tmp;
	max_I_tmp = -30000;
	min_I_tmp = 30000;

	I_acc_tmp = I_acc;
	num_I_meas_tmp = num_I_meas;
	I_acc = 0;
	num_I_meas = 0;

	sei();

	last_avg_I = I_acc_tmp/num_I_meas_tmp;


	As_count += time*last_avg_I;
	Ah_count = As_count/(60*60);
	int32_t SoC_tmp;
	SoC_tmp = (int32_t)pack_capacity_Ah*(int32_t)(60*60) + (int32_t)As_count;
	SoC_tmp *= 100;
	SoC = SoC_tmp/((int32_t)pack_capacity_Ah*(int32_t)(60*60));

	uint8_t batt_full_tmp = batt_full;
	if(SoC <= batt_full_reset_soc)
		batt_full = 0;

	if(batt_full_tmp != batt_full)
		eeprom_write_byte((uint8_t*)999, batt_full);

	if((++As_save > AS_SAVE_EVERY) && (last_As_count != As_count))
	{
		eeprom_write_dword((uint32_t*)1000, As_count);
		As_save = 0;
		last_As_count = As_count;
	}
}

inline void calc_voltage()
{
	uint32_t tmp = 0;
	for(uint8_t node = 0; node < num_nodes; node++)
	{
		tmp += (uint32_t)nodes[node].v;
	}

	cur_voltage = tmp/((uint32_t)1000);
}

inline void calc_range()
{
	range_left = ((int32_t)((int32_t)pack_capacity_Ah + (int32_t)Ah_count)*(int32_t)cur_voltage)/(int32_t)wh_per_km;
}

int main()
{

	uint16_t last_timer_val = 0;

	PRR0 = 0b11100110;
	PRR1 = 0b00111110;

	DDRJ = 0b00001110;
	PORTJ = 0b00000011; // ylosveto manseinputtiin ja manseoutput ykkoseksi.
	DDRC = 0xff;
	DDRG = 0b00000010;
	DDRD = 0b10000000;

	ADMUX = 0b11010000;  // 2.56V reference, differential ADC0-ADC1  1x.
	ADCSRB = 0b00000000; // free running mode

	ADCSRA = 0b11101110; // prescaler = 64: 125 kHz. auto trigger.
	DIDR0 = 0b00000011;

	PORTG = 0b00000001; // Button pull-up.

	TCCR1A = 0b00000000;
	TCCR1B = 0b00000101; // 1024 prescaler

	init_lcd();

	UART_init();



	load_config();
	if(timeout_per_node > 200000)
		timeout_per_node = 20000;

	load_bal_eeprom();
	last_clock_when_updated_balancer = balancer_unit_seconds - 2;

//	init_calibration();

	eep_load_temp_offsets();

	print_string("BMS Master started.\r\n");

	clock = 0;
	sei();
	last_timer_val = TCNT1;
	while(1)
	{
		uint8_t button_cnt = (~PING)&1;
		if(UART_byte_received())
		{
			char key = UDR1;
			if(key == 26) // CTRL+Z
			{
				_delay_ms(200);
				if(UART_byte_received() && UDR1 == 'z')
				{
					LOAD_OFF();
					CHARGER_OFF();
					HEATER_OFF();
					config();
				}
			}
		}

		utoa(clock, buf, 10);
		print_string("clock=");
		print_string(buf);
		print_char(' ');
		if(drive_en)    print_string("DRIVE ");
		if(charge_en)   print_string("CHARGE ");
                if(heat_en)     print_string("BATT_HEAT ");
		if(balancer_en) print_string("BALANCER ");
		if(batt_full)   print_string("BATT_FULL ");
		if(cur_message != 0)
		{
			print_string("lcdmsg=");
			utoa(cur_message, buf, 10);
			print_string(buf);
			print_char(' ');
		}

		button_cnt += (~PING)&1;

		if(clock%temperature_poll_rate == 0)
		{
			if(broadcast_poll(POLL_T))
			{
				print_string("T_POLL_ERROR ");
			}
			else
			{
				if(last_global_status != GLOBAL_COMM_OK)
				{
					print_global_error_flags();
					print_string("latest_problem_packet=");
					print_hex(latest_problem_packet.a);
					print_hex(latest_problem_packet.b);
					print_hex(latest_problem_packet.c);
					print_hex(latest_problem_packet.d);
					print_char(' ');
				}
			}


		}

		if(broadcast_poll(POLL_V))
		{
			print_string("V_POLL_ERROR ");
		}
		else
		{
			if(last_global_status != GLOBAL_COMM_OK)
			{
				print_global_error_flags();
				print_string("latest_problem_packet=");
				print_hex(latest_problem_packet.a);
				print_hex(latest_problem_packet.b);
				print_hex(latest_problem_packet.c);
				print_hex(latest_problem_packet.d);
				print_char(' ');
			}

		}

		button_cnt += (~PING)&1;

		uint16_t combined_flags = check_limits_all_nodes();

		if(clock%temperature_poll_rate == 0)
		{
			for(uint8_t i = 0; i < num_nodes; i++)
			{
				print_char('T');
				utoa(i+1, buf, 10);
				print_string(buf);
				print_char('=');
				itoa(K2C(nodes[i].t), buf, 10);
				print_string(buf);
				if(nodes[i].t_last_sample_time != clock)
					print_char('*');

				print_char(' ');
			}
		}

		for(uint8_t i = 0; i < num_nodes; i++)
		{
			print_char('V');
			utoa(i+1, buf, 10);
			print_string(buf);
			print_char('=');
			utoa(nodes[i].v, buf, 10);
			print_string(buf);
			if(nodes[i].v_last_sample_time != clock)
				print_char('*');

			print_char(' ');

		}

		button_cnt += (~PING)&1;

		for(uint8_t i = 0; i < num_nodes; i++)
		{
			if(nodes[i].flags != 0)
			{
				print_char('F');
				utoa(i+1, buf, 10);
				print_string(buf);
				print_char('=');
				print_hex(nodes[i].flags);
				print_char(' ');
			}
		}

		print_flags(combined_flags);

		for(uint8_t i = 0; i < num_nodes; i++)
		{
			if(nodes[i].flags&NODE_SHUNTING)
				shunt_node(i+1, 10);
		}

/*		uint16_t avg_v;
		uint32_t avg_v_acc = 0;
		for(uint8_t node = 0; node < num_nodes; node++)
			avg_v_acc += nodes[node].v;

		avg_v_acc /= num_nodes;
		avg_v = avg_v_acc; */

		button_cnt += (~PING)&1;

		if(balancer_en)
		{
			lcd_inst(LCD_LINES[3]);
			lcd_print("BAL");

			if((uint8_t)(clock - last_clock_when_updated_balancer) >= balancer_unit_seconds)
			{
				last_clock_when_updated_balancer = clock;

				uint8_t longest_balancing = 0;
				for(uint8_t node = 0; node < num_nodes; node++)
				{
					if(nodes[node].balance_time > 0)
					{
						print_string("Bal");
						utoa(node+1, buf, 10);
						print_string(buf);
						print_string("=");
						utoa(nodes[node].balance_time, buf, 10);
						print_string(buf);
						print_char(' ');

						shunt_node(node+1, balancer_unit_seconds+5);
						_delay_ms(2);

						if(nodes[node].balance_time > longest_balancing)
						{
							longest_balancing = nodes[node].balance_time;
						}

						nodes[node].balance_time--;
					}
				}
				if(longest_balancing == 0)
				{
					balancer_en = 0;
				}

				utoa(longest_balancing, buf, 10);
				lcd_print(buf);
				save_bal_eeprom();
			}

		}
		else
		{
			lcd_inst(LCD_LINES[3]);
			lcd_print("      ");
		}

		button_cnt += (~PING)&1;

		lcd_inst(LCD_LINES[2]);
		if(drive_en)
			lcd_print("DRIVE");
		else
			lcd_print("STOP ");

		lcd_inst(LCD_LINES[2]+7);
		if(charge_en)
		{
			if(batt_full)
			{
				lcd_print("ERR001");
				CHARGER_OFF();
			}
			else
				lcd_print("CHARGE");
		}
		else
		{
			if(batt_full)
				lcd_print(" FULL ");
			else if(heat_en)
				lcd_print(" COLD ");
			else
				lcd_print("ERR002");
		}

		lcd_inst(LCD_LINES[2]+13+3);
		if(heat_en)
			lcd_print("HEAT");
		else
			lcd_print("    ");

// -------++++++-------
// DRIVE  CHARGE   HEAT
// BAL123

// -------++++++-------
// STOP    FULL
//

		button_cnt += (~PING)&1;


		uint16_t new_timer_val;
		uint8_t last_clock = clock;
		// Make sure that no ISR takes longer than 1024 cycles,
		// which would cause a skipping of a count.
		while(1)
		{
			new_timer_val = TCNT1;
			if((new_timer_val - last_timer_val) == 7812)  // should be 7812.5
			{
				clock++;
				break;
			}
			else if((new_timer_val - last_timer_val) == 15625)
			{
				clock+=2;
				break;
			}
			else if((new_timer_val - last_timer_val) == 23437)
			{
				clock+=3;
				break;
			}
			else if((new_timer_val - last_timer_val) == 31250)
			{
				clock+=4;
				break;
			}
			else if((new_timer_val - last_timer_val) == 39062)
			{
				clock+=5;
				break;
			}
			else if((new_timer_val - last_timer_val) == 46875)
			{
				clock+=6;
				break;
			}
			else if((new_timer_val - last_timer_val) == 54687)
			{
				clock+=7;
				break;
			}
			else if((new_timer_val - last_timer_val) == 62500)
			{
				clock+=8;
				break;
			}
		}
		last_timer_val = new_timer_val;

		accumulate_As(clock - last_clock);

		//utoa(clock, buf, 10);
		//lcd_inst(LCD_LINES[3]+16);
		//lcd_print(buf); if(clock<10) lcd_data(' '); if(clock<100) lcd_data(' ');
		lcd_inst(LCD_LINES[3]+19);
		if(clock & 1)
			lcd_data('.');
		else
			lcd_data(':');

		print_string("lastMinI=");
		itoa(last_min_I, buf, 10);
		print_string(buf);

		print_string(" lastMaxI=");
		itoa(last_max_I, buf, 10);
		print_string(buf);

//		lcd_inst(LCD_LINES[1]);
//		for(uint8_t i = 0; i < 20; i++)
//			lcd_data(' ');

		lcd_inst(LCD_LINES[0]);

		print_string(" lastAvgI=");
		itoa(last_avg_I, buf, 10);
		print_string(buf);

		print_string(" AsCount=");
		itoa(As_count, buf, 10);
		print_string(buf);

		print_string(" AhCount=");
		itoa(Ah_count, buf, 10);
		print_string(buf);

		calc_voltage();

		print_string(" Vtot=");
		utoa(cur_voltage, buf, 10);
		print_string(buf);

		sprintf(buf, "%4dA %4dAh %2uV  ", last_avg_I, Ah_count, cur_voltage);
		lcd_print(buf);

		lcd_inst(LCD_LINES[1]);

		if(SoC > 99)
			lcd_data(255);
		else if(SoC < 3)
			lcd_data('!');
		else
			lcd_data(BLOCK_LEVELS[SoC/13]);

		if(SoC < 100)
			lcd_data(' ');
		itoa(SoC, buf, 10);
		lcd_print(buf);
		lcd_data('%');
		lcd_data(' ');

		print_string(" SoC=");
		print_string(buf);
		print_char(' ');

		calc_range();
		itoa(range_left, buf, 10);
		lcd_print(buf);
		lcd_print("km");
		print_string(" range_left=");
		print_string(buf);
		print_string("   ");

		print_string("\r\n\r\n");

		uint16_t smallest_v = 9999;
		uint16_t largest_v  = 0;

		int32_t temper = 0;

		for(uint8_t node = 0; node < num_nodes; node++)
		{
			if(nodes[node].v > largest_v)
				largest_v = nodes[node].v;

			if(nodes[node].v < smallest_v)
				smallest_v = nodes[node].v;

			temper += nodes[node].t;
		}
		last_avg_T = temper/num_nodes;

		char kakkulapio[10];
		char kukkapurkki[10];
		utoa(smallest_v, kakkulapio, 10);
		utoa(largest_v, kukkapurkki, 10);

		buf[0] = kakkulapio[0];
		buf[1] = '.';
		buf[2] = kakkulapio[1];
		buf[3] = '-';
		buf[4] = kukkapurkki[0];
		buf[5] = '.';
		buf[6] = kukkapurkki[1];
		buf[7] = 'V';
		buf[8] = 0;

		lcd_inst(LCD_LINES[1]+12);
		lcd_print(buf);

		lcd_inst(LCD_LINES[0]+16);
		itoa(K2C(last_avg_T), buf, 10);
		lcd_print(buf);
		lcd_data('C');

		if(button_cnt > 6)
			cur_message = 0;

		if(start_comm_err_cnt > 99)
			start_comm_err_cnt = 0;
		if(middle_comm_err_cnt > 99)
			middle_comm_err_cnt = 0;
		if(end_comm_err_cnt > 99)
			end_comm_err_cnt = 0;
		if(misc_comm_err_cnt > 99)
			misc_comm_err_cnt = 0;


		lcd_inst(LCD_LINES[3]+7);
		if(cur_message != 0)
			lcd_print(MESSAGES[cur_message]);
		else
		{
			lcd_data('S');
			utoa(start_comm_err_cnt, buf, 10);
			if(start_comm_err_cnt < 10) lcd_data(' ');
			lcd_print(buf);
			lcd_data('M');
			utoa(middle_comm_err_cnt, buf, 10);
			if(middle_comm_err_cnt < 10) lcd_data(' ');
			lcd_print(buf);
			lcd_data('E');
			utoa(end_comm_err_cnt, buf, 10);
			if(end_comm_err_cnt < 10) lcd_data(' ');
			lcd_print(buf);
			lcd_data('O');
			utoa(misc_comm_err_cnt, buf, 10);
			if(misc_comm_err_cnt < 10) lcd_data(' ');
			lcd_print(buf);

		}
	}


	return 0;
}
