#define F_CPU 16000000UL

#include <inttypes.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>

#include "../common/manchester.h"

uint8_t comm_error_cnt;
uint8_t start_comm_err_cnt;
uint8_t middle_comm_err_cnt;
uint8_t end_comm_err_cnt;
uint8_t misc_comm_err_cnt;

uint8_t drive_en;
uint8_t charge_en;
uint8_t heat_en;
uint8_t balancer_en;
uint8_t batt_full   = 1;
uint8_t batt_empty;

uint8_t batt_full_reset_soc = 97;


int16_t last_max_I;
int16_t last_min_I;
int16_t last_avg_I;
int16_t last_avg_T;
int32_t As_count;
int32_t last_As_count = -999;
int16_t Ah_count;
uint16_t cur_voltage;
int16_t range_left;

uint16_t pack_capacity_Ah;
int16_t SoC;

uint8_t over_SoC_limit = 110;

uint8_t no_idle_mode;

volatile int16_t max_I_tmp = -30000;
volatile int16_t min_I_tmp = 30000;
volatile int32_t I_acc;
volatile uint16_t num_I_meas;
volatile int32_t raw_I_acc;

#define print_char(byte) {while((UCSR1A & 0b00100000) == 0) ; UDR1 = (byte); }


uint8_t UART_byte_received()
{
	return (UCSR1A & 0b10000000);
}

#define UART_BYTE UDR1

#define POLL_T 1
#define POLL_V 0

#define TP_ON() sbi(PORTB, 6)
#define TP_OFF() cbi(PORTB, 6)

void print_hex(uint8_t num)
{
	uint8_t hi = (num & 0xF0) >> 4;
	uint8_t lo = num & 0x0F;

  	// Wait for empty transmit buffer 
  	while((UCSR1A & 0b00100000) == 0) ;

  	// Start transmission 
	if(hi>9)
  		UART_BYTE = hi+55;
	else
		UART_BYTE = hi+48;

  	// Wait for empty transmit buffer 
  	while((UCSR1A & 0b00100000) == 0) ;

  	// Start transmission 
	if(lo>9)
  		UART_BYTE = lo+55;
	else
		UART_BYTE = lo+48;

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

volatile uint8_t clock = 0;

// Unclear is handled as both LVC+HVC.

#define NODE_CHAN_BITS 0b00000011
#define NODE_ALT_LIMIT 0b00000100
#define NODE_CHA_ENA   0b00001000
#define NODE_DSCH_ENA  0b00010000
#define NODE_CURR_LIM  0b00100000
#define NODE_HEAT_ENA  0b01000000
#define NODE_COOL_ENA  0b10000000

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
#define POLL_CLOCK_MASK 0b00000111


int16_t adc_offset = 192;
int16_t adc_divider = 97;


#define K2C(x) ((int16_t)(x)-273)
#define C2K(x) ((int16_t)(x)+273)

// Temperature in Celsius (int8_t), -128 = invalid

typedef struct
{
	uint8_t flags;
	uint16_t v;
	int8_t t;
	uint8_t v_last_valid_time;
	uint8_t t_last_valid_time;
	uint8_t balance_time;
} node;

#define MAX_NODES 100

// index+1 is ID.
volatile node nodes[MAX_NODES];

uint8_t num_nodes = 8;

#define VOLT_MULT (uint32_t)9
#define VOLT_DIV (uint32_t)2

#define CHAN_RE_REG   PORTC
#define CHAN0_RE_IDX 7
#define CHAN1_RE_IDX 5
#define CHAN2_RE_IDX 3
#define CHAN3_RE_IDX 1

#define CHAN_DATA_REG PINC
#define CHAN0_DATA_MSK 0b01000000
#define CHAN1_DATA_MSK 0b00010000
#define CHAN2_DATA_MSK 0b00000100
#define CHAN3_DATA_MSK 0b00000001

#define CHAN_CLK_REG  PINE
#define CHAN0_CLK_MSK 0b00010000
#define CHAN1_CLK_MSK 0b00100000
#define CHAN2_CLK_MSK 0b01000000
#define CHAN3_CLK_MSK 0b10000000

// Channelfifo should interrupt data for only maximum of about 2 ms.
// Timeout after 5 ms.
// At 8 MHz, 5 ms is 400 000 cycles.
// About ten cycles per timeout loop.
#define CHAN_READ_TIMEOUT 40000

#define FIFO_ERR_TIMEOUT 254

#define HANDLE_MASTER_PACKET 1
#define HANDLE_ILLEGAL_NODE 2
#define HANDLE_COMM_ERROR 3

uint8_t handle_packet(data_t* packet)
{
	uint8_t node = packet->a;

	if(packet->b & 0b10000000)
		return HANDLE_MASTER_PACKET;
	if(node > num_nodes || node == 0)
		return HANDLE_ILLEGAL_NODE;

	node--;
	if((packet->b & 0b11110000) == 0b01000000)
	{
		// From 12-bit 2mvs to 16-bit mvs.
		nodes[node].v = (uint16_t)(((uint16_t)(packet->b & 0x0f) << 8) | packet->c) << 1;
		nodes[node].v_last_valid_time = clock;
	}
	else if((packet->b & 0b11110000) == 0b01100000)
	{
		uint16_t traw = ((uint16_t)(packet->b & 0x0f) << 8) | packet->c;
		nodes[node].t = K2C(traw>>2); // 12-bit to 10-bit, 1 LSB = 1 deg C
		nodes[node].t_last_valid_time = clock;
	}
	else if(packet->b == 0x03)
	{
		comm_error_cnt++;
		return HANDLE_COMM_ERROR;
	}

	return 0;
}

uint8_t fifo_rcv_fail_cnt = 0;

ISR(INT4_vect)
{
	// It always takes some time for channelfifo to start
	// sending data after receiving RE signal; we can use
	// about 20 cycles for init stuff after giving the signal.
	sbi(CHAN_RE_REG, CHAN0_RE_IDX);

	data_t data;
	data.abcd = 0;
	uint16_t timeout = CHAN_READ_TIMEOUT;
	uint8_t bit_cnt = 31;
	uint8_t fail = 1;

	while(timeout--)
	{
		if((CHAN_CLK_REG&CHAN0_CLK_MSK))
		{
			cbi(CHAN_RE_REG, CHAN0_RE_IDX);
			timeout = CHAN_READ_TIMEOUT;
			data.abcd >>= 1;
			if(CHAN_DATA_REG&CHAN0_DATA_MSK)
				data.abcd |= 0x80000000;

			if(!(bit_cnt--))
			{
				fail = 0;
				break;
			}
			// Normally, at this point, the clk is back to low.
			// If the channelfifo was interrupted by incoming packet,
			// clock may be held high.
			while((CHAN_CLK_REG&CHAN0_CLK_MSK))
			{
				if(--timeout == 0)
				{
					break;
				}
			}
		}

	}
	cbi(CHAN_RE_REG, CHAN0_RE_IDX);
	sbi(EIFR, 4);


	if(fail)
		fifo_rcv_fail_cnt++;
	else
		handle_packet(&data);

}


#define HEATER_OFF()  ; //cbi(PORTJ, 2);
#define LOAD_OFF()    ; // cbi(PORTJ, 2);
#define CHARGER_OFF() ; //cbi(PORTJ, 3);

#define HEATER_ON()  ; //sbi(PORTJ, 2);
#define LOAD_ON()    ; // sbi(PORTJ, 2);
#define CHARGER_ON() ; //sbi(PORTJ, 3);

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


int16_t uart_read_int16()
{
	char buf[8];
	uint8_t pos = 0;
	while(1)
	{
		if(UART_byte_received())
		{
			char key = UART_BYTE;
			if(key == '\r' || key == '\n' || key == ';')
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

#define NUM_CONF_PARAMS 24

const char* config_texts[NUM_CONF_PARAMS] =
{"n_cells",
"ch1_start_idx",
"ch2_start_idx",
"ch3_start_idx",
"ch4_start_idx",
"hvc_mv",
"lvc_mv",
"overtemp",
"undertemp",
"unclear_delay",
"lvc_delay",
"balancer_unit_sec",
"temp_poll_rate",
"pack_Ah",
"over_soc_shutdown",
"timeout_per_node_ms",
"shunt_limit",
"bat_heater_low_limit",
"wh_per_km",
"balancer_low_v",
"balancer_high_v",
"adc_offset",
"adc_divider"
};


const int16_t config_limit_mins[NUM_CONF_PARAMS] =
{1,
-1,
-1,
-1,
-1,
3000,
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
-10,
10,
2000,
2000,
-20000,
10};

const int16_t config_limit_maxs[NUM_CONF_PARAMS] =
{MAX_NODES,
MAX_NODES,
MAX_NODES,
MAX_NODES,
MAX_NODES,
4400,
4200,
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
30,
2000,
5000,
5000,
20000,
2000};


uint8_t channel_start_idxs[4];

// Resolve the channel and send the packet in the right channel.
// If config is illegal and channel cannot be resolved, data is
// sent on the last channel.
/*
void manchester_send_1node(bms_packet_t* data, uint8_t node)
{
	uint8_t chan;
	for(chan = 0; chan < 3; chan++)
	{
		if(node < channel_start_idxs[chan+1])
		{
			break;
		}
	}
	manchester_send_1chan(data, chan);
}
*/
void assign_node_ids()
{
	for(uint8_t chan = 0; chan < 4; chan++)
	{
		if(channel_start_idxs[chan] == 0)
			break;

		data_t data;
		data.a = 255;
		data.b = 0xb0;
		data.c = channel_start_idxs[chan];
//		manchester_send_1chan(&data, chan);
	}
}

void shunt_node(uint8_t node, uint8_t time)
{
	data_t data;
	data.a = node;
	data.b = 0b10101100;
	data.c = time;
	cli();
	manchester_send(&data);
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
	// gap, can be reused
	eeprom_write_word((uint16_t*)30, bat_heater_temp_limit);
	eeprom_write_word((uint16_t*)32, wh_per_km);
	eeprom_write_word((uint16_t*)34, balancer_low_v_point);
	eeprom_write_word((uint16_t*)36, balancer_high_v_point);
	eeprom_write_word((uint16_t*)38, adc_offset);
	eeprom_write_word((uint16_t*)40, adc_divider);
	eeprom_write_byte((uint8_t*)42, channel_start_idxs[0]);
	eeprom_write_byte((uint8_t*)44, channel_start_idxs[0]);
	eeprom_write_byte((uint8_t*)46, channel_start_idxs[0]);
	eeprom_write_byte((uint8_t*)48, channel_start_idxs[0]);
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
	// gap, can be reused
	bat_heater_temp_limit = eeprom_read_word((uint16_t*)30);
	wh_per_km = eeprom_read_word((uint16_t*)32);
	balancer_low_v_point = eeprom_read_word((uint16_t*)34);
	balancer_high_v_point = eeprom_read_word((uint16_t*)36);
	adc_offset = eeprom_read_word((uint16_t*)38);
	adc_divider = eeprom_read_word((uint16_t*)40);
	channel_start_idxs[0] = eeprom_read_byte((uint8_t*)42);
	channel_start_idxs[1] = eeprom_read_byte((uint8_t*)44);
	channel_start_idxs[2] = eeprom_read_byte((uint8_t*)46);
	channel_start_idxs[3] = eeprom_read_byte((uint8_t*)48);

	batt_full = eeprom_read_byte((uint8_t*)999);
	As_count = eeprom_read_dword((uint32_t*)1000);
}


uint8_t node_read_eeprom_8b(uint8_t node_id, uint8_t address, uint8_t* data)
{
	data_t msg, reply;
	msg.a = node_id;
	msg.b = 0b10010000; // Retrieve 8-bit eeprom value
	msg.c = address;

	manchester_send(&msg);
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
	manchester_send(&msg);
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
	manchester_send(&msg);
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

void config()
{
	char buf[12];
	data_t msg;

	while(1)
	{
		print_string("\r\n\r\nBMS Configuration");

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
				case 1: utoa(channel_start_idxs[0], buf, 10); break;
				case 2: utoa(channel_start_idxs[1], buf, 10); break;
				case 3: utoa(channel_start_idxs[2], buf, 10); break;
				case 4: utoa(channel_start_idxs[3], buf, 10); break;
				case 5: utoa(hvc_limit, buf, 10); break;
				case 6: utoa(lvc_limit_ocv, buf, 10); break;
				case 7: itoa(K2C(overtemp_limit), buf, 10); break;
				case 8: itoa(K2C(charge_temp_limit), buf, 10); break;
				case 9: utoa(unclear_time_limit, buf, 10); break;
				case 10: utoa(lvc_filter_limit, buf, 10); break;
				case 11: utoa(balancer_unit_seconds, buf, 10); break;
				case 12: utoa(temperature_poll_rate, buf, 10); break;
				case 13: utoa(pack_capacity_Ah, buf, 10); break;
				case 14: utoa(over_SoC_limit, buf, 10); break;
				case 15: utoa(timeout_per_node/534, buf, 10); break;
				case 16: utoa(shunt_limit, buf, 10); break;
				case 17: itoa(K2C(bat_heater_temp_limit), buf, 10); break;
				case 18: utoa(wh_per_km, buf, 10); break;
				case 19: utoa(balancer_low_v_point, buf, 10); break;
				case 20: utoa(balancer_high_v_point, buf, 10); break;
				case 21: itoa(adc_offset, buf, 10); break;
				case 22: itoa(adc_divider, buf, 10); break;
				default: break;
			}
			print_string(buf);

		}

		// Print channel configuration for sanity check:
		// ei tää näin mene
		uint8_t num_nodes_per_ch[4];
		num_nodes_per_ch[3] = num_nodes;
		for(uint8_t i = 1; i < 4; i++)
		{
			if(channel_start_idxs[i] == 0)
				num_nodes_per_ch[i] = 0;
			num_nodes_per_ch[3] -= (num_nodes_per_ch[i-1] = channel_start_idxs[i]-1);
		}

		print_string("\r\n\r\nChannel config: ");
		for(uint8_t i = 0; i < 4; i++)
		{
			print_string("CH");
			itoa(i, buf, 10); print_string(buf);
			print_string(": IDs ");
			itoa(channel_start_idxs[i], buf, 10); print_string(buf);
			print_string("...");
			itoa(channel_start_idxs[i]+num_nodes_per_ch[i], buf, 10); print_string(buf);
			print_string(" (");
			itoa(num_nodes_per_ch[i], buf, 10); print_string(buf);
			print_string(" nodes   ");
		}

		print_string("\r\n\r\n[1] Assign new node IDs");
		print_string("\r\n[2] Send custom message");
		print_string("\r\n[3] Locate node");
		print_string("\r\n[4] Re-zero current sensor offset");
		print_string("\r\n[5] Set current integrator value");

		print_string("\r\n[x] Save configuration (don't forget!)");
		print_string("\r\n[y] Reload configuration (undo changes)");
		print_string("\r\n[z] Quit configuration menu");

	  NEW_PROMPT:

		print_string("\r\nEnter your command: ");


		while(!UART_byte_received());

		char key = UART_BYTE;
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
				print_string("  OUT OF RANGE\r\n");
				goto NEW_PROMPT;
			}
			switch(key-'a')
			{
				case 0: num_nodes = val; break;
				case 1: channel_start_idxs[0] = val; break;
				case 2: channel_start_idxs[1] = val; break;
				case 3: channel_start_idxs[2] = val; break;
				case 4: channel_start_idxs[3] = val; break;
				case 5: hvc_limit = val; break;
				case 6: lvc_limit_ocv = val; break;
				case 7: overtemp_limit = C2K(val); break;
				case 8: charge_temp_limit = C2K(val); break;
				case 9: unclear_time_limit = val; break;
				case 10: lvc_filter_limit = val; break;
				case 11: balancer_unit_seconds = val; break;
				case 12: temperature_poll_rate = val; break;
				case 13: pack_capacity_Ah = val; break;
				case 14: over_SoC_limit = val; break;
				case 15: timeout_per_node = val*534; break;
				case 16: shunt_limit = val; break;
				case 17: bat_heater_temp_limit = C2K(val); break;
				case 18: wh_per_km = val; break;
				case 19: balancer_low_v_point = val; break;
				case 20: balancer_high_v_point = val; break;
				case 21: adc_offset = val; break;
				case 22: adc_divider = val; break;
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
						keys[i] = UART_BYTE;
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
			manchester_send(&msg);
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
			turha = UART_BYTE;

		}
		else if(key == '4')
		{
			print_string("\r\nMeasuring...");
			zero_I_offset();
			print_string(" adc_offset set. Check and save if happy.\r\n");
		}
		else if(key == '5')
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
	}

  QUIT_CONFIG:

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
	print_char((flags & NODE_CHA_ENA)?'+':'-');
	print_string("CHARGE ");

	print_char((flags & NODE_DSCH_ENA)?'+':'-');
	print_string("LOAD ");

	print_char((flags & NODE_CURR_LIM)?'+':'-');
	print_string("CURR_LIMIT ");

	print_char((flags & NODE_HEAT_ENA)?'+':'-');
	print_string("HEATING ");

	print_char((flags & NODE_COOL_ENA)?'+':'-');
	print_string("COOLING ");

}

ISR(ADC_vect)
{
	int16_t val = ADC<<6;
	raw_I_acc += val;
	val -= adc_offset;
	val /= adc_divider;

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

void init_can()
{
	CANGCON |= (1 << SWRES);

	// 125 kbps
//	CANBT1 = 0x06;
//	CANBT2 = 0x0c;
//	CANBT3 = 0x37;

	// 250 kbps
	CANBT1 = 0x06;
	CANBT2 = 0x0c;
	CANBT3 = 0x37;

	CANTCON = 100; // CAN timer prescaler - count at 10 kHz (0.1 ms resolution)

	// Initialize message objects as "disabled"
	for(uint8_t i = 0; i < 15; i++)
	{
		CANPAGE = (i << 4);
		CANCDMOB &= 0; // read-write might be required.
		CANSTMOB &= 0;
	}

	CANGCON = 0b00000010; // Controller enable command

//	while(!(CANGSTA & 0b100)) // check if really enabled
//		;

}

uint8_t send_can(uint8_t mob, uint8_t extended, uint32_t id, uint8_t* data, uint8_t datalen)
{
	if(datalen > 8 || mob < 0 || mob > 7)
		return 1;

	uint16_t timeout = 65535;
	while(CANEN2 & (1 << mob)) if(!timeout--) return 1;

	CANPAGE = mob << 4; // goes to MOB, sets fifo pointer to 0, autoincrement on.

	CANSTMOB &= 0;

	CANCDMOB = datalen;

	if(extended)
	{
		CANCDMOB |= (1 << 4); // extended ID

		CANIDT4 = (id & 0b11111) << 3; // RTR, RB1, RB0 set as zero.
		CANIDT3 = (id >> 5)&0xff;
		CANIDT2 = (id >> 13)&0xff;
		CANIDT1 = (id >> 21)&0xff;
	}
	else
	{	// 11-bit addressing:
		CANIDT1 = (id>>3)&0xff;
		CANIDT2 = (id&0b111)<<5;
		CANIDT3 = 0;
		CANIDT4 = 0; // RTR & RB0 bits.
	}


	for(uint8_t i = 0; i < datalen; i++)
	{
		CANMSG = data[i];
	}

	CANCDMOB |= 0b01000000;

	return 0;
}

typedef union
{
	uint16_t u16;
	int16_t s16;
	struct {uint8_t first; uint8_t second;};
	uint16_t block[2];
} union16_t;

uint16_t batt_voltage_min_discharge = 1500;
uint16_t batt_voltage_max_charge = 3650;
uint16_t batt_current_max_charge = 150;
uint16_t batt_current_max_discharge = 800;
uint16_t cur_pack_v = 3300;
uint16_t cur_pack_i = 0;
uint16_t cur_soc = 500;

//V = 330, MAXI = 80, Vmin = 150, maxV = 365, max_charge = 15, Imeas = 0, DOD = 50, 
void zytek_pulse(uint16_t voltage, uint16_t current, uint16_t soc)
{
//	uint8_t zytek_msgs[3][8] = {
//	{0x00, 0x00, 0xf0, 0x0a, 0x00, 0x00, 0x00, 0x00},
//	{0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x64, 0x00},
//	{0x00, 0xb0, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00}};

	uint8_t zytek_msg[8];
	zytek_msg[0] = current.first;
	zytek_msg[1] = current.second;
	zytek_msg[2] = voltage.first;
	zytek_msg[3] = voltage.second;
	zytek_msg[4] = soc.first;
	zytek_msg[5] = soc.second;
	zytek_msg[6] = 0;
	zytek_msg[7] = 0;

	send_can(0, 0, 0x301, zytek_msg, 8);

	zytek_msg[0] = 0; // General error active bit 0 - other bits undefined
	zytek_msg[1] = 0; // Limp home mode bit 0 - other bits undefined
	zytek_msg[2] = 0; // Isolation error bit 0 - other bits undefined
	zytek_msg[3] = 0;
	*((uint16_t*)(&zytek_msg[4])) = batt_voltage_min_discharge;
	*((uint16_t*)(&zytek_msg[6])) = batt_current_max_discharge;

	send_can(0, 0, 0x302, zytek_msg, 8);

	*((uint16_t*)(&zytek_msg[0])) = batt_voltage_max_charge;
	*((uint16_t*)(&zytek_msg[2])) = batt_current_max_charge;

	zytek_msg[4] = 0b100; // disable regen, enable discharge
	zytek_msg[5] = 0;
	zytek_msg[6] = 0;
	zytek_msg[7] = 0;

	send_can(0, 0, 0x303, zytek_msg, 8);

}


#define PSU_CAN_MOB 0
#define PSU_CAN_EXTENDED 1
#define PSU_CAN_ID 0x0240f010


// Voltage in 0.1V
// Current in 0.1A
void psu_pulse(uint8_t enable, int16_t voltage, int16_t current)
{
	voltage /= 10;
	current *= 15;
	if(voltage < 0)   voltage = 0;
	if(voltage > 255) voltage = 255;
	if(current < 0)   current = 0;
	if(current > 255) current = 255;

//	uint8_t can_data[3] = {enable, current, voltage};
	uint8_t can_data[3] = {0x01, 10, 10};
	send_can(PSU_CAN_MOB, PSU_CAN_EXTENDED, PSU_CAN_ID, can_data, 3);
}

int main()
{
	char buf[12];
	uint16_t last_timer_val = 0;

	DDRA  = 0b11111111; // 7..4 manchester outputs, 3..0 GPO driver outputs
	PORTA = 0b11110000; // Manchester outputs high.
	DDRB  = 0b01000111; // 7..4 PCB extra pins, 3..0 SD card
	PORTB = 0b10110000; // Pullups to PCB extra pins (unconnected so far)
//	DDRC  = 0b10101010; // Channel fifo RE and Data pins.
	DDRC  = 0b10000000; // Channel fifo RE and Data pins.
	DDRD  = 0b00101000; // unused, canRx, canTx, unused, tx, rx, int1, unused
	PORTD = 0b10010001; // Pullups to unused pins.
	DDRE  = 0b00000100; // 7..4 channelfifo interrupts, 3 unused, 2 SD PWR ena, PDO, PDI programming pins
	PORTE = 0b11111011; // Pullups to INTn pins and unused pins.
//	DDRF  = 0b11111000; // 7..4 GPO driver outputs, nLEM_ENA, unused, ADC differential pins.
//	PORTF = 0b00000100; // LEM power enable. Pull-up to unused pin.
	DDRG  = 0b00000010; // PG4,3 = 32kHz osc, PG2&0 = unused, PG1 = CAN_RS
	PORTG = 0b00000101; // Pullup to unused pins.

	UBRR1 = 16; // 16 MHz 115200 bps, remember to use 2x
	UCSR1A = 0b00000010; // 2x mode.
	// 8 data bits, 1 stop bit, no parity bit
	UCSR1C = 0b00000110;
	// Enable RX and TX, but no interrupts.
	UCSR1B = 0b00011000;

/*
	ADMUX = 0b11010000;  // 2.56V reference, differential ADC0-ADC1  1x.
	ADCSRB = 0b00000000; // free running mode
	ADCSRA = 0b11101110; // prescaler = 64: 125 kHz. auto trigger.
	DIDR0 = 0b00000011;
*/

	TCCR1A = 0b00000000;
	TCCR1B = 0b00000101; // 1024 prescaler

//	EICRB = 0b00000000;
	EIMSK = 0b00010000;

	load_config();
//	if(timeout_per_node > 200000)
//		timeout_per_node = 20000;

//	load_bal_eeprom();
//	last_clock_when_updated_balancer = balancer_unit_seconds - 2;

	init_can();

	print_string("BMS Master started.\r\n");

	while(1)
	{

		print_char('.');
		//V = 330, MAXI = 80, Vmin = 150, maxV = 365, max_charge = 15, Imeas = 0, DOD = 50, 
		union16_t volt;
		union16_t curr;
		union16_t soc;
		volt.u16 = 3300;
		curr.u16 = 0;
		soc.u16 = 500;
		zytek_pulse(volt, curr, soc);

		_delay_ms(100);
/*		uint8_t can_data[3] = {0x01, 0x10, tv};
		send_can_shit(1, 0x0240f010, can_data, 3);
		_delay_ms(2000);
		tv += 3;
		if(tv > 100)
			tv = 20;
*/
	}



	clock = 0;
	sei();
	last_timer_val = TCNT1;
	while(1)
	{

/*		data_t adata;
		adata.a = 0xff;
		adata.b = 0b10001100;
		adata.c = 0x00;
		cli();
		manchester_send(&adata);
		sei();
*/
		if(clock%temperature_poll_rate == 0)
		{
			_delay_ms(7);
			for(uint8_t i = 0; i < num_nodes; i++)
				_delay_ms(7);
			adata.a = 0xff;
			adata.b = 0b10001110;
			adata.c = 0x00;
			cli();
			manchester_send(&adata);
			sei();
		}

		if(UART_byte_received())
		{
			char key = UART_BYTE;
			if(key == 26) // CTRL+Z
			{
				_delay_ms(200);
				if(UART_byte_received() && UART_BYTE == 'z')
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
/*		if(drive_en)    print_string("DRIVE ");
		if(charge_en)   print_string("CHARGE ");
                if(heat_en)     print_string("BATT_HEAT ");
		if(balancer_en) print_string("BALANCER ");
		if(batt_full)   print_string("BATT_FULL ");
*/
/*
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
*/
//		uint16_t combined_flags = check_limits_all_nodes();

		_delay_ms(100);

		if(clock%temperature_poll_rate == 0)
		{
			for(uint8_t i = 0; i < num_nodes; i++)
			{
				print_char('T');
				utoa(i+1, buf, 10);
				print_string(buf);
				print_char('=');
				itoa(nodes[i].t, buf, 10);
				print_string(buf);
				if(nodes[i].t_last_valid_time != clock)
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
			if(nodes[i].v_last_valid_time != clock)
				print_char('*');

			print_char(' ');

		}

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

//		print_flags(combined_flags);

/*		for(uint8_t i = 0; i < num_nodes; i++)
		{
			if(nodes[i].flags&NODE_SHUNTING)
				shunt_node(i+1, 10);
		}*/
		if(balancer_en)
		{
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

				save_bal_eeprom();
			}

		}


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

		print_string("lastMinI=");
		itoa(last_min_I, buf, 10);
		print_string(buf);

		print_string(" lastMaxI=");
		itoa(last_max_I, buf, 10);
		print_string(buf);

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

		print_string(" SoC=");
		print_string(buf);
		print_char(' ');

		calc_range();
		itoa(range_left, buf, 10);
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


	}


	return 0;
}
