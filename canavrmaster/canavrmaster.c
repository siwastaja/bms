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


uint8_t UART_byte_received()
{
	return (UCSR1A & 0b10000000);
}

#define UART_BYTE UDR1


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

uint8_t clock = 0;

// Unclear is handled as both LVC+HVC.

#define NODE_CHAN_BITS 0b00000011
#define NODE_ALT_LIMIT 0b00000100
#define NODE_CHA_ENA   0b00001000
#define NODE_DSCH_ENA  0b00010000
#define NODE_CURR_LIM  0b00100000
#define NODE_HEAT_ENA  0b01000000
#define NODE_COOL_ENA  0b10000000


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

typedef struct
{
	uint16_t hvc;
	uint16_t lvc;
	int8_t overtemp;
	int8_t undertemp;
	int8_t heattemp;
	int8_t cooltemp;
} limits_type;

limits_type limits = {3600, 2700, 60, 0, 5, 30};

uint16_t wh_per_km = 150;

uint8_t unclear_time_limit = 4;
uint8_t lvc_filter_limit   = 4;

uint8_t  temperature_poll_rate = 10;

int16_t adc_offset = 192;
int16_t adc_divider = 97;


#define K2C(x) ((int16_t)(x)-273)
#define C2K(x) ((int16_t)(x)+273)

#define MAX_NODES 100

// index+1 is ID.
node nodes[MAX_NODES];

uint8_t num_nodes = 8;

#define VOLT_MULT (uint32_t)9
#define VOLT_DIV (uint32_t)2

#define CHAN_RE_REG   PORTC
#define CHAN_DATA_REG PINC
#define CHAN_CLK_REG  PINE

#define CHAN_READ_TIMEOUT 65000

#define FIFO_ERR_TIMEOUT 254

uint8_t read_channelfifo(uint8_t chan, data_t* packet)
{
	uint8_t data_pin = (3-chan)<<1;
	uint8_t re_pin = ((3-chan)<<1) + 1;
	uint8_t clk_pin = chan+4;
	uint16_t timeout = CHAN_READ_TIMEOUT;

	uint8_t bit_cnt = 31;

	packet.abcd = 0;

	sbi(CHAN_RE_REG, re_pin);

	while(timeout--)
	{
		// Wait until clk has falling edge:
		if((CHAN_CLK_REG&(1<<clk_pin)) &&
		   !(CHAN_CLK_REG&(1<<clk_pin)))
		{
			cbi(CHAN_RE_REG, re_pin);
			timeout = CHAN_READ_TIMEOUT;
			(*packet).abcd |= ( (CHAN_DATA_REG&(1<<data_pin)) >> data_pin ) << bit_cnt;
			if(bit_cnt > 0)
			{
				bit_cnt--;
			}
			else
			{
				return 0;
			}
		}

	}
	return FIFO_ERR_TIMEOUT;
}

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
	if((packet->b & 0b11110000) == 0b01000000)
	{
		// From 12-bit 2mvs to 16-bit mvs.
		nodes[node].v = (uint16_t)(((uint16_t)(packet->b & 0x0f) << 8) | packet->c) << 1;
		nodes[node].v_last_valid_time = clock;
	}
	else if(packet->b & 0b11110000) == 0b01100000)
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

ISR(INT4_VECT)
{
	data_t data;
	read_channelfifo(0, &data);
	if(bridge_mode_on)
	{
		print_hex(data.a);
		print_char(' ');
		print_hex(data.b);
		print_hex(data.c);
		print_char(' ');
		print_hex(data.d);
		print_string(";\r\n");
	}
	else
	{
		handle_packet(&data);
	}

}


#define HEATER_OFF()  cbi(PORTJ, 2);
#define LOAD_OFF()    ; // cbi(PORTJ, 2);
#define CHARGER_OFF() cbi(PORTJ, 3);

#define HEATER_ON()  sbi(PORTJ, 2);
#define LOAD_ON()    ; // sbi(PORTJ, 2);
#define CHARGER_ON() sbi(PORTJ, 3);

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
	uint8_t pos = 0;
	while(1)
	{
		if(UART_byte_received())
		{
			char key = UART_BYTE;
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
{"n_cells",
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
"invert_current",
"bat_heater_low_limit",
"wh_per_km",
"balancer_low_v",
"balancer_high_v",
"adc_offset",
"adc_divider"
};


const int16_t config_limit_mins[NUM_CONF_PARAMS] =
{1,
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
4350,
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

uint8_t bridge_mode_on = 0;
void bridge_mode(uint8_t calc_crc)
{
	bridge_mode_on = 1;
	while(1)
	{
		uint8_t chan;
		data_t packet;
		while(!UART_byte_received());
		chan = UART_BYTE-'0';
		if(chan > 3)
			return;
		for(uint8_t i = 0; i < 3; i++)
		{
			while(!UART_byte_received());
			packet.block[i] = UART_BYTE;
		}
		manchester_send_1chan(&packet, chan);
	}
	bridge_mode_on = 0;

}

void config()
{
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
		print_string("\r\n[4] RS232 bridge mode ([shift-4] w/ CRC generation)");
		print_string("\r\n[5] Re-zero current sensor offset");
		print_string("\r\n[6] Set current integrator value");

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
			turha = UART_BYTE;

		}
		else if(key == '4')
		{
			bridge_mode(0);
		}
		else if(key == '¤')
		{
			bridge_mode(1);
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

void init_can()
{
	// Initialize message objects as "disabled"
	for(uint8_t i; i < 15; i++)
	{
		CANPAGE = (i << 4);
		CANCDMOB = 0b00000000;
	}

	// 250 kbps, 0.250 us TQ
	CANBT1 = 0x02;
	CANBT2 = 0x0c;
	CANBT3 = 0x37;

	CANTCON = 100; // CAN timer prescaler - count at 10 kHz (0.1 ms resolution)

	CANGCON = 0b00000010; // Controller enable command


	(CANGSTA & 0b100) // check if really enabled
}

uint8_t send_can_shit(uint16_t id, uint8_t* data, uint8_t datalen)
{
	//can.Message(arbitration_id=0x301, data=[0, 0, 10, 10, 80, 0, 0, 0], extended_id=False) bus.send(msg)
	//can.Message(arbitration_id=0x302, data=[0, 0, 0, 0, 8, 8, 20, 20], extended_id=False) bus.send(msg)
	//can.Message(arbitration_id=0x303, data=[15, 15, 5, 5, 0b11, 0, 0, 0], extended_id=False) bus.send(msg)
	if(datalen > 8)
		return 1;


	uint8_t mob = 0;

	CANPAGE = (mob << 4) | (0); // goes to MOB, sets fifo pointer to 0, autoincrement on.

	CANCDMOB = 0b0100 | datalen;

	CANIDT1 = (id>>3)&0xff;
	CANIDT2 = (id&0b111)<<5;
	CANIDT4 = (0 << 2) | (0); // RTR & RB0 bits.

	for(uint8_t i = 0; i < datalen; i++)
	{
		CANMSG = data[i];
	}
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
		if(drive_en)    print_string("DRIVE ");
		if(charge_en)   print_string("CHARGE ");
                if(heat_en)     print_string("BATT_HEAT ");
		if(balancer_en) print_string("BALANCER ");
		if(batt_full)   print_string("BATT_FULL ");

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

		button_cnt += (~PING)&1;

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

		if(start_comm_err_cnt > 99)
			start_comm_err_cnt = 0;
		if(middle_comm_err_cnt > 99)
			middle_comm_err_cnt = 0;
		if(end_comm_err_cnt > 99)
			end_comm_err_cnt = 0;
		if(misc_comm_err_cnt > 99)
			misc_comm_err_cnt = 0;


	}


	return 0;
}
