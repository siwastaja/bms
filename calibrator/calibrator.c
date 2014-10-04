#include <inttypes.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdio_ext.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stropts.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <math.h>

//#include <iomanip.h>

#define CRC_INITIAL_REMAINDER 0x00
#define CRC_POLYNOMIAL 0x07 // As per CRC-8-CCITT

typedef union __attribute__ ((__packed__))
{
	struct __attribute__ ((__packed__))
	{
		uint8_t d;
		uint8_t c;
		uint8_t b;
		uint8_t a;
	};
	uint32_t abcd;
	uint8_t block[4];
} data_t;

int fd;
int num_nodes = 0;


uint8_t calc_crc_byte(uint8_t remainder)
{
	for(uint8_t bit = 8; bit > 0; --bit)
	{
		if((remainder) & 0b10000000)
			(remainder) = ((remainder) << 1) ^ CRC_POLYNOMIAL;
		else
			(remainder) = ((remainder) << 1);
	}
	return remainder;
}

void insert_crc(data_t* data)
{
	uint8_t remainder = CRC_INITIAL_REMAINDER;
	remainder ^= (*data).a;
	remainder = calc_crc_byte(remainder);
	remainder ^= (*data).b;
	remainder = calc_crc_byte(remainder);
	remainder ^= (*data).c;
	remainder = calc_crc_byte(remainder);
	(*data).d = remainder;
}

int kbhit()
{
	static const int STDIN = 0;
/*	static bool initialized = false;

	if(!initialized)
	{
		termios term;
		tcgetattr(STDIN, &term)
	}
*/
	int bytesWaiting;
	ioctl(STDIN, FIONREAD, &bytesWaiting);
	return bytesWaiting;
}

int set_interface_attribs(int fd, int speed, int parity, int should_block)
{
	struct termios tty;
	memset(&tty, 0, sizeof(tty));
	if(tcgetattr(fd, &tty) != 0)
	{
		printf("error %d from tcgetattr\n", errno);
		return -1;
	}

	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
	tty.c_iflag &= ~IGNBRK;
	tty.c_lflag = 0;
	tty.c_oflag = 0;
	tty.c_cc[VMIN] = 255; // should_block ? 1 : 0;
	tty.c_cc[VTIME] = 1; // 0.1s read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~(PARENB | PARODD);
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if(tcsetattr(fd, TCSANOW, &tty) != 0)
	{
		printf("error %d from tcsetattr\n", errno);
		return -1;
	}
	return 0;
}

int wr(int fd, const char* buf)
{
	int len = 0;
	while(buf[len] != 0) len++;
	if(len == 0) return 0;
	write(fd, buf, len);
}

void uart_send_packet(data_t* packet)
{
	insert_crc(packet);
	uint8_t buf[4] = {packet->a,packet->b,packet->c,packet->d};
	write(fd, buf, 4);
//	printf("Sent %x %x %x %x\n", buf[0], buf[1], buf[2], buf[3]);
}

int uart_read_packet(data_t* packet)
{
	uint8_t buf[4];
	int rcv_cnt = 0;
	while(rcv_cnt < 4)
	{
		rcv_cnt += read(fd, &(buf[rcv_cnt]), 4-rcv_cnt);
	}
	packet->a = buf[0];
	packet->b = buf[1];
	packet->c = buf[2];
	packet->d = buf[3];
//	printf("Rcvd %x %x %x %x\n", buf[0], buf[1], buf[2], buf[3]);
	return 0;
}

#define BROADCAST_ADDR 255
#define FACTORY_DEFAULT_ADDR 254

#define CMD_CHANGE_NODE_ID 0xb0
#define CMD_MEASURE        0x80
#define CMD_MEASURE_V      0
#define CMD_MEASURE_EXT_T  0b01
#define CMD_MEASURE_INT_T  0b10
#define CMD_MEASURE_REF_VCC 0
#define CMD_MEASURE_REF_1V1 0b100
#define CMD_MEASURE_CALIB 0b1000
#define CMD_MEASURE_LONG 0b10000

#define CMD_ENABLE_EEPROM 0xb1
#define CMD_WRITE_EEPROM  0xb2
#define CMD_SELFCHECK     0xb6

#define REPLY_OP_SUCCESS  0x00
#define REPLY_OP_FAILURE  0x01
#define REPLY_CMD_UNKNOWN 0x02
#define REPLY_COMM_ERROR  0x03
#define REPLY_EEPROM_CHECKSUM_ERROR 0x04
#define REPLY_EEPROM_DATA 0x05
#define REPLY_SELFCHECK_STATUS 0x06
#define REPLY_MEASUREMENT 0x40
#define REPLY_MEASUREMENT_SRC_MSK 0b00110000
#define REPLY_MEASUREMENT_SRC_SHIFT 4
#define REPLY_MEASUREMENT_MSBITS_MSK 0b00001111

#define MAX_NODES 255

int node_selfcheck(int node)
{
	if(node < 1 || node > 254 || node > num_nodes)
	{
		printf("node_selfcheck: invalid node number\n");
		return -1;
	}

	data_t cmd;
	cmd.a = node;
	cmd.b = CMD_SELFCHECK;
	cmd.c = 0;
	uart_send_packet(&cmd);

	data_t reply;
	uart_read_packet(&reply);

	if(reply.a == node && reply.b == REPLY_SELFCHECK_STATUS)
	{
		if(reply.c != 0)
			printf("node_selfcheck: Self-check reply indicates failure (%02x %02x %02x %02x)\n",
				reply.a, reply.b, reply.c, reply.d);
		return reply.c;
	}

	printf("node_selfcheck: Got unexpected data %02x %02x %02x %02x\n", reply.a, reply.b, reply.c, reply.d);
	return -2;
}

int node_write_eeprom(int node, int addr, uint8_t data)
{
	if(node < 1 || node > 254 || node > num_nodes)
	{
		printf("node_write_eeprom: invalid node number\n");
		return 1;
	}
	if(addr < 0 || addr > 127)
	{
		printf("node_write_eeprom: invalid eeprom address\n");
		return 2;
	}

	data_t cmd;
	cmd.a = node;
	cmd.b = CMD_ENABLE_EEPROM;
	cmd.c = addr;
	uart_send_packet(&cmd);

	cmd.b = CMD_WRITE_EEPROM;
	cmd.c = data;
	uart_send_packet(&cmd);

	data_t reply;
	uart_read_packet(&reply);
	if(reply.a != node || reply.b != REPLY_OP_SUCCESS || reply.c != CMD_WRITE_EEPROM)
	{
		printf("node_write_eeprom: invalid reply: %02x %02x %02x %02x\n", reply.a, reply.b, reply.c, reply.d);
		return 3;
	}
	return 0;

}

void parse_message(data_t* packet)
{
	if(packet->a == 255 && ((packet->b)&0b10000000))
		printf("    master's own broadcast message coming back\n");
	else if(packet->a == 255)
		printf("    ERR: node using 255 as its address or master sending crap (cmd < 0x80)\n");
	else if(packet->a == 0)
		printf("    ERR: illegal address 0 in packet\n");
	else if(packet->a == 254)
		printf("    Factory-programmed unsequenced node with default address (254)\n");
	else if(packet->a == 253)
		printf("    Message probably originated from active chain-end buffer (address 253)\n");
	else if(packet->a != 255 && ((packet->b)&0b10000000))
		printf("    Command coming back with address %u, maybe node not found\n", packet->a);
	else
		printf("    Node address = %u\n", packet->a);

	if((packet->b & 0b11100000) == 0b10000000)
	{
		printf("    Measurement command\n");
		if(packet->b & 16)
			printf("        Long acquire\n");
		else
			printf("        Short acquire\n");
		if(packet->b & 8)
			printf("        Calibration used\n");
		else
			printf("        Raw measurement\n");
		if(packet->b & 4)
			printf("        1V1 Voltage Reference\n");
		else
			printf("        Vcc Voltage Reference\n");
		printf("        Source: ");
		switch(packet->b & 0b11)
		{
			case 0b00: printf("Voltage\n"); break;
			case 0b01: printf("External sensor\n"); break;
			case 0b10: printf("Internal sensor\n"); break;
			case 0b11: printf("External sensor with pull-up resistor\n"); break;
		}
		printf("        Power consumption leveling value: %u\n", packet->c);
	}
	else if((packet->b & 0b11111100) == 0b10100000)
	{
		printf("    Shunt charge command\n        ");
		switch(packet->b & 0b11)
		{
			case 0b00: printf("Use CPU only (0.5 mA)\n"); break;
			case 0b01: printf("Use LED only (~15 mA)\n"); break;
			case 0b10: printf("Use measurement resistors only (~25 mA)\n"); break;
			case 0b11: printf("Use LED + resistors (~40 mA)\n"); break;
		}
		printf("        Timeout: %u seconds\n", packet->c);
	}
	else if(packet->b == 0xb0)
	{
		printf("    Change node ID command\n");
		printf("        New id = %u\n", packet->c);
		if(packet->a == 255)
			printf("        Sequencing command (sent with broadcast address)\n");
	}
	else if(packet->b == 0xb1)
	{
		printf("    Enable EEPROM write access command\n");
		printf("        EEPROM address = 0x%02x\n", packet->c);
	}
	else if(packet->b == 0xb2)
	{
		printf("    Write EEPROM byte command\n");
		printf("        Byte value = 0x%02x\n", packet->c);
	}
	else if(packet->b == 0xb3)
	{
		printf("    Set state command\n");
		if(packet->c & 1)
			printf("        Use Brown-Out Detector (default)\n");
		else
			printf("        Disable Brown-Out Detector (extra low power mode)\n");

		if(packet->c & 2)
			printf("        Visual mode (blink LED at rx/tx)\n");
		else
			printf("        LED visualization mode off (default)\n");
	}
	else if(packet->b == 0xb4)
		printf("    Reload shadowed variables from EEPROM\n");
	else if(packet->b == 0xb5)
	{
		printf("    Read EEPROM command\n");
		printf("        Read address: 0x%02x\n", packet->c);
	}
	else if(packet->b == 0xb6)
		printf("    Run self-check command\n");
	else if(packet->b & 0b10000000)
		printf("    Unrecognized command 0x%02x 0x%02x\n", packet->b, packet->c);
	else if(packet->b == 0x00)
		printf("    REPLY: Operation Success (0x%02x)\n", packet->c);
	else if(packet->b == 0x01)
		printf("    REPLY: Operation Failure (0x%02x)\n", packet->c);
	else if(packet->b == 0x02)
		printf("    REPLY: Unknown command (0x%02x)\n", packet->c);
	else if(packet->b == 0x03)
	{
		printf("    REPLY: Communication error\n");
		printf("        #0x%02x: ", packet->c);
		if(packet->c == 2)
			printf("Start transition (0->1) not received within window\n");
		else if(packet->c >= 0x40 && packet->c < 0x60)
			printf("High-low transition not received within window at bit %u\n", packet->c-0x40);
		else if(packet->c >= 0x60 && packet->c < 0x80)
			printf("Low-high transition not received within window at bit %u\n", packet->c-0x60);
		else if(packet->c >= 0x20 && packet->c < 0x40)
			printf("Signal too noisy or transitioning at a supposedly steady stage, at bit %u\n", packet->c-0x20);
		else if(packet->c == 255)
			printf("CRC8 checksum error\n");
		else
			printf("Unknown error code\n");
	}
	else if(packet->b == 0x05)
		printf("    REPLY: EEPROM data: 0x%02x\n", packet->c);
	else if(packet->b == 0x06)
	{
		printf("    REPLY: Self-check status\n");
		if(packet->c & 1)
			printf("        EEPROM corruption detected\n");
	}
	else if((packet->b&0b11000000) == 0b01000000)
	{
		printf("    REPLY: Measurement data\n");
		int val = ((packet->b&0x0f)<<8) | packet->c;
		printf("        Raw value: %u\n", val);
		printf("        Source: ");

		switch(packet->b & 0b00110000)
		{
			case 0b00000000: printf("Voltage\n");
				printf("        Interpreted as calibrated voltage: %lf V\n", (double)(val*2)/1000.0);
				printf("        Roughly interpreted as uncalibrated voltage: %lf V\n", (double)val*9.0/8000.0);
				break;
			case 0b00010000: printf("External sensor\n");
				break;
			case 0b00100000: printf("Internal sensor\n");
				printf("        Interpreted as temperature: %.4f deg C\n", (double)val/4.0-273.15);
				break;
			case 0b00110000: printf("0b11, this shouldn't happen\n");
				break;
		}
	}
	else if(packet->b < 0x80)
		printf("    Unrecognized message 0x%02x 0x%02x\n", packet->b, packet->c);
	else
		printf("    Internal error in parsing\n");
}

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
gain = (V2 - V1) / (RAW2 - RAW1)
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

gain (uint16_t): gain(uint16_t) + (temperature-temp_coeff_midpoint)(int8_t)*temp_coeff(int8_t)
result = gain(uint16_t) * data(uint16_t with 12 bits) >> shift

Typical values for voltage: gain = 9000, shift = 14

*/

#define V_MUX 0
#define T_MUX 2

typedef struct
{
	double v_act;
	double t_act;
	int raw[3];
} meas_point_t;

meas_point_t cal[MAX_NODES][3][3];


typedef struct
{
	int id;
	double v;
	double t;
	int raw[3];
	int offset[3];
	int gain[3];
	int t_coeff[3];
	int shift[3];
} node_t;

node_t nodes[MAX_NODES];

void calculate_calibration(int n)
{
	double gain[3];
	gain[1] = (double)(cal[n][1][2].v_act - cal[n][1][0].v_act) /
		(double)(cal[n][1][2].raw[V_MUX] - cal[n][1][0].raw[V_MUX]);

	//OFS1 = V1(RAW1 - RAW2) / (V2 - V1) + RAW1

	double ofs1 = (cal[n][1][0].v_act * (double)(cal[n][1][0].raw[V_MUX] - cal[n][1][2].raw[V_MUX])) /
		      ((double)(cal[n][1][2].v_act - cal[n][1][0].v_act)) + cal[n][1][0].raw[V_MUX];

	double ofs2 = (cal[n][1][2].v_act * (double)(cal[n][1][0].raw[V_MUX] - cal[n][1][2].raw[V_MUX])) /
		      ((double)(cal[n][1][2].v_act - cal[n][1][0].v_act)) + cal[n][1][2].raw[V_MUX];

	if(fabs(ofs1-ofs2) > 1.0)
		printf("V warning: |OFS1-OFS2| > 1.0 (OFS1=%lf, OFS2=%lf, diff=%lf)\n", ofs1, ofs2, fabs(ofs1-ofs2));

	double ofs = (ofs1+ofs2)/2.0;

	gain[0] = (double)(cal[n][0][2].v_act - cal[n][0][0].v_act) /
		(double)(cal[n][0][2].raw[V_MUX] - cal[n][0][0].raw[V_MUX]);

	gain[2] = (double)(cal[n][2][2].v_act - cal[n][2][0].v_act) /
		(double)(cal[n][2][2].raw[V_MUX] - cal[n][2][0].raw[V_MUX]);

	double t_coeff = (gain[2] - gain[0]) / (double)(cal[n][2][1].t_act - cal[n][0][1].t_act);

	// Maximize gain and offset values, without going over limits.

	int shift = 0;

	printf("V pre-shift: gain = %lf  ofs = %lf  t_coeff = %lf\n", gain[1], ofs, t_coeff);
	while(shift < 32)
	{
		if(gain[1] >= 32767.0 || t_coeff >= 127.0 || t_coeff <= -127.0)
			break;
		gain[1] *= 2.0;
		t_coeff *= 2.0;
		shift++;
	}
	printf("V post-shift: gain = %lf  ofs = %lf  t_coeff = %lf  shift = %u\n", gain[1], ofs, t_coeff, shift);


	nodes[n].offset[V_MUX] = (int)((ofs<0.0)?(ofs-0.5):(ofs+0.5));
	nodes[n].gain[V_MUX] = (int)(gain[1]+0.5);
	nodes[n].t_coeff[V_MUX] = (int)((t_coeff<0.0)?(t_coeff-0.5):(t_coeff+0.5));
	nodes[n].shift[V_MUX] = shift;

	printf("V Final: gain=%u  ofs = %d  t_coeff = %d  shift = %u\n", nodes[n].gain[V_MUX], nodes[n].offset[V_MUX], 
		nodes[n].t_coeff[V_MUX], nodes[n].shift[V_MUX]);


	// Do temperatures.
	double t_gain =
		(double)(cal[n][2][1].t_act - cal[n][0][1].t_act) /
		(double)(cal[n][2][1].raw[T_MUX] - cal[n][0][1].raw[T_MUX]);

	double t_ofs1 =
		(cal[n][0][1].t_act * (double)(cal[n][0][1].raw[T_MUX] - cal[n][2][1].raw[T_MUX])) /
		((double)(cal[n][2][1].t_act - cal[n][0][1].t_act)) + cal[n][0][1].raw[T_MUX];

	double t_ofs2 =
		(cal[n][2][1].t_act * (double)(cal[n][0][1].raw[T_MUX] - cal[n][2][1].raw[T_MUX])) /
		((double)(cal[n][2][1].t_act - cal[n][0][1].t_act)) + cal[n][2][1].raw[T_MUX];

	if(fabs(t_ofs1-t_ofs2) > 1.0)
		printf("T warning: |T_OFS1-T_OFS2| > 1.0 (T_OFS1=%lf, T_OFS2=%lf, diff=%lf)\n", t_ofs1, t_ofs2, fabs(t_ofs1-t_ofs2));

	double t_ofs = (t_ofs1+t_ofs2)/2.0;

	int t_shift = 0;

	printf("T pre-shift: gain = %lf  ofs = %lf\n", t_gain, t_ofs);
	while(t_shift < 32)
	{
		if(t_gain >= 32767.0)
			break;
		t_gain *= 2.0;
		t_shift++;
	}
	printf("T post-shift: gain = %lf  ofs = %lf  shift = %u\n", t_gain, t_ofs, t_shift);

	nodes[n].offset[T_MUX] = (int)((t_ofs<0.0)?(t_ofs-0.5):(t_ofs+0.5));
	nodes[n].gain[T_MUX] = (int)(t_gain+0.5);
	nodes[n].t_coeff[T_MUX] = 0;
	nodes[n].shift[T_MUX] = t_shift;

	printf("T Final: gain=%u  ofs = %d  t_coeff = %d  shift = %u\n", nodes[n].gain[T_MUX], nodes[n].offset[T_MUX], 
		nodes[n].t_coeff[T_MUX], nodes[n].shift[T_MUX]);

}

int get_values(int id, int get_t, int calibrated)
{
	int fail_cnt = 0;

	if(id < 0 || id > 255)
		return 1;

	int expected_replies = (id==255)?(num_nodes+1):1;

	data_t packet;
	packet.a = id;
	if(get_t)
		packet.b = CMD_MEASURE | CMD_MEASURE_INT_T | CMD_MEASURE_REF_1V1 | CMD_MEASURE_LONG;
	else
		packet.b = CMD_MEASURE | CMD_MEASURE_V | CMD_MEASURE_REF_1V1 | CMD_MEASURE_LONG;

	if(calibrated) packet.b |= CMD_MEASURE_CALIB;

	packet.c = 0;

	// First long request (start measuring)
	uart_send_packet(&packet);

	int num_replies = 0;
	while(1)
	{
		data_t reply;
		uart_read_packet(&reply);
		num_replies++;
		if(reply.abcd == packet.abcd)
			break;
		else if((reply.a > num_nodes) || ((reply.b&0b11000000) != 0b01000000))
		{
			printf("Warning: Ignoring unexpected packet %x %x %x %x\n", reply.a, reply.b, reply.c, reply.d);
			fail_cnt++;
		}
	}

	if(num_replies != expected_replies)
	{
		printf("Warning: expected %u messages, got %u\n", expected_replies, num_replies);
		fail_cnt++;
	}

	// Wait for at least 300 ms
	usleep(500000);

	// Get the results:
	uart_send_packet(&packet);

	while(1)
	{
		data_t reply;
		uart_read_packet(&reply);
		if(reply.abcd == packet.abcd)
			break;
		else if((reply.a > num_nodes) || ((reply.b&0b11000000) != 0b01000000))
		{
			printf("Warning: Ignoring unexpected packet %x %x %x %x\n", reply.a, reply.b, reply.c, reply.d);
			fail_cnt++;
		}
		else
		{
			if(reply.a < 1 || reply.a > 254)
			{
				printf("Warning: illegal node id %u\n", reply.a);
				fail_cnt++;
			}
			else
			{
				int val = ((int)(reply.b&REPLY_MEASUREMENT_MSBITS_MSK))<<8 | (int)reply.c;
				if(get_t)
				{
					nodes[reply.a-1].raw[T_MUX] = val;
					if(calibrated)
						nodes[reply.a-1].t = (double)val-273.15;
					else
						nodes[reply.a-1].t = (double)(val>>2)-273.15;
				}
				else
				{
					nodes[reply.a-1].raw[V_MUX] = val;
					if(calibrated)
						nodes[reply.a-1].v = (double)val*2.0/1000.0;
					else
						nodes[reply.a-1].v = (double)val*9.0/8000.0;
				}
			}
		}
	}

	if(num_replies != expected_replies)
	{
		printf("Warning: expected %u messages, got %u\n", expected_replies, num_replies);
		fail_cnt++;
	}

	usleep(500000);

	return fail_cnt;
}

#define HIBYTE(x) (((x)&0xff00)>>8)
#define LOBYTE(x) ((x)&0xff)

void calibration()
{
	int measurement_done[3][3] = {{0,0,0},{0,0,0},{0,0,0}};

	double calib_actual_temps[3] = {0.0, 23.0, 46.0};
	double calib_actual_volts[3] = {3.0, 3.5, 4.0};


	while(1)
	{
		printf("    actual V   %5.3f V [a] | %5.3f V [b] | %5.3f V [c]\n", calib_actual_volts[0], calib_actual_volts[1], calib_actual_volts[2]);
		printf("actual T      ---------------------------------------\n");
		for(uint8_t i = 0; i<3; i++)
		{
			printf("%5.1f C [%c]  |   %c  [%c]   |   %c  [%c]    |   %c  [%c]   |\n", 
				calib_actual_temps[i], 'd'+i,
				measurement_done[i][0]?'X':'-', '1'+3*i,
				measurement_done[i][1]?'X':'-', '2'+3*i,
				measurement_done[i][2]?'X':'-', '3'+3*i);
			printf("-----------------------------------------------------\n");

		}

		printf("\n");

		int all_done = 1;
		for(int i=0; i<3; i++) 
			for(int o=0; o<3; o++) 
				if(!measurement_done[i][o]) { all_done = 0; break; }


		if(all_done)
		{
			for(int n = 0; n < num_nodes; n++)
				calculate_calibration(n);
			printf("Calibration data done!\n");
		}

	CALIB_NEW_PROMPT:
		printf("\n");
		printf("   [a - f] Change actual voltages/temperatures\n");
		printf("   [1 - 9] Measure at this point\n");
		printf("       [p] Print calibration data\n");
		printf("       [s] Send calibration data to nodes\n");
		printf("       [Q] Abort and quit calibration\n");
		printf("> "); fflush(stdout);


		char key = getchar();
		if(key == 'Q')
		{
			printf("CALIBRATION ABORTED\n");
			return;
		}
		else if(key >= 'a' && key <= 'c')
		{
			printf("Enter actual voltage %c > ", key); fflush(stdout);
			double val;
			scanf("%lf", &val);
			if(val < 0.1 || val > 5.0)
			{
				printf(" OUT OF RANGE\n");
				goto CALIB_NEW_PROMPT;
			}
			calib_actual_volts[key-'a'] = val;
		}
		else if(key >= 'd' && key <= 'f')
		{
			printf("Enter actual temperature %c > ", key); fflush(stdout);
			double val;
			scanf("%lf", &val);
			if(val < -50.0 || val > 125.0)
			{
				printf(" OUT OF RANGE\n");
				goto CALIB_NEW_PROMPT;
			}
			calib_actual_temps[key-'d'] = val;
		}
		else if(key >= '1' && key <= '9')
		{
			if(get_values(255, 0, 0) || get_values(255, 1, 0))
			{
				printf("Error getting values.\n");
			}
			else
			{
				int t = (key-'1')/3;
				int v = (key-'1') - t*3;
				measurement_done[t][v] = 1;

				for(int n = 0; n < num_nodes; n++)
				{
					cal[n][t][v].v_act = calib_actual_volts[v];
					cal[n][t][v].t_act = calib_actual_temps[t];
					cal[n][t][v].raw[V_MUX] = nodes[n].raw[V_MUX];
					cal[n][t][v].raw[T_MUX] = nodes[n].raw[T_MUX];
				}
			}
		}
		else if(key == 'p')
		{
			printf("Actuals: (%lf, %lf, %lf) C, (%lf, %lf, %lf) V\n\n",
				calib_actual_temps[0], calib_actual_temps[1], calib_actual_temps[2],
				calib_actual_volts[0], calib_actual_volts[1], calib_actual_volts[2]);
			for(int n = 0; n < num_nodes; n++)
			{
				printf("NODE %u RAW VALUES (Traw, Vraw)\n", n);
				for(int t = 0; t < 3; t++)
				{
					for(int v = 0; v < 3; v++)
					{
						printf(" (%04u, %04u),", cal[n][t][v].raw[T_MUX], cal[n][t][v].raw[V_MUX]);
					}
					printf("\n");
				}
				printf("\n");
			}
			printf("\n");

			for(int n = 0; n < num_nodes; n++)
			{
				printf("NODE %u CALIBRATION FACTORS\n", n);
				printf("T_offset = %d, T_gain = %u, T_shift = %u\n", 
					nodes[n].offset[T_MUX], nodes[n].gain[T_MUX], nodes[n].shift[T_MUX]);
				printf("V_offset = %d, V_gain = %u, V_t_coeff = %d, V_shift = %u\n",
					nodes[n].offset[V_MUX], nodes[n].gain[V_MUX], nodes[n].t_coeff[V_MUX],
					nodes[n].shift[V_MUX]);
				printf("\n");
			}
		}
		else if(key == 's')
		{
			printf("Sending calibration to nodes...\n");

			for(int n = 0; n < num_nodes; n++)
			{
				int fail = 0;
				fail += node_write_eeprom(n+1, 0x21, nodes[n].offset[V_MUX]);
				fail += node_write_eeprom(n+1, 0x23, nodes[n].offset[T_MUX]);
				fail += node_write_eeprom(n+1, 0x24, LOBYTE(nodes[n].gain[V_MUX]));
				fail += node_write_eeprom(n+1, 0x25, HIBYTE(nodes[n].gain[V_MUX]));
				fail += node_write_eeprom(n+1, 0x28, LOBYTE(nodes[n].gain[T_MUX]));
				fail += node_write_eeprom(n+1, 0x29, HIBYTE(nodes[n].gain[T_MUX]));
				fail += node_write_eeprom(n+1, 0x2a, nodes[n].t_coeff[V_MUX]);
				fail += node_write_eeprom(n+1, 0x2c, nodes[n].t_coeff[T_MUX]);
				fail += node_write_eeprom(n+1, 0x2d, nodes[n].shift[V_MUX]);
				fail += node_write_eeprom(n+1, 0x2f, nodes[n].shift[T_MUX]);

				if(fail)
				{
					printf("Error sending data (%u failures at node %u)\n", fail, n+1);
				}
			}

			for(int n = 0; n < num_nodes; n++)
			{
				if(node_selfcheck(n+1))
				{
					printf("Node %u self-check failed!\n", n+1);
				}
			}

			printf("\nAll calibration data sent to nodes.\n");
		}

	}

}

void sequence_nodes()
{
	data_t packet;
	packet.a= BROADCAST_ADDR;
	packet.b = CMD_CHANGE_NODE_ID;
	packet.c = 1;

	uart_send_packet(&packet);

	for(int i = 0; i < MAX_NODES*2; i++)
	{
		data_t reply;
		uart_read_packet(&reply);
		if(reply.a == packet.a && reply.b == packet.b)
		{
			break;
		}
		else if(reply.b == REPLY_OP_SUCCESS && reply.c == CMD_CHANGE_NODE_ID)
		{
			if(reply.a != num_nodes+1)
				printf("Warning: Reply count %u disagrees with reported node ID %u\n", num_nodes+1, reply.a);
			nodes[num_nodes].id = reply.a;
			num_nodes++;
		}
		else
		{
			printf("Warning: Ignoring unexpected packet %x %x %x %x\n", reply.a, reply.b, reply.c, reply.d);
		}
	}

	printf("Found %u nodes.\n", num_nodes);


}

#define UP2LO(c) if((c) >= 'A' && (c) <= 'F') (c) = (c) - 'A' + 'a';
#define HEX2NUM(c) (((c)>='a')?((c)-'a'+10):((c)-'0'))
#define HEXVALID(c) ((((c)>='a' && (c)<='f') || ((c)>='0' && (c)<='9')))

void input_packet(data_t* packet)
{
	packet->abcd = 0;
	for(int i = 3; i > 0; --i)
	{
		char buf[500];
		scanf("%s", buf);
		int len = strlen(buf);
		if(len == 8)
		{
			for(int b = 0; b < 8; b++)
			{
				packet->block[i] <<= 1;
				if(buf[b] == '1')
					packet->block[i] |= 1;
				else if(buf[b] != '0')
					goto INPUT_FAIL;
			}
		}
		else if((len == 4 || len == 5) && buf[0] == '0' && buf[1] == 'x')
		{
			UP2LO(buf[2]);
			UP2LO(buf[3]);
			if(!HEXVALID(buf[2]) || !HEXVALID(buf[3]))
				goto INPUT_FAIL;

			packet->block[i] = (HEX2NUM(buf[2]) << 4) | HEX2NUM(buf[3]);
		}
		else if(len == 3 && buf[2] == 'h')
		{
			UP2LO(buf[0]);
			UP2LO(buf[1]);
			if(!HEXVALID(buf[0]) || !HEXVALID(buf[1]))
				goto INPUT_FAIL;
			packet->block[i] = (HEX2NUM(buf[0]) << 4) | HEX2NUM(buf[1]);
		}
		else if(len >= 1 && len <= 3)
		{
			if((buf[0] < '0' || buf[0] > '9')) goto INPUT_FAIL;
			if(len > 1 && (buf[1] < '0' || buf[1] > '9')) goto INPUT_FAIL;
			if(len > 2 && (buf[2] < '0' || buf[2] > '9')) goto INPUT_FAIL;
			packet->block[i] = atoi(buf);
		}
		else
			goto INPUT_FAIL;

		printf(" %02xh MORE>", packet->block[i]);
		fflush(stdout);

		continue;
		INPUT_FAIL:
		printf(" Try again (this byte)>");
		fflush(stdout);
		packet->block[i] = 0;
		i++;
	}
}

#define RELAY_MAX_REPLIES 300

int main(int argc, char** argv)
{
	char buf[5000];

	if(argc < 2)
	{
		printf("Usage: blah blah\n");
		return 1;
	}


	fd = open(argv[1], O_RDWR | O_NOCTTY | O_SYNC);
	if(fd < 0)
	{
		printf("error %d opening %s: %s\n", errno, argv[1], strerror(errno));
		return 1;
	}

	set_interface_attribs(fd, B115200, 0, 1);


	if(argc > 2 && argv[2][0] == 'q') // sequence
	{
		sequence_nodes();
	}
	if(argc > 2 && argv[2][0] == 'r') // Relay
	{
		while(1)
		{
			printf("Enter data (3 bytes) >");
			fflush(stdout);
			data_t packet;
			input_packet(&packet);
			insert_crc(&packet);
			printf(" OK, Sending %02x %02x %02x %02x (%3u %3u %3u)...\n",
				packet.a, packet.b, packet.c, packet.d, packet.a, packet.b, packet.c);

			printf("Press any key to stop waiting for replies.");
			fflush(stdout);
			uart_send_packet(&packet);

			int num_repl = 0;
			data_t replies[RELAY_MAX_REPLIES];

			while(1)
			{
				if(kbhit() || num_repl >= RELAY_MAX_REPLIES)
				{
					break;
				}
				int bytes_avail;
				ioctl(fd, FIONREAD, &bytes_avail);
				if(bytes_avail > 3)
				{
					uart_read_packet(&(replies[num_repl]));
					num_repl++;
				}
			}

			printf("\r                                             \r");
			for(int i = 0; i < num_repl; i++)
			{
				printf("Reply %3u: %02x %02x %02x %02x (%3u %3u %3u)\n",
					i+1, replies[i].a, replies[i].b, replies[i].c, replies[i].d,
					replies[i].a, replies[i].b, replies[i].c);
				parse_message(&(replies[i]));
			}
			__fpurge(stdin);

		}
	}
	else if(argc > 2 && argv[2][0] == 'c')  // Calibration
	{
		sequence_nodes();
		calibration();
	}
	else if(argc > 2 && argv[2][0] == 's') // Show
	{
		int calibrated = 0;
		int raw = 0;
		if(argv[2][1] == 'c')
			calibrated = 1;
		else if(argv[2][1] == 'r')
			raw = 1;
		sequence_nodes();
		while(1)
		{
			get_values(255, 1, calibrated);
			get_values(255, 0, calibrated);

			for(int i = 0; i < num_nodes; i++)
			{
				if(raw)
					printf("(%02u:T=%04u, V=%04u)\n", i+1, nodes[i].raw[T_MUX], nodes[i].raw[V_MUX]);
				else
					printf("(%02u:T=%4.3f, V=%5.4f)\n", i+1, nodes[i].t, nodes[i].v);
			}

			printf("\n");

			sleep(1);
			if(kbhit())
			{
				__fpurge(stdin);
				break;
			}
		}
	}

/*
	while(1)
	{
		sleep(1);
		data_t testi;
		testi.a = 123;
		testi.b = 0b10101010;
		testi.c = 45;
		printf("CRC: %u\n", testi.d);
		uart_send_packet(&testi);


	}
*/
//	int bytes;
//	bytes = read(fd, buf, 1000);
//	buf[bytes] = 0;

	close(fd);

	return 0;
}
