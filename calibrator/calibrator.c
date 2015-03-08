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
int psu_fd;
int oven_fd;
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

uint8_t check_crc_error(data_t* data)
{
	uint8_t remainder = CRC_INITIAL_REMAINDER;
	remainder ^= (*data).a;
	remainder = calc_crc_byte(remainder);
	remainder ^= (*data).b;
	remainder = calc_crc_byte(remainder);
	remainder ^= (*data).c;
	remainder = calc_crc_byte(remainder);
	if((*data).d != remainder)
		return 1;
	return 0;
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
	tty.c_iflag &= ~INLCR;
	tty.c_iflag &= ~ICRNL;
	tty.c_lflag = 0;
	tty.c_oflag = 0;
	tty.c_oflag &= ~ONLCR;
	tty.c_oflag &= ~OCRNL;
	tty.c_oflag &= ~ONLRET;
	tty.c_oflag &= ~ONOCR;

	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~(PARENB | PARODD);
	tty.c_cflag |= parity;
	tty.c_cflag |= CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	cfmakeraw(&tty);

	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 1;

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
	return len;
}

void uart_flush()
{
	tcflush(fd, TCIOFLUSH);
}

void uart_send_packet(data_t* packet)
{
	insert_crc(packet);
	uint8_t buf[4] = {packet->a,packet->b,packet->c,packet->d};
	write(fd, buf, 4);
//	printf("Sent %02x %02x %02x %02x\n", buf[0], buf[1], buf[2], buf[3]);
}

#define INTERBYTE_TIMEOUT 3

int uart_read_packet(data_t* packet, int timeout)
{
	uint8_t buf[4];
	int interbyte_timeout = INTERBYTE_TIMEOUT;
	int rcv_cnt = 0;
	while(rcv_cnt < 4)
	{
		usleep(1000);
		rcv_cnt += read(fd, &(buf[rcv_cnt]), 4-rcv_cnt);
		if(timeout)
			timeout--;
		if(timeout == 1)
			return 1;

		interbyte_timeout--;
		if(interbyte_timeout == 0)
			return 2;
	}
	packet->a = buf[0];
	packet->b = buf[1];
	packet->c = buf[2];
	packet->d = buf[3];
//	printf("Rcvd %02x %02x %02x %02x\n", buf[0], buf[1], buf[2], buf[3]);
	if(check_crc_error(packet))
	{
		printf("WARN: CRC error in packet %02x %02x %02x %02x\n", buf[0], buf[1], buf[2], buf[3]);
		return 3;
	}
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
#define CMD_RELOAD_SHADOW 0xb4
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

#define RX_WAIT_TIMEOUT_100MS 15

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
	uart_flush();
	uart_send_packet(&cmd);

	data_t reply;
	int ret;
	if((ret = uart_read_packet(&reply, RX_WAIT_TIMEOUT_100MS)))
	{
		printf("node_selfcheck: uart_read_packet returned %u\n", ret);
		return -3;
	}

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

int node_reload_shadow(int node)
{
	if(node < 1 || node > 254 || node > num_nodes)
	{
		printf("node_reload_shadow: invalid node number\n");
		return -1;
	}

	data_t cmd;
	cmd.a = node;
	cmd.b = CMD_RELOAD_SHADOW;
	cmd.c = 0;
	uart_flush();
	uart_send_packet(&cmd);

	data_t reply;
	int ret;
	if((ret = uart_read_packet(&reply, RX_WAIT_TIMEOUT_100MS)))
	{
		printf("node_reload_shadow: uart_read_packet returned %u\n", ret);
		return -3;
	}

	if(reply.a != node || reply.b != REPLY_OP_SUCCESS || reply.c != CMD_RELOAD_SHADOW)
	{
		printf("node_reload_shadow: Got unexpected data %02x %02x %02x %02x\n", reply.a, reply.b, reply.c, reply.d);
		return -2;
	}

	return 0;
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

	usleep(10000);

	cmd.b = CMD_WRITE_EEPROM;
	cmd.c = data;
	uart_flush();
	uart_send_packet(&cmd);

	data_t reply;
	int ret;
	if((ret = uart_read_packet(&reply, RX_WAIT_TIMEOUT_100MS)))
	{
		printf("node_write_eeprom: uart_read_packet returned %u\n", ret);
		return 4;
	}
	if(reply.a != node || reply.b != REPLY_OP_SUCCESS || reply.c != CMD_WRITE_EEPROM)
	{
		printf("node_write_eeprom: invalid reply: %02x %02x %02x %02x\n", reply.a, reply.b, reply.c, reply.d);
		return 3;
	}
	return 0;

}

void parse_message(data_t* packet)
{
	if(check_crc_error(packet))
		printf("    ERR: CRC8 mismatch\n");
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
} node_t;

node_t nodes[MAX_NODES];

void saturate(int* val, int min, int max)
{
	if(*val < min) *val = min;
	if(*val > max) *val = max;
}


double cels2bms(double celsius)
{
	return (celsius+273.15)*4.0;
}

void calculate_calibration(int n)
{
	printf("Calibration for node %u\n", n+1);
	for(int t = 0; t < 3; t++)
	{
		for(int v = 0; v < 3; v++)
		{
			printf("(T%5.1f V%5.3f -> T%4u V%4u) ", cal[n][t][v].t_act, cal[n][t][v].v_act,
				cal[n][t][v].raw[T_MUX], cal[n][t][v].raw[V_MUX]);
		}
		printf("\n");
	}

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

	double t_coeffs[3];

	for(int i = 0; i < 3; i++)
		t_coeffs[i] = 256.0*((double)(cal[n][0][i].raw[V_MUX] - cal[n][2][i].raw[V_MUX]) /
			(double)(cels2bms(cal[n][0][i].t_act) - cels2bms(cal[n][2][i].t_act)));


//	printf("V pre-shift: gain = %lf %lf %lf  ofs = %lf  t_coeff = %lf\n", gain[0], gain[1], gain[2], ofs, t_coeff);
	for(int i = 0; i < 3; i++)
	{
		gain[i] *= 16384.0*500.0;
	}
	printf("V post-shift: gains = %lf %lf %lf ofs = %lf  t_coeffs = %lf %lf %lf\n", gain[0], gain[1], gain[2], ofs, t_coeffs[0], t_coeffs[1], t_coeffs[2]);

	float t_coeff = (t_coeffs[0]+t_coeffs[1]+t_coeffs[2])/3;

	nodes[n].offset[V_MUX] = (int)((ofs<0.0)?(ofs-0.5):(ofs+0.5));
	nodes[n].gain[V_MUX] = (int)(gain[1]+0.5);
	nodes[n].t_coeff[V_MUX] = (int)((t_coeff<0.0)?(t_coeff-0.5):(t_coeff+0.5));

	saturate(&nodes[n].offset[V_MUX], -4096, 4095);
	saturate(&nodes[n].gain[V_MUX], 0, 65535);
	saturate(&nodes[n].t_coeff[V_MUX], -128, 127);

	printf("V Final: gain=%u  ofs = %d  t_coeff = %d\n", nodes[n].gain[V_MUX], nodes[n].offset[V_MUX], 
		nodes[n].t_coeff[V_MUX]);


	// Do temperatures.
	double t_gain =
		(double)(cels2bms(cal[n][2][1].t_act) - cels2bms(cal[n][0][1].t_act)) /
		(double)(cal[n][2][1].raw[T_MUX] - cal[n][0][1].raw[T_MUX]);

	double t_ofs1 =
		(cels2bms(cal[n][0][1].t_act) * (double)(cal[n][0][1].raw[T_MUX] - cal[n][2][1].raw[T_MUX])) /
		((double)(cels2bms(cal[n][2][1].t_act) - cels2bms(cal[n][0][1].t_act))) + cal[n][0][1].raw[T_MUX];

	double t_ofs2 =
		(cels2bms(cal[n][2][1].t_act) * (double)(cal[n][0][1].raw[T_MUX] - cal[n][2][1].raw[T_MUX])) /
		((double)(cels2bms(cal[n][2][1].t_act) - cels2bms(cal[n][0][1].t_act))) + cal[n][2][1].raw[T_MUX];

	if(fabs(t_ofs1-t_ofs2) > 1.0)
		printf("T warning: |T_OFS1-T_OFS2| > 1.0 (T_OFS1=%lf, T_OFS2=%lf, diff=%lf)\n", t_ofs1, t_ofs2, fabs(t_ofs1-t_ofs2));

	double t_ofs = (t_ofs1+t_ofs2)/2.0;

	printf("T pre-shift: gain = %lf  ofs = %lf\n", t_gain, t_ofs);
	t_gain *= 16384.0;
	printf("T post-shift: gain = %lf  ofs = %lf\n", t_gain, t_ofs);

	nodes[n].offset[T_MUX] = (int)((t_ofs<0.0)?(t_ofs-0.5):(t_ofs+0.5));
	nodes[n].gain[T_MUX] = (int)(t_gain+0.5);
	nodes[n].t_coeff[T_MUX] = 0;

	saturate(&nodes[n].offset[T_MUX], -4096, 4095);
	saturate(&nodes[n].gain[T_MUX], 0, 65535);

	printf("T Final: gain=%u  ofs = %d  t_coeff = %d\n", nodes[n].gain[T_MUX], nodes[n].offset[T_MUX], 
		nodes[n].t_coeff[T_MUX]);

}

int get_values(int id, int get_t, int calibrated, int long_acq)
{
	int num_replies;
	int fail_cnt = 0;

	if(id < 0 || id > 255)
		return 1;

	int expected_replies = (id==255)?(num_nodes+1):1;

	data_t packet;
	packet.a = id;
	packet.b = CMD_MEASURE | CMD_MEASURE_REF_1V1;
	if(long_acq)
		packet.b |= CMD_MEASURE_LONG;
	if(get_t)
		packet.b |= CMD_MEASURE_INT_T;
	else
		packet.b |= CMD_MEASURE_V;

	if(calibrated) packet.b |= CMD_MEASURE_CALIB;

	packet.c = 0;

	if(long_acq)
	{
		uart_flush();
		// First long request (start measuring)
		uart_send_packet(&packet);

		num_replies = 0;
		while(1)
		{
			data_t reply;
			int ret;
			if((ret = uart_read_packet(&reply, RX_WAIT_TIMEOUT_100MS)))
			{
				printf("get_values: uart_read_packet returned %u\n", ret);
				return -1;
			}
			num_replies++;
			if(reply.abcd == packet.abcd)
				break;
			else if((reply.a > num_nodes) || (((reply.b&0b11000000) != 0b01000000) && ((reply.b&0b11000000) != 0)))
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
	}

	// Get the results:
	uart_flush();
	uart_send_packet(&packet);

	num_replies = 0;
	while(1)
	{
		data_t reply;
		int ret;
		if((ret = uart_read_packet(&reply, RX_WAIT_TIMEOUT_100MS)))
		{
			printf("get_values: uart_read_packet returned %u\n", ret);
			return -2;
		}
		num_replies++;
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
					// calibrated or non-calibrated, same formula
					nodes[reply.a-1].t = ((double)val)/4.0-273.15;
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

	usleep(long_acq?500000:50000);

	return fail_cnt;
}

#define GET_VALUES_RETRIES 5

int get_values_retry(int id, int get_t, int calibrated, int long_acq)
{
	int retries = 0;
	while(get_values(id, get_t, calibrated, long_acq))
	{
		if(retries == GET_VALUES_RETRIES)
		{
			printf("ERR: get_values failed too many times.\n");
			return 1;
		}
		retries++;
		printf("WARN: get_values failed, retry %u...\n", retries);
		sleep(1);
	}
	return 0;
}

#define HIBYTE(x) (((x)&0xff00)>>8)
#define LOBYTE(x) ((x)&0xff)

int send_calibration(int n, int v, int t, int copy)
{
	int fail = 0;
	if(v)
	{
		fail += node_write_eeprom(n+1, 0x21, LOBYTE(nodes[n].offset[V_MUX]));
		fail += node_write_eeprom(n+1, 0x22, HIBYTE(nodes[n].offset[V_MUX]));
		fail += node_write_eeprom(n+1, 0x27, LOBYTE(nodes[n].gain[V_MUX]));
		fail += node_write_eeprom(n+1, 0x28, HIBYTE(nodes[n].gain[V_MUX]));

		if(copy)
		{
			fail += node_write_eeprom(n+1, 0x41, LOBYTE(nodes[n].offset[V_MUX]));
			fail += node_write_eeprom(n+1, 0x42, HIBYTE(nodes[n].offset[V_MUX]));
			fail += node_write_eeprom(n+1, 0x47, LOBYTE(nodes[n].gain[V_MUX]));
			fail += node_write_eeprom(n+1, 0x48, HIBYTE(nodes[n].gain[V_MUX]));

		}
	}
	if(t)
	{
		fail += node_write_eeprom(n+1, 0x25, LOBYTE(nodes[n].offset[T_MUX]));
		fail += node_write_eeprom(n+1, 0x26, HIBYTE(nodes[n].offset[T_MUX]));
		fail += node_write_eeprom(n+1, 0x2b, LOBYTE(nodes[n].gain[T_MUX]));
		fail += node_write_eeprom(n+1, 0x2c, HIBYTE(nodes[n].gain[T_MUX]));
		fail += node_write_eeprom(n+1, 0x2f, 0);

		if(copy)
		{
			fail += node_write_eeprom(n+1, 0x45, LOBYTE(nodes[n].offset[T_MUX]));
			fail += node_write_eeprom(n+1, 0x46, HIBYTE(nodes[n].offset[T_MUX]));
			fail += node_write_eeprom(n+1, 0x4b, LOBYTE(nodes[n].gain[T_MUX]));
			fail += node_write_eeprom(n+1, 0x4c, HIBYTE(nodes[n].gain[T_MUX]));
			fail += node_write_eeprom(n+1, 0x4f, 0);
		}
	}
	if(v&&t)
	{
		fail += node_write_eeprom(n+1, 0x2d, nodes[n].t_coeff[V_MUX]);

		if(copy)
		{
			fail += node_write_eeprom(n+1, 0x4d, nodes[n].t_coeff[V_MUX]);
		}
	}
	return fail;
}

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


	CALIB_NEW_PROMPT:
		printf("\n");
		printf("   [a - f] Change actual voltages/temperatures\n");
		printf("   [1 - 9] Measure at this point\n");
		printf("       [p] Print calibration data\n");
		printf("       [s] Send all calibration data to nodes [v] voltage only [t] temp only\n");
		printf("       [o] Open from file [w] write to file\n");
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
			if(get_values_retry(255, 0, 0, 1) || get_values_retry(255, 1, 0, 1))
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

			int all_done = 1;
			for(int i=0; i<3; i++) 
				for(int o=0; o<3; o++) 
					if(!measurement_done[i][o]) { all_done = 0; break; }


			if(all_done)
			{
				for(int n = 0; n < num_nodes; n++)
					calculate_calibration(n);
				printf("All points measured; calibration data done!\n");
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
			//		printf("\n");
				}
				printf("\n");
			}
			printf("\n");

			for(int n = 0; n < num_nodes; n++)
			{
				printf("NODE %u CALIBRATION FACTORS\n", n);
				printf("T_offset = %d, T_gain = %u, ", 
					nodes[n].offset[T_MUX], nodes[n].gain[T_MUX]);
				printf("V_offset = %d, V_gain = %u, V_t_coeff = %d\n",
					nodes[n].offset[V_MUX], nodes[n].gain[V_MUX], nodes[n].t_coeff[V_MUX]);
			}
			printf("\n");
		}
		else if(key == 's' || key == 'v' || key == 't')
		{
			printf("Sending calibration to nodes...\n");

			for(int n = 0; n < num_nodes; n++)
			{
				if(send_calibration(n, (key=='v')||(key=='s'), (key=='t')||(key=='s'), 1))
				{
					printf("Error sending data to node %u\n", n+1);
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
		else if(key == 'w')
		{
			printf("Writing to calib.bin\n");
			FILE* calfile = fopen("calib.bin", "wb");
			putc(num_nodes, calfile);
			printf("sizeof(cal) == %u. sizeof(nodes) == %u\n", sizeof(cal), sizeof(nodes));
			fwrite((void*)cal, 1, sizeof(cal), calfile); 
			fwrite((void*)nodes, 1, sizeof(nodes), calfile); 
			fwrite((void*)measurement_done, 1, sizeof(measurement_done), calfile);
			fwrite((void*)calib_actual_volts, 1, sizeof(calib_actual_volts), calfile);
			fwrite((void*)calib_actual_temps, 1, sizeof(calib_actual_temps), calfile);
			fclose(calfile);
		}
		else if(key == 'o')
		{
			printf("Reading from calib.bin\n");
			FILE* calfile = fopen("calib.bin", "rb");
			num_nodes = getc(calfile);
			fread((void*)cal, 1, sizeof(cal), calfile);
			fread((void*)nodes, 1, sizeof(nodes), calfile);
			fread((void*)measurement_done, 1, sizeof(measurement_done), calfile);
			fread((void*)calib_actual_volts, 1, sizeof(calib_actual_volts), calfile);
			fread((void*)calib_actual_temps, 1, sizeof(calib_actual_temps), calfile);
			fclose(calfile);
			printf("OK. num_nodes = %u\n", num_nodes);
		}

	}

}

int psu_clear_err()
{
	int cnt = 0;
	char buf[1000];
	while(1)
	{
		wr(psu_fd, "SYST:ERR?\r\n");
		buf[0] = 0; buf[1] = 0;
		usleep(100000);
		int kak = read(psu_fd, buf, 1000);
		buf[kak] = 0;
		if(buf[0] == '0' || buf[1] == '0')
			break;

		printf("PSU: err %u: %s\n", cnt, buf);

		cnt++;
		if(cnt > 20)
		{
			printf("PSU: Error clearing errors (>20 errors).\n");
			return 20;
		}
	}
	if(cnt > 0)
		printf("PSU: %u errors cleared\n", cnt);
	return cnt;
}

void update_max(int val, int* max)
{
	if(val > *max)
		*max = val;
}

void update_min(int val, int* min)
{
	if(val < *min)
		*min = val;
}

// -1 = timeout
// 0 = nl not found, buffer full
// others = location of terminating 0 (string length)
int read_until_nl_timeout(int fd, char* buf, int buflen, int timeout_ms)
{
	int loc = 0;
	char* nl_loc = 0;
	int timeout = 0;
	do
	{
		if(loc >= buflen-1)
			break;
		loc += read(fd, &(buf[loc]), buflen-1-loc);
		buf[loc] = 0;
		usleep(1000);
		timeout++;
		if(timeout == timeout_ms)
			break;
	}
	while((nl_loc = strchr(buf, '\n')) == 0);

	if(loc >= buflen-1)
		return 0;

	if(timeout == timeout_ms)
		return -1;

	return (int)(nl_loc - buf);
}

int replace_strchr(char* str, char from, char to)
{
	int n = 0;
	char* p_loc;
	while((p_loc = strchr(str, from)) != 0)
	{
		*p_loc = to;
		n++;
	}
	return n;
}

int sequence_nodes(int start_addr)
{
	data_t packet;
	packet.a= BROADCAST_ADDR;
	packet.b = CMD_CHANGE_NODE_ID;
	packet.c = start_addr;


	num_nodes = 0;
	uart_flush();

	uart_send_packet(&packet);

	for(int i = 0; i < MAX_NODES*2; i++)
	{
		data_t reply;
		int ret;
		if((ret = uart_read_packet(&reply, RX_WAIT_TIMEOUT_100MS)))
		{
			printf("sequence_nodes: uart_read_packet returned %u\n", ret);
			return 1;
		}
		if(reply.a == packet.a && reply.b == packet.b)
		{
			break;
		}
		else if(reply.b == REPLY_OP_SUCCESS && reply.c == CMD_CHANGE_NODE_ID)
		{
			if(reply.a != start_addr)
				printf("Warning: Reply count %u disagrees with reported node ID %u\n", start_addr, reply.a);
			nodes[num_nodes].id = reply.a;
			num_nodes++;
			start_addr++;
		}
		else
		{
			printf("Warning: Ignoring unexpected packet %x %x %x %x\n", reply.a, reply.b, reply.c, reply.d);
		}
	}

	printf("Found %u nodes.\n", num_nodes);
	return 0;

}



void auto_calibration(int load_from_file)
{
	char buf[1000];

	psu_clear_err();

	wr(psu_fd, "OUTP OFF\r\n");
	wr(psu_fd, "SENS:SWE:POIN 4096\r\n");
	wr(psu_fd, "SENS:SWE:TINT 4E-5\r\n");
	sleep(1);
	wr(psu_fd, "CURR 1\r\n");
	wr(psu_fd, "VOLT 3.6\r\n");
	wr(psu_fd, "OUTP ON\r\n");
	sleep(1);
	sequence_nodes(1);

	if(load_from_file)
	{
		printf("Reading from autocal.bin\n");
		FILE* calfile = fopen("autocal.bin", "rb");
		if(!calfile)
		{
			printf("Couldn't open autocal.bin! Aborting.\n");
			return;
		}
		num_nodes = getc(calfile);
		fread((void*)cal, 1, sizeof(cal), calfile);
		fread((void*)nodes, 1, sizeof(nodes), calfile);
		fclose(calfile);
		printf("OK. num_nodes = %u\n", num_nodes);
		goto SKIP_CALIBRATION;
	}


	double voltage_cmds[3] = {2.9, 3.5, 4.1};
	double actual_t = 0.0;
	int temp_cmds[3] = {6, 23, 40};

	for(int t_cnt = 0; t_cnt < 3; t_cnt++)
	{
		printf("Commanding temperature %u.0, starting oven.\n", temp_cmds[t_cnt]);
		sleep(2);
		tcflush(oven_fd, TCIOFLUSH);
		sleep(1);

		sprintf(buf, ";SET%u;", temp_cmds[t_cnt]);
		wr(oven_fd, buf);
		usleep(10000);
		wr(oven_fd, ";ON;");
		int stable_cnt = 0;
		for(int i = 0;; i++)
		{
			int loc = read_until_nl_timeout(oven_fd, buf, 1000, 5000);
			if(loc == -1)
			{
				printf("\nERROR: Temperature controller data fetch timeouted.\n");
			}
			else if(loc == 0)
			{
				printf("\nERROR: Temperature controller buffer full without newline.\n");
			}
			else
			{
				if(i < 3)
				{
//					printf("ignoring a few first rounds...\n");
					printf(".");
					fflush(stdout);
					continue;
				}

				replace_strchr(buf, '\r', ' ');
				replace_strchr(buf, '\n', ' ');
				char* p_tavg  = strstr(buf, "Tavg=");
				char* p_tdiff = strstr(buf, "Tdiff=");
				char* p_tset  = strstr(buf, "Tset=");
				char* p_stable = strstr(buf, "STABLE");

				float tavg, tdiff, tset;

				sscanf(p_tavg, "Tavg=%f", &tavg);
				sscanf(p_tdiff, "Tdiff=%f", &tdiff);
				sscanf(p_tset, "Tset=%f", &tset);

				printf("%s  STA=%u               \r", buf,stable_cnt);
				fflush(stdout);

				if((int)tset != temp_cmds[t_cnt])
				{
					printf("\nFATAL ERROR: Tset differs from temperature command (cmd=%d, Tset=%f)\n",
						temp_cmds[t_cnt], tset);
					// todo: stop & quit
				}

				if((fabs(tavg - tset) < 0.5) && p_stable && tdiff < 0.4)
					stable_cnt++;

				if(stable_cnt > 10)
				{
					printf("\nOven stable, measuring nodes...\n");
					actual_t = tavg;
					break;
				}

			}

		}

		for(int v_cnt = 0; v_cnt < 3; v_cnt++)
		{
			printf("Measuring at %.3f V ", voltage_cmds[v_cnt]);
			fflush(stdout);
			psu_clear_err();
			sleep(1);
			sprintf(buf, "VOLT %f\r\n", voltage_cmds[v_cnt]);
			wr(psu_fd, buf);
			usleep(100000);
			wr(psu_fd, "OUTP ON\r\n");
			sleep(5);
			wr(psu_fd, "MEAS:VOLT?\r\n");
			usleep(100000);
			int loc = read_until_nl_timeout(psu_fd, buf, 1000, 5000);
			if(loc == -1)
			{
				printf("\nERROR: PSU data fetch timeouted.\n");
			}
			else if(loc == 0)
			{
				printf("\nERROR: PSU buffer full without newline.\n");
			}

			double actual_v = atof(buf);
			printf("(actual: %.3f V)\n", actual_v);
			if(actual_v < voltage_cmds[v_cnt]-0.05 || actual_v > voltage_cmds[v_cnt]+0.05)
				printf("WARN: Measured voltage (%f) out of spec.\n", actual_v);

			if(get_values_retry(255, 0, 0, 1) || get_values_retry(255, 1, 0, 1))
			{
				printf("Error getting values from nodes.\n");
			}
			else
			{
				for(int n = 0; n < num_nodes; n++)
				{
					cal[n][t_cnt][v_cnt].v_act = actual_v;
					cal[n][t_cnt][v_cnt].t_act = actual_t;
					cal[n][t_cnt][v_cnt].raw[V_MUX] = nodes[n].raw[V_MUX];
					cal[n][t_cnt][v_cnt].raw[T_MUX] = nodes[n].raw[T_MUX];
				}
			}
		}

		sprintf(buf, "VOLT %f\r\n", voltage_cmds[1]);
		wr(psu_fd, buf);

		printf("Commanding oven off.\n");
		wr(oven_fd, ";OFF;");
		if(t_cnt < 2)
		{
			int wait = 120;
			while(--wait)
			{
				printf("%u...  \r", wait);
				fflush(stdout);
				sleep(1);
			}
		}
	}


	SKIP_CALIBRATION:

	for(int n = 0; n < num_nodes; n++)
	{
		calculate_calibration(n);
	}

	int max_offset[3] = {-999999,-999999,-999999};
	int min_offset[3] = {999999,999999,999999};
	int max_gain[3] = {0,0,0};
	int min_gain[3] = {999999,999999,999999};
	int max_t_coeff[3] = {-999999,-999999,-999999};
	int min_t_coeff[3] = {999999,999999,999999};
	int64_t offset_acc[3] = {0,0,0};
	int64_t gain_acc[3] = {0,0,0};
	int64_t t_coeff_acc[3] = {0,0,0};

	for(int n = 0; n < num_nodes; n++)
	{
		for(int mux = 0; mux < 3; mux++)
		{
			update_max(nodes[n].offset[mux], &max_offset[mux]);
			update_min(nodes[n].offset[mux], &min_offset[mux]);
			update_max(nodes[n].gain[mux], &max_gain[mux]);
			update_min(nodes[n].gain[mux], &min_gain[mux]);
			update_max(nodes[n].t_coeff[mux], &max_t_coeff[mux]);
			update_min(nodes[n].t_coeff[mux], &min_t_coeff[mux]);
			offset_acc[mux] += nodes[n].offset[mux];
			gain_acc[mux] += nodes[n].gain[mux];
			t_coeff_acc[mux] += nodes[n].t_coeff[mux];
		}
	}

	double avg_offset[3], avg_gain[3], avg_t_coeff[3];

	for(int i = 0; i < 3; i++)
	{
		avg_offset[i] = (double)offset_acc[i]/(double)num_nodes;
		avg_gain[i] = (double)gain_acc[i]/(double)num_nodes;
		avg_t_coeff[i] = (double)t_coeff_acc[i]/(double)num_nodes;
	}

	printf("Statistics: \n");
	printf("V offset:  min=%6d   avg=%7.1f   max=%6d\n", min_offset[V_MUX], avg_offset[V_MUX], max_offset[V_MUX]);
	printf("V gain:    min=%6d   avg=%7.1f   max=%6d\n", min_gain[V_MUX], avg_gain[V_MUX], max_gain[V_MUX]);
	printf("V t_coeff: min=%6d   avg=%7.1f   max=%6d\n", min_t_coeff[V_MUX], avg_t_coeff[V_MUX], max_t_coeff[V_MUX]);
	printf("T offset:  min=%6d   avg=%7.1f   max=%6d\n", min_offset[T_MUX], avg_offset[T_MUX], max_offset[T_MUX]);
	printf("T gain:    min=%6d   avg=%7.1f   max=%6d\n", min_gain[T_MUX], avg_gain[T_MUX], max_gain[T_MUX]);

	const int do_check[3] = {1, 0, 1};
	const int offset_max_diff[3]  = {150, 0, 50};
	const int gain_max_diff[3]    = {1000, 0, 1000};
	const int t_coeff_max_diff[3] = {40, 0, 40};

	for(int n = 0; n < num_nodes; n++)
	{
		for(int mux = 0; mux<3; mux++)
		{
			if(!do_check[mux]) continue;

			if(abs(nodes[n].offset[mux] - (int)avg_offset[mux]) > offset_max_diff[mux])
			{
				printf("WARN: node %u: offset[%u]=%d differs too much from average (%d)\n",
					n, mux, nodes[n].offset[mux], (int)avg_offset[mux]);
			}
			if(abs(nodes[n].gain[mux] - (int)avg_gain[mux]) > gain_max_diff[mux])
			{
				printf("WARN: node %u: gain[%u]=%d differs too much from average (%d)\n",
					n, mux, nodes[n].gain[mux], (int)avg_gain[mux]);
			}
			if(abs(nodes[n].t_coeff[mux] - (int)avg_t_coeff[mux]) > t_coeff_max_diff[mux])
			{
				printf("WARN: node %u: t_coeff[%u]=%d differs too much from average (%d)\n",
					n, mux, nodes[n].t_coeff[mux], (int)avg_t_coeff[mux]);
			}
		}
	}

	printf("Saving calibration data to autocal.bin\n");

	FILE* calfile = fopen("autocal.bin", "wb");
	if(!calfile)
	{
		printf("Couldn't open the file for writing. Not saving.\n");
	}
	else
	{
		putc(num_nodes, calfile);
		printf("sizeof(cal) == %u. sizeof(nodes) == %u\n", sizeof(cal), sizeof(nodes));
		fwrite((void*)cal, 1, sizeof(cal), calfile);
		fwrite((void*)nodes, 1, sizeof(nodes), calfile);
		fclose(calfile);
	}

	__fpurge(stdin);
	printf("Send calibration to nodes? [a]ll | [v]oltage | [t]emperature | [q]uit  >");
	AUTOCAL_CMD_TRY_AGAIN:
	fflush(stdout);
	char key = getchar();
	printf("\n");
	if(key == 'a' || key == 'v' || key == 't')
	{
		printf("Sending calibration to nodes...\n");
		for(int n = 0; n < num_nodes; n++)
		{
			if(send_calibration(n, (key=='a')||(key=='v'), (key=='a')||(key=='t'), 1))
			{
				printf("WARN: Error sending to node %u, trying again.\n", n+1);
				sleep(2);
				if(send_calibration(n, (key=='a')||(key=='v'), (key=='a')||(key=='t'), 1))
				{
					printf("Resend failed, giving up.\n");
				}

			}
		}

		printf("Self-checking nodes & commanding calibration shadow memory reload...\n");
		for(int n = 0; n < num_nodes; n++)
		{
			if(node_selfcheck(n+1))
			{
				printf("WARN: Node %u self-check failed!\n", n+1);
			}
			if(node_reload_shadow(n+1))
			{
				printf("WARN: Node %u shadow reload failed!\n", n+1);
			}
		}
	}
	else if(key == 'q')
		return;
	else
	{
		printf(" Try again > ");
		goto AUTOCAL_CMD_TRY_AGAIN;
	}


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
	if(argc < 2)
	{
		printf("Usage: calibrate <bms_serial_device> <q | r | c[a|o] | s[c|r]> [power_supply_device] [oven_device]\n");
		printf("q = seQuence nodes from 1 and quit.\n");
		printf("r = message relay mode. (doesn't sequence nodes.)\n");
		printf("c = enter calibration menu.\n");
		printf("        a = automated calibration: add HP 6632B power supply device file name.\n");
		printf("        o = automated calibration: open data from autocal.bin.\n");
		printf("s = show values.\n");
		printf("        c = use calibration in nodes. When not specified, default rough conversion used.\n");
		printf("        r = show raw values (as received) instead of converted numbers.\n");
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
		int start = 1;
		if(argc > 3)
		{
			start = atoi(argv[3]);
			if(start < 0 || start > 254)
			{
				printf("Invalid start parameter\n");
				start = 1;
			}
		}

		printf("Sequencing from %u.\n", start);
		sequence_nodes(start);
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
			uart_flush();
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
					uart_read_packet(&(replies[num_repl]), 0);
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
			uart_flush();
			__fpurge(stdin);

		}
	}
	else if(argc > 2 && argv[2][0] == 'c')  // Calibration
	{
		if(argv[2][1] == 'a' || argv[2][1] == 'o')
		{
			if(argc > 4)
			{
				psu_fd = open(argv[3], O_RDWR | O_NOCTTY | O_SYNC);
				if(psu_fd < 0)
				{
					printf("error %d opening Power Supply %s: %s\n", errno, argv[3], strerror(errno));
					return 1;
				}

				set_interface_attribs(psu_fd, B9600, 0, 1);

				oven_fd = open(argv[4], O_RDWR | O_NOCTTY | O_SYNC);

				if(oven_fd < 0)
				{
					printf("error %d opening Oven %s: %s\n", errno, argv[4], strerror(errno));
					return 1;
				}
				set_interface_attribs(oven_fd, B115200, 0, 1);

				auto_calibration(argv[2][1]=='o');
				close(psu_fd);
			}
			else
			{
				printf("Calibrate Auto was defined, but power supply and oven device names are missing.\n");
				return 1;
			}
		}
		else
		{
			sequence_nodes(1);
			calibration();
		}
	}
	else if(argc > 2 && argv[2][0] == 's') // Show
	{
		int calibrated = 0;
		int long_acq = 0;
		if(strchr(argv[2], 'c'))
			calibrated = 1;
		if(strchr(argv[2], 'l'))
			long_acq = 1;
		sequence_nodes(1);
		while(1)
		{
			get_values_retry(255, 1, calibrated, long_acq);
			get_values_retry(255, 0, calibrated, long_acq);

			for(int i = 0; i < num_nodes; i++)
			{
					printf("(%02u:T=%4.3f, V=%5.4f)  ", i+1, nodes[i].t, nodes[i].v);
					printf("(DATA:T=%04u, V=%04u)\n", nodes[i].raw[T_MUX], nodes[i].raw[V_MUX]);
			}

			printf("\n");

//			sleep(1);
			usleep(50000);
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
