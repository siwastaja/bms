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
				printf("        Interpreted as calibrated voltage: %f V\n", (double)(val*2)/1000.0);
				printf("        Roughly interpreted as uncalibrated voltage: %f V\n", (double)val*9.0/8000.0);
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

typedef struct
{
	int id;
	double v;
	double t;
	int v_raw;
	int t_raw;
	int offset;
	int gain;
	int v_t_coeff;
	int bitshift;
} node_t;

node_t nodes[MAX_NODES];

int get_raw_voltages(int id)
{
	if(id < 0 || id > 255)
		return 1;

	int expected_replies = (id==255)?2:(num_nodes+1);

	data_t packet;
	packet.a = id;
	packet.b = CMD_MEASURE | CMD_MEASURE_V | CMD_MEASURE_REF_1V1 | CMD_MEASURE_LONG;
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
		else if((reply.a > num_nodes) || ((reply.b&0b11110000) != 0b01000000))
			printf("Warning: Ignoring unexpected packet %x %x %x %x\n", reply.a, reply.b, reply.c, reply.d);
	}

	if(num_replies != expected_replies)
	{
		printf("Warning: expected %u messages, got %u\n", expected_replies, num_replies);
	}

	// Wait for at least 300 ms
	sleep(1);

	// Get the results:
	uart_send_packet(&packet);

	while(1)
	{
		data_t reply;
		uart_read_packet(&reply);
		if(reply.abcd == packet.abcd)
			break;
		else if((reply.a > num_nodes) || ((reply.b&0b11110000) != 0b01000000))
			printf("Warning: Ignoring unexpected packet %x %x %x %x\n", reply.a, reply.b, reply.c, reply.d);
		else
		{
			if(reply.a < 1 || reply.a > 254)
			{
				printf("Warning: illegal node id %u\n", reply.a);
			}
			else
			{
				nodes[reply.a-1].v_raw = ((int)(reply.b&REPLY_MEASUREMENT_MSBITS_MSK))<<8 |
					(int)reply.c;
			}
		}
	}

	if(num_replies != expected_replies)
	{
		printf("Warning: expected %u messages, got %u\n", expected_replies, num_replies);
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
		if(reply.abcd == packet.abcd)
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

	if(argc > 2 && argv[2][0] == 'r')
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
