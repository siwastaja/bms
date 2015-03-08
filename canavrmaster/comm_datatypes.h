#ifndef __comm_datatypes_h
#define __comm_datatypes_h

#include <inttypes.h>

typedef union
{
	struct
	{
		uint8_t crc;
		uint8_t val;
		uint8_t cmd;
		uint8_t addr;
	};
	uint32_t clump;
	uint8_t bytes[4];
} bms_packet_t;


typedef struct
{
	uint8_t status;
	bms_packet_t data;
} channelfifo_packet_t;

#endif
