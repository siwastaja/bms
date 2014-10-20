// EEPROM MAP
// 0x00-0x1f: Non-checksummed area
// 0x00-0x0f: Boot counters, saturated at 255, indexed with MCUSR.
// 0x10: Node SW version number

// 0x20 - 0x3e: checksummed area
// 0x20: Node ID
// 0x21, 0x22: V offset (int16).
// 0x23, 0x24: Ext sens offset (int16)
// 0x25, 0x26: Int Temp offset (int16).
// 0x27, 0x28: V gain (uint16)
// 0x29, 0x2a: Ext sens gain (uint16)
// 0x2b, 0x2c: Int temp gain (uint16)
// 0x2d: V temp coeff (int8)
// 0x2e: Ext sens temp coeff (int8)
// 0x2f: Int temp temp coeff (int8). Should always be 0.
// 0x30: Checksum of all previous (LSbyte of sum of bytes)

// ...0x3e: Reserved for future use within node code


// 0x40...0x7f: For free use by master.

#include <inttypes.h>
#include <stdio.h>

struct __attribute__ ((__packed__)) 
	{int16_t offsets[3]; uint16_t gains[3]; int8_t t_coeffs[3];} calib =
{300,0,0,
9000, 16384, 4096,
0,0,0};

int main(int argc, char** argv)
{
	FILE* eep;
	int i;

	if(argc != 3 && argc != 2)
	{
		printf("Usage: eepromgen <filename> [serial no]\n");
		return 1;
	}

	uint8_t version = 3;
	uint32_t serial;

	if(argc == 3)
		serial = atoi(argv[2]);
	else
	{
		FILE* serialfile;
		serialfile = fopen("cur_serial", "rb");
		if(!serialfile)
		{
			printf("Serial number file cur_serial not found, starting new from 10\n.");
			serial = 10;
		}
		else
		{
			fscanf(serialfile, "%u", &serial);
			fclose(serialfile);
		}

		serial++;

		serialfile = fopen("cur_serial", "wb");
		if(!serialfile)
		{
			printf("Couldn't open cur_serial for incrementing.\n");
		}
		else
		{
			fprintf(serialfile, "%u", serial);
			fclose(serialfile);
		}
	}

	if(serial < 10 || serial > 2000000000)
	{
		printf("Invalid serial!\n");
		serial = 0;
	}

	printf("Using serial %u\n", serial);

	eep = fopen(argv[1], "wb");

	// Boot counters
	for(i = 0; i < 16; i++)
		putc(0, eep);

	putc(version, eep);
	putc( serial&0x000000ff, eep);
	putc((serial&0x0000ff00)>>8, eep);
	putc((serial&0x00ff0000)>>16, eep);
	putc((serial&0xff000000)>>24, eep);

	int checksum = 0; 

	checksum += version;
	checksum += serial&0xff;
	checksum += (serial&0xff00)>>8;
	checksum += (serial&0xff0000)>>16;
	checksum += (serial&0xff000000)>>24;

	putc(checksum&0xff, eep);

	for(i = 0; i < 10; i++)
		putc(255, eep); // Padding until 0x20

	checksum = 0;

	putc(254, eep);
	checksum += 254;

	uint8_t* p_calib = &(calib.offsets[0]);
	for(i = 0; i < 15; i++)
	{
		putc(*p_calib, eep);
		checksum += *p_calib;
		p_calib++;
	}
	putc(checksum&0xff, eep);

	for(i = 0; i < 79; i++)
		putc(255, eep);

	fclose(eep);
	return 0;
}
