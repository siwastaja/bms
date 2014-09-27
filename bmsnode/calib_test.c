/*
	Calibration test code. Same code as in node.
	Use this to quickly test different values on a PC.
	Example cli: nano calib_test.c; gcc calib_test.c -o calib_test; ./calib_test
*/

#include <inttypes.h>
#include <stdio.h>

int main()
{
	int8_t offset = 0;
	uint16_t gain = 9000;
	int8_t t_coeff = 0;
	uint8_t shift = 6;

	uint16_t t = 283;
	int8_t last_temp_diff = (int16_t)t-273-23;

	uint32_t val_accum = 800;

	printf("raw 10-bit: %u\n", val_accum);
	val_accum *= 2048;

	val_accum >>= 9;

	printf("raw 12-bit: %u\n", val_accum);

	val_accum -= offset;
	printf("offset-corrected: %u\n", val_accum);

	printf("gain: %u\n", gain);

	printf("last_temp_diff: %d\n", last_temp_diff);

	int16_t temp_corr = (int16_t)t_coeff * (int16_t)last_temp_diff;
	printf("temp_corr: %d\n", temp_corr);
	temp_corr >>= 8;
	printf("temp_corr: %d\n", temp_corr);

	gain += temp_corr;
	printf("gain (temp corrected): %u\n", gain);

	val_accum *= gain;

	printf("corrected value: %u\n", val_accum);

	val_accum >>= 8 + shift;

	printf("corrected value: %u\n", val_accum);

	printf("value interpreted as 2mvs: %u\n", val_accum<<1);

	return 0;
}
