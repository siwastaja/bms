#ifndef __manchester_h
#define __manchester_h

#include <inttypes.h>

#ifndef MANCH_OUTPORT
//#error "Must define MANCH_OUTPORT (defaulting to PORTB)"
#define MANCH_OUTPORT PORTB
#endif

#ifndef MANCH_INPORT
//#error "Must define MANCH_INPORT (defaulting to PINB)"
#define MANCH_INPORT PINB
#endif

#ifndef MANCH_INPORT_PULLUP
//#error "Must define MANCH_INPORT_PULLUP (defaulting to PORTB)"
#define MANCH_INPORT_PULLUP PORTB
#endif

#ifndef MANCH_OUTPIN
//#error "Must define MANCH_OUTPIN (defaulting to 1)"
#define MANCH_OUTPIN 1
#endif

#ifndef MANCH_INPIN
//#error "Must define MANCH_INPIN (defaulting to 3)"
#define MANCH_INPIN 3
#endif

#define cbi(x,y) x &= ~(1<<(y))
#define sbi(x,y) x |= (1<<(y))

#define pinstat ((MANCH_INPORT) & (1<<MANCH_INPIN))
#define pinval  (((MANCH_INPORT) & (1<<MANCH_INPIN))>>MANCH_INPIN)

void manchester_wait_data_block();
uint8_t manchester_wait_data_block_timeout(uint32_t cnt); // return 0 if timeout, 1 if rx.

typedef union
{
	struct
	{
		uint8_t d;
		uint8_t c;
		uint8_t b;
		uint8_t a;
	};
	uint32_t abcd;
	uint8_t block[4];
} data_t;


uint8_t manchester_receive(data_t* data);
uint8_t manchester_send(data_t* data);

#endif
