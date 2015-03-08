#ifndef __multichan_manchester_tx_h
#define __multichan_manchester_tx_h

#include <inttypes.h>
#include "comm_datatypes.h"

#define MANCH_OUTPORT PORTA
#define MANCH_OUTPORT_OFFSET 4

#define cbi(x,y) x &= ~(1<<(y))
#define sbi(x,y) x |= (1<<(y))


void manchester_send_1chan(bms_packet_t* data, uint8_t chan);
void manchester_send_4chan(bms_packet_t* datas);

#endif
