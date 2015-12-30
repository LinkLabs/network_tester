#ifndef BYTE_FIFO_H
#define BYTE_FIFO_H

#include <stdint.h>

#define NUM_FIFO_BYTES           (512)

typedef struct
{
    uint8_t buf[NUM_FIFO_BYTES];
    uint16_t take;
    uint16_t put;
} byte_fifo_t;


void fifo_init(byte_fifo_t* f);
void fifo_push(byte_fifo_t* f, uint8_t* buf, uint16_t len);
uint8_t fifo_get(byte_fifo_t* f, uint8_t* p_byte);


#endif // BYTE_FIFO_H
