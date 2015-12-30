#include "byte_fifo.h"
#include "string.h"
#include "bsp.h"

void fifo_init(byte_fifo_t* f)
{
    memset(f->buf, 0, NUM_FIFO_BYTES);
    f->put = 0;
    f->take = 0;
}

/**
 * @brief
 *  Initiate a Tx operation over the Module USART.
 *
 * @details
 *
 * @param[in] uint8_t* buf
 *  A buffer of uint8_t values to send out
 *
 * @param[in] uint16_t  len
 *  How many bytes to send out
 *
 * @return None.
 */
void fifo_push(byte_fifo_t* f, uint8_t* buf, uint16_t len)
{
    // Put queued bytes on the FIFO
    uint16_t i;

    // TODO: Check if the queue is full for some reason and return an error value.

    for (i=0; i<len; i++)
    {
        // Put new byte on the FIFO
        f->buf[f->put] = buf[i];

        // Increment index, checking for rollover
        if (++f->put >= NUM_FIFO_BYTES)
        {
            f->put = 0;
        }
    }

    LL_ASSERT(f->put != f->take);
}

/**
 * @brief
 *  Checks if there are bytes available to transmit..
 *
 * @details
 *
 * @return True if bytes are available on the queue.
 */
static uint8_t fifo_peek(byte_fifo_t* f)
{
    // Are there more bytes left to tx?
    return (f->put != f->take);
}

/**
 * @brief
 *  Check to see if there are any bytes to Tx over module ifc
 *
 * @details
 *
 * @param[out] uint8_t* p_byte
 *  This pointer will be written with the byte to write if return value = 1
 *
 * @return byte_to_tx:
 *   1 = there is a byte that needs to be transmitted over host ifc
 *   0 = no bytes to Tx at this time
 */
uint8_t fifo_get(byte_fifo_t* f, uint8_t* p_byte)
{
    uint8_t i8_ret = 0;
    // Are there more bytes left to tx?
    // TODO: Should we peek here, or will the peek always be used prior to getting the byte?
    if (fifo_peek(f))
    {
        i8_ret = 1;

        *p_byte = f->buf[f->take];

        if (++f->take >= NUM_FIFO_BYTES)
        {
            f->take = 0;
        }
    }

    return i8_ret;
}
