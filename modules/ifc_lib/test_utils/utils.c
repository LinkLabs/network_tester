#include "ll_ifc.h"
#include "embc/fifo.h"
#include "embc/log.h"
#include <stdint.h>
#include <string.h> // memset, memcpy
#include "ll_ifc_symphony.h"

#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include "cmockery.h"

#define TRANSPORT_LEN_MAX       256
#define TRANSPORT_INVOKE_MAX    16

struct fifo8_s * rd_fifo;

int32_t ll_ul_max_port = 127;

int32_t ll_config_set(uint32_t net_token, const uint8_t app_token[APP_TOKEN_LEN], enum ll_downlink_mode dl_mode, uint8_t qos)
{
    (void) net_token;
    (void) app_token;
    (void) dl_mode;
    (void) qos;
    return 0;
}

int32_t ll_config_get(uint32_t *net_token, uint8_t app_token[APP_TOKEN_LEN], enum ll_downlink_mode * dl_mode, uint8_t *qos)
{
    (void) net_token;
    (void) app_token;
    *dl_mode = LL_DL_OFF;
    (void) qos;
    return 0;
}

uint32_t log_timestamp_us( void ) {
    return 0;
}

void log_printf(const char * format, ...) {
    (void) format;
}

void dbc_assert(char const *file, unsigned line, const char * msg) {
    (void) file;
    (void) line;
    (void) msg;
}

int32_t sleep_ms(int32_t millis)
{
    (void) millis;
    return 0;
}

struct transport_single_s {
    uint8_t wr_buf[TRANSPORT_LEN_MAX];
    uint16_t wr_len;
    uint8_t rd_buf[TRANSPORT_LEN_MAX];
    uint16_t rd_len;
};

struct transport_expect_s {
    struct transport_single_s buffer[TRANSPORT_INVOKE_MAX];
    int head;
    int tail;

    uint8_t message_num;
    uint8_t msg_buf[TRANSPORT_LEN_MAX];
    uint16_t msg_len;
};

struct transport_expect_s transport_expect_;

uint16_t compute_checksum(uint8_t const * buf, uint16_t len)
{
    uint16_t i;
    uint16_t crc = 0;
    for(i = 0; i < len; i++)
    {
        crc = (crc >> 8) | (crc << 8);
        crc ^= buf[i];
        crc ^= (crc & 0xff) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0xff) << 5;
    }
    return crc;
}

void transport_expect(
        uint8_t opcode,
        uint8_t const * wr_buf, uint16_t wr_len,
        uint8_t ack,
        uint8_t const * rd_buf, uint16_t rd_len)
{
    uint16_t crc;
    struct transport_expect_s * self = &transport_expect_;
    struct transport_single_s * s;
    int next = self->head + 1;
    if (next >= TRANSPORT_INVOKE_MAX)
    {
        next = 0;
    }
    assert_true(next != self->tail);
    assert_true(wr_len <= TRANSPORT_LEN_MAX - 11);
    assert_true(rd_len <= TRANSPORT_LEN_MAX - 8);
    s = &self->buffer[self->head];

    // construct command
    s->wr_len = 11 + wr_len;
    s->wr_buf[0] = 0xff;
    s->wr_buf[1] = 0xff;
    s->wr_buf[2] = 0xff;
    s->wr_buf[3] = 0xff;
    s->wr_buf[4] = 0xc4;
    s->wr_buf[5] = opcode;
    s->wr_buf[6] = self->message_num;
    s->wr_buf[7] = (wr_len >> 8) & 0xff;
    s->wr_buf[8] = (wr_len >> 0) & 0xff;
    memcpy(&s->wr_buf[9], wr_buf, wr_len);
    crc = compute_checksum(s->wr_buf + 4, s->wr_len - 6);
    s->wr_buf[s->wr_len - 2] = (crc >> 8) & 0xff;
    s->wr_buf[s->wr_len - 1] = (crc >> 0) & 0xff;

    // construct response
    s->rd_len = 8 + rd_len;
    s->rd_buf[0] = 0xc4;
    s->rd_buf[1] = opcode;
    s->rd_buf[2] = self->message_num;
    s->rd_buf[3] = ack;
    s->rd_buf[4] = (rd_len >> 8) & 0xff;
    s->rd_buf[5] = (rd_len >> 0) & 0xff;
    memcpy(&s->rd_buf[6], rd_buf, rd_len);
    crc = compute_checksum(s->rd_buf, s->rd_len - 2);
    s->rd_buf[s->rd_len - 2] = (crc >> 8) & 0xff;
    s->rd_buf[s->rd_len - 1] = (crc >> 0) & 0xff;

    self->head = next;
    self->message_num++;
}

int32_t transport_write(uint8_t *buff, uint16_t len)
{
    struct transport_expect_s * self = &transport_expect_;
    struct transport_single_s * s;

    //assert_int_not_equal(self->head, self->tail); // "write when none expected");
    assert_true(len + self->msg_len <= TRANSPORT_LEN_MAX); // "receive buffer overflow");
    memcpy(&self->msg_buf[self->msg_len], buff, len);
    self->msg_len += len;

    s = &self->buffer[self->tail];
    if (self->msg_len >= s->wr_len)
    {
        int next = self->tail + 1;
        if (next >= TRANSPORT_INVOKE_MAX)
        {
            next = 0;
        }
        assert_memory_equal(s->wr_buf, self->msg_buf, s->wr_len);
        memcpy(self->msg_buf, self->msg_buf + s->wr_len, self->msg_len - s->wr_len);
        self->msg_len -= s->wr_len;
        if (s->rd_len) {
            fifo_size_t actual_size;
            assert_int_equal(0, fifo8_write(rd_fifo, s->rd_buf, (fifo_size_t) s->rd_len, &actual_size));
            assert_int_equal(actual_size, s->rd_len);
        }
        self->tail = next;
    }
    return 0;
}

int32_t transport_read(uint8_t *buff, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; ++i) {
        if (fifo8_read_u8(rd_fifo, buff++)) {
            return 1;
        }
    }
    return 0;
}

void ifc_utils_setup(void)
{
    memset(&transport_expect_, 0, sizeof(transport_expect_));
    assert_int_equal(0, fifo8_alloc(&rd_fifo, TRANSPORT_LEN_MAX + TRANSPORT_INVOKE_MAX));
    assert_true(0 != rd_fifo);
}

void ifc_utils_teardown(void)
{
    fifo8_free(rd_fifo);
}
