#include "ll_ifc_ftp.h"
#include "ll_ifc.h"
#include "ftp_data.h"

#include <string.h> // memset, memcpy
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <time.h>
#include <sys/time.h>
#include "cmockery.h"

typedef struct
{
    uint32_t crc;
    uint32_t file_id;
    uint32_t file_version;
    uint32_t file_size;
    uint32_t msg_type;
    uint8_t* payload;
    uint8_t length;
} ftp_msg_t;

typedef struct
{
    struct ll_ftp ftp;
    ll_ftp_callbacks_t cb;
} fixture_ftp_t;


extern uint8_t LL_FTP_MAX_NUM_RETRIES;
extern uint8_t LL_FTP_RETRY_INTERVAL;
extern uint8_t LL_FTP_PORT;

uint8_t* g_truth_buf;
uint8_t g_truth_len;


static ll_ftp_return_code_t ftp_open(uint32_t file_id, uint32_t file_version, uint32_t file_size)
{
    assert_int_equal(FILE_ID, file_id);
    assert_int_equal(FILE_VERSION, file_version);
    assert_int_equal(FILE_SIZE, file_size);
    // return LL_FTP_OK or LL_FTP_NO_ACTION
    return (int32_t)mock();
}

static ll_ftp_return_code_t ftp_read(uint32_t file_id, uint32_t file_version, uint32_t offset, uint8_t* payload, uint16_t len)
{
    (void)file_id;
    (void)file_version;
    (void)offset;

    uint8_t *tmp = mock();
    memcpy(payload, tmp, len);
    // return LL_FTP_OK or LL_FTP_NO_ACTION
    return (int32_t)mock();
}

static ll_ftp_return_code_t ftp_write(uint32_t file_id, uint32_t file_version, uint32_t offset, uint8_t* payload, uint16_t len)
{
    (void)file_id;
    (void)file_version;
    (void)offset;
    (void)payload;
    (void)len;

    // return LL_FTP_OK or LL_FTP_NO_ACTION
    return (int32_t)mock();
}
static ll_ftp_return_code_t ftp_close(uint32_t file_id, uint32_t file_version)
{
    assert_int_equal(FILE_ID, file_id);
    assert_int_equal(FILE_VERSION, file_version);
    // return LL_FTP_OK or LL_FTP_NO_ACTION
    return (int32_t)mock();
}

static ll_ftp_return_code_t ftp_apply(uint32_t file_id, uint32_t file_version, uint32_t file_size)
{
    assert_int_equal(FILE_ID, file_id);
    assert_int_equal(FILE_VERSION, file_version);
    assert_int_equal(FILE_SIZE, file_size);
    // return LL_FTP_OK or LL_FTP_NO_ACTION
    return (int32_t)mock();
}

static ll_ftp_return_code_t ftp_uplink(const uint8_t* buf, uint8_t len, bool acked, uint8_t port)
{
    assert_int_equal(true, acked);
    assert_int_equal(LL_FTP_PORT, port);
    if(NULL != g_truth_buf)
    {
        assert_memory_equal(buf, mock(), len);
    }
    // return LL_FTP_OK or LL_FTP_NO_ACTION
    return (int32_t)mock();
}

static ll_ftp_return_code_t ftp_dl_config(bool downlink_on)
{
    assert_int_equal((int)mock(), downlink_on);
    return LL_FTP_OK;
}

static void set_default_ftp_callbacks(struct ll_ftp* ftp, ll_ftp_callbacks_t* cb)
{
    cb->open = ftp_open;
    cb->read = ftp_read;
    cb->write = ftp_write;
    cb->close = ftp_close;
    cb->apply = ftp_apply;
    cb->uplink = ftp_uplink;
    cb->config = ftp_dl_config;

    ll_ftp_init(ftp, cb);
    assert_true(ftp->cb.open == ftp_open);
    assert_true(ftp->cb.read == ftp_read);
    assert_true(ftp->cb.write == ftp_write);
    assert_true(ftp->cb.close == ftp_close);
    assert_true(ftp->cb.apply == ftp_apply);
    assert_true(ftp->cb.uplink == ftp_uplink);
    assert_true(ftp->cb.config == ftp_dl_config);
}

void setUpFtpInit(void **state)
{
    ll_reset_state();

    *state = malloc(sizeof(fixture_ftp_t));
    fixture_ftp_t* f = *state;
    memset(&f->ftp, 0, sizeof(fixture_ftp_t));

    set_default_ftp_callbacks(&f->ftp, &f->cb);

    g_truth_buf = MSG_ACK_INIT;
    g_truth_len = sizeof(MSG_ACK_INIT);
}

void setUpFtpSegment(void **state)
{
    ll_reset_state();

    *state = malloc(sizeof(fixture_ftp_t));
    fixture_ftp_t* f = *state;
    memset(&f->ftp, 0, sizeof(fixture_ftp_t));
    set_default_ftp_callbacks(&f->ftp, &f->cb);

    f->ftp.state = SEGMENT;
    f->ftp.num_segs = 2;
    f->ftp.file_id = FILE_ID;
    f->ftp.file_version = FILE_VERSION;
    f->ftp.file_size = FILE_SIZE;
    f->ftp.num_segs = NUM_SEGS;

    g_truth_buf = MSG_ACK_SEG;
    g_truth_len = sizeof(MSG_ACK_SEG);
}

void setUpFtpApply(void **state)
{
    ll_reset_state();

    *state = malloc(sizeof(fixture_ftp_t));
    fixture_ftp_t* f = *state;
    memset(&f->ftp, 0, sizeof(fixture_ftp_t));
    set_default_ftp_callbacks(&f->ftp, &f->cb);

    f->ftp.state = APPLY;
    f->ftp.num_segs = 2;
    f->ftp.file_id = FILE_ID;
    f->ftp.file_version = FILE_VERSION;
    f->ftp.file_size = FILE_SIZE;
    f->ftp.num_segs = NUM_SEGS;
    f->ftp.rx_segs[0] = 3;

    g_truth_buf = MSG_ACK_APPLY;
    g_truth_len = sizeof(MSG_ACK_APPLY);
}

void tearDownFtp(void **state)
{
    free(*state);
}

//TODO: full retransmission

void test_ftp_INIT_init_accept(void **state)
{
    fixture_ftp_t* f = *state;
    will_return(ftp_open, LL_FTP_OK);
    will_return(ftp_uplink, MSG_ACK_INIT);
    will_return(ftp_uplink, LL_FTP_OK);
    will_return(ftp_dl_config, true);
    assert_int_equal(IDLE, f->ftp.state);

    assert_int_equal(LL_FTP_OK, ll_ftp_msg_process(&f->ftp, (uint8_t*)MSG_INIT, sizeof(MSG_INIT)));

    assert_int_equal(SEGMENT, f->ftp.state);
}

void test_ftp_INIT_init_reject(void **state)
{
    fixture_ftp_t* f = *state;
    will_return(ftp_open, LL_FTP_NO_ACTION);
    will_return(ftp_uplink, MSG_NACK_INIT_FILE_INFO_ZERO);
    will_return(ftp_uplink, LL_FTP_OK);
    assert_int_equal(IDLE, f->ftp.state);

    assert_int_equal(LL_FTP_NO_ACTION, ll_ftp_msg_process(&f->ftp, (uint8_t*)MSG_INIT, sizeof(MSG_INIT)));

    assert_int_equal(IDLE, f->ftp.state);
}

void test_ftp_INIT_seg_accept(void **state)
{
    fixture_ftp_t* f = *state;
    will_return(ftp_open, LL_FTP_OK);
    will_return(ftp_uplink, MSG_ACK_INIT);
    will_return(ftp_uplink, LL_FTP_OK);
    will_return(ftp_dl_config, true);
    assert_int_equal(IDLE, f->ftp.state);

    assert_int_equal(LL_FTP_OK, ll_ftp_msg_process(&f->ftp, (uint8_t*)MSG_SEG1, sizeof(MSG_SEG1)));

    assert_int_equal(SEGMENT, f->ftp.state);
}

void test_ftp_INIT_seg_reject(void **state)
{
    fixture_ftp_t* f = *state;
    will_return(ftp_open, LL_FTP_NO_ACTION);
    will_return(ftp_uplink, MSG_NACK_INIT_FILE_INFO_ZERO);
    will_return(ftp_uplink, LL_FTP_OK);
    assert_int_equal(IDLE, f->ftp.state);
    assert_int_equal(LL_FTP_NO_ACTION, ll_ftp_msg_process(&f->ftp, (uint8_t*)MSG_SEG1, sizeof(MSG_SEG1)));
    assert_int_equal(IDLE, f->ftp.state);
}

void test_ftp_SEGMENT_cancel_accept(void **state)
{
    fixture_ftp_t* f = *state;
    will_return(ftp_close, LL_FTP_OK);
    will_return(ftp_dl_config, false);
    assert_int_equal(SEGMENT, f->ftp.state);
    assert_int_equal(LL_FTP_OK, ll_ftp_msg_process(&f->ftp, (uint8_t*)MSG_CANCEL, sizeof(MSG_CANCEL)));
    assert_int_equal(IDLE, f->ftp.state);
}

void test_ftp_SEGMENT_cancel_reject(void **state)
{
    fixture_ftp_t* f = *state;
    will_return(ftp_close, LL_FTP_NO_ACTION);
    assert_int_equal(SEGMENT, f->ftp.state);
    assert_int_equal(LL_FTP_ERROR, ll_ftp_msg_process(&f->ftp, (uint8_t*)MSG_CANCEL, sizeof(MSG_CANCEL)));
    assert_int_equal(SEGMENT, f->ftp.state);
}

void test_ftp_SEGMENT_init_accept(void **state)
{
    fixture_ftp_t* f = *state;
    will_return(ftp_open, LL_FTP_OK);
    will_return(ftp_uplink, MSG_ACK_INIT);
    will_return(ftp_uplink, LL_FTP_OK);
    will_return(ftp_dl_config, true);
    assert_int_equal(SEGMENT, f->ftp.state);
    f->ftp.file_id += 1;

    assert_int_equal(LL_FTP_OK, ll_ftp_msg_process(&f->ftp, (uint8_t*)MSG_INIT, sizeof(MSG_INIT)));

    assert_int_equal(SEGMENT, f->ftp.state);
}

void test_ftp_SEGMENT_init_reject(void **state)
{
    fixture_ftp_t* f = *state;
    will_return(ftp_open, LL_FTP_NO_ACTION);
    will_return(ftp_uplink, MSG_NACK_INIT);
    will_return(ftp_uplink, LL_FTP_OK);
    assert_int_equal(SEGMENT, f->ftp.state);
    f->ftp.file_id += 1;

    assert_int_equal(LL_FTP_NO_ACTION, ll_ftp_msg_process(&f->ftp, (uint8_t*)MSG_INIT, sizeof(MSG_INIT)));
    assert_int_equal(SEGMENT, f->ftp.state);
}

void test_ftp_SEGMENT_new_seg_accept_init_accept(void **state)
{
    fixture_ftp_t* f = *state;
    will_return(ftp_open, LL_FTP_OK);
    will_return(ftp_uplink, MSG_ACK_INIT);
    will_return(ftp_uplink, LL_FTP_OK);
    will_return(ftp_dl_config, true);
    assert_int_equal(SEGMENT, f->ftp.state);
    assert_int_equal(0, f->ftp.rx_segs[0]);
    f->ftp.file_id += 1;
    assert_int_equal(FILE_ID+1, f->ftp.file_id);

    assert_int_equal(LL_FTP_OK, ll_ftp_msg_process(&f->ftp, (uint8_t*)MSG_SEG1, sizeof(MSG_SEG1)));

    assert_int_equal(0, f->ftp.rx_segs[0]);
    assert_int_equal(SEGMENT, f->ftp.state);
    assert_int_equal(FILE_ID, f->ftp.file_id);
}

void test_ftp_SEGMENT_new_seg_accept_init_reject(void **state)
{
    fixture_ftp_t* f = *state;
    will_return(ftp_open, LL_FTP_NO_ACTION);
    will_return(ftp_uplink, MSG_NACK_INIT);
    will_return(ftp_uplink, LL_FTP_OK);
    assert_int_equal(SEGMENT, f->ftp.state);
    assert_int_equal(0, f->ftp.rx_segs[0]);
    f->ftp.file_id += 1;
    assert_int_equal(FILE_ID+1, f->ftp.file_id);

    assert_int_equal(LL_FTP_NO_ACTION, ll_ftp_msg_process(&f->ftp, (uint8_t*)MSG_SEG1, sizeof(MSG_SEG1)));

    assert_int_equal(0, f->ftp.rx_segs[0]);
    assert_int_equal(SEGMENT, f->ftp.state);
    assert_int_equal(FILE_ID+1, f->ftp.file_id);
}

void test_ftp_SEGMENT_new_seg_accept_write_fail(void **state)
{
    fixture_ftp_t* f = *state;
    will_return(ftp_write, LL_FTP_ERROR);
    assert_int_equal(SEGMENT, f->ftp.state);
    assert_int_equal(0, f->ftp.rx_segs[0]);

    assert_int_equal(LL_FTP_NO_ACTION, ll_ftp_msg_process(&f->ftp, (uint8_t*)MSG_SEG1, sizeof(MSG_SEG1)));

    assert_int_equal(0, f->ftp.rx_segs[0]);
    assert_int_equal(SEGMENT, f->ftp.state);
}

void test_ftp_SEGMENT_new_seg_accept_write(void **state)
{
    fixture_ftp_t* f = *state;
    will_return(ftp_write, LL_FTP_OK);
    assert_int_equal(SEGMENT, f->ftp.state);
    assert_int_equal(0, f->ftp.rx_segs[0]);

    assert_int_equal(LL_FTP_OK, ll_ftp_msg_process(&f->ftp, (uint8_t*)MSG_SEG1, sizeof(MSG_SEG1)));

    assert_int_equal(1, f->ftp.rx_segs[0]);
    assert_int_equal(SEGMENT, f->ftp.state);
}

void test_ftp_SEGMENT_new_seg_complete_short(void **state)
{
    uint32_t offset;
    uint8_t tmpbuf[16];
    uint32_t file_crc;
    fixture_ftp_t* f = *state;

    will_return(ftp_write, LL_FTP_OK);
    file_crc = 0x90af182a;
    will_return(ftp_read, &file_crc); // file_crc, assumes all 0's, file_size 153
    will_return(ftp_read, LL_FTP_OK);
    will_return(ftp_read, &f->ftp.file_size);
    will_return(ftp_read, LL_FTP_OK);
    will_return(ftp_read, &f->ftp.file_id);
    will_return(ftp_read, LL_FTP_OK);
    will_return(ftp_read, &f->ftp.file_version);
    will_return(ftp_read, LL_FTP_OK);
    for (offset = 0; offset < (12 + f->ftp.file_size); offset += sizeof(tmpbuf))
    {
        memset(tmpbuf, 0, sizeof(tmpbuf));
        will_return(ftp_read, tmpbuf);
        will_return(ftp_read, LL_FTP_OK);
    }
    will_return(ftp_close, LL_FTP_OK);
    will_return(ftp_uplink, MSG_ACK_SEG);
    will_return(ftp_uplink, LL_FTP_OK);
    assert_int_equal(SEGMENT, f->ftp.state);
    f->ftp.rx_segs[0] = 1;

    assert_int_equal(LL_FTP_OK, ll_ftp_msg_process(&f->ftp, (uint8_t*)MSG_SEG2, sizeof(MSG_SEG2)));

    assert_int_equal(3, f->ftp.rx_segs[0]);
    assert_int_equal(APPLY, f->ftp.state);
}

void test_ftp_SEGMENT_new_seg_complete_long(void **state)
{
    uint32_t offset;
    uint8_t tmpbuf[16];
    uint32_t file_crc;
    fixture_ftp_t* f = *state;

    will_return(ftp_write, LL_FTP_OK);
    file_crc = 0x90af182a;
    will_return(ftp_read, &file_crc); // file_crc
    will_return(ftp_read, LL_FTP_OK);
    will_return(ftp_read, &f->ftp.file_size);
    will_return(ftp_read, LL_FTP_OK);
    will_return(ftp_read, &f->ftp.file_id);
    will_return(ftp_read, LL_FTP_OK);
    will_return(ftp_read, &f->ftp.file_version);
    will_return(ftp_read, LL_FTP_OK);
    for (offset = 0; offset < (12 + f->ftp.file_size); offset += sizeof(tmpbuf))
    {
        memset(tmpbuf, 0, sizeof(tmpbuf));
        will_return(ftp_read, tmpbuf);
        will_return(ftp_read, LL_FTP_OK);
    }
    will_return(ftp_close, LL_FTP_OK);
    will_return(ftp_uplink, MSG_ACK_SEG);
    will_return(ftp_uplink, LL_FTP_OK);
    assert_int_equal(SEGMENT, f->ftp.state);
    f->ftp.rx_segs[0] = 0xFFFFFFFD;
    f->ftp.rx_segs[1] = 0x7;
    f->ftp.num_segs = 35;

    assert_int_equal(LL_FTP_OK, ll_ftp_msg_process(&f->ftp, (uint8_t*)MSG_SEG2, sizeof(MSG_SEG2)));

    assert_int_equal(0xFFFFFFFF, f->ftp.rx_segs[0]);
    assert_int_equal(0x7, f->ftp.rx_segs[1]);
    assert_int_equal(APPLY, f->ftp.state);
}

void test_ftp_SEGMENT_new_seg_complete_long_bad_crc(void **state)
{
    uint32_t offset;
    uint8_t tmpbuf[16];
    uint32_t file_crc;
    fixture_ftp_t* f = *state;

    will_return(ftp_write, LL_FTP_OK);
    file_crc = 0x90af182a ^ 0xabababab;
    will_return(ftp_read, &file_crc); // file_crc
    will_return(ftp_read, LL_FTP_OK);
    will_return(ftp_read, &f->ftp.file_size);
    will_return(ftp_read, LL_FTP_OK);
    will_return(ftp_read, &f->ftp.file_id);
    will_return(ftp_read, LL_FTP_OK);
    will_return(ftp_read, &f->ftp.file_version);
    will_return(ftp_read, LL_FTP_OK);
    will_return(ftp_open, LL_FTP_OK);
    for (offset = 0; offset < (12 + f->ftp.file_size); offset += sizeof(tmpbuf))
    {
        memset(tmpbuf, 0, sizeof(tmpbuf));
        will_return(ftp_read, tmpbuf);
        will_return(ftp_read, LL_FTP_OK);
    }
    will_return(ftp_close, LL_FTP_OK);
    assert_int_equal(SEGMENT, f->ftp.state);
    f->ftp.rx_segs[0] = 0xFFFFFFFD;
    f->ftp.rx_segs[1] = 0x7;
    f->ftp.num_segs = 35;

    assert_int_equal(LL_FTP_OK, ll_ftp_msg_process(&f->ftp, (uint8_t*)MSG_SEG2, sizeof(MSG_SEG2)));

    // The transfer should have been restarted and remained in the SEGMENT state
    assert_int_equal(0x0, f->ftp.rx_segs[0]);
    assert_int_equal(0x0, f->ftp.rx_segs[1]);
    assert_int_equal(SEGMENT, f->ftp.state);
}

void test_ftp_SEGMENT_apply_incomplete(void **state)
{
    fixture_ftp_t* f = *state;
    will_return(ftp_uplink, MSG_NAK_SEG);
    will_return(ftp_uplink, LL_FTP_OK);
    assert_int_equal(SEGMENT, f->ftp.state);
    f->ftp.rx_segs[0] = 1;

    assert_int_equal(LL_FTP_OK, ll_ftp_msg_process(&f->ftp, (uint8_t*)MSG_APPLY, sizeof(MSG_APPLY)));

    assert_int_equal(1, f->ftp.rx_segs[0]);
    assert_int_equal(SEGMENT, f->ftp.state);
}

void test_ftp_SEGMENT_timeout_request_segs(void **state)
{
    fixture_ftp_t* f = *state;
    will_return(ftp_write, LL_FTP_OK);
    assert_int_equal(SEGMENT, f->ftp.state);

    assert_int_equal(LL_FTP_OK, ll_ftp_msg_process(&f->ftp, (uint8_t*)MSG_SEG1, sizeof(MSG_SEG1)));

    // call timeout LL_FTP_RETRY_INTERVAL times
    will_return(ftp_uplink, MSG_NAK_SEG);
    will_return(ftp_uplink, LL_FTP_OK);
    for(int i = 0; i < LL_FTP_RETRY_INTERVAL; i++)
    {
        assert_int_equal(LL_FTP_OK, ll_ftp_msg_process(&f->ftp, NULL, 0));
    }

    assert_int_equal(1, f->ftp.retry_count);
    assert_int_equal(1, f->ftp.rx_segs[0]);
    assert_int_equal(SEGMENT, f->ftp.state);
}

void test_ftp_SEGMENT_timeout_request_segs_clear_timeout(void **state)
{
    fixture_ftp_t* f = *state;
    will_return(ftp_write, LL_FTP_OK);
    will_return(ftp_write, LL_FTP_NO_ACTION);
    assert_int_equal(SEGMENT, f->ftp.state);

    assert_int_equal(LL_FTP_OK, ll_ftp_msg_process(&f->ftp, (uint8_t*)MSG_SEG1, sizeof(MSG_SEG1)));

    // call timeout LL_FTP_RETRY_INTERVAL times
    will_return(ftp_uplink, MSG_NAK_SEG);
    will_return(ftp_uplink, LL_FTP_OK);
    for(int i = 0; i < LL_FTP_RETRY_INTERVAL; i++)
    {
        assert_int_equal(LL_FTP_OK, ll_ftp_msg_process(&f->ftp, NULL, 0));
    }

    assert_int_equal(LL_FTP_NO_ACTION, ll_ftp_msg_process(&f->ftp, (uint8_t*)MSG_SEG1, sizeof(MSG_SEG1)));

    assert_int_equal(0, f->ftp.retry_count);
    assert_int_equal(1, f->ftp.rx_segs[0]);
    assert_int_equal(SEGMENT, f->ftp.state);
}

void test_ftp_SEGMENT_timeout_cancel(void **state)
{
    fixture_ftp_t* f = *state;
    will_return(ftp_write, LL_FTP_OK);
    will_return(ftp_close, LL_FTP_OK);
    assert_int_equal(SEGMENT, f->ftp.state);
    will_return(ftp_dl_config, false);

    assert_int_equal(LL_FTP_OK, ll_ftp_msg_process(&f->ftp, (uint8_t*)MSG_SEG1, sizeof(MSG_SEG1)));

    // retry enough times to cancel
    for(int j = 0; j < LL_FTP_MAX_NUM_RETRIES + 1; j++)
    {
        if(j < LL_FTP_MAX_NUM_RETRIES)
        {
            will_return(ftp_uplink, MSG_NAK_SEG);
            will_return(ftp_uplink, LL_FTP_OK);
        }

        // call timeout LL_FTP_RETRY_INTERVAL times
        for(int i = 0; i < LL_FTP_RETRY_INTERVAL; i++)
        {
            assert_int_equal(LL_FTP_OK, ll_ftp_msg_process(&f->ftp, NULL, 0));
        }

        assert_int_equal((j+1) % (LL_FTP_MAX_NUM_RETRIES + 1), f->ftp.retry_count);
        //assert_int_equal((j+1), f->ftp.retry_count);
        if(j < LL_FTP_MAX_NUM_RETRIES)
        {
            assert_int_equal(SEGMENT, f->ftp.state);
        }
        else
        {
            assert_int_equal(IDLE, f->ftp.state);
        }
    }

    assert_int_equal(0, f->ftp.rx_segs[0]);
    assert_int_equal(IDLE, f->ftp.state);
}

void test_ftp_APPLY_init_accept(void **state)
{
    fixture_ftp_t* f = *state;
    will_return(ftp_open, LL_FTP_OK);
    will_return(ftp_uplink, MSG_ACK_INIT);
    will_return(ftp_uplink, LL_FTP_OK);
    will_return(ftp_dl_config, true);
    assert_int_equal(APPLY, f->ftp.state);

    g_truth_buf = MSG_ACK_INIT;
    g_truth_len = sizeof(MSG_ACK_INIT);
    f->ftp.file_id += 1;

    assert_int_equal(LL_FTP_OK, ll_ftp_msg_process(&f->ftp, (uint8_t*)MSG_INIT, sizeof(MSG_INIT)));

    assert_int_equal(SEGMENT, f->ftp.state);
}

void test_ftp_APPLY_init_reject(void **state)
{
    fixture_ftp_t* f = *state;
    will_return(ftp_open, LL_FTP_NO_ACTION);
    will_return(ftp_uplink, MSG_NACK_INIT);
    will_return(ftp_uplink, LL_FTP_OK);
    assert_int_equal(APPLY, f->ftp.state);

    g_truth_buf = MSG_ACK_INIT;
    g_truth_len = sizeof(MSG_ACK_INIT);
    f->ftp.file_id += 1;

    assert_int_equal(LL_FTP_NO_ACTION, ll_ftp_msg_process(&f->ftp, (uint8_t*)MSG_INIT, sizeof(MSG_INIT)));

    assert_int_equal(APPLY, f->ftp.state);
}

void test_ftp_APPLY_apply_accept(void **state)
{
    fixture_ftp_t* f = *state;
    will_return(ftp_apply, LL_FTP_OK);
    will_return(ftp_close, LL_FTP_OK);
    will_return(ftp_uplink, MSG_ACK_APPLY);
    will_return(ftp_uplink, LL_FTP_OK);
    will_return(ftp_dl_config, false);
    assert_int_equal(APPLY, f->ftp.state);

    assert_int_equal(LL_FTP_OK, ll_ftp_msg_process(&f->ftp, (uint8_t*)MSG_APPLY, sizeof(MSG_APPLY)));

    assert_int_equal(IDLE, f->ftp.state);
}

void test_ftp_APPLY_apply_reject(void **state)
{
    fixture_ftp_t* f = *state;
    will_return(ftp_apply, LL_FTP_ERROR);
    will_return(ftp_close, LL_FTP_OK);
    assert_int_equal(APPLY, f->ftp.state);
    will_return(ftp_dl_config, false);

    assert_int_equal(LL_FTP_ERROR, ll_ftp_msg_process(&f->ftp, (uint8_t*)MSG_APPLY, sizeof(MSG_APPLY)));

    assert_int_equal(IDLE, f->ftp.state);
}


//TODO: write tests for apply state an various message types


void test_ftp_num_segments_get_fail(void **state)
{
    (void)state;
    ll_ftp_t f;
    f.rx_segs[0] = 0;

    f.num_segs = -1;
    assert_int_equal(-1, ll_ftp_num_missing_segs_get(&f));

    f.num_segs = MAX_NUM_SEGMENTS + 1;
    assert_int_equal(-1, ll_ftp_num_missing_segs_get(&f));
}

void test_ftp_num_segments_get_success(void **state)
{
    (void)state;
    ll_ftp_t f;
    f.num_segs = MAX_NUM_SEGMENTS;

    memset(f.rx_segs, 0, sizeof(f.rx_segs));
    assert_int_equal(MAX_NUM_SEGMENTS, ll_ftp_num_missing_segs_get(&f));

    memset(f.rx_segs, 0xFF, sizeof(f.rx_segs));
    assert_int_equal(0, ll_ftp_num_missing_segs_get(&f));

    memset(f.rx_segs, 0xFF, sizeof(f.rx_segs));
    f.rx_segs[0] = 0xFFFFFFF8;
    assert_int_equal(3, ll_ftp_num_missing_segs_get(&f));

    memset(f.rx_segs, 0xFF, sizeof(f.rx_segs));
    f.rx_segs[0] = 0xFFFF8FFF;
    assert_int_equal(3, ll_ftp_num_missing_segs_get(&f));

    memset(f.rx_segs, 0xA5, sizeof(f.rx_segs));
    assert_int_equal(MAX_NUM_SEGMENTS / 2, ll_ftp_num_missing_segs_get(&f));
}


