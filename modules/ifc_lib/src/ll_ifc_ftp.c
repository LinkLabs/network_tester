#include <string.h>  // memmove
#include <math.h> // for ceil
#include <stdio.h> // for printf
#include "ll_ifc_ftp.h"
#include "ll_ifc_utils.h"

#define LL_FTP_MSG_PACKET_TYPE_INDEX     (0)
#define LL_FTP_MSG_CRC_INDEX             (1)
#define LL_FTP_MSG_FILE_ID_INDEX         (5)
#define LL_FTP_MSG_FILE_VERSION_INDEX    (9)
#define LL_FTP_MSG_FILE_SIZE_INDEX       (13)
#define LL_FTP_MSG_SEG_NUM_INDEX         (17)
#define LL_FTP_MSG_NUM_SEGS_INDEX        (19)
#define LL_FTP_MSG_PAYLOAD_INDEX         (21)
#define LL_FTP_ACK_ACK_TYPE_INDEX        (5)
#define LL_FTP_ACK_FILE_ID_INDEX         (6)
#define LL_FTP_ACK_FILE_VERSION_INDEX    (10)
#define LL_FTP_NAK_FILE_SEGS_INDEX       (14)

#define LL_FTP_MSG_MIN_SIZE              (17)

#define IS_MY_FILE ((ftp_msg.file_id == f->file_id) && (ftp_msg.file_version == f->file_version))

#define UINT16_FROM_BYTESTREAM(p)      (p[0] | (p[1] << 8))
#define UINT32_FROM_BYTESTREAM(p)      (p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24))

#define UINT32_TO_BYTESTREAM(dst, src) dst[0] = src & 0xFF; \
                                        dst[1] = (src >> 8) & 0xFF; \
                                        dst[2] = (src >> 16) & 0xFF; \
                                        dst[3] = (src >> 24) & 0xFF;

const uint8_t LL_FTP_MAX_NUM_RETRIES = 5;
const uint8_t LL_FTP_RETRY_INTERVAL = 16;
const uint8_t LL_FTP_PORT = 128;

extern uint8_t ll_ul_max_port;

typedef enum ll_ftp_msg_type
{
    TX_INIT,
    TX_CANCEL,
    TX_APPLY,
    TX_SEGMENT,
    ACK_INIT,
    ACK_SEGMENT,
    ACK_APPLY,
    TICK,
    NUM_MSG_TYPES,
} ll_ftp_msg_type_t;

typedef enum ll_ftp_ack_type
{
    ACK_ACK = 0x00,
    ACK_NAK_SEGMENT = 0xFD,
    ACK_NAK = 0xFF,
} ll_ftp_ack_type_t;

typedef struct ll_ftp_msg
{
    uint32_t crc;
    uint32_t file_id;
    uint32_t file_version;
    uint32_t file_size;
    ll_ftp_msg_type_t msg_type;
    uint16_t segment_number;
    uint16_t num_segs;
    uint16_t payload_len;
    uint8_t* payload;
} ll_ftp_msg_t;

// Modifies the max port so we can send the message.
static ll_ftp_return_code_t ll_ftp_send_uplink(ll_ftp_t* f, size_t len)
{
    uint8_t old_max_port = ll_ul_max_port;
    ll_ul_max_port = LL_FTP_PORT;
    ll_ftp_return_code_t ret = f->cb.uplink(f->tx_buf, len, true, LL_FTP_PORT);
    ll_ul_max_port = old_max_port;
    gettime(&f->time_last_msg);
    return ret;
}

// reset retry and timeout counters
static void ll_ftp_reset(ll_ftp_t *f)
{
    gettime(&f->time_last_msg);
    f->retry_count = 0;
}

// Close and reopen the file
static ll_ftp_return_code_t ll_ftp_file_reopen(ll_ftp_t *f)
{
    ll_ftp_return_code_t cb_ret;

    cb_ret = f->cb.close(f->file_id, f->file_version);
    if (LL_FTP_OK == cb_ret)
    {
        cb_ret = f->cb.open(f->file_id, f->file_version, f->file_size);
    }

    return cb_ret;
}

static int32_t ll_ftp_parse_rx_msg(const uint8_t* buf, uint8_t len, ll_ftp_msg_t* ftp_msg)
{
    // NULL pointer signifies TICK message
    if(NULL == buf)
    {
        ftp_msg->msg_type = TICK;
        return LL_FTP_OK;
    }

    // Catch invalid values
    if(LL_FTP_MSG_MIN_SIZE > len)
    {
        return LL_FTP_INVALID_VALUE;
    }
    if(NUM_MSG_TYPES <= buf[LL_FTP_MSG_PACKET_TYPE_INDEX])
    {
        return LL_FTP_OOR;
    }

    // Stuff all messages have
    ftp_msg->msg_type = buf[LL_FTP_MSG_PACKET_TYPE_INDEX];
    ftp_msg->crc = buf[LL_FTP_MSG_CRC_INDEX + 0] << 0;
    ftp_msg->crc = UINT32_FROM_BYTESTREAM( (&buf[LL_FTP_MSG_CRC_INDEX]) );
    ftp_msg->file_id = UINT32_FROM_BYTESTREAM( (&buf[LL_FTP_MSG_FILE_ID_INDEX]) );
    ftp_msg->file_version = UINT32_FROM_BYTESTREAM( (&buf[LL_FTP_MSG_FILE_VERSION_INDEX]) );
    ftp_msg->file_size = UINT32_FROM_BYTESTREAM( (&buf[LL_FTP_MSG_FILE_SIZE_INDEX]) );

    // Stuff only segments have
    if(TX_SEGMENT == ftp_msg->msg_type)
    {
        ftp_msg->segment_number = UINT16_FROM_BYTESTREAM( (&buf[LL_FTP_MSG_SEG_NUM_INDEX]) );
        ftp_msg->num_segs = UINT16_FROM_BYTESTREAM( (&buf[LL_FTP_MSG_NUM_SEGS_INDEX]) );
        ftp_msg->payload = (uint8_t*) &buf[LL_FTP_MSG_PAYLOAD_INDEX];
        ftp_msg->payload_len = len - LL_FTP_MSG_PAYLOAD_INDEX; // implicit payload length
    }

    // check crc
    uint32_t crc = crc32(0, (uint8_t*) &buf[LL_FTP_MSG_FILE_ID_INDEX], len - LL_FTP_MSG_FILE_ID_INDEX);
    int32_t ret = (crc == ftp_msg->crc) ? LL_FTP_OK : LL_FTP_ERROR;

    return ret;
}

// Update segments cache array for successfully received and written segments
static void ll_ftp_seg_num_track(ll_ftp_t* f, uint16_t seg_num)
{
    if(MAX_NUM_SEGMENTS >= seg_num)
    {
        uint16_t base = seg_num >> 5; // divide by 32
        f->rx_segs[base] |= (1 << (0x1F & seg_num)); // bit shift a maximum of 5 bits
    }
}

static bool ll_ftp_transfer_is_complete(ll_ftp_t* f)
{
    bool ret = true;
    uint32_t i;
    uint16_t base_max = f->num_segs >> 5; // divide by 32

    for(i = 0; i <= base_max; i++)
    {
        if(i != base_max)
        {
            if(0xFFFFFFFF != f->rx_segs[i])
            {
                ret = false;
                break;
            }
        }
        else
        {
            uint32_t j;
            uint32_t num_segs_mod = f->num_segs & 0x1F; // mod by 32
            for(j = 0; j < num_segs_mod; j++)
            {
                if(!((f->rx_segs[base_max] >> j) & 0x1))
                {
                    ret = false;
                    break;
                }
            }
        }
    }

    return ret;
}

static uint16_t ll_ftp_get_missing_segs(ll_ftp_t* f, bool fill_buf)
{
    uint16_t num_missing_segs = 0;
    uint32_t i, j;
    uint16_t seg_num;
    uint8_t num_segs_base;
    uint8_t idx;

    uint16_t base_max = f->num_segs >> 5; // divide by 32
    for(i = 0; i <= base_max; i++)
    {
        num_segs_base = (i != base_max) ? 32 : f->num_segs & 0x1F; // mod by 32

        for(j = 0; j < num_segs_base; j++)
        {
            if(!((f->rx_segs[i] >> j) & 0x1))
            {
                if((MAX_NUM_RETRY_SEGS <= num_missing_segs) && fill_buf)
                {
                    // request complete retransmission
                    return 0xFFFF;
                }
                seg_num = (i << 5) + j; // i * 32 + j

                idx = num_missing_segs * 2;
                if(fill_buf)
                {
                    f->tx_buf[LL_FTP_NAK_FILE_SEGS_INDEX + idx] = seg_num & 0x00FF;
                    f->tx_buf[LL_FTP_NAK_FILE_SEGS_INDEX + idx + 1] = (seg_num >> 8) & 0x00FF;
                }
                num_missing_segs++;
            }
        }
    }

    return num_missing_segs;
}

static uint8_t ll_ftp_ack_init_generate(ll_ftp_t* f, bool ack)
{
    f->tx_buf[LL_FTP_MSG_PACKET_TYPE_INDEX] = ACK_INIT;
    if (ack)
    {
        f->tx_buf[LL_FTP_ACK_ACK_TYPE_INDEX] = ACK_ACK;
    }
    else
    {
        f->tx_buf[LL_FTP_ACK_ACK_TYPE_INDEX] = ACK_NAK;
    }
    UINT32_TO_BYTESTREAM((&f->tx_buf[LL_FTP_ACK_FILE_ID_INDEX]), f->file_id);
    UINT32_TO_BYTESTREAM((&f->tx_buf[LL_FTP_ACK_FILE_VERSION_INDEX]), f->file_version);


    uint32_t crc = crc32(0, &f->tx_buf[LL_FTP_ACK_ACK_TYPE_INDEX], BASE_UL_MSG_LEN - LL_FTP_ACK_ACK_TYPE_INDEX);
    UINT32_TO_BYTESTREAM((&f->tx_buf[LL_FTP_MSG_CRC_INDEX]), crc);

    return BASE_UL_MSG_LEN;
}

static uint8_t ll_ftp_ack_segs_complete_generate(ll_ftp_t* f)
{
    f->tx_buf[LL_FTP_MSG_PACKET_TYPE_INDEX] = ACK_SEGMENT;
    f->tx_buf[LL_FTP_ACK_ACK_TYPE_INDEX] = ACK_ACK;
    UINT32_TO_BYTESTREAM((&f->tx_buf[LL_FTP_ACK_FILE_ID_INDEX]), f->file_id);
    UINT32_TO_BYTESTREAM((&f->tx_buf[LL_FTP_ACK_FILE_VERSION_INDEX]), f->file_version);

    uint32_t crc = crc32(0, &f->tx_buf[LL_FTP_ACK_ACK_TYPE_INDEX], BASE_UL_MSG_LEN - LL_FTP_ACK_ACK_TYPE_INDEX);
    UINT32_TO_BYTESTREAM((&f->tx_buf[LL_FTP_MSG_CRC_INDEX]), crc);

    return BASE_UL_MSG_LEN;
}

static uint8_t ll_ftp_ack_segs_request_generate(ll_ftp_t* f)
{
    f->tx_buf[LL_FTP_MSG_PACKET_TYPE_INDEX] = ACK_SEGMENT;
    f->tx_buf[LL_FTP_ACK_ACK_TYPE_INDEX] = ACK_NAK_SEGMENT;
    UINT32_TO_BYTESTREAM((&f->tx_buf[LL_FTP_ACK_FILE_ID_INDEX]), f->file_id);
    UINT32_TO_BYTESTREAM((&f->tx_buf[LL_FTP_ACK_FILE_VERSION_INDEX]), f->file_version);

    // check max length number of retry segments
    uint16_t num_missing_segs = ll_ftp_get_missing_segs(f, true);
    uint8_t return_len = BASE_UL_MSG_LEN;
    if(MAX_NUM_RETRY_SEGS <= num_missing_segs)
    {
        f->tx_buf[LL_FTP_ACK_ACK_TYPE_INDEX] = ACK_NAK;
    }
    else
    {
        return_len += num_missing_segs * sizeof(uint16_t);
    }

    uint32_t crc = crc32(0, &f->tx_buf[LL_FTP_ACK_ACK_TYPE_INDEX], return_len - LL_FTP_ACK_ACK_TYPE_INDEX);
    UINT32_TO_BYTESTREAM((&f->tx_buf[LL_FTP_MSG_CRC_INDEX]), crc);

    return return_len;
}

static uint8_t ll_ftp_ack_apply_generate(ll_ftp_t* f)
{
    f->tx_buf[LL_FTP_MSG_PACKET_TYPE_INDEX] = ACK_APPLY;
    f->tx_buf[LL_FTP_ACK_ACK_TYPE_INDEX] = ACK_ACK;
    UINT32_TO_BYTESTREAM((&f->tx_buf[LL_FTP_ACK_FILE_ID_INDEX]), f->file_id);
    UINT32_TO_BYTESTREAM((&f->tx_buf[LL_FTP_ACK_FILE_VERSION_INDEX]), f->file_version);

    uint32_t crc = crc32(0, &f->tx_buf[LL_FTP_ACK_ACK_TYPE_INDEX], BASE_UL_MSG_LEN - LL_FTP_ACK_ACK_TYPE_INDEX);
    UINT32_TO_BYTESTREAM((&f->tx_buf[LL_FTP_MSG_CRC_INDEX]), crc);

    return BASE_UL_MSG_LEN;
}

static void ll_ftp_new_file_initialize(ll_ftp_t* f, ll_ftp_msg_t* msg)
{
    ll_ftp_reset(f);
    f->file_id = msg->file_id;
    f->file_version = msg->file_version;
    f->file_size = msg->file_size;
    f->num_segs = (uint16_t)ceil((double)f->file_size / (double)MAX_FILE_SEGMENT_BYTES);
    memset(f->rx_segs, 0, sizeof(f->rx_segs[0]) * NUM_RX_SEGS_BITMASK);

    // send ACK_INIT
    uint8_t len = ll_ftp_ack_init_generate(f, true);
    ll_ftp_send_uplink(f, len);
}

static void ll_ftp_state_set(ll_ftp_t* f, ll_ftp_state_t new_state)
{
    f->state = new_state;
}

static int32_t ll_ftp_idle_start(ll_ftp_t* f)
{
    int32_t ret = LL_FTP_NO_ACTION;

    int32_t cb_ret = f->cb.close(f->file_id, f->file_version);
    if(LL_FTP_OK == cb_ret)
    {
        f->cb.config(false);
    }
    ret = (LL_FTP_OK == cb_ret) ? LL_FTP_OK : LL_FTP_ERROR;

    return ret;
}

static int32_t ll_ftp_segment_start(ll_ftp_t* f, ll_ftp_msg_t* msg)
{
    int32_t ret = LL_FTP_NO_ACTION;

    int32_t cb_ret = f->cb.open(msg->file_id, msg->file_version, msg->file_size);
    if (LL_FTP_OK == cb_ret)
    {
        ll_ftp_new_file_initialize(f, msg);
        f->cb.config(true);
        ret = LL_FTP_OK;
    }
    else if (TX_INIT == msg->msg_type || TX_SEGMENT == msg->msg_type)
    {
        // send ACK_INIT
        uint8_t len = ll_ftp_ack_init_generate(f, false);
        ll_ftp_send_uplink(f, len);
    }

    return ret;
}

static int32_t ll_ftp_apply_start(ll_ftp_t* f)
{
    int32_t ret = LL_FTP_ERROR;

    int32_t cb_ret = f->cb.close(f->file_id, f->file_version);
    if(LL_FTP_OK == cb_ret)
    {
        uint8_t len = ll_ftp_ack_segs_complete_generate(f);
        ll_ftp_send_uplink(f, len);
        ret = LL_FTP_OK;
    }

    return ret;
}

static int32_t ll_ftp_verify_file_info(ll_ftp_t* f, uint32_t *file_crc)
{
    int32_t ret = LL_FTP_OK;
    uint8_t tmp_buf[4];
    uint32_t file_size = 0;
    uint32_t file_id = 0;
    uint32_t file_version = 0;
    ll_ftp_return_code_t cb_ret;

    cb_ret = f->cb.read(f->file_id, f->file_version, LL_FTP_HDR_OFFSET_CRC, tmp_buf, sizeof(tmp_buf));
    if (LL_FTP_OK != cb_ret)
    {
        return cb_ret;
    }
    *file_crc = UINT32_FROM_BYTESTREAM(tmp_buf);

    cb_ret = f->cb.read(f->file_id, f->file_version, LL_FTP_HDR_OFFSET_SIZE, tmp_buf, sizeof(tmp_buf));
    if (LL_FTP_OK != cb_ret)
    {
        return cb_ret;
    }
    file_size = UINT32_FROM_BYTESTREAM(tmp_buf);

    cb_ret = f->cb.read(f->file_id, f->file_version, LL_FTP_HDR_OFFSET_ID, tmp_buf, sizeof(tmp_buf));
    if (LL_FTP_OK != cb_ret)
    {
        return cb_ret;
    }
    file_id = UINT32_FROM_BYTESTREAM(tmp_buf);

    cb_ret = f->cb.read(f->file_id, f->file_version, LL_FTP_HDR_OFFSET_VERSION, tmp_buf, sizeof(tmp_buf));
    if (LL_FTP_OK != cb_ret)
    {
        return cb_ret;
    }
    file_version = UINT32_FROM_BYTESTREAM(tmp_buf);

    if (file_size != f->file_size ||
        file_id != f->file_id ||
        file_version != f->file_version)
    {
        ret = LL_FTP_ERROR;
    }
    return ret;
}

static int32_t ll_ftp_compute_file_crc(ll_ftp_t* f, uint32_t *crc_out)
{
    uint8_t tmp_buf[16];
    uint32_t offset;
    uint32_t len = sizeof(tmp_buf);
    uint32_t crc = 0;

    for (offset = LL_FTP_HDR_OFFSET_SIZE; offset < (LL_FTP_HDR_LEN + f->file_size); offset += len)
    {
        ll_ftp_return_code_t cb_ret;

        if ((offset + len) > (LL_FTP_HDR_LEN + f->file_size))
        {
            len = (LL_FTP_HDR_LEN + f->file_size) - offset;
        }

        cb_ret = f->cb.read(f->file_id, f->file_version, offset, tmp_buf, len);
        if (LL_FTP_OK == cb_ret)
        {
            crc = crc32(crc, tmp_buf, len);
        }
    }
    *crc_out = crc;

    return LL_FTP_OK;
}

static int32_t ll_ftp_verify_file(ll_ftp_t* f)
{
    int32_t ret = LL_FTP_OK;
    uint32_t file_crc = 0;

    ret = ll_ftp_verify_file_info(f, &file_crc);

    if (LL_FTP_OK == ret)
    {
        uint32_t crc;
        ret = ll_ftp_compute_file_crc(f, &crc);
        if (LL_FTP_OK == ret && (crc == file_crc))
        {
            ret = LL_FTP_OK;
        }
        else
        {
            ret = LL_FTP_ERROR;
        }
    }

    return ret;
}

static int32_t ll_ftp_write_segment(ll_ftp_t* f, ll_ftp_msg_t* msg)
{
    int32_t ret = LL_FTP_NO_ACTION;

    int32_t offset = msg->segment_number * MAX_FILE_SEGMENT_BYTES;
    int32_t cb_ret = f->cb.write(msg->file_id, msg->file_version, offset, msg->payload, msg->payload_len);

    if(LL_FTP_OK == cb_ret)
    {
        ll_ftp_seg_num_track(f, msg->segment_number);
        ret = LL_FTP_OK;
    }

    return ret;
}

static int32_t ll_ftp_idle_process_msg(ll_ftp_t* f, uint8_t* buf, uint8_t len)
{
    ll_ftp_msg_t ftp_msg;
    int32_t ret = ll_ftp_parse_rx_msg(buf, len, &ftp_msg);
    if(LL_FTP_OK != ret)
    {
        return ret;
    }

    if((TX_INIT != ftp_msg.msg_type) && (TX_SEGMENT != ftp_msg.msg_type))
    {
        return LL_FTP_INVALID_VALUE;
    }

    // State transition to SEGMENT
    ret = ll_ftp_segment_start(f, &ftp_msg);
    if(LL_FTP_OK == ret)
    {
        ll_ftp_state_set(f, SEGMENT);
    }

    return ret;
}

static int32_t ll_ftp_transition_out_of_segment(ll_ftp_t* f, ll_ftp_msg_t* msg, ll_ftp_state_t next_state)
{
    int32_t ret;

    switch(next_state)
    {
        case IDLE:
            {
                ret = ll_ftp_idle_start(f);
                if(LL_FTP_OK == ret)
                {
                    ll_ftp_state_set(f, IDLE);
                    ll_ftp_init(f, &f->cb);
                }
            }
            break;
        case SEGMENT:
            {
                ret = ll_ftp_segment_start(f, msg);
                if(LL_FTP_OK == ret)
                {
                    ll_ftp_state_set(f, SEGMENT);
                }
            }
            break;
        case APPLY:
            {
                ret = ll_ftp_apply_start(f);
                if(LL_FTP_OK == ret)
                {
                    ll_ftp_state_set(f, APPLY);
                }
                else
                {
                    ll_ftp_state_set(f, IDLE);
                }
            }
            break;
        default:
            break;
    }

    return ret;
}

static int32_t ll_ftp_segment_process_msg(ll_ftp_t* f, uint8_t* buf, uint8_t len)
{
    ll_ftp_msg_t ftp_msg;
    int32_t ret = ll_ftp_parse_rx_msg(buf, len, &ftp_msg);
    if(LL_FTP_OK != ret)
    {
        return ret;
    }

    int32_t next_state = -1;

    ret = LL_FTP_NO_ACTION; //default value
    if((TX_INIT == ftp_msg.msg_type) && !(IS_MY_FILE))
    {
        next_state = SEGMENT;
    }
    else if ((TX_CANCEL == ftp_msg.msg_type) && (IS_MY_FILE))
    {
        next_state = IDLE;
    }
    else if (TX_SEGMENT == ftp_msg.msg_type)
    {
        if(!(IS_MY_FILE))
        {
            next_state = SEGMENT; // attempt to restart SEGMENT state
        }
        else
        {
            // reset retry and timeout counters
            ll_ftp_reset(f);

            ret = ll_ftp_write_segment(f, &ftp_msg);
            if(ll_ftp_transfer_is_complete(f))
            {
                if(LL_FTP_OK == ll_ftp_verify_file(f))
                {
                    next_state = APPLY;
                }
                else
                {
                    //keep file transfer, but clear file progress and request all segs
                    memset(f->rx_segs, 0, sizeof(f->rx_segs[0]) * NUM_RX_SEGS_BITMASK);
                    ll_ftp_reset(f);

                    ret = ll_ftp_file_reopen(f);
                    if (LL_FTP_OK != ret)
                    {
                        next_state = IDLE;
                    }
                }
            }
        }
    }
    else if (TX_APPLY == ftp_msg.msg_type)
    {
        // Only able to apply in APPLY state
        // send NAK with retry segs
        uint8_t len = ll_ftp_ack_segs_request_generate(f);
        ll_ftp_send_uplink(f, len);
        ret = LL_FTP_OK;
    }
    else if(TICK == ftp_msg.msg_type)
    {
        struct time time_now;

        // Rely on a periodic 1 Hz tick to request segments if necessary
        gettime(&time_now);
        if(ll_difftime(&f->time_last_msg, &time_now) >= LL_FTP_RETRY_INTERVAL)
        {
            if(f->retry_count < LL_FTP_MAX_NUM_RETRIES)
            {
                uint8_t len = ll_ftp_ack_segs_request_generate(f);
                ll_ftp_send_uplink(f, len);
                f->retry_count++;
            }
            else
            {
                next_state = IDLE;
            }
        }
        ret = LL_FTP_OK;
    }

    // Do state transitions out of SEGMENT
    if(next_state > -1)
    {
        ret = ll_ftp_transition_out_of_segment(f, &ftp_msg, next_state);
    }

    return ret;
}

static int32_t ll_ftp_transition_out_of_apply(ll_ftp_t* f, ll_ftp_msg_t* msg, ll_ftp_state_t next_state, bool is_error)
{
    int32_t ret;

    switch (next_state)
    {
        case IDLE:
            {
                ret = ll_ftp_idle_start(f);
                ll_ftp_init(f, &f->cb);
                if(LL_FTP_OK == ret)
                {
                    ll_ftp_state_set(f, IDLE);
                }
            }
            break;
        case SEGMENT:
            {
                ret = ll_ftp_segment_start(f, msg);
                if(LL_FTP_OK == ret)
                {
                    ll_ftp_state_set(f, SEGMENT);
                }
            }
            break;
        case APPLY:
            {
                ret = LL_FTP_NO_ACTION;
            }
            break;
        default:
            ret = LL_FTP_ERROR;
            break;
    }

    if(is_error)
    {
        ret = LL_FTP_ERROR;
    }

    return ret;
}

static int32_t ll_ftp_apply_process_msg(ll_ftp_t* f, uint8_t* buf, uint8_t len)
{
    ll_ftp_msg_t ftp_msg;
    int32_t ret = ll_ftp_parse_rx_msg(buf, len, &ftp_msg);

    if(LL_FTP_OK != ret)
    {
        return ret;
    }

    int32_t next_state = -1;

    ret = LL_FTP_NO_ACTION; //default value
    if((TX_INIT == ftp_msg.msg_type) && !(IS_MY_FILE))
    {
        next_state = SEGMENT;
    }
    else if ((TX_CANCEL == ftp_msg.msg_type) && (IS_MY_FILE))
    {
        next_state = IDLE;
    }
    else if(TX_SEGMENT == ftp_msg.msg_type)
    {
        if(IS_MY_FILE)
        {
            // reset retry and timeout counters
            ll_ftp_reset(f);
        }
        else
        {
            next_state = SEGMENT;
        }
    }
    else if ((TX_APPLY == ftp_msg.msg_type) && (IS_MY_FILE))
    {
        int32_t cb_ret = f->cb.apply(f->file_id, f->file_version, f->file_size);
        if(LL_FTP_OK == cb_ret)
        {
            uint8_t len = ll_ftp_ack_apply_generate(f);
            ll_ftp_send_uplink(f, len);
            next_state = IDLE;
        }
        else
        {
            // Apply rejected. Forget File.
            ret = LL_FTP_ERROR;
            next_state = IDLE;
        }
    }
    else if(TICK == ftp_msg.msg_type)
    {
        struct time time_now;

        // Rely on a periodic 1 Hz tick to request segments if necessary
        gettime(&time_now);
        if(ll_difftime(&f->time_last_msg, &time_now) >= LL_FTP_RETRY_INTERVAL)
        {
            if(f->retry_count < LL_FTP_MAX_NUM_RETRIES)
            {
                uint8_t len = ll_ftp_ack_segs_complete_generate(f);
                ll_ftp_send_uplink(f, len);
                f->retry_count++;
            }
            else
            {
                next_state = IDLE;
            }
        }
        ret = LL_FTP_OK;
    }

    // Do state transitions out of APPLY
    if(next_state > -1)
    {
        ret = ll_ftp_transition_out_of_apply(f, &ftp_msg, next_state, (LL_FTP_ERROR == ret));
    }

    return ret;
}

int32_t ll_ftp_num_missing_segs_get(ll_ftp_t* f)
{
    if((0 >= f->num_segs) || (MAX_NUM_SEGMENTS < f->num_segs))
    {
        return -1;
    }

    return ll_ftp_get_missing_segs(f, false);
}

int32_t ll_ftp_msg_process(ll_ftp_t* f, uint8_t* buf, uint8_t len)
{
    int32_t ret = 0;
    switch (f->state)
    {
        case IDLE:
            ret = ll_ftp_idle_process_msg(f, buf, len);
            break;
        case SEGMENT:
            ret = ll_ftp_segment_process_msg(f, buf, len);
            break;
        case APPLY:
            ret = ll_ftp_apply_process_msg(f, buf, len);
            break;
        default:
            ret = LL_FTP_ERROR;
            break;
    }

    return ret;
}

int32_t ll_ftp_init(ll_ftp_t* f, ll_ftp_callbacks_t* cb)
{
    if((NULL == cb->open) || (NULL == cb->read) ||
       (NULL == cb->write) || (NULL == cb->close) ||
       (NULL == cb->apply) || (NULL == cb->uplink) ||
       (NULL == cb->config))
    {
        return LL_FTP_INVALID_VALUE;
    }

    ll_ftp_state_set(f, IDLE);
    f->num_segs = 0;
    memset(f->rx_segs, 0, sizeof(f->rx_segs[0]) * NUM_RX_SEGS_BITMASK);
    f->file_id = 0;
    f->file_version = 0;
    f->file_size = 0;
    f->time_last_msg.tv_sec = 0;
    f->time_last_msg.tv_nsec = 0;
    f->retry_count = 0;
    f->is_processing = false;
    memset(f->tx_buf, 0, sizeof(f->tx_buf[0]) * LL_FTP_TX_BUF_SIZE);

    f->cb = *cb;

    return LL_FTP_OK;
}
