#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "osp.h"
//#include "iodefine.h"

#define UINT16_DECODE_INC(var, p_buf)        var = osp_uint16_decode(p_buf); p_buf+=2;
#define UINT32_DECODE_INC(var, p_buf)        var = osp_uint32_decode(p_buf); p_buf+=4;

#define OSP_START_SEQ0                  (0xA0)
#define OSP_START_SEQ1                  (0xA2)
#define OSP_END_SEQ0                    (0xB0)
#define OSP_END_SEQ1                    (0xB3)
#define OSP_LENGTH_MIN                  (8)
#define OSP_PAYLOAD_LENGTH_MAX          (0x7FFF)
#define OSP_MAX_SUPPORTED_PAYLOAD_LEN   (128)

#define NUM_BYTES_RX_FIFO               (512)
#define NUM_BYTES_TX_FIFO               (128)

/* Input Message Identifiers and sub types*/
#define MID_SW_VER_POLL                 (132)
#define MID_POS_REQ                     (210)

/* Output Message Identifiers and sub types*/
#define MID_NAV_DATA                    (2)
#define MID_SW_VERSION                  (6)
#define MID_CMD_ACK                     (11)
#define MID_CMD_NACK                    (12)
#define MID_OK_TO_SEND                  (18)
#define MID_GEODETIC_NAV                (41)
#define MID_POS_MEAS_RESP               (69)
#define  POS_RESP                       (1)
#define  MEAS_RESP                      (2)
#define  PARTPOS_RESP                   (3)
#define MID_SW_TOOLBOX                  (178)
#define  TRACKER_CONFIG                 (2)
#define  TRACKER_POLL                   (9)
#define  TRACKER_RESP                   (10)

#define TCXO_FREQUENCY                  (26000000)

#define READ_ONLY

static gps_data_t s_gps_data;
static uint8_t s_payload[OSP_MAX_SUPPORTED_PAYLOAD_LEN];    /* payload buffer */
static uint8_t prev_byte = 0x00;

static uint8_t s_num_long_packets = 0;
static uint8_t s_num_bad_packets = 0;
static uint8_t s_num_bad_checksum = 0;
static uint8_t s_ok_to_send = 0;
static uint16_t s_valid = 0;

/**@brief Function for decoding a uint16 value.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value.
 */
static inline uint16_t osp_uint16_decode(const uint8_t * p_encoded_data)
{
        return ( (((uint16_t)((uint8_t *)p_encoded_data)[0]) << 8) |
                 (((uint16_t)((uint8_t *)p_encoded_data)[1])));
}

/**@brief Function for decoding a uint32 value.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value.
 */
static inline uint32_t osp_uint32_decode(const uint8_t * p_encoded_data)
{
    return ( (((uint32_t)((uint8_t *)p_encoded_data)[0]) << 24) |
             (((uint32_t)((uint8_t *)p_encoded_data)[1]) << 16) |
             (((uint32_t)((uint8_t *)p_encoded_data)[2]) << 8)  |
             (((uint32_t)((uint8_t *)p_encoded_data)[3]) << 0 ));
}

static uint16_t osp_checksum(uint8_t *payload, uint16_t length)
{
    uint16_t checksum = 0;
    int i = 0;

    if (payload == NULL || length == 0)
    {
        return 0;
    }

    while (i < length)
    {
        checksum += payload[i];
        checksum &= 0x7FFF;
        i++;
    }
    return checksum;
}

static void print_osp(uint8_t *buf, uint16_t length)
{
    uint16_t payload_len;
    uint16_t checksum;
    int i;

    if (length < OSP_LENGTH_MIN)
    {
        return;
    }

    //printf("Start sequence: 0x%02x %02x\n", buf[0], buf[1]);
    payload_len = (buf[2] << 8 | buf[3]) & 0x7FFF;
    //printf("Payload length: 0x%04x\n", payload_len);

    if (payload_len > (length - 8))
    {
        //printf("invalid packet %d > %d\n", payload_len, (length - 4));
        return;
    }
    //printf("Payload: ");
    for (i = 4; i < (4 + payload_len); i++)
    {
        //printf("%02x ", buf[i]);
    }
    //printf("\n");

    checksum = (buf[i] << 8 | buf[i+1]) & 0x7FFF;
    //printf("Checksum: 0x%04x\n", checksum);
    //printf("Checksum: 0x%04x\n", osp_checksum(&buf[4], payload_len));
    //printf("Stop sequence: 0x%02x %02x\n", buf[i+2], buf[i+3]);
}

#ifndef READ_ONLY
static void write_byte(uint8_t byte)
{
    uint32_t ret;
    ret = app_uart_put(byte);

    /* TODO: timeout */
    while (NRF_SUCCESS != ret)
    {
        /* block until fifo is available */
        ret = app_uart_put(byte);
    }
}

static void osp_send_message(uint8_t *payload, uint16_t payload_length)
{
    int i;
    uint16_t checksum;

    if (payload == NULL || payload_length == 0)
    {
        return;
    }

    if (!s_ok_to_send)
    {
        return;
    }

    write_byte(OSP_START_SEQ0);
    write_byte(OSP_START_SEQ1);
    write_byte(payload_length >> 8);
    write_byte(payload_length & 0xFF);
    for (i = 0; i < payload_length; i++)
    {
        write_byte(payload[i]);
    }
    checksum = osp_checksum(payload, payload_length);
    write_byte(checksum >> 8);
    write_byte(checksum & 0xFF);
    write_byte(OSP_END_SEQ0);
    write_byte(OSP_END_SEQ1);
}

typedef struct {
    uint8_t mid;
    uint8_t mode;
    uint8_t gga_period;
    uint8_t gga_checksum;
    uint8_t gll_period;
    uint8_t gll_checksum;
    uint8_t gsa_period;
    uint8_t gsa_checksum;
    uint8_t gsv_period;
    uint8_t gsv_checksum;
    uint8_t rmc_period;
    uint8_t rmc_checksum;
    uint8_t vtg_period;
    uint8_t vtg_checksum;
    uint8_t mss_period;
    uint8_t mss_checksum;
    uint8_t epe_period;
    uint8_t epe_checksum;
    uint8_t zda_period;
    uint8_t zda_checksum;
    uint8_t unused_0;
    uint8_t unused_1;
    uint16_t bitrate;
} switch_to_nmea_t;

static void switch_to_nmea(uint16_t baudrate)
{
    static switch_to_nmea_t payload = {
        0x81, 0x02,
        0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x05, 0x01, 0x01, 0x01,
        0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01,
        0x0000};
    payload.bitrate = __builtin_bswap16(baudrate);
        /* TODO: handle big-endianness */
    osp_send_message((uint8_t *)&payload, sizeof(payload));
}

typedef struct {
    uint8_t mid;
    uint8_t control;
} sw_version_poll_t;
void osp_sw_version_poll(void)
{
    static sw_version_poll_t payload = {MID_SW_VER_POLL, 0x00};
    osp_send_message((uint8_t *)&payload, sizeof(payload));
}

typedef struct {
    uint8_t mid;
    uint8_t pos_req_id;
    uint8_t num_fixes;
    uint8_t time_btw_fixes;
    uint8_t hori_error_max;
    uint8_t vert_error_max;
    uint8_t resp_time_max;
    uint8_t time_acc_priority;
    uint8_t location_method;
} pos_req_t;
static void osp_position_request(void)
{
    static uint8_t req_cnt = 0;
    static pos_req_t payload =
        {
            MID_POS_REQ,
            0,
            0,
            2, /* seconds */
            0, /* no maximum */
            7, /* no maximum */
            0, /* no time limit */
            0, /* no priority imposed */
            0x01 /* MS based */
        };

    payload.pos_req_id = req_cnt++;
    osp_send_message((uint8_t *)&payload, sizeof(payload));
}

typedef struct __attribute__ ((__packed__)) {
    uint8_t mid;
    uint8_t sid;
    uint32_t clock_freq;
    uint16_t clock_startup_delay;
    uint32_t clock_uncertainty;
    int32_t  clock_drift;
    uint8_t lna;
    uint8_t config_enable;
    uint8_t pin_config[22];
    uint8_t uart_wakeup_preamble;
    uint8_t uart_idle_byte;
    uint32_t uart_baudrate;
    uint8_t uart_flow_ctrl;
    uint16_t i2c_master_address;
    uint16_t i2c_slave_address;
    uint8_t i2c_rate;
    uint8_t i2c_mode;
    uint16_t i2c_max_len;
    uint8_t pwr_ctrl;
    uint8_t pwr_config;
} tracker_config_t;

void osp_tracker_config(void)
{
    static tracker_config_t payload =
        {
            MID_SW_TOOLBOX,
            TRACKER_CONFIG,
            __builtin_bswap32(26000000u),
            __builtin_bswap16(0x03FF), /* default */
            __builtin_bswap32(3000), /* initial uncertainty unknown */
            __builtin_bswap32(120), /* initial drift unknown */
            0, /* LNA - 0 for high gain, 1 for low gain */
            0, /* ignore pin config */
            {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
            0,
            0,
            __builtin_bswap32(115200),
            0,
            __builtin_bswap16(98),
            __builtin_bswap16(96),
            1,
            1,
            __builtin_bswap16(500),
            0,
            2 /* external voltage */
        };

    osp_send_message((uint8_t *)&payload, sizeof(payload));
}

void osp_tracker_poll(void)
{
    static uint8_t payload[2];

    payload[0] = MID_SW_TOOLBOX;
    payload[1] = TRACKER_POLL;
    osp_send_message(payload, sizeof(payload));
}
#endif
typedef struct __attribute__ ((__packed__))  {
    uint8_t mid;
    int32_t x_pos;
    int32_t y_pos;
    int32_t z_pos;
    int16_t x_velocity;
    int16_t y_velocity;
    int16_t z_velocity;
    uint8_t mode_1;
    uint8_t hdop2;
    uint8_t mode_2;
    uint16_t gps_week;
    uint32_t gps_tow;
    uint8_t num_sv;
    uint8_t prn_ch1;
    uint8_t prn_ch2;
    uint8_t prn_ch3;
    uint8_t prn_ch4;
    uint8_t prn_ch5;
    uint8_t prn_ch6;
    uint8_t prn_ch7;
    uint8_t prn_ch8;
    uint8_t prn_ch9;
    uint8_t prn_ch10;
    uint8_t prn_ch11;
    uint8_t prn_ch12;
} nav_data_t;
void osp_parse_nav_data(uint8_t *payload, uint16_t payload_len)
{
    static uint8_t last_sv_num = 0;
    static nav_data_t nav_data = {0};

    if (NULL == payload || payload_len != 41|| payload[0] != MID_NAV_DATA)
    {
        return;
    }
    nav_data.mid = *payload++;
    nav_data.x_pos = *payload++ << 24;
    nav_data.x_pos |= *payload++ << 16;
    nav_data.x_pos |= *payload++ << 8;
    nav_data.x_pos |= *payload++;
    nav_data.y_pos = *payload++ << 24;
    nav_data.y_pos |= *payload++ << 16;
    nav_data.y_pos |= *payload++ << 8;
    nav_data.y_pos |= *payload++;
    nav_data.z_pos = *payload++ << 24;
    nav_data.z_pos |= *payload++ << 16;
    nav_data.z_pos |= *payload++ << 8;
    nav_data.z_pos |= *payload++;

    nav_data.x_velocity = *payload++ << 8;
    nav_data.x_velocity |= *payload++;
    nav_data.y_velocity = *payload++ << 8;
    nav_data.y_velocity |= *payload++;
    nav_data.z_velocity = *payload++ << 8;
    nav_data.z_velocity |= *payload++;
    nav_data.mode_1 = *payload++;
    nav_data.hdop2 = *payload++;
    nav_data.mode_2 = *payload++;
    nav_data.gps_week = *payload++ << 8;
    nav_data.gps_week |= *payload++;
    nav_data.gps_tow = *payload++ << 24;
    nav_data.gps_tow |= *payload++ << 16;
    nav_data.gps_tow |= *payload++ << 8;
    nav_data.gps_tow |= *payload++;
    nav_data.num_sv = *payload++;
    if (nav_data.num_sv > 0)
    {
        last_sv_num = nav_data.num_sv;
    }
}
#ifndef READ_ONLY
void osp_parse_ok_to_send(uint8_t *payload, uint16_t payload_len)
{
    if (NULL == payload || payload_len != 2 || payload[0] != MID_OK_TO_SEND)
    {
        return;
    }

    if (s_ok_to_send == 0 && payload[1] == 1)
    {
        s_ok_to_send = payload[1];
        osp_tracker_poll();
        //osp_sw_version_poll();
        //osp_position_request();
    }
    s_ok_to_send = payload[1];
}
#endif
typedef struct __attribute__ ((__packed__))  {
    uint8_t mid;
    uint16_t nav_valid;
    uint16_t nav_type;
    uint16_t ext_week_num;
    uint32_t tow;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint32_t sat_id_list;
    int32_t latitude;
    int32_t longitude;
    int32_t altitude_ellipsoid;
    int32_t altitude_msl;
    uint8_t map_datum;
    uint16_t sog; /*speed over ground*/
    uint16_t cog; /*course over ground*/
    int16_t magnetic_variation;
    int16_t climb_rate;
    int16_t heading_rate; /* SiRFDRive only */
    uint32_t ehpe; /* estimated horizontal position error */
    uint32_t evpe; /* estimated vertical position error */
    uint32_t ete;  /* estimated time error SiRFDRive only */
    uint16_t ehve; /* estimated horizontal velocity error  SiRFDRive only */
    uint32_t clock_bias;
    uint32_t clock_bias_error; /* SiRFDRive only */
    int32_t clock_drift;
    uint32_t clock_drift_error; /* SiRFDRive only */
    /* TODO - missing fields */
} geodetic_data_t;

void osp_set_fix_valid(uint16_t valid)
{
    s_valid = valid;
}

uint16_t osp_get_fix_valid(void)
{
    return(s_valid);
}


void osp_parse_geodetic(uint8_t *payload, uint16_t payload_len)
{
    static uint8_t last_sv_num = 0;
    static geodetic_data_t geo_data = {0};

    if (NULL == payload || payload_len != 91 || payload[0] != MID_GEODETIC_NAV)
    {
        return;
    }
    geo_data.mid = *payload++;
    UINT16_DECODE_INC(geo_data.nav_valid, payload);
    UINT16_DECODE_INC(geo_data.nav_type, payload);
    UINT16_DECODE_INC(geo_data.ext_week_num, payload);
    UINT32_DECODE_INC(geo_data.tow, payload);
    UINT16_DECODE_INC(geo_data.year, payload);
    geo_data.month = *payload++;
    geo_data.day = *payload++;
    geo_data.hour = *payload++;
    geo_data.minute = *payload++;
    UINT16_DECODE_INC(geo_data.second, payload);
    UINT32_DECODE_INC(geo_data.sat_id_list, payload);
    UINT32_DECODE_INC(geo_data.latitude, payload);
    UINT32_DECODE_INC(geo_data.longitude, payload);
    UINT32_DECODE_INC(geo_data.altitude_ellipsoid, payload);
    UINT32_DECODE_INC(geo_data.altitude_msl, payload);
    geo_data.map_datum = *payload++;
    UINT16_DECODE_INC(geo_data.sog, payload);
    UINT16_DECODE_INC(geo_data.cog, payload);
    UINT16_DECODE_INC(geo_data.magnetic_variation, payload);
    UINT16_DECODE_INC(geo_data.climb_rate, payload);
    UINT16_DECODE_INC(geo_data.heading_rate, payload);
    UINT32_DECODE_INC(geo_data.ehpe, payload);
    UINT32_DECODE_INC(geo_data.evpe, payload);
    UINT32_DECODE_INC(geo_data.ete, payload);
    UINT16_DECODE_INC(geo_data.ehve, payload);
    UINT32_DECODE_INC(geo_data.clock_bias, payload);
    UINT32_DECODE_INC(geo_data.clock_bias_error, payload);
    UINT32_DECODE_INC(geo_data.clock_drift, payload);
    UINT32_DECODE_INC(geo_data.clock_drift_error, payload);

    if (geo_data.sat_id_list > 0)
    {
        last_sv_num = geo_data.sat_id_list;
    }
    s_gps_data.latitude = geo_data.latitude;
    s_gps_data.longitude = geo_data.longitude;
    s_gps_data.altitude = geo_data.altitude_msl / 100;
    s_gps_data.sat_id_list = geo_data.sat_id_list;
    s_gps_data.ehpe = geo_data.ehpe;
    s_gps_data.evpe = geo_data.evpe;
    s_gps_data.cnt++;

    osp_set_fix_valid(geo_data.nav_valid == 0);

}


void osp_parse_meas_resp(uint8_t *payload, uint16_t payload_len)
{
    uint32_t s_num_mes_resp = 0;
    (void)payload;
    (void)payload_len;

    s_num_mes_resp++;
}

void osp_parse_sw_version(uint8_t *payload, uint16_t payload_len)
{
    uint32_t s_num_sw_version = 0;
    (void)payload;
    (void)payload_len;

    s_num_sw_version++;
}

static uint32_t g_last_freq = 0;
void osp_parse_sw_toolbox(uint8_t *payload, uint16_t payload_len)
{
#if 0
    static uint8_t pkt_cnt = 0;
    static uint8_t reset_done = 0;
#endif
    uint8_t mid;
    uint8_t sid;
    uint32_t curr_freq;

    mid = payload[0];
    sid = payload[1];

    if (mid == MID_SW_TOOLBOX && sid == TRACKER_RESP)
    {
        curr_freq = payload[2] << 24 | payload[3] << 16 | payload[4] << 8 | payload[5];
        g_last_freq = curr_freq;
#if 0
        if (curr_freq != TCXO_FREQUENCY)
        {
            payload[1] = TRACKER_CONFIG;
            payload[2] = (TCXO_FREQUENCY >> 24) & 0xFF;
            payload[3] = (TCXO_FREQUENCY >> 16) & 0xFF;
            payload[4] = (TCXO_FREQUENCY >> 8) & 0xFF;
            payload[5] = (TCXO_FREQUENCY) & 0xFF;

            osp_send_message(payload, payload_len - 2);
            if (pkt_cnt % 4 == 0)
            {
                pkt_cnt = 1;
            }
            pkt_cnt++;
            osp_tracker_poll();
            osp_tracker_poll();
        }
        else if (gps_is_clk_reset_done() == 0)
        {
            gps_reset(); //RICKY: this has to happen for clock to be correct
            s_ok_to_send = 0;
            gps_set_clk_reset_done(1);
        }
#endif
    }
}

int osp_parse_cmd_ack(uint8_t *payload, uint16_t payload_len)
{
    uint8_t mid;
    uint8_t sid;
    uint8_t aci_id;
    if (NULL == payload || payload_len > 3 || payload_len < 2 || payload[0] != MID_CMD_ACK)
    {
        return -1;
    }
    mid = *payload++;
    sid = *payload++;
    aci_id = *payload++;

    if (aci_id == MID_SW_TOOLBOX || sid == MID_SW_TOOLBOX)
    {
        //gps_reset();
        return 2;
    }
    return 1;
}

void osp_parse_packet(uint8_t *payload, uint16_t payload_len)
{
    static uint32_t s_num_unknown_packets = 0;
    #ifndef READ_ONLY
    static uint8_t s_sw_version_recvd = 0;
    #endif

    if (NULL == payload || payload_len < 1)
    {
        return;
    }

    switch (payload[0])
    {
        case MID_NAV_DATA:
            osp_parse_nav_data(payload, payload_len);
            break;
        case MID_SW_VERSION:
            osp_parse_sw_version(payload, payload_len);
            //osp_position_request();
            #ifndef READ_ONLY
            s_sw_version_recvd = 1;
            #endif
            break;
        case MID_CMD_ACK:
            osp_parse_cmd_ack(payload, payload_len);
            break;
        case MID_CMD_NACK:
            break;
        case MID_OK_TO_SEND:
            #ifndef READ_ONLY
            osp_parse_ok_to_send(payload, payload_len);
            #endif
            break;
        case MID_GEODETIC_NAV:
            osp_parse_geodetic(payload, payload_len);
            break;
        case MID_POS_MEAS_RESP:
            osp_parse_meas_resp(payload, payload_len);
            break;
        case MID_SW_TOOLBOX:
            osp_parse_sw_toolbox(payload, payload_len);
            break;
        default:
            #ifndef READ_ONLY
            if (s_sw_version_recvd == 0)
            {
                osp_sw_version_poll();
            }
            osp_tracker_poll(); //ask for clock
            #endif
            s_num_unknown_packets++;
            break;
    }
}

void osp_get_latest(gps_data_t *gps_data)
{
    if (NULL == gps_data)
    {
        return;
    }
    memcpy(gps_data, &s_gps_data, sizeof(gps_data_t));
}

//void osp_task(void)
void osp(uint8_t new_byte)
{
    typedef enum
    {
        STATE_START_SEQ,        /* Waiting for start sequence */
        STATE_PAYLOAD_LEN_MSB,  /* Expecting payload length MSB */
        STATE_PAYLOAD_LEN_LSB,  /* Expecting payload length LSB */
        STATE_PAYLOAD,          /* Expecting payload (variable number of bytes) */
        STATE_CHECKSUM_MSB,     /* Expecting checksum MSB */
        STATE_CHECKSUM_LSB,     /* Expecting checksum LSB */
        STATE_END_SEQ_MSB,      /* Waiting for end sequence MSB */
        STATE_END_SEQ_LSB       /* Waiting for end sequence LSB */
    } rx_state_t;

    /* Start out waiting for start sequence */
    static rx_state_t s_rx_state = STATE_START_SEQ;
    static uint16_t s_payload_len;
    static uint16_t s_curr_pos;
    static uint16_t s_checksum;

    /* Rx Handler State Machine */
    switch (s_rx_state)
    {
        case STATE_START_SEQ:
            if (prev_byte == OSP_START_SEQ0 && new_byte == OSP_START_SEQ1)
            {
                s_rx_state = STATE_PAYLOAD_LEN_MSB;
            }
            break;
        case STATE_PAYLOAD_LEN_MSB:
            s_rx_state = STATE_PAYLOAD_LEN_LSB;
            break;
        case STATE_PAYLOAD_LEN_LSB:
            s_payload_len = (prev_byte << 8) | new_byte;
            if (s_payload_len < OSP_PAYLOAD_LENGTH_MAX)
            {
                if (s_payload_len > OSP_MAX_SUPPORTED_PAYLOAD_LEN)
                {
                    s_num_long_packets++;
                }
                s_rx_state = STATE_PAYLOAD;
                s_curr_pos = 0;
            }
            else
            {
                s_rx_state = STATE_START_SEQ;
            }
            break;
        case STATE_PAYLOAD:
            if (s_payload_len <= OSP_MAX_SUPPORTED_PAYLOAD_LEN)
            {
                s_payload[s_curr_pos++] = new_byte;
            }
            else
            {
                s_curr_pos++;
            }
            if (s_curr_pos >= s_payload_len)
            {
                s_rx_state = STATE_CHECKSUM_MSB;
            }
            break;
        case STATE_CHECKSUM_MSB:
            s_rx_state = STATE_CHECKSUM_LSB;
            break;
        case STATE_CHECKSUM_LSB:
            s_rx_state = STATE_END_SEQ_MSB;
            s_checksum = (prev_byte << 8) | new_byte;
            break;
        case STATE_END_SEQ_MSB:
            s_rx_state = STATE_END_SEQ_LSB;
            break;
        case STATE_END_SEQ_LSB:
            if (prev_byte == OSP_END_SEQ0 && new_byte == OSP_END_SEQ1)
            {
                uint16_t checksum;
                checksum = osp_checksum(s_payload, s_payload_len);
                if (checksum == s_checksum)
                {
                    // Successful packet
                    osp_parse_packet(s_payload, s_payload_len);
                }
                else
                {
                    s_num_bad_checksum++;
                    // bad packet
                }
            }
            else
            {
                s_num_bad_packets++;
                // Bad packet
            }
            s_rx_state = STATE_START_SEQ;
            break;
        default:
            s_rx_state = STATE_START_SEQ;
            s_payload[0] = 0;
            s_payload_len = 0;
    }
    prev_byte = new_byte;
}

uint8_t osp_get_num_sats(gps_data_t* data)
{
    uint8_t num_sats = 0;
    uint8_t i;
    for(i=0; i<32; i++)
    {
        num_sats += (data->sat_id_list >> i) & 0x01;
    }

    return num_sats;
}

#if 0
int main(int argc, char *argv[])
{
    uint32_t ret;
    app_uart_comm_params_t osp_uart_params;

    osp_uart_params.baud_rate = UART_BAUDRATE_BAUDRATE_Baud115200;
    osp_uart_params.rx_pin_no = GPIO_GPS_UART_RX;
    osp_uart_params.tx_pin_no = GPIO_GPS_UART_TX;
    osp_uart_params.use_parity = false;
    osp_uart_params.flow_control = APP_UART_FLOW_CONTROL_DISABLED;
    APP_UART_FIFO_INIT(&osp_uart_params, NUM_BYTES_RX_FIFO, NUM_BYTES_TX_FIFO, osp_uart_event_handler, APP_IRQ_PRIORITY_LOW, ret);
    if (NRF_SUCCESS != ret)
    {
        return -1;
    }
    return 0;
}
#endif
