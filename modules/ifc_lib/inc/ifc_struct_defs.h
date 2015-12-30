#ifndef __IFC_STRUCT_DEFS_H_
#define __IFC_STRUCT_DEFS_H_

#include <stdint.h>

#define NET_INFO_BUFF_SIZE (30)
#define DL_BAND_CFG_SIZE (3 * 4 + 2)
#define STATS_SIZE (10 * 4)

#ifdef __GNU_C__
    #define PACKED __attribute ((__packed__))
#else
    #ifndef PACKED
        #define PACKED
    #endif
#endif

typedef enum
{
    LLABS_CONNECT_INITIAL = 0,
    LLABS_CONNECT_DISCONNECTED,
    LLABS_CONNECT_CONNECTED,
    LLABS_NUM_CONNECT_STATUSES
} llabs_connect_status_t;

typedef struct PACKED llabs_network_info_t
{
    uint32_t network_id_node;
    uint32_t network_id_gw;
    int8_t gateway_channel;
    uint32_t gateway_frequency;
    uint32_t last_rx_tick;
    int16_t rssi;
    uint8_t snr;
    llabs_connect_status_t connection_status;
    uint8_t is_scanning_gateways;
    uint64_t gateway_id;
} llabs_network_info_t;

// Defines the band-specific frequency parameters (FCC 902-928, etc...)
typedef struct PACKED llabs_dl_band_cfg
{
    uint32_t band_edge_lower;
    uint32_t band_edge_upper;
    uint32_t band_edge_guard;
    uint8_t chan_step_size;
    uint8_t chan_step_offset;
} llabs_dl_band_cfg_t;

typedef struct llabs_stats
{
    uint32_t num_send_calls;            // Number of times SendMessage has been called successfully
    uint32_t num_pkts_transmitted;      // Number of packet transmissions (includes retries)
    uint32_t num_gateway_scans;         // Number of gateway scans
    uint32_t num_collisions;            // Number of CSMA collisions detected
    uint32_t num_ack_successes;         // Number of successful acknowledgments
    uint32_t num_ack_failures;          // Number of failed acknowledgments
    uint32_t num_sync_failures;         // Number of Sync failures
    uint32_t num_canceled_pkts_ack;     // Number of times packet was canceled due to LLABS_ACK_FAIL_RETRIES
    uint32_t num_canceled_pkts_csma;    // Number of times packet was canceled due to LLABS_MAX_CSMA_COLLISIONS
    uint32_t num_rx_errors;             // Number of times we received a Rx error from the back end
} llabs_stats_t;

void ll_net_info_deserialize(const uint8_t buff[NET_INFO_BUFF_SIZE], llabs_network_info_t * net_info);
uint16_t ll_net_info_serialize(const llabs_network_info_t * net_info, uint8_t buff[NET_INFO_BUFF_SIZE]);
void ll_dl_band_cfg_deserialize(const uint8_t buff[DL_BAND_CFG_SIZE], llabs_dl_band_cfg_t * dl_cfg);
uint16_t ll_dl_band_cfg_serialize(const llabs_dl_band_cfg_t * dl_cfg, uint8_t buff[DL_BAND_CFG_SIZE]);
void ll_stats_deserialize(const uint8_t buff[STATS_SIZE], llabs_stats_t * stats);
uint16_t ll_stats_serialize(const llabs_stats_t * stats, uint8_t buff[STATS_SIZE]);

#endif
