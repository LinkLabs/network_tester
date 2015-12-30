#include <stddef.h>
#include "ifc_struct_defs.h"

uint8_t read_uint8(const uint8_t buff[], size_t * index) 
{
    return buff[(*index)++];
}

uint16_t read_uint16(const uint8_t buff[], size_t * index) 
{
    uint16_t msb = read_uint8(buff, index);
    uint16_t lsb = read_uint8(buff, index);
    return (msb << 8) | lsb;
}

uint32_t read_uint32(const uint8_t buff[], size_t * index) 
{
    uint32_t msb = read_uint16(buff, index);
    uint32_t lsb = read_uint16(buff, index);
    return (msb << 16) | lsb;
}

uint64_t read_uint64(const uint8_t buff[], size_t *index)
{
    uint64_t msb = read_uint32(buff, index);
    uint64_t lsb = read_uint32(buff, index);
    return (msb << 32) | lsb;
}

void write_uint8(uint8_t x, uint8_t **buff)
{
    *((*buff)++) = x;
}

void write_uint16(uint16_t x, uint8_t **buff)
{
    write_uint8(x >> 8, buff);
    write_uint8(x, buff);
}

void write_uint32(uint32_t x, uint8_t **buff) {
    write_uint16(x >> 16, buff);
    write_uint16(x, buff);
}

void write_uint64(uint64_t x, uint8_t **buff) {
    write_uint32(x >> 32, buff);
    write_uint32(x, buff);
}

// Parse a serialized llabs_network_info_t struct.
void ll_net_info_deserialize(const uint8_t buff[NET_INFO_BUFF_SIZE], llabs_network_info_t *net_info)
{
    size_t i = 0;
    net_info->network_id_node = read_uint32(buff, &i);
    net_info->network_id_gw = read_uint32(buff, &i);
    net_info->gateway_channel = read_uint8(buff, &i);
    net_info->gateway_frequency = read_uint32(buff, &i);
    net_info->last_rx_tick = read_uint32(buff, &i);
    net_info->rssi = read_uint16(buff, &i);
    net_info->snr = read_uint8(buff, &i);
    net_info->connection_status = read_uint8(buff, &i);
    net_info->is_scanning_gateways = read_uint8(buff, &i);
    net_info->gateway_id = read_uint64(buff, &i);
}

// Serializes an llabs_network_info_t struct into a buffer to be sent over the host interface.
// Returns the size of the serialized struct in the buffer.
uint16_t ll_net_info_serialize(const llabs_network_info_t *net_info, uint8_t buff[NET_INFO_BUFF_SIZE])
{
    uint8_t * buff_cpy = buff;
    write_uint32(net_info->network_id_node, &buff_cpy);
    write_uint32(net_info->network_id_gw, &buff_cpy);
    write_uint8(net_info->gateway_channel, &buff_cpy);
    write_uint32(net_info->gateway_frequency, &buff_cpy);
    write_uint32(net_info->last_rx_tick, &buff_cpy);
    write_uint16(net_info->rssi, &buff_cpy);
    write_uint8(net_info->snr, &buff_cpy);
    write_uint8(net_info->connection_status, &buff_cpy);
    write_uint8(net_info->is_scanning_gateways, &buff_cpy);
    write_uint64(net_info->gateway_id, &buff_cpy);
    return buff_cpy - buff;
}

void ll_dl_band_cfg_deserialize(const uint8_t buff[DL_BAND_CFG_SIZE], llabs_dl_band_cfg_t *dl_cfg)
{
    size_t i = 0;
    dl_cfg->band_edge_lower = read_uint32(buff, &i);
    dl_cfg->band_edge_upper = read_uint32(buff, &i);
    dl_cfg->band_edge_guard = read_uint32(buff, &i);
    dl_cfg->chan_step_size = read_uint8(buff, &i);
    dl_cfg->chan_step_offset = read_uint8(buff, &i);
}

uint16_t ll_dl_band_cfg_serialize(const llabs_dl_band_cfg_t *dl_cfg, uint8_t buff[DL_BAND_CFG_SIZE])
{
    uint8_t * buff_cpy = buff;
    write_uint32(dl_cfg->band_edge_lower, &buff_cpy);
    write_uint32(dl_cfg->band_edge_upper, &buff_cpy);
    write_uint32(dl_cfg->band_edge_guard, &buff_cpy);
    write_uint8(dl_cfg->chan_step_size, &buff_cpy);
    write_uint8(dl_cfg->chan_step_offset, &buff_cpy);
    return buff_cpy - buff;
}

void ll_stats_deserialize(const uint8_t buff[STATS_SIZE], llabs_stats_t *stats)
{
    size_t i = 0;
    stats->num_send_calls = read_uint32(buff, &i);
    stats->num_pkts_transmitted = read_uint32(buff, &i);
    stats->num_gateway_scans = read_uint32(buff, &i);
    stats->num_collisions = read_uint32(buff, &i);
    stats->num_ack_successes = read_uint32(buff, &i);
    stats->num_ack_failures = read_uint32(buff, &i);
    stats->num_sync_failures = read_uint32(buff, &i);
    stats->num_canceled_pkts_ack = read_uint32(buff, &i);
    stats->num_canceled_pkts_csma = read_uint32(buff, &i);
    stats->num_rx_errors = read_uint32(buff, &i);
}

uint16_t ll_stats_serialize(const llabs_stats_t *stats, uint8_t buff[STATS_SIZE])
{
    uint8_t * buff_cpy = buff;
    write_uint32(stats->num_send_calls, &buff_cpy);
    write_uint32(stats->num_pkts_transmitted, &buff_cpy);
    write_uint32(stats->num_gateway_scans, &buff_cpy);
    write_uint32(stats->num_collisions, &buff_cpy);
    write_uint32(stats->num_ack_successes, &buff_cpy);
    write_uint32(stats->num_ack_failures, &buff_cpy);
    write_uint32(stats->num_sync_failures, &buff_cpy);
    write_uint32(stats->num_canceled_pkts_ack, &buff_cpy);
    write_uint32(stats->num_canceled_pkts_csma, &buff_cpy);
    write_uint32(stats->num_rx_errors, &buff_cpy);
    return buff_cpy - buff;
}
