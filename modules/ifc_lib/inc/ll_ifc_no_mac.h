#ifndef __LL_IFC_NO_MAC_H
#define __LL_IFC_NO_MAC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief
 *   RSSI Scan set
 *
 * @param[out]
 *   Stats:
 *   All multi byte values are sent over in little-endian mode.
 *   uint32_t u1
 *   uint32_t u2
 *   uint32_t u3
 *   uint32_t u4
 *
 * @return
 *   0 - success, negative otherwise
 */
int32_t ll_rssi_scan_set(uint32_t u1, uint32_t u2, uint32_t u3, uint32_t u4);

/**
 * @brief
 *   TBD
 *
 * @details
 *   TBD
 *
 * @param[out] buf
 *   byte array containing the data payload
 *
 * @param[in] len
 *   size of the output buffer in bytes,
 *
 * @param[out] bytes_received
 *   number of bytes received
 *
 * @return
 *   0 - success, negative otherwise
 */
int32_t ll_rssi_scan_get(uint8_t buf[], uint16_t len, uint8_t *bytes_received);

/**
* @brief
*   Gets the radio parameters of the module
*
* @details
*
* @param[out] uint8_t sf: 6..12
*
* @param[out] uint8_t cr: 1=CR4/5 ... 4=CR4/8
*
* @param[out] uint8_t bw: 0=62.5kHz ... 3=500kHz
*
* @param[out] uint32_t freq: center frequency in Hz
*
* @param[out] uint16_t preamble_syms: length of preamble in number of symbols
*
* @param[out] uint8_t header_enabled: boolean indicating whether LoRa header is enabled
*
* @param[out] uint8_t crc_enabled: boolean indicating whether LoRa CRC is enabled
*
* @param[out] uint8_t iq_inverted: boolean indicating whether IQ inversion is enabled
*
* @return
*   0 - success, negative otherwise
*/
int32_t ll_radio_params_get(uint8_t *sf, uint8_t *cr, uint8_t *bw, uint32_t *freq,
                            uint16_t *preamble_syms, uint8_t *header_enabled, uint8_t *crc_enabled,
                            uint8_t *iq_inverted);

/**
* @brief
*   Sets multiple radio parameters at once
*
* @details
*
* @param[in] uint8_t flags:
*       RADIO_PARAM_FLAGS_SF
*       RADIO_PARAM_FLAGS_CR
*       RADIO_PARAM_FLAGS_BW
*       RADIO_PARAM_FLAGS_FREQ
*
* @param[in] uint8_t sf: 6..12
*
* @param[in] uint8_t cr: 1=CR4/5 ... 4=CR4/8
*
* @param[in] uint8_t bw: 0=62.5kHz ... 3=500kHz
*
* @param[in] uint32_t freq: center frequency in Hz
*
* @return
*   0 - success, negative otherwise
*/
int32_t ll_radio_params_set(uint8_t flags, uint8_t sf, uint8_t cr, uint8_t bw, uint32_t freq,
                            uint16_t preamble_syms, uint8_t enable_header, uint8_t enable_crc,
                            uint8_t enable_iq_inversion);

/**
 * @brief
 *   Set the RF bandwidth to a number between 0 and 3 corresponding to the
 *   bandwidths listed below.
 *
 * @details
 *   This function sets the RF bandwidth.
 *     - 0 - 62.5 kHz
 *     - 1 - 125  kHz
 *     - 2 - 250  kHz
 *     - 3 - 500  kHz
 *
 * @param[in] bandwidth
 *   bandwidth index (0 to 3)
 *
 * @return
 *   0 - success, negative otherwise
 */
int32_t ll_bandwidth_set(uint8_t bandwidth);

/**
 * @brief
 *   Set the spreading factor for the modulation.  Both the receiver and the transmitter must have the same
 *   setting.
 *
 * @details
 *   This function sets the modulation spreading factor and must be a number in the range 6 to 12.
 *
 * @param[in] sf
 *   spreading factor number 6 to 12
 *
 * @return
 *   0 - success, negative otherwise
 */
int32_t ll_spreading_factor_set(uint8_t sf);

/**
 * @brief
 *   Set the coding rate.  Both the receiver and the transmitter must have the same
 *   setting.
 *
 * @details
 *   This function sets the coding rate from 4/5 to 4/8:
 *     - 1 - 4/5
 *     - 2 - 4/6
 *     - 3 - 4/7
 *     - 4 - 4/8
 *
 * @param[in] coding_rate
 *   coding_rate number from 1 to 4
 *
 * @return
 *   0 - success, negative otherwise
 */
int32_t ll_coding_rate_set(uint8_t coding_rate);

/**
 * @brief
 *   Set the transmitter output power.
 *
 * @details
 *   This function sets the tx output power
 *
 * @param[in] pwr
 *   tx power in dBm
 *      - For the low power module, +2 to 20 dBm
 *      - For the high power module, +10 to 26 dBm
 *
 * @return
 *   0 - success, negative otherwise
 */
int32_t ll_tx_power_set(int8_t pwr);

/**
 * @brief
 *   Get the transmitter output power.
 *
 * @details
 *   Get the tx output power
 *
 * @param[out] pwr
 *   tx power in dBm
 *      - For the low power module, +2 to 20 dBm
 *      - For the high power module, +10 to 26 dBm
 *
 * @return
 *   0 - success, negative otherwise
 */
int32_t ll_tx_power_get(int8_t *pwr);

/**
 * @brief
 *   Set the tuned frequency.  Both the receiver and the transmitter
 *   must have the same setting.
 *
 * @details
 *   This function sets the RF frequency
 *
 * @param[in] freq
 *   frequency in Hz - 820 MHz to 1020 MHz
 *
 * @note
 *   The modules is only designed for the 902 to 928 MHz band
 *
 * @return
 *   0 - success, negative otherwise
 */
int32_t ll_frequency_set(uint32_t freq);

/**
 * @brief
 *   Set the preamble length.
 *
 * @details
 *   This function sets the length of the LoRa preamble, in number of symbols.
 *
 * @param[in] num_syms
 *   length of the preamble in symbols.
 *
 * @return
 *   0 - success, negative otherwise
 */
int32_t ll_preamble_syms_set(uint16_t num_syms);

/**
 * @brief
 *   Enable or disable the LoRa header
 *
 * @details
 *   This function enables or disable the LoRa header. This is also known as the 'fixed-length' LoRa option.
 *
 * @param[in] enabled
 *   Boolean indicating whether to enable or disable the header.
 *
 * @return
 *   0 - success, negative otherwise
 */
int32_t ll_header_enabled_set(uint8_t enabled);

/**
 * @brief
 *   Enable or disable the LoRa CRC
 *
 * @details
 *   This function enables or disable the CRC attached to every LoRa packet.
 *
 * @param[in] enabled
 *   Boolean indicating whether to enable or disable the CRC.
 *
 * @return
 *   0 - success, negative otherwise
 */
int32_t ll_crc_enabled_set(uint8_t enabled);

/**
 * @brief
 *   Enable or disable the I/Q inversion.
 *
 * @param[in] enabled
 *   Boolean. A non-zero value indicates that I/Q inversion will be turned on.
 *
 * @return
 *   0 - success, negative otherwise
 */
int32_t ll_iq_inversion_set(uint8_t inverted);

/**
 * @brief
 *   Set the No MAC sync word
 *
 * @details
 *   This function sets the LoRa sync word
 *
 * @param[in] sync_word
 *
 * @return
 *   0 - success, negative otherwise
 */
int32_t ll_sync_word_set(uint8_t sync_word);

/**
 * @brief
 *   Get the LoRa sync word
 *
 * @details
 *   Get the LoRa sync word
 *
 * @param[out] sync_word
 *
 * @return
 *   0 - success, negative otherwise
 */
int32_t ll_sync_word_get(uint8_t *sync_word);

/**
 * @brief
 *   Put the module into echo mode
 *
 * @details
 *   In echo mode, the module is in a continous Receive mode.  When a packet
 *   is received, it transmits the payload and received RSSI, SNR.
 *
 * @return
 *   0 - success, negative otherwise
 */
int32_t ll_echo_mode();

/**
 * @brief
 *   Cancel any active receive window and send a single Tx packet immediately.
 *
 * @details
 *   When the packet finishes transmission the module will return to the idle state.
 *   It is possible to queue a Rx window, by following this command with
 *   ll_packet_recv.  In this case, the module will enter Rx mode when the
 *   transmission is completed.
 *
 * @param[in] buf
 *   byte array containing the data payload
 *
 * @param[in] len
 *   length of the input buffer in bytes
 *
 * @return
 *   positive number of bytes queued,
 *   negative if an error
 */
int32_t ll_packet_send(uint8_t buf[], uint16_t len);

/**
 * @brief
 *   Cancel any active receive window and queue a packet for Tx.
 *
 * @details
 *   When the packet finishes transmission the module will return to the idle state.
 *
 * @param[in] buf
 *   byte array containing the data payload
 *
 * @param[in] len
 *   length of the input buffer in bytes
 *
 * @return
 *   int32_t 1 if packet queued
 *   int32_t 0 if queue is full
 *   negative if an error occured
 */
int32_t ll_packet_send_queue(uint8_t buf[], uint16_t len);

/**
 * @brief
 *   Cancel any active receive window and queue a packet for transmission
 *   at a specified time.
 *
 * @details
 *   When the packet finishes transmission the module will return to the
 *   idle state.  Also see ll_packet_send_queue().
 *
 * @param[in] timestamp_us
 *   The timestamp in microseconds when the next packet will be
 *   transmitted.  Use ll_timestamp_get() and ll_timestamp_set()
 *   to determine the desired timestamp.
 *
 * @param[in] buf
 *   byte array containing the data payload
 *
 * @param[in] len
 *   length of the input buffer in bytes
 *
 * @return
 *   int32_t 1 if packet queued
 *   int32_t 0 if queue is full
 *   negative if an error occurred
 */
int32_t ll_packet_send_timestamp(uint32_t timestamp_us, uint8_t buf[], uint16_t len);

/**
 * Commands the module to transmit a CW at the current frequency and power.
 */
int32_t ll_transmit_cw(void);

/**
 * @brief
 *   Receive over-the-air data from the module continuously, with RSSI and SNR prepended to the payload
 *
 * @details
 *   This function puts the module into continuous receive mode and checks for any new packets.
 *   Once in continuous receive mode, this function must be called to poll for new packets.
 *
 * @param[out] buf
 *   byte array containing the data payload
 *
 * @param[in] len
 *   size of the output buffer in bytes,
 *
 * @param[out] bytes_received
 *   number of bytes received
 *
 * @note
 *   The first 2 bytes of the payload will be the int16_t rssi in units of dBm
 *   The third byte of the payload will be the estimated SnR
 *   The remaining bytes are the user payload
 *
 * @return
 *   0 - success, negative otherwise
 */
 int32_t ll_packet_recv_cont(uint8_t buf[], uint16_t len, uint8_t *bytes_received);

/**
 * @brief
 *   Receive over-the-air data from the module
 *
 * @details
 *   This function puts the module into receive mode and checks for any new packets.
 *   Once in receive mode, this function must be called to poll for new packets.
 *
 * @param[in] num_timeout_symbols
 *   When search for a preamble, the modem will timeout after this number of symbols
 *
 * @param[out] buf
 *   byte array containing the data payload
 *
 * @param[in] len
 *   size of the output buffer in bytes,
 *
 * @param[out] bytes_received
 *   number of bytes received
 *
 * @return
 *   0 - success, negative otherwise
 */
int32_t ll_packet_recv(uint16_t num_timeout_symbols, uint8_t buf[], uint16_t len, uint8_t *bytes_received);

/**
 * @brief
 *   Receive over-the-air data from the module with RSSI and SNR prepended to the payload
 *
 * @details
 *   This function puts the module into receive mode and checks for any new packets.
 *   Once in receive mode, this function must be called to poll for new packets.
 *
 * @param[in] num_timeout_symbols
 *   When search for a preamble, the modem will timeout after this number of symbols
 *
 * @param[out] buf
 *   byte array containing the data payload
 *
 * @param[in] len
 *   size of the output buffer in bytes,
 *
 * @param[out] bytes_received
 *   number of bytes received
 *
 * @note
 *   The first 2 bytes of the payload will be the int16_t rssi in units of dBm
 *   The third byte of the payload will be the estimated SnR
 *   The remaining bytes are the user payload
 *
 * @return
 *   0 - success, negative otherwise
 */
int32_t ll_packet_recv_with_rssi(uint16_t num_timeout_symbols, uint8_t buf[], uint16_t len, uint8_t *bytes_received);

#ifdef __cplusplus
}
#endif

#endif
