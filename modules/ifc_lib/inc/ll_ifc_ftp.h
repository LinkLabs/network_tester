#ifndef __LL_IFC_FTP_H
#define __LL_IFC_FTP_H

#include <stdint.h>
#include <stdbool.h>
#include "ll_ifc.h"
#include "ll_ifc_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup Link_Labs_Interface_Library
 * @{
 */

/**
 * @defgroup SYMFTP_Interface Symphony Link File Transfer Protocol
 *
 * @brief Symphony Link File Transfer Protocol.
 * Symphony FTP handles the detials for transfering a file over a Symphony Link network.
 *
 * The file transfer is initiated at the Conductor Client-Edge.  Each file transfers
 * is identified by a 32-bit file ID and a 32-bit file version.  The file ID and file
 * version should be defined by the user of this library.  The maximum file size is 256 kB.
 *
 * Note: There is a unique file ID reserved for Link Labs module firmware images.
 *
 * In order to use this protocol the module must be configured to use one of the two available
 * downlink modes ("downlink always on" or "downlink mailbox").
 *
 * When using downlink mailbox the protocol server must be provided with a worse-case mailbox polling interval.
 * When the module receives the first Symphony FTP packet, it will call the dl_config callback and the user must
 * configure the module for downlink always on mode during the file transfer state.  Once the file transfer
 * is complete, the module can return to downlink mailbox mode -- waiting for the apply message.
 *
 * The state machine for the client-side of this protocol is as follows:
 *
 * <pre>
 *   /------\     /----------\     /----------\
 *   | Idle | --> | File     | --> | Wait for |  --> return to Idle
 *   |      |     | Transfer |     | Apply    |
 *   \\------/     \\----------/     \\----------/
 * </pre>
 * * Idle - wait for the file transfer to begin
 * * File Transfer - receive segments requesting missing segments until the file transfer is complete
 * * Wait for Apply - wait for the server to apply the file transfer (e.g. flash the new firmware image)
 *
 *
 * ### Unicast ###
 * This is a file transfer from Conductor to a single Symphony module identified
 * by it's Symphony node address (e.g. $301$0-0-0-030000001)
 *
 * ### Multicast ###
 * This is a file transfer from Conductor to a group of Symphony modules identified by
 * application token.  The group of modules can be reduced to all Symphony modules connected
 * through a particular gateway.
 *
 *
 * @{
 */

#define MAX_NUM_SEGMENTS          (2400)
#define MAX_FILE_SEGMENT_BYTES    (107)
#define NUM_RX_SEGS_BITMASK       (MAX_NUM_SEGMENTS / 32)
#define BASE_UL_MSG_LEN           (14)
#define MAX_NUM_RETRY_SEGS        (16)
#define LL_FTP_TX_BUF_SIZE        (BASE_UL_MSG_LEN + MAX_NUM_RETRY_SEGS * 2)


// The file header contains the crc, file size, file id, and file version
// at the following offsets.
#define LL_FTP_HDR_OFFSET_CRC            (0)
#define LL_FTP_HDR_OFFSET_SIZE           (4)
#define LL_FTP_HDR_OFFSET_ID             (8)
#define LL_FTP_HDR_OFFSET_VERSION        (12)
#define LL_FTP_HDR_LEN                   (16)

/**
 * @brief
 *   Forward declaration of Symphony FTP type definition
 */
struct ll_ftp;


/**
 * @brief
 *   Symphonty FTP return codes
 */
typedef enum ll_ftp_return_code
{
    LL_FTP_OK,            ///< Success
    LL_FTP_OOR,           ///< Out of range
    LL_FTP_INVALID_VALUE, ///< Invalid input value
    LL_FTP_NO_ACTION,     ///< No action
    LL_FTP_ERROR          ///< Error
} ll_ftp_return_code_t;


/**
 * @brief
 *   File open callback, this is called at the beginning of the file transfer.
 *
 * @param[in] file_id
 *   The 32-bit unique ID of the file.
 *
 * @param[in] file_version
 *   The 32-bit unique file version.
 *
 * @param[in] file_size
 *   File size in bytes.
 *
 * @return
 *   LL_FTP_OK - success, see ::ll_ftp_return_code.
 */
typedef ll_ftp_return_code_t (*ll_ftp_open_t)(uint32_t file_id, uint32_t file_version, uint32_t file_size);

/**
 * @brief
 *   File read callback, this is called at the end of the file transfer to compute the
 *   file CRC integrity check.
 *
 * @param[in] file_id
 *   The 32-bit unique ID of the file.
 *
 * @param[in] file_version
 *   The 32-bit unique file version.
 *
 * @param[in] offset
 *   offset of first byte to be read.
 *
 * @param[out] payload
 *   pointer to the buffer where the bytes will be stored.
 *
 * @param[in] len
 *   Number of bytes to read.
 *
 * @return
 *   LL_FTP_OK - success, see ::ll_ftp_return_code.
 */
typedef ll_ftp_return_code_t (*ll_ftp_read_t)(uint32_t file_id, uint32_t file_version, uint32_t offset, uint8_t* payload, uint16_t len);

/**
 * @brief
 *   File write callback, this is called each time a segment is received to store the received segment
 *   to storage.
 *
 * @param[in] file_id
 *   The 32-bit unique ID of the file.
 *
 * @param[in] file_version
 *   The 32-bit unique file version.
 *
 * @param[in] offset
 *   offset of first byte to be written.
 *
 * @param[in] payload
 *   pointer to the buffer of bytes.
 *
 * @param[in] len
 *   Number of bytes to write.
 *
 * @return
 *   LL_FTP_OK - success, see ::ll_ftp_return_code.
 */
typedef ll_ftp_return_code_t (*ll_ftp_write_t)(uint32_t file_id, uint32_t file_version, uint32_t offset, uint8_t* payload, uint16_t len);

/**
 * @brief
 *   File close callback, this is called once all segments have been received, when a file transfer is interrupted, or 
 *   a file transfer is canceled.
 *
 * @param[in] file_id
 *   The 32-bit unique ID of the file.
 *
 * @param[in] file_version
 *   The 32-bit unique file version.
 *
 * @return
 *   LL_FTP_OK - success, see ::ll_ftp_return_code.
 */
typedef ll_ftp_return_code_t (*ll_ftp_close_t)(uint32_t file_id, uint32_t file_version);

/**
 * @brief
 *   File apply callback, this is called after all segments have been received, the file has been closed, and the server
 *   is ready to proceed with the final step of the process, if any.
 *
 * @param[in] file_id
 *   The 32-bit unique ID of the file.
 *
 * @param[in] file_version
 *   The 32-bit unique file version.
 *
 * @param[in] file_size
 *   File size in bytes.
 *
 * @return
 *   LL_FTP_OK - success, see ::ll_ftp_return_code.
 */
typedef ll_ftp_return_code_t (*ll_ftp_apply_t)(uint32_t file_id, uint32_t file_version, uint32_t file_size);

/**
 * @brief
 *   Send uplink callback, the user of this library must send responses to the server over Symphony Link on behalf of the 
 *   Symphony FTP protocol.
 *
 * @param[in] buf
 *   byte array containing the data payload.
 *
 * @param[in] len
 *   length of the input buffer.
 *
 * @param[in] acked
 *   Whether or not to request an ACK from the gateway.
 *
 * @param[in] port
 *   The port number to send to.
 *
 * @return
 *   LL_FTP_OK - success, see ::ll_ftp_return_code.
 */
typedef ll_ftp_return_code_t (*ll_ftp_send_uplink_t)(const uint8_t* buf, uint8_t len, bool acked, uint8_t port);

/**
 * @brief
 *   downlink config callback, this is called to toggle between the default downlink state and the downlink always on
 *   state to support the different phases of the file transfer.
 *
 * @note
 *   If the default mode is downlink always on, then this function has nothing to do.
 *
 * @return
 *   LL_FTP_OK - success, see ::ll_ftp_return_code.
 */
typedef ll_ftp_return_code_t (*ll_ftp_dl_config_t)(bool downlink_on);

/**
 * Symphony FTP Download state
 */
typedef enum ll_ftp_state
{
    IDLE,       ///< Idle
    SEGMENT,    ///< File Transfer
    APPLY,      ///< Wait for Apply
} ll_ftp_state_t;

/**
 * Symphony FTP required callbacks
 */
typedef struct ll_ftp_callbacks
{
    ll_ftp_open_t open;
    ll_ftp_read_t read;
    ll_ftp_write_t write;
    ll_ftp_close_t close;
    ll_ftp_apply_t apply;
    ll_ftp_send_uplink_t uplink;
    ll_ftp_dl_config_t config;
} ll_ftp_callbacks_t;

/**
 * Symphony FTP object.  A reference to an ll_ftp_t object is used as the context pointer.
 */
typedef struct ll_ftp
{
    ll_ftp_state_t state;
    uint16_t num_segs;
    uint32_t rx_segs[NUM_RX_SEGS_BITMASK];
    uint32_t file_id;
    uint32_t file_version;
    uint32_t file_size;
    struct time time_last_msg;
    uint8_t retry_count;
    bool is_processing;
    uint8_t tx_buf[LL_FTP_TX_BUF_SIZE];
    ll_ftp_callbacks_t cb;
} ll_ftp_t;

/**
 * @brief
 *   Initializes an ll_ftp_t object and registers application-specific callbacks.
 *
 * @param[ctx] f
 *   context pointer for the current transfer.
 *
 * @param[in] cb
 *   pointer to a structure containing the required callbacks.
 *
 * @return
 *   0 - [LL_FTP_OK] success
 *   2 - [LL_FTP_INVALID_VALUE] otherwise
 */
int32_t ll_ftp_init(ll_ftp_t* f, ll_ftp_callbacks_t* cb);

/**
 * @brief
 *   Signal Symphony FTP entry point.  Processes a received downlink message for Symphony FTP or a timeout. This code
 *   expects this function to be called at least once per second.
 *
 * @param[ctx] f
 *   context pointer for the current transfer.
 *
 * @param[in] msg
 *   pointer to a buffer containing bytes received by the Symphony Link module
 *   NULL signifies a timeout event.
 *
 * @param[in] len
 *   number of bytes in msg.
 *
 * @return
 *   0 - [LL_FTP_OK] success
 *   1 - [LL_FTP_OOR] len is out of range
 *   2 - [LL_FTP_INVALID_VALUE] msg contains unexpected content
 *   3 - [LL_FTP_NO_ACTION] msg was not necessary
 *   4 - [LL_FTP_ERROR] otherwise
 */
int32_t ll_ftp_msg_process(ll_ftp_t* f, uint8_t* msg, uint8_t len);

/**
 * @brief
 *   Determines the number of outstanding file segments still to be downloaded.
 *
 * @param[ctx] f
 *   context pointer for the current transfer.
 *
 * @return
 *   number of outstanding segments.
 *   -1 - if context pointer, f, is not initialized.
 */
int32_t ll_ftp_num_missing_segs_get(ll_ftp_t* f);


/** @} (end defgroup Symphony_Interface) */

/** @} (end addtogroup Link_Labs_Interface_Library) */


#ifdef __cplusplus
}
#endif

#endif
