#ifndef __LL_IFC_CONSTS_H
#define __LL_IFC_CONSTS_H

#include <stdint.h>

/**
 * @addtogroup Link_Labs_Interface_Library
 * @{
 */

/**
 * @addtogroup Module_Interface
 * @brief
 * @{
 */

#define IFC_VERSION_MAJOR (0)
#define IFC_VERSION_MINOR (5)
#define IFC_VERSION_TAG (0)

#define APP_TOKEN_LEN (10)
#define MAX_RX_MSG_LEN (128)

extern const uint32_t OPEN_NET_TOKEN;   ///< Open network token (0x4f50454e)

typedef enum
{
    OP_VERSION = 0,                 ///< 0x00
    OP_IFC_VERSION = 1,             ///< 0x01
    OP_STATE = 2,                   ///< 0x02
    OP_TX_STATE = 3,                ///< 0x03
    OP_RX_STATE = 4,                ///< 0x04
    OP_FREQUENCY = 6,               ///< 0x06
    OP_TX_POWER_SET = 7,            ///< 0x07
    OP_RESET_SETTINGS = 8,          ///< 0x08
    OP_GET_RADIO_PARAMS = 9,        ///< 0x09
    OP_SET_RADIO_PARAMS = 10,       ///< 0x0A
    OP_PKT_SEND_QUEUE = 11,         ///< 0x0B
    OP_TX_POWER_GET = 12,           ///< 0x0C
    OP_SYNC_WORD_SET = 13,          ///< 0x0D
    OP_SYNC_WORD_GET = 14,          ///< 0x0E
    OP_IRQ_FLAGS = 15,              ///< 0x0F
    OP_IRQ_FLAGS_MASK = 16,         ///< 0x10
    OP_SLEEP = 20,                  ///< 0x14
    OP_SLEEP_BLOCK = 21,            ///< 0x15
    OP_PKT_ECHO = 31,               ///< 0x1F
    OP_PKT_RECV = 40,               ///< 0x28
    OP_MSG_RECV_RSSI = 41,          ///< 0x29 - Deprecated for Symphony mode
    OP_PKT_RECV_CONT = 42,          ///< 0x2A
    OP_MSG_RECV = 43,               ///< 0x2B
    OP_MODULE_ID = 50,              ///< 0x32
    OP_STORE_SETTINGS = 51,         ///< 0x33
    OP_DELETE_SETTINGS = 52,        ///< 0x34
    OP_RESET_MCU = 60,              ///< 0x3C
    OP_TRIGGER_BOOTLOADER = 61,     ///< 0x3D
    OP_MAC_MODE_SET = 70,           ///< 0x46
    OP_MAC_MODE_GET = 71,           ///< 0x47
//STRIPTHIS!START
    OP_RESERVED_LIFERAFT0 = 72,     ///< 0x48 - Liferaft Specific (Do not use!)
//STRIPTHIS!STOP
    OP_MSG_SEND_ACK = 90,           ///< 0x5A - Deprecated
    OP_MSG_SEND_UNACK = 91,         ///< 0x5B - Deprecated
    OP_MSG_SEND = 92,               ///< 0x5C
    OP_CONN_FILT_SET = 93,          ///< 0x5D
    OP_CONN_FILT_GET = 94,          ///< 0x5E
//STRIPTHIS!START
    OP_RESERVED0 = 96,              ///< 0x60
    OP_RESERVED1 = 97,              ///< 0x60
//STRIPTHIS!STOP
    OP_TX_CW = 98,                  ///< 0x61
    OP_SYSTEM_TIME_GET = 108,       ///< 0x6C
    OP_SYSTEM_TIME_SYNC = 109,      ///< 0x6D
    OP_RX_MODE_SET = 110,           ///< 0x6E
    OP_RX_MODE_GET = 111,           ///< 0x6F
    OP_QOS_REQUEST = 112,           ///< 0x70
    OP_QOS_GET     = 113,           ///< 0x71
    OP_ANTENNA_SET = 114,           ///< 0x72
    OP_ANTENNA_GET = 115,           ///< 0x73
    OP_NET_TOKEN_SET  = 116,        ///< 0x74
    OP_NET_TOKEN_GET  = 117,        ///< 0x75
    OP_NET_INFO_GET= 118,           ///< 0x76
    OP_STATS_GET   = 119,           ///< 0x77
    OP_RSSI_SET    = 120,           ///< 0x78
    OP_RSSI_GET    = 121,           ///< 0x79
    OP_DL_BAND_CFG_GET       = 122, ///< 0x7A
    OP_DL_BAND_CFG_SET       = 123, ///< 0x7B
    OP_APP_TOKEN_SET         = 124, ///< 0x7C
    OP_APP_TOKEN_GET         = 125, ///< 0x7D
    OP_APP_TOKEN_REG_GET     = 126, ///< 0x7E
    OP_CRYPTO_KEY_XCHG_REQ   = 128, ///< 0x80
    OP_MAILBOX_REQUEST       = 129, ///< 0x81
//STRIPTHIS!START
    OP_RESERVED_LIFERAFT1    = 130, ///< 0x82 - Liferaft Specific (Do not use!)
    OP_TIMESTAMP             = 131, ///< 0x83  reserved, not fully implemented
    OP_SEND_TIMESTAMP        = 132, ///< 0x84  reserved, not fully implemented
//STRIPTHIS!STOP

//STRIPTHIS!START
    OP_FCC_TEST = 245,              ///< 0xF5
    OP_PER_TEST_TX = 246,           ///< 0xF6
    OP_PER_TEST_RX = 247,           ///< 0xF7
    OP_GET_ASSERT = 248,            ///< 0xF8
    OP_SET_ASSERT = 249,            ///< 0xF9
//STRIPTHIS!STOP
    OP_HARDWARE_TYPE = 254,         ///< 0xFE
    OP_FIRMWARE_TYPE = 255          ///< 0xFF
} opcode_t;

typedef enum
{
    FRAME_START = 0xC4              ///< 0xC4
} frame_t;

typedef enum
{
    LORA_NO_MAC = 0,                ///< 0x00
    DEPRECATED_MAC1,                ///< 0x01
    DEPRECATED_MAC2,                ///< 0x02
    SYMPHONY_LINK,                  ///< 0x03
    NUM_MACS,                       ///< Total number of MAC modes
    MAC_INVALID = 255,              ///< 0xFF
} ll_mac_type_t;

/**
 * version struct
 */
typedef struct {
    uint8_t major;
    uint8_t minor;
    uint16_t tag;
} ll_version_t;

#define VERSION_LEN                 (4)

// Hardware Types
typedef enum
{
    UNAVAILABLE    = 0,             ///< 0x00
    LLRLP20_V2     = 1,             ///< 0x01
    LLRXR26_V2     = 2,             ///< 0x02
    LLRLP20_V3     = 3,             ///< 0x03
    LLRXR26_V3     = 4,             ///< 0x04
    LLREPEATER     = 5              ///< 0x05  AES_TODO - Replace with official part number?
} ll_hardware_type_t;

/**
 * Firmware identifiers
 */
typedef enum
{
    CPU_EFM32TG210F32    = 0,      ///< 0x00
    CPU_EFM32G210F128    = 1,      ///< 0x01
    CPU_R5F51115ADNE     = 2,      ///< 0x02
    CPU_R5F51116ADNE     = 3,      ///< 0x03
    CPU_EFM32GG232F1024  = 4       ///< 0x04
} cpu_code_t;

typedef enum
{
    GATEWAY_TX_ONLY = 0,            ///< 0x00
    MODULE_END_NODE = 1,            ///< 0x01
    REPEATER_HOST   = 2             ///< 0x02
    // TBD - How to define others ?
} functionality_code_t;

/**
 * Link Labs LORA enumeration identifiers for Bandwidth
 */
typedef enum
{
    PROPERTY_LORA_BW_MIN    = 0,    ///< range limit value (lower)
    PROPERTY_LORA_BW_62_5   = 0,    ///< 62.5KHz BW
    PROPERTY_LORA_BW_125    = 1,    ///< 125KHz BW
    PROPERTY_LORA_BW_250    = 2,    ///< 250KHz BW
    PROPERTY_LORA_BW_500    = 3,    ///< 500KHz BW
    PROPERTY_LORA_BW_MAX,           ///< range limit value (upper)
} property_bw_t;

/**
 * Link Labs LORA enumeration identifiers for Spreading Factor
 */
typedef enum
{
    PROPERTY_LORA_SF_MIN    = 6,    ///< range limit value (lower)
    PROPERTY_LORA_SF_6      = 6,    ///< SF 6
    PROPERTY_LORA_SF_7      = 7,    ///< SF 7
    PROPERTY_LORA_SF_8      = 8,    ///< SF 8
    PROPERTY_LORA_SF_9      = 9,    ///< SF 9
    PROPERTY_LORA_SF_10     = 10,   ///< SF 10
    PROPERTY_LORA_SF_11     = 11,   ///< SF 11
    PROPERTY_LORA_SF_12     = 12,   ///< SF 12
    PROPERTY_LORA_SF_MAX,           ///< range limit value (upper)
} property_sf_t;

/**
 * Link Labs LORA enumeration identifiers for Coding rate
 */
typedef enum
{
    PROPERTY_LORA_CR_MIN    = 1,    ///< range limit value (lower)
    PROPERTY_LORA_CR_4_5    = 1,    ///< 0x01 -- 4/5
    PROPERTY_LORA_CR_4_6    = 2,    ///< 0x02 -- 4/6
    PROPERTY_LORA_CR_4_7    = 3,    ///< 0x03 -- 4/7
    PROPERTY_LORA_CR_4_8    = 4,    ///< 0x04 -- 4/8
    PROPERTY_LORA_CR_MAX,           ///< range limit value (upper)
} property_cr_t;

typedef struct {
    uint16_t cpu_code;
    uint16_t functionality_code;
} ll_firmware_type_t;

#define FIRMWARE_TYPE_LEN           (4)

/** Possible downlink modes for OP_DOWNLINK_MODE */
typedef enum
{
    DOWNLINK_MODE_OFF = 0,          ///< 0x00
    DOWNLINK_MODE_ALWAYS_ON = 1,    ///< 0x01
    DOWNLINK_MODE_MAILBOX = 2,      ///< 0x02
    DOWNLINK_MODE_PERIODIC = 3,     ///< 0x03
    NUM_DOWNLINK_MODES,
    DOWNLINK_MODE_FAILURE = 255,    ///< 0xFF
} downlink_mode_t;

/** ACK/NACK Codes */
#define LL_IFC_ACK                          (0)   ///< All good.
#define LL_IFC_NACK_CMD_NOT_SUPPORTED       (1)   ///< Command not supported.
#define LL_IFC_NACK_INCORRECT_CHKSUM        (2)   ///< Incorrect Checksum.
#define LL_IFC_NACK_PAYLOAD_LEN_OOR         (3)   ///< Length of payload sent in command was out of range.
#define LL_IFC_NACK_PAYLOAD_OOR             (4)   ///< Payload sent in command was out of range.
#define LL_IFC_NACK_BOOTUP_IN_PROGRESS      (5)   ///< Not allowed since firmware bootup still in progress. Wait.
#define LL_IFC_NACK_BUSY_TRY_AGAIN          (6)   ///< Operation prevented by temporary event. Re-try should work.
#define LL_IFC_NACK_APP_TOKEN_REG           (7)   ///< Application token is not registered for this node.
#define LL_IFC_NACK_PAYLOAD_LEN_EXCEEDED    (8)   ///< Payload length is greater than the max supported length
#define LL_IFC_NACK_NOT_IN_MAILBOX_MODE     (9)   ///< Module must be in DOWNLINK_MAILBOX mode
#define LL_IFC_NACK_PAYLOAD_BAD_PROPERTY    (10)  ///< Invalid property specified in command
#define LL_IFC_NACK_NODATA                  (11)  ///< No data is available to be returned
#define LL_IFC_NACK_QUEUE_FULL              (12)  ///< Data could not be enqueued for transmission (queue full)
#define LL_IFC_NACK_OTHER                   (99)
/* When adding a new value, update ll_return_code_name() and ll_return_code_description() */

/** Error Codes */
/* Note: Error codes -1 to -99 map to NACK codes received from the radio */
typedef enum ll_ifc_error_codes_e {
    LL_IFC_ERROR_INCORRECT_PARAMETER        = -101, ///< The parameter value was invalid.
    LL_IFC_ERROR_INCORRECT_RESPONSE_LENGTH  = -102, ///< Module response was not the expected size.
    LL_IFC_ERROR_MESSAGE_NUMBER_MISMATCH    = -103, ///< Message number in response doesn't match expected
    LL_IFC_ERROR_CHECKSUM_MISMATCH          = -104, ///< Checksum mismatch
    LL_IFC_ERROR_COMMAND_MISMATCH           = -105, ///< Command mismatch (responding to a different command)
    LL_IFC_ERROR_HOST_INTERFACE_TIMEOUT     = -106, ///< Timed out waiting for Rx bytes from interface
    LL_IFC_ERROR_BUFFER_TOO_SMALL           = -107, ///< Response larger than provided output buffer
    LL_IFC_ERROR_START_OF_FRAME             = -108, ///< transport_read failed getting FRAME_START
    LL_IFC_ERROR_HEADER                     = -109, ///< transport_read failed getting header
    LL_IFC_ERROR_TIMEOUT                    = -110, ///< The operation timed out.
    LL_IFC_ERROR_INCORRECT_MESSAGE_SIZE     = -111, ///< The message size from the device was incorrect.
    LL_IFC_ERROR_NO_NETWORK                 = -112, ///< No network was available.
    /* When adding a new value, update ll_return_code_name() and ll_return_code_description() */
} ll_ifc_error_codes_t;


/** Bit Definitions for OP_SET_RADIO_PARAMS */
#define RADIO_PARAM_FLAGS_SF        (1u<<0u)
#define RADIO_PARAM_FLAGS_CR        (1u<<1u)
#define RADIO_PARAM_FLAGS_BW        (1u<<2u)
#define RADIO_PARAM_FLAGS_FREQ      (1u<<3u)
#define RADIO_PARAM_FLAGS_PREAMBLE  (1u<<4u)
#define RADIO_PARAM_FLAGS_HEADER    (1u<<5u)
#define RADIO_PARAM_FLAGS_CRC       (1u<<6u)
#define RADIO_PARAM_FLAGS_IQ        (1u<<7u)

/** Bit Definitions for OP_IRQ_FLAGS */
#define IRQ_FLAGS_WDOG_RESET                  (0x00000001UL)  ///< Set every time the module reboots after a Watchdog reboot
#define IRQ_FLAGS_RESET                       (0x00000002UL)  ///< Set every time the module reboots for any reason
#define IRQ_FLAGS_TX_DONE                     (0x00000010UL)  ///< Set every time a Tx Queue goes from non-empty to empty
#define IRQ_FLAGS_TX_ERROR                    (0x00000020UL)  ///< Set every time there is a Tx Error
#define IRQ_FLAGS_RX_DONE                     (0x00000100UL)  ///< Set every time a new packet is received
#define IRQ_FLAGS_MAILBOX_EMPTY               (0x00000200UL)  ///< Set when a GW reports an empty mailbox
#define IRQ_FLAGS_CONNECTED                   (0x00001000UL)  ///< Set every time we transition from the disconnected -> connected state
#define IRQ_FLAGS_DISCONNECTED                (0x00002000UL)  ///< Set every time we transition from the connected -> disconnected state
//STRIPTHIS!START
// LifeRaft IRQ flags
#define IRQ_FLAGS_ENROLL_GRANT                (0x00004000UL)  ///< Set when the module has been granted an enrollment by the gateway
//STRIPTHIS!STOP
#define IRQ_FLAGS_CRYPTO_ESTABLISHED          (0x00010000UL)  ///< Set every time we transition from the crypto not established -> crytpto established state
#define IRQ_FLAGS_APP_TOKEN_CONFIRMED         (0x00020000UL)  ///< Set every time an application token is confirmed by Conductor
#define IRQ_FLAGS_DOWNLINK_REQUEST_ACK        (0x00040000UL)  ///< Set every time a downlink registration request is acknowledged
#define IRQ_FLAGS_INITIALIZATION_COMPLETE     (0x00080000UL)  ///< Set every time the MAC has completed initialization
#define IRQ_FLAGS_CRYPTO_ERROR                (0x00100000UL)  ///< Set when a crypto exchange attempt fails
#define IRQ_FLAGS_APP_TOKEN_ERROR             (0x00200000UL)  ///< Set when an application token registration fails
//STRIPTHIS!START
// LifeRaft IRQ flags
#define IRQ_FLAGS_STATUS_REQ                  (0x00400000UL)  ///< Set when we want to request the status of the host controller
#define IRQ_FLAGS_FIRMWARE_REQ                (0x00800000UL)  ///< Set when we want to request the firmware data of the host controller
//STRIPTHIS!STOP
#define IRQ_FLAGS_ASSERT                      (0x80000000UL)  ///< Set every time we transition from the connected->disconnected state

/**
 * @brief
 *   The operations for ll_timestamp_set().
 */
typedef enum ll_timestamp_operation_e {
    /**
     * @brief
     *   No set operation.
     *
     * @details
     *   Just get the current timestamp.
     */
    LL_TIMESTAMP_NO_OPERATION,

    /**
     * @brief
     *   Directly set the timestamp from the provided value.
     *
     * @details
     *   The value is not applied until the command is processed by the
     *   module, and it does not account for transmission delay.
     */
    LL_TIMESTAMP_SET_IMMEDIATE,

    /**
     * @brief
     *   Synchronize the timestamp using the provided value which
     *   corresponds to the most recent event.
     *
     * @details
     *   Use this mechanism when the host directly controls the trigger
     *   event received by both the reference time source and the module.
     *   This mechanism guarantees the module that the reference value
     *   aligns to the module's timestamp value when the most recent
     *   trigger event occurred.
     */
    LL_TIMESTAMP_SYNC_IMMEDIATE,

    /**
     * @brief
     *   Intelligently synchronize the timestamp using the provided value
     *   accounting for possible mismatched trigger events.
     *
     * @details
     *   Use this mechanism when the host does not control the trigger
     *   event.  When the trigger event is free-running such as with a
     *   GPS pulse-per-seconds (PPS) output, the reference value may
     *   be one event behind the module's value.  This mechanism allows
     *   the module to detect and account for this variability.  To
     *   correctly update the module's timestamp using this mechanism:
     *
     *   1. Read the reference timestamp.
     *   2. Call ll_timestamp_set()
     *   3. Read the reference timestamp.
     *   4. Call ll_timestamp_set()
     */
    LL_TIMESTAMP_SYNC
} ll_timestamp_operation_t;

/** @} (end addtogroup Module_Interface) */




/** @} (end defgroup Module_Interface) */

/** @} (end addtogroup Link_Labs_Interface_Library) */

#endif /* __LL_IFC_CONSTS_H */
