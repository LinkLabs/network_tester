#ifndef __LL_IFC_CONSTS_H
#define __LL_IFC_CONSTS_H

#include <stdint.h>

#define IFC_VERSION_MAJOR (0)
#define IFC_VERSION_MINOR (3)
#define IFC_VERSION_TAG (1)

#define APP_TOKEN_LEN (10)
#define MAX_RX_MSG_LEN (128)

extern const uint32_t OPEN_NET_TOKEN;

typedef enum
{
    OP_VERSION = 0,                 // 0x00
    OP_IFC_VERSION = 1,             // 0x01
    OP_STATE = 2,                   // 0x02
    OP_TX_STATE = 3,                // 0x03
    OP_RX_STATE = 4,                // 0x04
    OP_FREQUENCY = 6,               // 0x06
    OP_TX_POWER_SET = 7,            // 0x07
    OP_RESET_SETTINGS = 8,          // 0x08
    OP_GET_RADIO_PARAMS = 9,        // 0x09
    OP_SET_RADIO_PARAMS = 10,       // 0x0A
    OP_PKT_SEND_QUEUE = 11,         // 0x0B
    OP_TX_POWER_GET = 12,           // 0x0C
    OP_SYNC_WORD_SET = 13,          // 0x0D
    OP_SYNC_WORD_GET = 14,          // 0x0E
    OP_IRQ_FLAGS = 15,              // 0x0F
    OP_IRQ_FLAGS_MASK = 16,         // 0x10
    OP_SLEEP = 20,                  // 0x14
    OP_SLEEP_BLOCK = 21,            // 0x15
    OP_PKT_ECHO = 31,               // 0x1F
    OP_PKT_RECV = 40,               // 0x28
    OP_PKT_RECV_RSSI = 41,          // 0x29
    OP_PKT_RECV_CONT = 42,          // 0x2A
    OP_MODULE_ID = 50,              // 0x32
    OP_STORE_SETTINGS = 51,         // 0x33
    OP_DELETE_SETTINGS = 52,        // 0x34
    OP_RESET_MCU = 60,              // 0x3C
    OP_TRIGGER_BOOTLOADER = 61,     // 0x3D
    OP_MAC_MODE_SET = 70,           // 0x46
    OP_MAC_MODE_GET = 71,           // 0x47
    OP_PKT_SEND_ACK = 90,           // 0x5A
    OP_PKT_SEND_UNACK = 91,         // 0x5B
    OP_TX_CW = 98,                  // 0x61
    OP_RX_MODE_SET = 110,           // 0x6E
    OP_RX_MODE_GET = 111,           // 0x6F
    OP_QOS_REQUEST = 112,           // 0x70
    OP_QOS_GET     = 113,           // 0x71
    OP_ANTENNA_SET = 114,           // 0x72
    OP_ANTENNA_GET = 115,           // 0x73
    OP_NET_TOKEN_SET  = 116,        // 0x74
    OP_NET_TOKEN_GET  = 117,        // 0x75
    OP_NET_INFO_GET= 118,           // 0x76
    OP_STATS_GET   = 119,           // 0x77
    OP_RSSI_SET    = 120,           // 0x78
    OP_RSSI_GET    = 121,           // 0x79
    OP_DL_BAND_CFG_GET       = 122, // 0x7A
    OP_DL_BAND_CFG_SET       = 123, // 0x7B
    OP_APP_TOKEN_SET         = 124, // 0x7C
    OP_APP_TOKEN_GET         = 125, // 0x7D
    OP_APP_TOKEN_REG_GET     = 126, // 0x7E
    OP_CRYPTO_KEY_XCHG_REQ   = 128, // 0x80
    OP_MAILBOX_REQUEST       = 129, // 0x81
    OP_HARDWARE_TYPE = 254,         // 0xFE
    OP_FIRMWARE_TYPE = 255          // 0xFF
} opcode_t;

typedef enum
{
    FRAME_START = 0xC4
} frame_t;

typedef enum
{
    LORA_NO_MAC = 0,
    LORAWAN_EU,
    LORAWAN_FCC,
    SYMPHONY_LINK,
    NUM_MACS,
    MAC_INVALID = 255,
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
    UNAVAILABLE    = 0,             // 0x00
    LLRLP20_V2     = 1,             // 0x01
    LLRXR26_V2     = 2,             // 0x02
    LLRLP20_V3     = 3,             // 0x03
    LLRXR26_V3     = 4,             // 0x04
} ll_hardware_type_t;

/**
 * Firmware identifiers
 */
typedef enum
{
    CPU_EFM32TG210F32 = 0,          // 0x00
    CPU_EFM32G210F128 = 1,          // 0x01
    CPU_R5F51115ADNE  = 2,          // 0x02
    CPU_R5F51116ADNE  = 3,          // 0x03
} cpu_code_t;

typedef enum
{
    GATEWAY_TX_ONLY = 0,            // 0x00
    MODULE_END_NODE = 1             // 0x01
    // TBD - How to define others ?
} functionality_code_t;

typedef struct {
    uint16_t cpu_code;
    uint16_t functionality_code;
} ll_firmware_type_t;

#define FIRMWARE_TYPE_LEN           (4)

/** Possible downlink modes for OP_DOWNLINK_MODE */
typedef enum
{
    DOWNLINK_MODE_OFF = 0,          // 0x00
    DOWNLINK_MODE_ALWAYS_ON = 1,    // 0x01
    DOWNLINK_MODE_MAILBOX = 2,      // 0x02
    DOWNLINK_MODE_PERIODIC = 3,     // 0x03
    NUM_DOWNLINK_MODES,
    DOWNLINK_MODE_FAILURE = 255,    // 0xFF
} downlink_mode_t;

/** ACK/NACK Codes */
#define LL_IFC_ACK                          (0)   // All good.
#define LL_IFC_NACK_CMD_NOT_SUPPORTED       (1)   // Command not supported.
#define LL_IFC_NACK_INCORRECT_CHKSUM        (2)   // Incorrect Checksum.
#define LL_IFC_NACK_PAYLOAD_LEN_OOR         (3)   // Length of payload sent in command was out of range.
#define LL_IFC_NACK_PAYLOAD_OOR             (4)   // Payload sent in command was out of range.
#define LL_IFC_NACK_BOOTUP_IN_PROGRESS      (5)   // Not allowed since firmware bootup still in progress. Wait.
#define LL_IFC_NACK_BUSY_TRY_AGAIN          (6)   // Operation prevented by temporary event. Re-try should work.
#define LL_IFC_NACK_APP_TOKEN_REG           (7)   // Application token is not registered for this node.
#define LL_IFC_NACK_PAYLOAD_LEN_EXCEEDED    (8)   // Payload length is greater than the max supported length
#define LL_IFC_NACK_NOT_IN_MAILBOX_MODE     (9)   // Module must be in DOWNLINK_MAILBOX mode
#define LL_IFC_NACK_OTHER                   (99)

/** Error Codes */
/* Note: Error codes -1 to -99 map to NACK codes received from the radio */
#define LL_IFC_ERROR_INCORRECT_PARAMETER    (-101)
// TODO: Define more error codes

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
#define IRQ_FLAGS_WDOG_RESET                  (0x00000001UL)  // Set every time the module reboots after a Watchdog reboot
#define IRQ_FLAGS_RESET                       (0x00000002UL)  // Set every time the module reboots for any reason
#define IRQ_FLAGS_TX_DONE                     (0x00000010UL)  // Set every time a Tx Queue goes from non-empty to empty
#define IRQ_FLAGS_TX_ERROR                    (0x00000020UL)  // Set every time there is a Tx Error
#define IRQ_FLAGS_RX_DONE                     (0x00000100UL)  // Set every time a new packet is received
#define IRQ_FLAGS_CONNECTED                   (0x00001000UL)  // Set every time we transition from the disconnected -> connected state
#define IRQ_FLAGS_DISCONNECTED                (0x00002000UL)  // Set every time we transition from the connected -> disconnected state
#define IRQ_FLAGS_ENROLL_GRANT                (0x00004000UL)  // Set when the module has been granted an enrollment by the gateway
#define IRQ_FLAGS_CRYPTO_ESTABLISHED          (0x00010000UL)  // Set every time we transition from the crypto not established -> crytpto established state
#define IRQ_FLAGS_APP_TOKEN_CONFIRMED         (0x00020000UL)  // Set every time an application token is confirmed by Conductor
#define IRQ_FLAGS_DOWNLINK_REQUEST_ACK        (0x00040000UL)  // (NOT IMPLEMENTED) Set every time a downlink registration request is acknowledged
#define IRQ_FLAGS_CRYPTO_ERROR                (0x00100000UL)  // Set when a crypto exchange attempt fails
#define IRQ_FLAGS_APP_TOKEN_ERROR             (0x00200000UL)  // Set when an application token registration fails
#define IRQ_FLAGS_STATUS_REQ                  (0x00400000UL)  // Set when we want to request the status of the host controller
#define IRQ_FLAGS_FIRMWARE_REQ                (0x00800000UL)  // Set when we want to request the firmware data of the host controller
#define IRQ_FLAGS_ASSERT                      (0x80000000UL)  // Set every time we transition from the connected->disconnected state


#endif /* __LL_IFC_CONSTS_H */
