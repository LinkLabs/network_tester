#ifndef SUPERVISOR_H_INCLUDED
#define SUPERVISOR_H_INCLUDED

#include "ll_ifc.h"

#define LL_FTP_FIRMWARE_ID_MODULE  (0x726c706d)
#define LL_FTP_FIRMWARE_ID_NETTEST (0x6e657477)

#define NET_TOKEN_OPEN         (0x4f50454e)

typedef enum
{
    MSG_WAITING_FOR_ACK,
    MSG_ACKED,
    MSG_ERROR,
    MSG_INIT
} msg_success_t;

typedef struct
{
    uint8_t         msg;
    msg_success_t   acked;
} msg_record_t;

typedef struct
{
    uint32_t crc;
    uint32_t size;
    uint32_t id;
    uint32_t version;
} ftp_header_data_t;

extern const uint8_t DEFAULT_APPLICATION_TOKEN[];
extern const uint8_t OFFLINE_APPLICATION_TOKEN[];

// returns -1 if the flash vars are null
int32_t load_ftp_flash_vars(uint32_t start_addr, ftp_header_data_t* data);

void sup_get_gw_status(llabs_network_info_t* gw_info_ptr);

// returns true if GW connected, false otherwise.  Stores RSSI in rssi_ptr if GW connected
bool sup_get_GW_rssi(int16_t* rssi_ptr);

uint8_t init_supervisor_task(void);

uint64_t sup_get_MAC_address(void);

ll_version_t sup_get_version(void);

#endif /* SUPERVISOR_H_INCLUDED */
