#ifndef SUPERVISOR_H_INCLUDED
#define SUPERVISOR_H_INCLUDED
/*********************************************************************/
/*****INCLUDES********************************************************/
#include "ll_ifc.h"
/*********************************************************************/
/*****PUBLIC TYPEDEFS*************************************************/
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

/*********************************************************************/
/*****PUBLIC DEFS*****************************************************/
/*****PUBLIC FUNCTIONS************************************************/
void sup_get_gw_status(llabs_network_info_t* gw_info_ptr);
bool sup_get_GW_rssi(int16_t* rssi_ptr);    // returns true if GW connected, false otherwise.  Stores RSSI in rssi_ptr if GW connected
uint8_t init_supervisor_task(void);
uint64_t sup_get_MAC_address(void);
/*********************************************************************/
#endif /* SUPERVISOR_H_INCLUDED */
