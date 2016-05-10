#ifndef UI_TASK_H_INCLUDED
#define UI_TASK_H_INCLUDED
#include "supervisor.h"
#include "gps_task.h"
#include "bme280_driver.h"
/*********************************************************************/
/*****PUBLIC DEFS*****************************************************/
#define UI_TX_ON_DEMAND 0xFFFFFFFF
#define NUM_NOTIFY_CYCLES_MSG_RX               2       // number of pulses in notify action
#define NUM_NOTIFY_CYCLES_LOST_GW              1       // number of pulses in notify action
/*****PUBLIC TYPES****************************************************/
typedef enum
{
    MAIN_MENU = 0,
    GPS_MENU,
    SENSOR_MODE_MENU,
    DOWNLINK_MENU,
    GPS_DIAG_MENU,
    SENSOR_DIAG_MENU,
    NETWORK_DIAG_MENU,
    UART_PASS_DIAG_MENU,
    DRIVE_MODE_DIAG_MENU,
    ACK_MODE_DIAG_MENU,
    NETWORK_TOKEN_MENU,
} menu_names_t;

typedef enum
{
    SINGLE_LONG_BEEP    = 0,
    SINGLE_SHORT_BEEP   = 1,
    DOUBLE_LONG_BEEP    = 2,
    DOUBLE_SHORT_BEEP   = 3,
    TRIPLE_LONG_BEEP    = 4,
    TRIPLE_SHORT_BEEP   = 5,
} notifications_t;

/*****PUBLIC FUNCTIONS************************************************/
uint8_t init_user_interface(void);
menu_names_t ui_get_menu(void);
bool ui_tx_trigger_rd(void);
uint32_t ui_get_tx_interval(void);
void ui_display_msg_record(msg_record_t *record);
void ui_display_gps_diagnostics(gps_fix_t* fix_ptr);
void ui_display_sensor_diagnostics(bme280_data_t* env_ptr, uint16_t lux);
void ui_display_network_diagnostics(llabs_network_info_t* net_info, uint32_t ll_state);
void ui_display_gw_info(llabs_network_info_t* net_info, uint32_t ll_state);
void ui_display_dl_msg(uint8_t* msg_buf, uint8_t msg_len);
void ui_refresh_display(void);
void ui_notify_user(notifications_t beep_type);
void ui_register_wipe_callback(void (*hook) (void));
void ui_register_uart_pass_callbacks(void (*h_set) (bool), bool (*h_get) (void));
void ui_register_drive_mode_callbacks(bool (*h_set) (bool), bool (*h_get) (void));
void ui_register_ack_mode_callbacks(bool (*h_set) (bool), bool (*h_get) (void));
void ui_uart_passthru_force(void);
/*********************************************************************/
#endif /* UI_TASK_H_INCLUDED */
