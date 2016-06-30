/*********************************************************************/
/*********************************************************************/
//
// \file    supervisor.c
// \brief   Supervisor Task
// \author  Mark Bloechl
// \author  Thomas Steinholz
// \version 0.0.1
//
// \copyright LinkLabs, 2015
//
/*********************************************************************/
/*****INCLUDES********************************************************/
//-----Standard Libraries-----//
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
//-----EFM Libraries-----//
#include "em_chip.h"
#include "em_gpio.h"
#include "gpiointerrupt.h"
//-----My Libraries-----//
#include "osp.h"
#include "gps_task.h"
#include "ui_task.h"
#include "sensor_task.h"
#include "bsp.h"
#include "bsp_uart.h"
#include "bsp_watchdog.h"
#include "bsp_timer.h"
#include "iomap.h"
#include "ll_ifc.h"
#include "ll_ifc_symphony.h"
#include "ll_ifc_consts.h"
#include "supervisor.h"
#include "debug_print.h"
//-----FreeRTOS Libraries-----//
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "task_mgmt.h"
#include "queue.h"
#include "timers.h"
/*********************************************************************/

#define MIN_VERSION_MAJOR   (1)
#define MIN_VERSION_MINOR   (1)
#define MIN_VERSION_TAG     (0)


/*****DEFINES*********************************************************/
#define SUP_STATUS_TIMEOUT_TICKS     (1000 / portTICK_RATE_MS)
#define SUP_TIMER_TIMEOUT_TICKS     (100 / portTICK_RATE_MS)
#define SUP_QUEUE_TIMEOUT_TICKS     (250 / portTICK_RATE_MS)
#define SUP_TASK_PRIORITY           (tskIDLE_PRIORITY + 1)
#define SUP_TASK_STACK_SIZE         (300u)
#define SUP_TASK_QUEUE_SIZE         (8u)

#define WIPE_MODULE                         true    // set to true to wipe module settings on startup
#define GW_POLL_PERIOD_MS                   500
#define MSG_RECORD_LENGTH                   3
#define MODULE_RESET_TIME_S                 3
#define MAX_MSG_COUNT                       64
#define DISCONNECTED_RESET_INTERVAL_S       50
#define DISCONNECTED_RESET_INTERVAL_TICKS   (DISCONNECTED_RESET_INTERVAL_S*1000/portTICK_PERIOD_MS)

#define NET_TOKEN_OPEN         (0x4f50454e)
#define APP_TOKEN_LEVEREGE     0xd6, 0x84, 0x2f, 0x77, 0x4f, 0xe1, 0x7d, 0x8d, 0x0e, 0x81

#define LLABS_DL_CHAN_SPACING             (526595)
/*********************************************************************/


/*****TYPEDEFS/STRUCTS************************************************/
typedef struct ll_vars
{
    ll_version_t         fw_version;
    llabs_network_info_t net_info;
    uint64_t             mac_address;
    enum ll_state        state;
} ll_vars_t;

typedef enum
{
    SUP_STATE_LL_UNINITIALIZED,
    SUP_STATE_LL_AFTER_RESET,
    SUP_STATE_LL_INITIALIZING,
    SUP_STATE_LL_DISCONNECTED,
    SUP_STATE_LL_IDLE,
    SUP_STATE_LL_TX,
    SUP_STATE_UART_PASSTHRU,
    NUM_SUP_STATES
} supervisor_state_t;


typedef enum sup_task_command
{
    SUP_CMD_LL_IRQ_FLAG_SET,
    SUP_CMD_LL_WAS_RESET,
    SUP_CMD_LL_CONNECTED,
    SUP_CMD_LL_DISCONNECTED,
    SUP_CMD_LL_TX_DONE,
    SUP_CMD_LL_RX_DONE,
    SUP_CMD_LL_ERASE_FLASH,
    SUP_CMD_STATUS_TIMEOUT,
    SUP_CMD_UART_PASSTHRU,
    SUP_CMD_LL_DL_BAND_CONFIG,
    NUM_SUP_EVENTS
} sup_task_event_t;

typedef struct sup_queue_msg
{
    sup_task_event_t event;
    uint8_t *DataPtr;
    uint8_t  Data;
    uint16_t DataSize;
} sup_queue_msg_t;


#ifdef DEBUG
// For debug - printing state transitions
static const char* sup_state_strings[NUM_SUP_STATES] = {
        "LL_UNINITIALIZED",
        "LL_AFTER_RESET",
        "LL_INITIALIZING",
        "LL_DISCONNECTED",
        "LL_IDLE",
        "LL_TX",
        "UART_PASS",
};

static const char* sup_cmd_strings[NUM_SUP_EVENTS] = {
        "LL_IRQ_FLAG_SET",
        "LL_RESET",
        "LL_CONNECTED",
        "LL_DISCONNECTED",
        "LL_TX_DONE",
        "LL_RX_DONE",
        "LL_ERASE_FLASH",
        "STATUS_TIMEOUT",
        "UART_PASS",
        "DL_BAND_CONFIG",
};

#endif

const llabs_dl_band_cfg_t DL_BAN_FCC  = { 902000000, 928000000, 386703, 3, 0 }; // USA / Mexico
const llabs_dl_band_cfg_t DL_BAN_BRA  = { 916000000, 928000000, 386703, 3, 0 }; // Brazil
const llabs_dl_band_cfg_t DL_BAN_AUS  = { 918000000, 926000000, 386703, 3, 0 }; // Australia
const llabs_dl_band_cfg_t DL_BAN_NZL  = { 921000000, 928000000, 386703, 3, 0 }; // New Zealand
const llabs_dl_band_cfg_t DL_BAN_ETSI = { 869100000, 871000000, 386703, 1, 0 }; // Europe
/*********************************************************************/
/*****CONSTANTS*******************************************************/

/*********************************************************************/
/*****VARIABLES*******************************************************/
static uint8_t          s_app_token[] = {APP_TOKEN_LEVEREGE};

// rtos variables
static xTaskHandle      s_sup_task_handle;
static wdg_handler_t    s_sup_task_wdg_handler;
static xQueueHandle     s_sup_task_queue;
static TimerHandle_t    s_status_timer;

// ll variables
static ll_vars_t s_ll_vars = {
    .mac_address = 0,
};

// supervisor task variables
static supervisor_state_t   s_state;
static TickType_t           s_last_tx_tick;
static TickType_t           s_last_rst_tick;
static uint8_t              s_msg_count = 0;
static msg_record_t         s_msg_record[MSG_RECORD_LENGTH] = {{0,MSG_INIT},{0,MSG_INIT},{0,MSG_INIT}};
static uint8_t              s_dl_msg_buf[128];
static uint8_t              s_dl_msg_len;
static bool                 s_is_drive_mode = false;
static bool                 s_ack_mode = true;
/*********************************************************************/


/*****PRIVATE FUNCTION PROTOTYPES*************************************/
static portTASK_FUNCTION_PROTO(supervisor_task, param);
static void sup_state_set(supervisor_state_t new_state);
static supervisor_state_t sup_state_get(void);
/*********************************************************************/


/*****PRIVATE FUNCTIONS***********************************************/
void gpio_callback_irq_flag(void)
{
    BaseType_t  xHigherPriorityTaskWoken;
    BaseType_t ret;
    sup_queue_msg_t msg;
    msg.event = SUP_CMD_LL_IRQ_FLAG_SET;
    ret = xQueueSendFromISR(s_sup_task_queue, &msg, &xHigherPriorityTaskWoken);
    EFM_ASSERT(pdPASS == ret);

    // Did sending to the queue unblock a higher priority task?
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Called from button task context
void sup_eraseflash_callback(void)
{
    sup_queue_msg_t msg;
    msg.event = SUP_CMD_LL_ERASE_FLASH;
    BaseType_t ret = xQueueSend(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS);
    EFM_ASSERT(pdPASS == ret);
}

void sup_uart_pass_set_callback(bool enable)
{
    sup_queue_msg_t msg;
    if(enable)
    {
        msg.event = SUP_CMD_UART_PASSTHRU;
    }
    else
    {
        msg.event = SUP_CMD_LL_WAS_RESET;
    }

    BaseType_t ret = xQueueSend(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS);
    EFM_ASSERT(pdPASS == ret);
}

bool sup_uart_pass_get_callback(void)
{
    return (SUP_STATE_UART_PASSTHRU == sup_state_get());
}

bool sup_drive_mode_set_callback(bool enable)
{
    sup_queue_msg_t msg;
    bool ret = false;
    if(enable)
    {
        // can only enable if currently connected to a GW
        if(LL_STATE_IDLE_CONNECTED == s_ll_vars.state)
        {
            msg.Data = 1; //enable
            ret = true;
        }
    }
    else
    {
        // always able to disable
        msg.Data = false; //disable
        ret = true;
    }

    if(ret)
    {
        msg.event = SUP_CMD_LL_DL_BAND_CONFIG;
        BaseType_t pd_ret = xQueueSend(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS);
        EFM_ASSERT(pdPASS == pd_ret);
    }

    return ret;
}

bool sup_drive_mode_get_callback(void)
{
    return s_is_drive_mode;
}

bool sup_ack_mode_set_callback(bool enable)
{
    s_ack_mode = enable;
}

bool sup_ack_mode_get_callback(void)
{
    return s_ack_mode;
}

void sup_uart_user_rx_callback(char byte)
{
    if(SUP_STATE_UART_PASSTHRU == sup_state_get())
    {
        bsp_uart_module_tx((uint8_t*) &byte, 1);
    }
}

void sup_ll_bypass_rx_callback(char byte)
{
    if(SUP_STATE_UART_PASSTHRU == sup_state_get())
    {
        bsp_uart_user_tx((uint8_t*) &byte, 1);
    }
}

void sup_status_timer_callback(TimerHandle_t xTimer)
{
    (void) xTimer;

    uint32_t num_spaces_available = uxQueueSpacesAvailable(s_sup_task_queue);
    sup_queue_msg_t msg;
    if(num_spaces_available > 1)
    {
        msg.event = SUP_CMD_STATUS_TIMEOUT;
    }
    else
    {
        // clear queue content
        xQueueReset(s_sup_task_queue);
        msg.event = SUP_CMD_LL_DISCONNECTED;
    }

    BaseType_t  ret = xQueueSend(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS);
    EFM_ASSERT(pdPASS == ret);
}

static int32_t sup_tx_msg_build(menu_names_t active_menu, uint8_t* tmp_buf)
{
    if(GPS_MENU == active_menu)
    {
        // load gps data into buffer
        gps_build_packet(tmp_buf, s_msg_count);
    }
    else if(SENSOR_MODE_MENU == active_menu)
    {
        // load sensor data into buffer
        sensor_build_packet(tmp_buf, s_msg_count);
    }
    else
    {
        return -1;
    }

    return 0;
}

void sup_tx_record_shift()
{
    // Shift tx history by one placement
    uint32_t i;
    for(i=0; i < (MSG_RECORD_LENGTH-1); i++)
    {
        s_msg_record[i] = s_msg_record[i+1];
        if(MSG_WAITING_FOR_ACK == s_msg_record[i].acked)
        {
            // Force error if no response TX response on previous message (bug?)
            s_msg_record[i].acked = MSG_ERROR;
            Debug_Printf("No response from last Tx.\n");
        }
    }

    // Increment message count here
    s_msg_count++;
    s_msg_count %= MAX_MSG_COUNT;

    // add latest transmission to display record
    s_msg_record[MSG_RECORD_LENGTH-1].msg = s_msg_count;
    s_msg_record[MSG_RECORD_LENGTH-1].acked = MSG_WAITING_FOR_ACK;
}

static int32_t sup_msg_attempt_transmit(menu_names_t active_menu)
{
    int32_t ret;
    uint8_t tmp_bfr[10];

    if(sup_tx_msg_build(active_menu, tmp_bfr) < 0)
    {
        return -1;
    }

    // Latch time
    s_last_tx_tick = xTaskGetTickCount();

    // Transmit message
    if (s_ack_mode)
    {
        ret = ll_packet_send_ack(tmp_bfr, sizeof(tmp_bfr));
    }
    else
    {
        ret = ll_packet_send_unack(tmp_bfr, sizeof(tmp_bfr));
    }

    // Update msg display record if
    if(ret >= 0)
    {
        sup_tx_record_shift();
    }

    // Update message history display
    ui_display_msg_record(s_msg_record);

    return ret;
}

// Get message based on which menu is active
static bool sup_next_msg_send(void)
{
    menu_names_t active_menu = ui_get_menu();
    bool ret = sup_msg_attempt_transmit(active_menu) >= 0;
    return ret;
}

// check transmit mode, if we're supposed to transmit on demand, then check to see if a tx is requested
// otherwise, check to see if the correct amount of time has elapsed.
static bool sup_need_to_tx(void)
{
    bool ret = false;
    if((ui_get_tx_interval() == UI_TX_ON_DEMAND))
    {
        if(ui_tx_trigger_rd() == true)
        {
            ret = true;
        }
    }
    else
    {
        uint32_t seconds_since_last_tx = ((uint32_t)(xTaskGetTickCount() - s_last_tx_tick))*portTICK_PERIOD_MS/1000;
        if((seconds_since_last_tx > ui_get_tx_interval()) || ui_tx_trigger_rd())    // if enough time has elapsed (or someone's requested one) kick off a TX
        {
            ret = true;
        }
    }

    return ret;
}

void sup_report_status(void)
{
    int32_t ret;
    // poll for state
    ret = ll_get_state(&s_ll_vars.state, NULL, NULL);
    EFM_ASSERT(ret >= 0);


    ret = ll_net_info_get(&s_ll_vars.net_info);
    EFM_ASSERT(ret >= 0);
    ui_display_gw_info(&s_ll_vars.net_info, s_ll_vars.state);              // display gateway rssi bar (if appropriate)
    ui_display_network_diagnostics(&s_ll_vars.net_info, s_ll_vars.state);  // display network diagnostics (if appropriate)
}

static supervisor_state_t sup_state_get(void)
{
    return s_state;
}

static void sup_state_set(supervisor_state_t new_state)
{
    Debug_Printf("\n(%d ms) SUP State Change %s => %s\n\n", xTaskGetTickCount()*portTICK_RATE_MS, sup_state_strings[s_state], sup_state_strings[new_state]);

    //set the state
    s_state = new_state;
}

static void sup_wait_after_reset(uint32_t millisecs)
{
    uint32_t i;
    const uint32_t delay = 250; //millisecs
    uint32_t num_cycles = millisecs / delay;

    // Beep
    ui_notify_user(TRIPLE_LONG_BEEP);

    // Blink and delay
    for(i=0; i<num_cycles; i++)
    {
        GW_CONNECTED_TOGGLE();
        vTaskDelay(delay / portTICK_PERIOD_MS);
        wdg_refresh(s_sup_task_wdg_handler);
    }
    GW_CONNECTED_LED(OFF);
}

// Should be called only from SUP task context
static void sup_check_irq_flags()
{
    // Get flags over host ifc
    uint32_t flags = 0;
    int32_t ret = ll_irq_flags(0xffffffff, &flags);
    EFM_ASSERT(ret >= 0);

    Debug_Printf("IRQ_FLAGS = %08X\n", flags);

    // Local vars
    BaseType_t  pdRet;
    sup_queue_msg_t msg;

    // Process Flags
    if(flags & IRQ_FLAGS_WDOG_RESET)
    {
        EFM_ASSERT(0);
    }
    if(flags & IRQ_FLAGS_RESET)
    {
        msg.event = SUP_CMD_LL_WAS_RESET;
        pdRet = xQueueSend(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS);
        EFM_ASSERT(pdPASS == pdRet);
    }
    if(flags & IRQ_FLAGS_TX_DONE)
    {
        msg.event = SUP_CMD_LL_TX_DONE;
        msg.Data = 1; //OK
        pdRet = xQueueSend(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS);
        EFM_ASSERT(pdPASS == pdRet);
    }
    if(flags & IRQ_FLAGS_TX_ERROR)
    {
        msg.event = SUP_CMD_LL_TX_DONE;
        msg.Data = 0; //ERROR
        pdRet = xQueueSend(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS);
        EFM_ASSERT(pdPASS == pdRet);
    }
    if(flags & IRQ_FLAGS_RX_DONE)
    {
        msg.event = SUP_CMD_LL_RX_DONE;
        pdRet = xQueueSend(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS);
        EFM_ASSERT(pdPASS == pdRet);
    }
    if(flags & IRQ_FLAGS_CONNECTED)
    {
        msg.event = SUP_CMD_LL_CONNECTED;
        pdRet = xQueueSend(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS);
        EFM_ASSERT(pdPASS == pdRet);
    }
    if(flags & IRQ_FLAGS_DISCONNECTED)
    {
        msg.event = SUP_CMD_LL_DISCONNECTED;
        pdRet = xQueueSend(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS);
        EFM_ASSERT(pdPASS == pdRet);
    }
    if((flags & IRQ_FLAGS_APP_TOKEN_ERROR) || (flags & IRQ_FLAGS_CRYPTO_ERROR))
    {
        // TODO
    }
}

void sup_ll_module_reset(bool do_wait)
{
    // Stop status timer
    BaseType_t pd_ret = xTimerStop(s_status_timer, SUP_TIMER_TIMEOUT_TICKS);
    EFM_ASSERT(pdPASS == pd_ret);

    // clear queue content
    xQueueReset(s_sup_task_queue);

    // Reset the module
    MODULE_RST(1);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    MODULE_RST(0);

     // Wait a little
    if(do_wait)
    {
        sup_wait_after_reset(MODULE_RESET_TIME_S * 1000);
    }
}

// Erases persistent settings from module
static void sup_flash_erase(void)
{
    // Stop status timer
    BaseType_t pd_ret = xTimerStop(s_status_timer, SUP_TIMER_TIMEOUT_TICKS);
    EFM_ASSERT(pdPASS == pd_ret);

    // Delete settings
    Debug_Printf("ERASING MODULE FLASH SETTINGS...\n");
    int32_t ret = ll_settings_delete();
    EFM_ASSERT(ret >= 0);

    //TODO :this shouldn't be necessary
    sup_wait_after_reset(MODULE_RESET_TIME_S * 1000);
}

// Determines drive mode based on previously stored DL band config
// stored on the module
//
// Note: Workaround - until the network tester has persistent storage
static bool sup_drive_mode_init(void)
{
    int ret;
    llabs_dl_band_cfg_t cfg = {0};

    ret = ll_dl_band_cfg_get(&cfg);
    if (ret < 0)
    {
        // Assume default band cfg
        return false;
    }
    else
    {
        return cfg.chan_step_size == 100 && cfg.chan_step_offset < 50;
    }
}

static void sup_dl_band_config(bool is_single_channel)
{
    // load the dl band cfg
    llabs_dl_band_cfg_t cfg = {0};
    int32_t ret = ll_dl_band_cfg_get(&cfg);
    EFM_ASSERT(ret >= 0);

    // get module's connection info
    ret = ll_net_info_get(&s_ll_vars.net_info);
    EFM_ASSERT(ret >= 0);

    cfg.chan_step_offset = is_single_channel ? s_ll_vars.net_info.gateway_channel : 0;
    cfg.chan_step_size   = is_single_channel ? 100 : cfg.chan_step_size;

    // set new dl band in module
    ret = ll_dl_band_cfg_set(&cfg);
    EFM_ASSERT(ret >= 0);

    // save status
    s_is_drive_mode = is_single_channel;
}

// Retrieve the received message from the module and pass to ...
static void sup_message_retrieve(void)
{
    int32_t ret = ll_retrieve_message(s_dl_msg_buf, &s_dl_msg_len, NULL, NULL);
    EFM_ASSERT(ret >= 0);

    if(s_dl_msg_len > 0)
    {
        ui_display_dl_msg(s_dl_msg_buf, s_dl_msg_len);
        ui_notify_user(DOUBLE_LONG_BEEP);
    }
    else
    {
        Debug_Print("RX message length ZERO.\n");
        EFM_ASSERT(0);
    }
}

static void sup_tx_done(bool is_error)
{
    s_msg_record[MSG_RECORD_LENGTH-1].msg = s_msg_count;
    s_msg_record[MSG_RECORD_LENGTH-1].acked = is_error ? MSG_ERROR : MSG_ACKED;
    ui_display_msg_record(s_msg_record);

    MESSAGE_SENT_LED(OFF);
}

// Reset the module and wait for the reset IRQ flag
static void sup_ll_uninitialized_state()
{
    // Module IRQ Flag interrupt pin
    GPIO_IntConfig(gpioPortD, 2, true, false, true); //set falling edge interrupt
    NVIC_SetPriority(GPIO_EVEN_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn); // Enable interrupt in core for even and odd gpio interrupts
    NVIC_EnableIRQ(GPIO_EVEN_IRQn);

    // Register callback functions and enable interrupts
    GPIOINT_CallbackRegister(2, gpio_callback_irq_flag);
    GPIO_IntEnable(1<<2);

    // Reset the module
    sup_ll_module_reset(true);

    // Can't leave this state until the module has set the RESET IRQ flag or pass thru is selected
    bool b_exit_state = false;
    sup_queue_msg_t msg;
    supervisor_state_t next_state = SUP_STATE_LL_UNINITIALIZED;
    while(!b_exit_state)
    {
        wdg_refresh(s_sup_task_wdg_handler);

        if (xQueueReceive(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS))
        {
            Debug_Printf("(%d ms) sup_task CMD: %s in state %s\n", xTaskGetTickCount()*portTICK_RATE_MS, sup_cmd_strings[msg.event], sup_state_strings[sup_state_get()]);

            if (SUP_CMD_LL_WAS_RESET == msg.event)
            {
                b_exit_state = true;
                next_state = SUP_STATE_LL_AFTER_RESET;
            }
            else if (SUP_CMD_LL_ERASE_FLASH == msg.event)
            {
                sup_flash_erase();
            }
            else if (SUP_CMD_STATUS_TIMEOUT == msg.event)
            {
                // Do nothing
            }
            else if (SUP_CMD_LL_IRQ_FLAG_SET == msg.event)
            {
                sup_check_irq_flags();
            }
            else if (SUP_CMD_UART_PASSTHRU == msg.event)
            {
                b_exit_state = true;
                next_state = SUP_STATE_UART_PASSTHRU;
            }
            else
            {
                Debug_Print("IGNORED.\n");
                EFM_ASSERT(false);
            }
        }
    }

    // Set next state
    sup_state_set(next_state);
}

bool sup_is_fw_ok(ll_version_t* fw)
{
    uint32_t min_ver = (MIN_VERSION_MAJOR << 24) | (MIN_VERSION_MINOR << 16) | MIN_VERSION_TAG;
    uint32_t module_ver = (fw->major << 24) | (fw->minor << 16) | (fw->tag);

    return (module_ver >= min_ver);
}

// Get module's basic info and pass to configuration state
static void sup_ll_after_reset_state()
{
    int32_t ret = 0;
    supervisor_state_t next_state = SUP_STATE_LL_UNINITIALIZED;

    // set GW LED off
    GW_CONNECTED_LED(OFF);

    // Get module ID
    ret = ll_unique_id_get(&s_ll_vars.mac_address);
    EFM_ASSERT(ret >= 0);

    // Check MAC mode
    ll_mac_type_t mac_mode;
    ret = ll_mac_mode_get(&mac_mode);
    EFM_ASSERT(ret >= 0);
    if(mac_mode != SYMPHONY_LINK)
    {
        // set to symphony mode
        ret = ll_mac_mode_set(SYMPHONY_LINK);
        EFM_ASSERT(ret >= 0);
    }

    // Get firmware version
    ll_version_t fw_ver;
    ret = ll_version_get(&fw_ver);
    EFM_ASSERT(ret >= 0);
    if(!sup_is_fw_ok(&fw_ver))
    {
        //report to ui
        Debug_Printf("Firmware Version is too old: v%d.%d.%d\n", fw_ver.major, fw_ver.minor, fw_ver.tag);
        Debug_Printf("SUP: Need FW upgrade to v%d.%d.%d\n", MIN_VERSION_MAJOR, MIN_VERSION_MINOR, MIN_VERSION_TAG);
        Debug_Printf("SUP: Entering INFINITE loop\n");
        ui_uart_passthru_force();
        next_state = SUP_STATE_UART_PASSTHRU;
    }
    else
    {
        // Set drive mode by reading dl_band_cfg
        s_is_drive_mode = sup_drive_mode_init();

        // Set DL Band config to default
        // sup_dl_band_config(s_is_drive_mode);

        // update display with module info
        ui_refresh_display();

        // Change state
        next_state = SUP_STATE_LL_INITIALIZING;

        // Start status timer
        BaseType_t pd_ret = xTimerStart(s_status_timer, SUP_TIMER_TIMEOUT_TICKS);
        EFM_ASSERT(pdPASS == pd_ret);
    }

    // Set next state
    sup_state_set(next_state);
}

// Making sure the module's configuration is properly set
static void sup_initializing_state()
{
    GW_CONNECTED_LED(OFF);

    // Set module configuration
    uint8_t app_token[APP_TOKEN_LEN], qos;
    enum ll_downlink_mode dl_mode;
    uint32_t net_token;
    int32_t ret = ll_config_get(&net_token, app_token, &dl_mode, &qos);
    EFM_ASSERT(ret >= 0);
    ret = ll_config_set(net_token, s_app_token, LL_DL_ALWAYS_ON, 0);
    EFM_ASSERT(ret >= 0);

    // Stay here until state is satisfied
    sup_queue_msg_t msg;
    supervisor_state_t next_state = SUP_STATE_LL_UNINITIALIZED;
    bool b_exit_state = false;
    while(!b_exit_state)
    {
        wdg_refresh(s_sup_task_wdg_handler);

        if (xQueueReceive(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS))
        {
            Debug_Printf("(%d ms) sup_task CMD: %s in state %s\n", xTaskGetTickCount()*portTICK_RATE_MS, sup_cmd_strings[msg.event], sup_state_strings[sup_state_get()]);

            if (SUP_CMD_LL_WAS_RESET == msg.event)
            {
                b_exit_state = true;
                next_state = SUP_STATE_LL_AFTER_RESET;
            }
            else if (SUP_CMD_LL_CONNECTED == msg.event)
            {
                // ignore
            }
            else if (SUP_CMD_LL_DISCONNECTED == msg.event)
            {
                b_exit_state = true;
                next_state = SUP_STATE_LL_INITIALIZING;
            }
            else if (SUP_CMD_LL_ERASE_FLASH == msg.event)
            {
                sup_flash_erase();
            }
            else if (SUP_CMD_STATUS_TIMEOUT == msg.event)
            {
                sup_report_status();
            }
            else if (SUP_CMD_LL_IRQ_FLAG_SET == msg.event)
            {
                sup_check_irq_flags();
            }
            else if (SUP_CMD_UART_PASSTHRU == msg.event)
            {
                b_exit_state = true;
                next_state = SUP_STATE_UART_PASSTHRU;
            }
            else if (SUP_CMD_LL_DL_BAND_CONFIG == msg.event)
            {
                sup_dl_band_config(false);
                b_exit_state = true;
                next_state = SUP_STATE_LL_INITIALIZING;
            }
            else if (SUP_CMD_LL_TX_DONE == msg.event)
            {
                // ignore
            }
            else
            {
                Debug_Print("IGNORED.\n");
                EFM_ASSERT(false);
            }
        }
        else
        {
            // poll for state
            ret = ll_get_state(&s_ll_vars.state, NULL, NULL);
            EFM_ASSERT(ret >= 0);

            if(LL_STATE_IDLE_CONNECTED == s_ll_vars.state)
            {
                b_exit_state = true;
                next_state = SUP_STATE_LL_IDLE;
            }
            else if (LL_STATE_INITIALIZING == s_ll_vars.state)
            {
                ret = ll_net_info_get(&s_ll_vars.net_info);
                EFM_ASSERT(ret >= 0);

                // Blink if initializing
                if(!s_ll_vars.net_info.is_scanning_gateways)
                {
                    GW_CONNECTED_TOGGLE();
                }
            }
            else if (LL_STATE_IDLE_DISCONNECTED == s_ll_vars.state)
            {
                Debug_Printf("No GWs found.  Trying again.\n");
                sup_ll_module_reset(true);
                b_exit_state = true;
                next_state = SUP_STATE_LL_INITIALIZING;
            }
            else if (LL_STATE_ERROR == s_ll_vars.state)
            {
                // Start from scratch
                Debug_Print("LL in error state.\n");
                sup_ll_module_reset(true);
                next_state = SUP_STATE_LL_INITIALIZING;
                b_exit_state = true;
            }
        }
    }

    // Set next state
    sup_state_set(next_state);
}

// Trying to reconnect
static void sup_disconnected_state()
{
    // Notify UI
    GW_CONNECTED_LED(OFF);
    ui_notify_user(TRIPLE_SHORT_BEEP);

    // go back to initialization
    supervisor_state_t next_state = SUP_STATE_LL_INITIALIZING;

    // Set next state
    sup_state_set(next_state);
}

// Ready to send/receive messages over Symphony
static void sup_idle_state()
{
    // Notify UI
    GW_CONNECTED_LED(ON);
    ui_notify_user(SINGLE_SHORT_BEEP);

    sup_queue_msg_t msg;
    supervisor_state_t next_state = SUP_STATE_LL_UNINITIALIZED;
    bool b_exit_state = false;
    while(!b_exit_state)
    {
        wdg_refresh(s_sup_task_wdg_handler);

        if (xQueueReceive(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS))
        {
            Debug_Printf("(%d ms) sup_task CMD: %s in state %s\n", xTaskGetTickCount()*portTICK_RATE_MS, sup_cmd_strings[msg.event], sup_state_strings[sup_state_get()]);

            if (SUP_CMD_LL_WAS_RESET == msg.event)
            {
                b_exit_state = true;
                next_state = SUP_STATE_LL_AFTER_RESET;
            }
            else if (SUP_CMD_LL_CONNECTED == msg.event)
            {
                b_exit_state = true;
                next_state = SUP_STATE_LL_IDLE;
            }
            else if (SUP_CMD_LL_DISCONNECTED == msg.event)
            {
                b_exit_state = true;
                next_state = SUP_STATE_LL_DISCONNECTED;
            }
            else if (SUP_CMD_LL_RX_DONE == msg.event)
            {
                sup_message_retrieve();
            }
            else if (SUP_CMD_LL_ERASE_FLASH == msg.event)
            {
                sup_flash_erase();
            }
            else if (SUP_CMD_STATUS_TIMEOUT == msg.event)
            {
                sup_report_status();
            }
            else if (SUP_CMD_LL_IRQ_FLAG_SET == msg.event)
            {
                sup_check_irq_flags();
            }
            else if (SUP_CMD_UART_PASSTHRU == msg.event)
            {
                b_exit_state = true;
                next_state = SUP_STATE_UART_PASSTHRU;
            }
            else if (SUP_CMD_LL_DL_BAND_CONFIG == msg.event)
            {
                sup_dl_band_config(msg.Data);
                b_exit_state = true;
                next_state = SUP_STATE_LL_INITIALIZING;
            }
            else if (SUP_CMD_LL_TX_DONE == msg.event)
            {
                // Ignore and stay in idle
            }
            else
            {
                Debug_Print("IGNORED.\n");
                EFM_ASSERT(false);
            }
        }
        else
        {
            // Get stuff to do
            if(sup_need_to_tx())
            {
                next_state = SUP_STATE_LL_TX;
                b_exit_state = true;
            }
        }
    }

    // Set next state
    sup_state_set(next_state);
}

// Send a message to the module.  Stays here until module reports TX_DONE or TX_ERROR.
static void sup_tx_state()
{
    supervisor_state_t next_state = SUP_STATE_LL_UNINITIALIZED;
    uint32_t check_count = 0;


    // Attempt to send message
    bool was_msg_sent = sup_next_msg_send();

    // Return if message was not sent
    if(!was_msg_sent)
    {
        MESSAGE_SENT_LED(OFF);
        next_state = SUP_STATE_LL_IDLE;
        return;
    }
    Debug_Printf("(%d ms) Mesage queued.\n", xTaskGetTickCount()*portTICK_RATE_MS);

    // Turn TX LED on
    MESSAGE_SENT_LED(ON);

    // Wait here until TX_DONE or TX_ERROR (or connected or disconnected or dl band config)
    sup_queue_msg_t msg;
    bool b_exit_state = false;
    while(!b_exit_state)
    {
        wdg_refresh(s_sup_task_wdg_handler);

        if (xQueueReceive(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS))
        {
            Debug_Printf("(%d ms) sup_task CMD: %s in state %s\n", xTaskGetTickCount()*portTICK_RATE_MS, sup_cmd_strings[msg.event], sup_state_strings[sup_state_get()]);

            if (SUP_CMD_LL_WAS_RESET == msg.event)
            {
                b_exit_state = true;
                next_state = SUP_STATE_LL_AFTER_RESET;
            }
            else if (SUP_CMD_LL_TX_DONE == msg.event)
            {
                bool is_error = !msg.Data;
                sup_tx_done(is_error);
                next_state = SUP_STATE_LL_IDLE;
                b_exit_state = true;
            }
            else if (SUP_CMD_LL_RX_DONE == msg.event)
            {
                sup_message_retrieve();
            }
            else if (SUP_CMD_LL_ERASE_FLASH == msg.event)
            {
                sup_flash_erase();
            }
            else if (SUP_CMD_STATUS_TIMEOUT == msg.event)
            {
                sup_report_status();
            }
            else if (SUP_CMD_LL_IRQ_FLAG_SET == msg.event)
            {
                sup_check_irq_flags();
            }
            else if (SUP_CMD_UART_PASSTHRU == msg.event)
            {
                b_exit_state = true;
                next_state = SUP_STATE_UART_PASSTHRU;
            }
            else if (SUP_CMD_LL_DL_BAND_CONFIG == msg.event)
            {
                sup_dl_band_config(msg.Data);
                b_exit_state = true;
                next_state = SUP_STATE_LL_INITIALIZING;
            }
            else if (SUP_CMD_LL_CONNECTED == msg.event)
            {
                sup_tx_done(true);
                next_state = SUP_STATE_LL_IDLE;
                b_exit_state = true;
            }
            else if (SUP_CMD_LL_DISCONNECTED == msg.event)
            {
                sup_tx_done(true);
                next_state = SUP_STATE_LL_DISCONNECTED;
                b_exit_state = true;
            }
            else
            {
                Debug_Print("IGNORED.\n");
                EFM_ASSERT(false);
            }
        }
        else
        {
            // poll for state
            int32_t ret = ll_get_state(&s_ll_vars.state, NULL, NULL);
            EFM_ASSERT(ret >= 0);

            if(LL_STATE_IDLE_CONNECTED != s_ll_vars.state)
            {
                b_exit_state = true;
                next_state = SUP_STATE_LL_INITIALIZING;
            }

            // check if stuck
            if(check_count > 120)
            {
                Debug_Printf("Tx Timeout.");
                sup_ll_module_reset(true);
                check_count = 0;
            }

            check_count++;
        }
    }

    // Set next state
    sup_state_set(next_state);
}

void sup_uart_pass_state(void)
{
    supervisor_state_t next_state = SUP_STATE_LL_UNINITIALIZED;

    // Turn off GW LED
    GW_CONNECTED_LED(OFF);

    // Enter UART bypass
    bsp_module_bypass_enable(true, sup_ll_bypass_rx_callback);

    // Wait here until module is reset
    sup_queue_msg_t msg;
    bool b_exit_state = false;
    while(!b_exit_state)
    {
        wdg_refresh(s_sup_task_wdg_handler);
        LED2_TOGGLE();

        if (xQueueReceive(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS/5))
        {
            Debug_Printf("(%d ms) sup_task CMD: %s in state %s\n", xTaskGetTickCount()*portTICK_RATE_MS, sup_cmd_strings[msg.event], sup_state_strings[sup_state_get()]);

            if (SUP_CMD_LL_WAS_RESET == msg.event)
            {
                LED2(OFF); // off
                b_exit_state = true;
                if(0 == s_ll_vars.mac_address)
                {
                    //never initialized
                    next_state = SUP_STATE_LL_UNINITIALIZED;
                }
                else
                {
                    next_state = SUP_STATE_LL_AFTER_RESET;
                }
            }
            else if (SUP_CMD_LL_IRQ_FLAG_SET == msg.event)
            {
                // do nothing
            }
            else if (SUP_CMD_STATUS_TIMEOUT == msg.event)
            {
                // do nothing
            }
            else if (SUP_CMD_LL_CONNECTED == msg.event)
            {
                // do nothing
            }
            else
            {
                Debug_Print("IGNORED.\n");
                EFM_ASSERT(false);
            }
        }
        else
        {
            // check if Rx bytes

            // check for Tx bytes
        }
    }

    // Exit UART bypass
    bsp_module_bypass_enable(false, NULL);

    // Set next state
    sup_state_set(next_state);
}

static portTASK_FUNCTION(supervisor_task, param)
{
    (void) param;
    supervisor_state_t curr_state;

    sup_state_set(SUP_STATE_LL_UNINITIALIZED);

    s_last_tx_tick  = xTaskGetTickCount();
    s_last_rst_tick = xTaskGetTickCount();
    while (true)
    {
        curr_state = sup_state_get();
        switch(curr_state)
        {
            case SUP_STATE_LL_UNINITIALIZED:
                sup_ll_uninitialized_state();
                break;
            case SUP_STATE_LL_AFTER_RESET:
                sup_ll_after_reset_state();
                break;
            case SUP_STATE_LL_INITIALIZING:
                sup_initializing_state();
                break;
            case SUP_STATE_LL_DISCONNECTED:
                sup_disconnected_state();
                break;
            case SUP_STATE_LL_IDLE:
                sup_idle_state();
                break;
            case SUP_STATE_LL_TX:
                sup_tx_state();
                break;
            case SUP_STATE_UART_PASSTHRU:
                sup_uart_pass_state();
                break;
            default:
                Debug_Printf("Unknown State: %d\n", curr_state);
                EFM_ASSERT(false);
                break;
       }

        wdg_refresh(s_sup_task_wdg_handler);
    }
}

/*****PUBLIC FUNCTIONS************************************************/
void sup_get_gw_status(llabs_network_info_t* gw_info_ptr)
{
    *gw_info_ptr = s_ll_vars.net_info;
}

bool sup_get_GW_rssi(int16_t* rssi_ptr)
{
    if(s_ll_vars.net_info.connection_status == LLABS_CONNECT_CONNECTED)
    {
        *rssi_ptr = s_ll_vars.net_info.rssi;
        return(true);
    }

    return(false);
}

uint64_t sup_get_MAC_address(void)
{
    return(s_ll_vars.mac_address);
}

uint8_t init_supervisor_task(void)
{
    // Register User UART receive callback
    register_usart0_rx_callback(sup_uart_user_rx_callback);

    // Register "Wipe" callback
    ui_register_wipe_callback(sup_eraseflash_callback);

    // Register UART Pass-though callbacks
    ui_register_uart_pass_callbacks(sup_uart_pass_set_callback, sup_uart_pass_get_callback);

    // Register "Drive Mode" callbacks
    ui_register_drive_mode_callbacks(sup_drive_mode_set_callback, sup_drive_mode_get_callback);

    // Register "Drive Mode" callbacks
    ui_register_ack_mode_callbacks(sup_ack_mode_set_callback, sup_ack_mode_get_callback);

    // Create Task
    if (pdPASS != xTaskCreate(supervisor_task, (const portCHAR *)"supervisor_task", SUP_TASK_STACK_SIZE, NULL, SUP_TASK_PRIORITY, &s_sup_task_handle))
    {
        return EXIT_FAILURE;
    }

    // Create Queue
    s_sup_task_queue = xQueueCreate(SUP_TASK_QUEUE_SIZE, sizeof(sup_queue_msg_t));
    if (NULL == s_sup_task_queue)
    {
        return false;
    }

    // Create Status Timer
    s_status_timer = xTimerCreate("status", SUP_STATUS_TIMEOUT_TICKS, pdTRUE, sup_status_timer_callback, sup_status_timer_callback);
    if (NULL == s_status_timer)
    {
        return false;
    }

    // Initialize Watchdog
    s_sup_task_wdg_handler = wdg_register("supervisor");
    if (WDG_HANDLER_ERROR == s_sup_task_wdg_handler)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
