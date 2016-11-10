#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include "em_chip.h"
#include "em_msc.h"
#include "em_gpio.h"
#include "gpiointerrupt.h"
#include "main.h"
#include "osp.h"
#include "gps_task.h"
#include "ui_task.h"
#include "sensor_task.h"
#include "bsp.h"
#include "bsp_uart.h"
#include "bsp_watchdog.h"
#include "bsp_timer.h"
#include "iomap.h"
#include "ll_ifc_ftp.h"
#include "ll_ifc_symphony.h"
#include "ll_ifc_consts.h"
#include "ll_ifc_xmodem.h"
#include "supervisor.h"
#include "debug_print.h"
#include "network_tester_version.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "task_mgmt.h"
#include "queue.h"
#include "timers.h"

#define MIN_VERSION_MAJOR   (1)
#define MIN_VERSION_MINOR   (4)
#define MIN_VERSION_TAG     (0)

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

#define FTP_PORT (128)
#define DL_BUFF_MAX_LEN (142)

#define NET_TOKEN_OPEN         (0x4f50454e)
#define NT_APP_TOKEN     0xd6, 0x84, 0x2f, 0x77, 0x4f, 0xe1, 0x7d, 0x8d, 0x0e, 0x81

#define LLABS_DL_CHAN_SPACING             (526595)

#define LL_FTP_NT_FIRMWARE_ADDR_START     (0x0006f000)
#define LL_FTP_MAX_NT_FIRMWARE_SIZE       (0x6d000)
#define LL_FTP_MODULE_FIRMWARE_ADDR_START (0x000e0000)
#define LL_FTP_MAX_MODULE_FIRMWARE_SIZE   (131072)

#define UINT16_FROM_BYTESTREAM(p)      (p[0] | (p[1] << 8))
#define UINT32_FROM_BYTESTREAM(p)      (p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24))

#define UINT32_TO_BYTESTREAM(dst, src) dst[0] = src & 0xFF; \
                                        dst[1] = (src >> 8) & 0xFF; \
                                        dst[2] = (src >> 16) & 0xFF; \
                                        dst[3] = (src >> 24) & 0xFF;

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
    SUP_STATE_LL_FTP,
    SUP_STATE_UART_PASSTHRU,
    SUP_STATE_MODULE_UPGRADE,
    SUP_STATE_SELF_UPGRADE,
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
    SUP_CMD_LL_START_FTP,
    SUP_CMD_LL_UPDATE_RLP,
    SUP_CMD_LL_UPDATE_SELF,
    SUP_CMD_LL_RLP_CRC,
    SUP_CMD_LL_BOOTLOADER,
    SUP_CMD_LL_RLP_XMODEM_SEND_FW,
    SUP_CMD_LL_UPDATE_DONE,
    SUP_CMD_LL_UPDATE_FAILED,
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
        "LL_FTP",
        "UART_PASS",
        "Module Upgrade",
        "Network Tester Upgrade",
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
        "LL_START_FTP",
        "LL_UPDATE_RLP",
        "LL_UPDATE_SELF",
        "LL_RLP_CRC",
        "LL_BOOTLOADER",
        "LL_RLP_XMODEM_SEND_FW",
        "LL_RLP_UPDATE_DONE",
        "LL_RLP_UPDATE_FAILED"
};

static const char* sup_ftp_strings[3] = {
    "Idle",
    "Segment",
    "Apply"
};

#endif

static uint8_t            s_app_token[] = {NT_APP_TOKEN};

// ftp variables
static ll_ftp_t           ftp;
static ll_ftp_callbacks_t ftp_callbacks;

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
static uint8_t              s_dl_msg_buf[DL_BUFF_MAX_LEN];
static uint8_t              s_dl_msg_len;
static uint8_t              s_dl_port;
static bool                 s_is_drive_mode = false;
static bool                 s_ack_mode = true;

static portTASK_FUNCTION_PROTO(supervisor_task, param);
static void sup_state_set(supervisor_state_t new_state);
static supervisor_state_t sup_state_get(void);

static ll_ftp_return_code_t ftp_open_callback(uint32_t file_id, uint32_t file_version,
       uint32_t file_size)
{
    uint32_t addr;

    if (LL_FTP_FIRMWARE_ID_MODULE == file_id)
    {
        if (LL_FTP_MAX_MODULE_FIRMWARE_SIZE < file_size)
        {
            return LL_FTP_ERROR;
        }

        addr = LL_FTP_MODULE_FIRMWARE_ADDR_START;
    }
    else if (LL_FTP_FIRMWARE_ID_NETTEST == file_id)
    {
        if (LL_FTP_MAX_NT_FIRMWARE_SIZE < file_size)
        {
            return LL_FTP_ERROR;
        }

        addr = LL_FTP_NT_FIRMWARE_ADDR_START;
    }
    else
    {
        // We do not support this firmware.
        return LL_FTP_NO_ACTION;
    }

    ftp_header_data_t header = { 0 };
    load_ftp_flash_vars((uint32_t*)addr, &header);

    if (header.version == file_version && header.id == file_id && header.size == file_size)
    {
        return LL_FTP_NO_ACTION;  // This file has already been downloaded.
    }

    taskENTER_CRITICAL();

    MSC_Init();

    for (uint32_t i = addr; i < (addr + file_size); i += FLASH_PAGE_SIZE)
    {
        MSC_ErasePage((uint32_t*)i);
        wdg_refresh(s_sup_task_wdg_handler);
    }

    taskEXIT_CRITICAL();

    ui_ftp_activate_download();

    ui_notify_user(SINGLE_SHORT_BEEP);
    return LL_FTP_OK;
}

static ll_ftp_return_code_t ftp_read_callback(uint32_t file_id, uint32_t file_version,
       uint32_t offset, uint8_t* payload, uint16_t len)
{
    (void) file_version;

    if (LL_FTP_FIRMWARE_ID_MODULE == file_id)
    {
        memcpy(payload, (void*)LL_FTP_MODULE_FIRMWARE_ADDR_START + offset, (size_t)len);
    }
    else if (LL_FTP_FIRMWARE_ID_NETTEST == file_id)
    {
        memcpy(payload, (void*)LL_FTP_NT_FIRMWARE_ADDR_START + offset, (size_t)len);
    }
    else
    {
        return LL_FTP_NO_ACTION;
    }

    return LL_FTP_OK;
}

static ll_ftp_return_code_t ftp_write_callback(uint32_t file_id, uint32_t file_version,
       uint32_t offset, uint8_t* payload, uint16_t len)
{
    (void) file_version;

    uint32_t addr;

    // Set the word alligned address based on the firmware type.
    if (LL_FTP_FIRMWARE_ID_MODULE == file_id)
    {
        addr = (LL_FTP_MODULE_FIRMWARE_ADDR_START + offset) & ~(0x3);
    }
    else if (LL_FTP_FIRMWARE_ID_NETTEST == file_id)
    {
        addr = (LL_FTP_NT_FIRMWARE_ADDR_START + offset) & ~(0x3);
    }
    else
    {
        // Do not write to flash, we do not support this firmware.
        return LL_FTP_NO_ACTION;
    }

    // Find how much data need to be copied to send buffer from flash.
    uint8_t used_data = (uint8_t)(offset - (addr - LL_FTP_MODULE_FIRMWARE_ADDR_START));

    // Find the proper length for the buffer to be alligned
    uint16_t buff_len = ((len + used_data) % 4 == 0)
        ? (len + used_data) : (len + used_data) + (4 - ((len + used_data) % 4));

    // Create the send buffer.
    uint8_t buff[buff_len];

    // Make sure there is no random memory in the buffer.
    memset(buff, 0xff, (size_t)buff_len);

    // Populate send buffer with current memory.
    memcpy((void*)buff, (void*)addr, (size_t)used_data);

    // Populate send buffer with payload.
    memcpy((void*)(buff + used_data), (void*)payload, (size_t)len);

    // Send the buffer to flash.
    taskENTER_CRITICAL();
    msc_Return_TypeDef msc_ret = MSC_WriteWord((uint32_t*)addr, buff, buff_len);
    taskEXIT_CRITICAL();

    // Check the staus of the write.
    if (msc_ret != mscReturnOk)
    {
        return LL_FTP_ERROR;
    }

    // Update UI
    ui_display_ftp_download_status(&ftp);

    // Tell Fota the write operation went well.
    return LL_FTP_OK;
}

static ll_ftp_return_code_t ftp_close_callback(uint32_t file_id, uint32_t file_version)
{
    (void) file_version;

    if (LL_FTP_FIRMWARE_ID_MODULE == file_id || LL_FTP_FIRMWARE_ID_NETTEST == file_id)
    {
        taskENTER_CRITICAL();
        MSC_Deinit();
        taskEXIT_CRITICAL();
    }

    return LL_FTP_OK;
}

static ll_ftp_return_code_t ftp_apply_callback(uint32_t file_id, uint32_t file_version, uint32_t file_size)
{
    (void) file_version;
    (void) file_size;

    sup_queue_msg_t msg;

    if (LL_FTP_FIRMWARE_ID_MODULE == file_id)
    {
        msg.event = SUP_CMD_LL_UPDATE_RLP;
    }
    else if (LL_FTP_FIRMWARE_ID_NETTEST == file_id)
    {
        msg.event = SUP_CMD_LL_UPDATE_SELF;
    }
    else
    {
        // Do not write to flash, we do not support this firmware.
        return LL_FTP_NO_ACTION;
    }

    BaseType_t ret = xQueueSend(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS);
    return pdPASS == ret ? LL_FTP_OK : LL_FTP_ERROR;
}

static ll_ftp_return_code_t ftp_send_uplink_callback(uint8_t* buf, uint8_t len, bool acked, uint8_t port)
{
    int32_t ret = ll_message_send(buf, len, acked, port);
    return ret == 0 ? LL_FTP_OK : LL_FTP_ERROR;
}

static ll_ftp_return_code_t ftp_dl_config_callback(bool downlink_on)
{
    uint8_t app_token[APP_TOKEN_LEN], qos;
    enum ll_downlink_mode dl_mode;
    uint32_t net_token;

    if (LL_FTP_OK != (ll_ftp_return_code_t) ll_config_get(&net_token, app_token, &dl_mode, &qos))
    {
        return LL_FTP_ERROR;
    }

    dl_mode = (downlink_on) ? LL_DL_ALWAYS_ON : LL_DL_MAILBOX;

    return (ll_ftp_return_code_t) ll_config_set(net_token, s_app_token, dl_mode, 0);
}

static int32_t ll_xmodem_progress_callback(uint32_t sent, uint32_t total)
{
    wdg_refresh(s_sup_task_wdg_handler);
    ui_progress_bar(3, sent, total);
    return 0;
}

void gpio_callback_irq_flag(void)
{
    BaseType_t  xHigherPriorityTaskWoken;
    BaseType_t ret;
    sup_queue_msg_t msg;
    msg.event = SUP_CMD_LL_IRQ_FLAG_SET;
    ret = xQueueSendFromISR(s_sup_task_queue, &msg, &xHigherPriorityTaskWoken);
    LL_ASSERT(pdPASS == ret);

    // Did sending to the queue unblock a higher priority task?
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Called from button task context
void sup_eraseflash_callback(void)
{
    sup_queue_msg_t msg;
    msg.event = SUP_CMD_LL_ERASE_FLASH;
    BaseType_t ret = xQueueSend(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS);
    LL_ASSERT(pdPASS == ret);
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
    LL_ASSERT(pdPASS == ret);
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
        LL_ASSERT(pdPASS == pd_ret);
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
    LL_ASSERT(pdPASS == ret);
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

    if (128 == s_dl_port)
    {
        return ll_message_send(s_dl_msg_buf, s_dl_msg_len, s_ack_mode, s_dl_port);
    }
    else
    {
        uint8_t tmp_bfr[10];

        if(sup_tx_msg_build(active_menu, tmp_bfr) < 0)
        {
            return -1;
        }

        // Latch time
        s_last_tx_tick = xTaskGetTickCount();

        // Transmit message
        ret = ll_message_send(tmp_bfr, sizeof(tmp_bfr), true, s_ack_mode);

        // Update msg display record if
        if(ret >= 0)
        {
            sup_tx_record_shift();
        }

        // Update message history display
        ui_display_msg_record(s_msg_record);

        return ret;
    }
}

// Get message based on which menu is active
static bool sup_next_msg_send(void)
{
    menu_names_t active_menu = ui_get_menu();
    return sup_msg_attempt_transmit(active_menu) >= 0;
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
    LL_ASSERT(ret >= 0);

    ret = ll_net_info_get(&s_ll_vars.net_info);
    LL_ASSERT(ret >= 0);
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
    LL_ASSERT(ret >= 0);

    Debug_Printf("IRQ_FLAGS = %08X\n", flags);

    // Local vars
    BaseType_t  pdRet;
    sup_queue_msg_t msg;

    // Process Flags
    if(flags & IRQ_FLAGS_WDOG_RESET)
    {
        LL_ASSERT(0);
    }
    if(flags & IRQ_FLAGS_RESET)
    {
        msg.event = SUP_CMD_LL_WAS_RESET;
        pdRet = xQueueSend(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS);
        LL_ASSERT(pdPASS == pdRet);
    }
    if(flags & IRQ_FLAGS_TX_DONE)
    {
        msg.event = SUP_CMD_LL_TX_DONE;
        msg.Data = 1; //OK
        pdRet = xQueueSend(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS);
        LL_ASSERT(pdPASS == pdRet);
    }
    if(flags & IRQ_FLAGS_TX_ERROR)
    {
        msg.event = SUP_CMD_LL_TX_DONE;
        msg.Data = 0; //ERROR
        pdRet = xQueueSend(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS);
        LL_ASSERT(pdPASS == pdRet);
    }
    if(flags & IRQ_FLAGS_RX_DONE)
    {
        msg.event = SUP_CMD_LL_RX_DONE;
        pdRet = xQueueSend(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS);
        LL_ASSERT(pdPASS == pdRet);
    }
    if(flags & IRQ_FLAGS_CONNECTED)
    {
        msg.event = SUP_CMD_LL_CONNECTED;
        pdRet = xQueueSend(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS);
        LL_ASSERT(pdPASS == pdRet);
    }
    if(flags & IRQ_FLAGS_DISCONNECTED)
    {
        msg.event = SUP_CMD_LL_DISCONNECTED;
        pdRet = xQueueSend(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS);
        LL_ASSERT(pdPASS == pdRet);
    }
    if((flags & IRQ_FLAGS_APP_TOKEN_ERROR) || (flags & IRQ_FLAGS_CRYPTO_ERROR))
    {
        // TODO
    }
}

void sup_ll_module_bootloader_mode()
{
    ui_log("Preparing Module...");
    int32_t ret = ll_xmodem_prepare_module(true);
    if (ret < 0)
    {
        ui_log("FAILED! (%i)", ret);
    }
    LL_ASSERT(ret >= 0);
    ui_log("               done.");
    ui_log("Module is ready.");

    sup_queue_msg_t msg;
    msg.event = SUP_CMD_LL_RLP_XMODEM_SEND_FW;
    BaseType_t bret = xQueueSend(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS);
    LL_ASSERT(pdPASS == bret);
}

void sup_ll_module_send_xmodem_fw(uint32_t size)
{
    ll_xmodem_callbacks_t cb;
    ll_version_t vers = { 0 };
    cb.progress = ll_xmodem_progress_callback;
    ui_log("");
    ui_log("Sending Firmware...");
    ui_log(""); // put progress bar here
    int32_t ret = ll_xmodem_send(&cb, (uint8_t*)(LL_FTP_MODULE_FIRMWARE_ADDR_START + LL_FTP_HDR_LEN), (size_t)size);

    wdg_refresh(s_sup_task_wdg_handler);
    switch (ret)
    {
        case 0:
            ui_log("               done.");
            ui_log("Success!");
            sleep_ms(1500);
            wdg_refresh(s_sup_task_wdg_handler);
            ll_version_get(&vers);
            ui_log("Updated to v%i.%i.%i!",
                (int)vers.major,
                (int)vers.minor,
                (int)vers.tag);
            ui_log("Rebooting Module...");
            break;
        case -2:
            ui_log("ERR: Timeout!");
            break;
        case -3:
            ui_log("ERR: Canceled!");
            break;
        case -4:
            ui_log("ERR: Max TX Atmpts!");
            break;
        case -5:
            ui_log("ERR: XModem UNinit!");
            break;
        case -6:
            ui_log("ERR: Tranfer Failed!");
            break;
        case -7:
            ui_log("ERR: FW not verified!");
            break;
        case -8:
            ui_log("ERR: FW not activated!");
            break;
        default:
            ui_log("ERR: IFC ERROR!");
            break;
    }

    sleep_ms(3000);
    wdg_refresh(s_sup_task_wdg_handler);

    sup_queue_msg_t msg;
    msg.event = ret < 0 ? SUP_CMD_LL_UPDATE_FAILED : SUP_CMD_LL_UPDATE_DONE;
    BaseType_t bret = xQueueSend(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS);
    LL_ASSERT(pdPASS == bret);
}

void sup_ll_upgrade_failed()
{
    ui_log("Firmware Upgrade Fail!");
    sleep_ms(1500);
    sup_queue_msg_t msg;
    msg.event = SUP_CMD_LL_UPDATE_DONE;
    BaseType_t bret = xQueueSend(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS);
    LL_ASSERT(pdPASS == bret);
}

void sup_ll_module_reset(bool do_wait)
{
    // Stop status timer
    BaseType_t pd_ret = xTimerStop(s_status_timer, SUP_TIMER_TIMEOUT_TICKS);
    LL_ASSERT(pdPASS == pd_ret);

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
    LL_ASSERT(pdPASS == pd_ret);

    // Delete settings
    Debug_Printf("ERASING MODULE FLASH SETTINGS...\n");
    int32_t ret = ll_settings_delete();
    LL_ASSERT(ret >= 0);

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
    LL_ASSERT(ret >= 0);

    // get module's connection info
    ret = ll_net_info_get(&s_ll_vars.net_info);
    LL_ASSERT(ret >= 0);

    cfg.chan_step_offset = is_single_channel ? s_ll_vars.net_info.gateway_channel : 0;
    cfg.chan_step_size = is_single_channel ? 100 : cfg.chan_step_size;

    // set new dl band in module
    ret = ll_dl_band_cfg_set(&cfg);
    LL_ASSERT(ret >= 0);

    // save status
    s_is_drive_mode = is_single_channel;
}

// Retrieve the received message from the module and pass to ...
static void sup_message_retrieve(void)
{
    s_dl_msg_len = sizeof(s_dl_msg_buf) / sizeof(s_dl_msg_buf[0]);
    uint8_t port, raw_snr;
    uint16_t rssi;
    int32_t ret = ll_retrieve_message(s_dl_msg_buf, &s_dl_msg_len, &port, &rssi, &raw_snr);
    if (ret < 0)
    {
        Debug_Printf("Retrieve Message Failed: %i\n", ret);
    }

    if (FTP_PORT == port)
    {
        ll_ftp_return_code_t fret = ll_ftp_msg_process(&ftp, s_dl_msg_buf, s_dl_msg_len);
        LL_ASSERT(LL_FTP_OK == fret || LL_FTP_NO_ACTION == fret);
        if (SEGMENT == ftp.state)
        {
            ui_display_ftp_download_status(&ftp);
            wdg_refresh(s_sup_task_wdg_handler);
        }
    }
    else
    {
        if(s_dl_msg_len > 0)
        {
            ui_display_dl_msg(s_dl_msg_buf, s_dl_msg_len);
            ui_notify_user(DOUBLE_LONG_BEEP);
        }
        else
        {
            Debug_Print("RX message length ZERO.\n");
            LL_ASSERT(0);
        }
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
                LL_ASSERT(false);
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
    uint32_t ret = 0;

    supervisor_state_t next_state = SUP_STATE_LL_UNINITIALIZED;

    // set GW LED off
    GW_CONNECTED_LED(OFF);

    // Get module ID
    ret = ll_unique_id_get(&s_ll_vars.mac_address);
    LL_ASSERT(ret >= 0);

    // Check MAC mode
    ll_mac_type_t mac_mode;
    ret = ll_mac_mode_get(&mac_mode);
    LL_ASSERT(ret >= 0);
    if(mac_mode != SYMPHONY_LINK)
    {
        // set to symphony mode
        ret = ll_mac_mode_set(SYMPHONY_LINK);
        LL_ASSERT(ret >= 0);
    }

    // Get firmware version
    ll_version_t fw_ver;
    ret = ll_version_get(&fw_ver);
    s_ll_vars.fw_version = fw_ver;
    LL_ASSERT(ret >= 0);
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

        // update display with module info
        ui_refresh_display();

        // Change state
        next_state = SUP_STATE_LL_INITIALIZING;

        // Start status timer
        BaseType_t pd_ret = xTimerStart(s_status_timer, SUP_TIMER_TIMEOUT_TICKS);
        LL_ASSERT(pdPASS == pd_ret);
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
    LL_ASSERT(ret >= 0);
    ret = ll_config_set(net_token, s_app_token, LL_DL_ALWAYS_ON, 0);
    LL_ASSERT(ret >= 0);

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

            switch (msg.event)
            {
                case SUP_CMD_LL_IRQ_FLAG_SET:
                    sup_check_irq_flags();
                    break;
                case SUP_CMD_LL_WAS_RESET:
                    b_exit_state = true;
                    next_state = SUP_STATE_LL_AFTER_RESET;
                    break;
                case SUP_CMD_LL_DISCONNECTED:
                    b_exit_state = true;
                    next_state = SUP_STATE_LL_INITIALIZING;
                    break;
                case SUP_CMD_LL_RX_DONE:
                    break;
                case SUP_CMD_LL_ERASE_FLASH:
                    sup_flash_erase();
                    break;
                case SUP_CMD_STATUS_TIMEOUT:
                    sup_report_status();
                    ui_refresh_display();
                    break;
                case SUP_CMD_UART_PASSTHRU:
                    b_exit_state = true;
                    next_state = SUP_STATE_UART_PASSTHRU;
                    break;
                case SUP_CMD_LL_DL_BAND_CONFIG:
                    sup_dl_band_config(false);
                    b_exit_state = true;
                    next_state = SUP_STATE_LL_INITIALIZING;
                    break;
                case SUP_CMD_LL_TX_DONE:
                case SUP_CMD_LL_CONNECTED:
                    // ignore
                    break;
                default:
                    Debug_Print("IGNORED.\n");
                    LL_ASSERT(false);
                    break;
            }
        }
        else
        {
            // poll for state
            ret = ll_get_state(&s_ll_vars.state, NULL, NULL);
            LL_ASSERT(ret >= 0);

            if(LL_STATE_IDLE_CONNECTED == s_ll_vars.state)
            {
                b_exit_state = true;
                next_state = SUP_STATE_LL_IDLE;
            }
            else if (LL_STATE_INITIALIZING == s_ll_vars.state)
            {
                ret = ll_net_info_get(&s_ll_vars.net_info);
                LL_ASSERT(ret >= 0);

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

        if (ftp.state == SEGMENT)
        {
            b_exit_state = true;
            next_state = SUP_STATE_LL_FTP;
        }

        if (xQueueReceive(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS))
        {
            Debug_Printf("(%d ms) sup_task CMD: %s in state %s\n", xTaskGetTickCount()*portTICK_RATE_MS, sup_cmd_strings[msg.event], sup_state_strings[sup_state_get()]);

	        switch (msg.event)
            {
        	    case SUP_CMD_LL_IRQ_FLAG_SET:
		            sup_check_irq_flags();
        		    break;
        		case SUP_CMD_LL_WAS_RESET:
        		    b_exit_state = true;
        		    next_state = SUP_STATE_LL_AFTER_RESET;
        		    break;
        		case SUP_CMD_LL_CONNECTED:
        		    b_exit_state = true;
        		    next_state = SUP_STATE_LL_IDLE;
        		    break;
        		case SUP_CMD_LL_DISCONNECTED:
        		    b_exit_state = true;
        		    next_state = SUP_STATE_LL_DISCONNECTED;
        		    break;
                case SUP_CMD_LL_TX_DONE:
        		    //ignore
        		    break;
        		case SUP_CMD_LL_RX_DONE:
        		    sup_message_retrieve();
        		    break;
        		case SUP_CMD_LL_ERASE_FLASH:
        		    sup_flash_erase();
        		    break;
                case SUP_CMD_STATUS_TIMEOUT:
        		    sup_report_status();
		            break;
                case SUP_CMD_UART_PASSTHRU:
        		    b_exit_state = true;
        		    next_state = SUP_STATE_UART_PASSTHRU;
        		    break;
        		case SUP_CMD_LL_DL_BAND_CONFIG:
        		    sup_dl_band_config(msg.Data);
        		    b_exit_state = true;
        		    next_state = SUP_STATE_LL_INITIALIZING;
                case SUP_CMD_LL_UPDATE_RLP:
                    b_exit_state = true;
                    next_state = SUP_STATE_MODULE_UPGRADE;
        		    break;
                case SUP_CMD_LL_UPDATE_SELF:
                    b_exit_state = true;
                    next_state = SUP_STATE_SELF_UPGRADE;
                    break;
                default:
        		    Debug_Print("IGNORED.\n");
        		    LL_ASSERT(false);
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

static void sup_ftp_state(void)
{
    bool b_exit_state = false;
    sup_queue_msg_t msg;
    supervisor_state_t next_state = SUP_STATE_LL_IDLE;

    while (!b_exit_state)
    {
        wdg_refresh(s_sup_task_wdg_handler);
        ll_ftp_msg_process(&ftp, NULL, 0);

        if (ftp.state == IDLE)
        {
            b_exit_state = true;
        }

        if (xQueueReceive(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS))
        {
            Debug_Printf("(%d ms) sup_task CMD: %s in state %s\n", xTaskGetTickCount()*portTICK_RATE_MS, sup_cmd_strings[msg.event], sup_state_strings[sup_state_get()]);
            Debug_Printf("(%d ms) FTP State: %s || Transfered: %i / %i\n", xTaskGetTickCount()*portTICK_RATE_MS, sup_ftp_strings[ftp.state], ftp.num_segs - ll_ftp_num_missing_segs_get(&ftp), ftp.num_segs);
	        switch (msg.event)
            {
        	    case SUP_CMD_LL_IRQ_FLAG_SET:
		            sup_check_irq_flags();
        		    break;
        		case SUP_CMD_LL_CONNECTED:
        		    break;
        		case SUP_CMD_LL_DISCONNECTED:
        		    b_exit_state = true;
        		    next_state = SUP_STATE_LL_DISCONNECTED;
        		    break;
        		case SUP_CMD_LL_RX_DONE:
        		    sup_message_retrieve();
        		    break;
                case SUP_CMD_LL_UPDATE_RLP:
                    b_exit_state = true;
                    next_state = SUP_STATE_MODULE_UPGRADE;
        		    break;
                case SUP_CMD_LL_UPDATE_SELF:
                    b_exit_state = true;
                    next_state = SUP_STATE_SELF_UPGRADE;
                    break;
                default:
        		    Debug_Print("IGNORED.\n");
            }
        }
    }

    sup_state_set(next_state);
}

// Send a message to the module.  Stays here until module reports TX_DONE or TX_ERROR.
static void sup_tx_state()
{
    supervisor_state_t next_state = ftp.state != IDLE ? SUP_STATE_LL_FTP : SUP_STATE_LL_UNINITIALIZED;
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
        ll_ftp_msg_process(&ftp, NULL, 0);
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
                LL_ASSERT(false);
            }
        }
        else
        {
            // poll for state
            int32_t ret = ll_get_state(&s_ll_vars.state, NULL, NULL);
            LL_ASSERT(ret >= 0);

            if(LL_STATE_IDLE_CONNECTED != s_ll_vars.state)
            {
                b_exit_state = true;
                next_state = SUP_STATE_LL_INITIALIZING;
            }

            // check if stuck
            if(check_count > 120)
            {
                Debug_Printf("Tx Timeout.\n");
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
                LL_ASSERT(false);
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

static void sup_module_upgrade_state()
{
    for (int i = 0; i < 5; i++)
    {
        sleep_ms(2000);
        wdg_refresh(s_sup_task_wdg_handler);
    }

    supervisor_state_t next_state = SUP_STATE_LL_UNINITIALIZED;
    sup_queue_msg_t msg;
    bool b_exit_state = false;
    bool pending = true;

    // Check if firmware is downloaded.
    ftp_header_data_t header = { 0 };
    ll_version_t curr_vers   = { 0 };

    if (load_ftp_flash_vars(LL_FTP_MODULE_FIRMWARE_ADDR_START, &header) < 0)
    {
        ui_log("FW is not downloaded");
        sleep_ms(3000);
        wdg_refresh(s_sup_task_wdg_handler);
        msg.event = SUP_CMD_LL_UPDATE_FAILED;
    }

    int32_t ret = ll_version_get(&curr_vers);
    LL_ASSERT(ret >= 0);

    ui_set_log_title("RLP Module Upgrade!");
    ui_log("  v%i.%i.%i > v%i.%i.%i  ",
            (int)curr_vers.major,
            (int)curr_vers.minor,
            (int)curr_vers.tag,
            (int)(header.version >> 24),
            (int)(header.version >> 16) & 255,
            (int)(header.version >> 8)  & 255);
    sleep_ms(2000);
    wdg_refresh(s_sup_task_wdg_handler);

    msg.event = SUP_CMD_LL_BOOTLOADER;
    BaseType_t bret = xQueueSend(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS);
    LL_ASSERT(pdPASS == bret);

    // Wait in this state while RLP is getting upgraded
    while(!b_exit_state)
    {
        wdg_refresh(s_sup_task_wdg_handler);

        if (xQueueReceive(s_sup_task_queue, &msg, SUP_QUEUE_TIMEOUT_TICKS/5))
        {
            Debug_Printf("(%d ms) sup_task CMD: %s in state %s\n", xTaskGetTickCount()*portTICK_RATE_MS, sup_cmd_strings[msg.event], sup_state_strings[sup_state_get()]);

            switch (msg.event)
            {
                case SUP_CMD_LL_UPDATE_DONE:
                    b_exit_state = true;
                    ui_activate_main_menu();
                    next_state = SUP_STATE_LL_UNINITIALIZED;
                    break;
                case SUP_CMD_LL_UPDATE_FAILED:
                    sup_ll_upgrade_failed();
                    break;
                case SUP_CMD_LL_RLP_CRC:
                    // TODO?
                    break;
                case SUP_CMD_LL_BOOTLOADER:
                    sup_ll_module_bootloader_mode();
                    break;
                case SUP_CMD_LL_RLP_XMODEM_SEND_FW:
                    sup_ll_module_send_xmodem_fw(header.size);
                    break;
                default:
                    Debug_Printf("IGNORED %s in %s State!\n", sup_cmd_strings[msg.event], sup_state_strings[sup_state_get()]);
                    LL_ASSERT(false);
                    break;
            }
        }
    }

    // Set next state
    sup_state_set(next_state);
}

static void sup_self_upgrade_state()
{
    for (int i = 0; i < 5; i++)
    {
        sleep_ms(2000);
        wdg_refresh(s_sup_task_wdg_handler);
    }

    supervisor_state_t next_state = SUP_STATE_LL_IDLE;
    sup_queue_msg_t msg;

    ui_set_log_title("NT Firmware Upgrade!");

    // Check if firmware is downloaded.
    ftp_header_data_t header = {0};

    if (load_ftp_flash_vars(LL_FTP_NT_FIRMWARE_ADDR_START, &header) < 0)
    {
        ui_log("FW is not downloaded");
        ui_log("Nothing will be done");
        ui_log("Rebooting...");
        sleep_ms(3000);
        wdg_refresh(s_sup_task_wdg_handler);
        NVIC_SystemReset();
    }

    ui_log("  v%i.%i.%i > v%i.%i.%i  ",
            (int)VERSION_MAJOR,
            (int)VERSION_MINOR,
            (int)VERSION_TAG,
            (int)(header.version >> 24),
            (int)(header.version >> 16) & 255,
            (int)(header.version >> 8)  & 255);
    sleep_ms(2000);
    wdg_refresh(s_sup_task_wdg_handler);

    ui_log("Firmware is Verfied");
    ui_log("Preparing to Install");
    ui_log("Rebooting...");

    sleep_ms(2000);
    wdg_refresh(s_sup_task_wdg_handler);
    NVIC_SystemReset();
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
            case SUP_STATE_LL_FTP:
                sup_ftp_state();
                break;
            case SUP_STATE_LL_TX:
                sup_tx_state();
                break;
            case SUP_STATE_UART_PASSTHRU:
                sup_uart_pass_state();
                break;
            case SUP_STATE_MODULE_UPGRADE:
                sup_module_upgrade_state();
                break;
            case SUP_STATE_SELF_UPGRADE:
                sup_self_upgrade_state();
                break;
            default:
                Debug_Printf("Unknown State: %d\n", curr_state);
                LL_ASSERT(false);
                break;
       }

        ll_ftp_msg_process(&ftp, NULL, 0);
        wdg_refresh(s_sup_task_wdg_handler);
    }
}

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

ll_version_t sup_get_version(void)
{
    return (s_ll_vars.fw_version);
}

int32_t load_ftp_flash_vars(uint32_t start_addr, ftp_header_data_t* data)
{
    uint8_t *raw_crc  = (uint8_t*)start_addr + LL_FTP_HDR_OFFSET_CRC;
    uint8_t *raw_size = (uint8_t*)start_addr + LL_FTP_HDR_OFFSET_SIZE;
    uint8_t *raw_id   = (uint8_t*)start_addr + LL_FTP_HDR_OFFSET_ID;
    uint8_t *raw_vers = (uint8_t*)start_addr + LL_FTP_HDR_OFFSET_VERSION;

    data->crc     = UINT32_FROM_BYTESTREAM(raw_crc);
    data->size    = UINT32_FROM_BYTESTREAM(raw_size);
    data->id      = UINT32_FROM_BYTESTREAM(raw_id);
    data->version = UINT32_FROM_BYTESTREAM(raw_vers);

    if (data->crc == 0xffffff || data->size == 0xffffff
            || data->id == 0xffffff || data->version == 0xffffff)
    {
        return -1;
    }

    return 0;
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
    ui_register_ack_mode_callbacks(sup_ack_mode_set_callback, sup_ack_mode_get_callback);

    // Initalize FTP
    ftp_callbacks.open    = ftp_open_callback;
    ftp_callbacks.read    = ftp_read_callback;
    ftp_callbacks.write   = ftp_write_callback;
    ftp_callbacks.close   = ftp_close_callback;
    ftp_callbacks.apply   = ftp_apply_callback;
    ftp_callbacks.uplink  = ftp_send_uplink_callback;
    ftp_callbacks.config  = ftp_dl_config_callback;

    ll_ftp_return_code_t ret = ll_ftp_init(&ftp, &ftp_callbacks);
    LL_ASSERT(LL_FTP_OK == ret);

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
