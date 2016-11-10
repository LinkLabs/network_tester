#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "em_chip.h"
#include "bsp.h"
#include "bsp_io.h"
#include "bsp_timer.h"
#include "bsp_watchdog.h"
#include "debug_print.h"
#include "debug_print.h"
#include "iomap.h"
#include "lcd_nhd.h"
#include "ll_ifc.h"
#include "ll_ifc_consts.h"
#include "ll_ifc_symphony.h"
#include "network_tester_version.h"
#include "supervisor.h"
#include "ui_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "task_mgmt.h"

#define BUTTON_POLL_PERIOD_MS           100
#define BUTTON_TIMER_ID                 0

#define CURSOR_GLYPH                    0x7E // right arrow
#define SMILE_GLYPH                     0xAF // smile face
#define ERROR_GLYPH                     0xEB // small x
#define LOAD_SEGMENT_GLYPH              0xFF // cursor block

#define MENU_TITLE_LINE                 0
#define MENU_LINE_1                     1
#define MENU_LINE_2                     2
#define MENU_LINE_3                     3

#define CURSOR_LINE_1                   0
#define CURSOR_LINE_2                   1
#define CURSOR_LINE_3                   2
#define CURSOR_LINE_4                   3
#define CURSOR_LINE_5                   4
#define CURSOR_LINE_6                   5
#define CURSOR_LINE_7                   6
#define CURSOR_LINE_8                   7
#define CURSOR_LINE_9                   8
#define CURSOR_LINE_10                  9
#define CURSOR_LINE_11                  10
#define CURSOR_LINE_12                  11

#define RSSI_METER_MIN                  130     // absolute val of min RSSI meter value
#define RSSI_METER_STEP                 10

#define DOWNLINK_BUZZ_EN_POS            0
#define DOWNLINK_CLEAR_SCREEN_POS       1

#define LONG_BEEP_LENGTH_MS                  300
#define SHORT_BEEP_LENGTH_MS                 100

typedef struct
{
    const char *menu_title;
    const char *menu_text;
    uint32_t    cursor_min;
    uint32_t    cursor_max;
    uint32_t    num_lines;
} menu_info_t;

typedef struct
{
    uint32_t    beep_on_time;
    uint32_t    beep_off_time;
    uint32_t    num_cycles;
} beep_t;

// notification beeps
const beep_t beeps[6] = {{LONG_BEEP_LENGTH_MS,LONG_BEEP_LENGTH_MS,1},
                         {SHORT_BEEP_LENGTH_MS,SHORT_BEEP_LENGTH_MS,1},
                         {LONG_BEEP_LENGTH_MS,LONG_BEEP_LENGTH_MS,2},
                         {SHORT_BEEP_LENGTH_MS,SHORT_BEEP_LENGTH_MS,2},
                         {LONG_BEEP_LENGTH_MS,LONG_BEEP_LENGTH_MS,3},
                         {SHORT_BEEP_LENGTH_MS,SHORT_BEEP_LENGTH_MS,3}};
// menus
const char main_menu_title[LCD_COLUMNS] = "                    ";
const char main_menu_text[][LCD_COLUMNS] = {" GPS Test           ",
                                            " Sensor Test        ",
                                            " Downlink Messages  ",
                                            " GPS Diagnostics    ",
                                            " Sensor Diagnostics ",
                                            " Network Diagnostics",
                                            " UART Pass-Through  ",
                                            " Drive Test Mode    ",
                                            " Uplink Ack Mode    ",
                                            " Set Network Token  ",
                                            " Set Downlink Band  ",
                                            " Connection Filter  "};

const char gps_menu_title[LCD_COLUMNS] = "GW: N/A             ";
const char gps_menu_text[][LCD_COLUMNS] = {" Mode: On Demand    ",
                                           " Send now?          ",
                                           "                    "};
const char sensor_menu_title[LCD_COLUMNS] = "GW: N/A             ";
const char sensor_menu_text[][LCD_COLUMNS] = {" Mode: On Demand    ",
                                              " Send now?          ",
                                              "                    "};
const char downlink_menu_title[LCD_COLUMNS] = "GW: N/A             ";
const char downlink_menu_text[][LCD_COLUMNS] = {"\x7EOptions: Buzz on   ","\x7EOptions: Clear?    "}; // downlink menu text array is used differently from other menus
const char gps_diag_menu_title[LCD_COLUMNS] = "GPS Diagnostics     ";
const char gps_diag_menu_text[][LCD_COLUMNS] = {"Lat:                ",
                                                "Lon:                ",
                                                "Alt:                "};
const char sensor_diag_menu_title[LCD_COLUMNS] = "Sensor Diagnostics  ";
const char sensor_diag_menu_text[][LCD_COLUMNS] = {"Temp: loading...    ",
                                                   "Humidity: loading...",
                                                   "Press: loading...   "};
const char network_diag_menu_title[LCD_COLUMNS] = "Network Diagnostics ";
const char network_diag_menu_text[][LCD_COLUMNS] = {"RSSI: loading...     ",
                                                    "ID: loading...     ",
                                                    "Chan/Freq:          "};
const char uart_pass_diag_menu_title[LCD_COLUMNS] = "UART Pass-Through  ";
const char uart_pass_diag_menu_text[][LCD_COLUMNS] = {" Net Tester bypass  ",
                                                      " Direct to module   ",
                                                      "                    "};
const char drive_mode_diag_menu_title[LCD_COLUMNS] = "Drive Test Mode     ";
const char drive_mode_diag_menu_text[][LCD_COLUMNS] = {" Bypass GW Scan     ",
                                                       " Using last good GW ",
                                                       "                    "};
const char ack_mode_diag_menu_title[LCD_COLUMNS] = "Uplink Ack Mode     ";
const char ack_mode_diag_menu_text[][LCD_COLUMNS] = {" Request ACK      ",
                                                       " on each uplink   ",
                                                       "                    "};
const char network_token_menu_title[LCD_COLUMNS] = "Set Network Token  ";
const char network_token_menu_text[][LCD_COLUMNS] = {"                   ",
                                                     "                   ",
                                                     " Apply             "};
const char set_dl_band_title[LCD_COLUMNS] = "Set Downlink Band   ";
const char set_dl_band_text[][LCD_COLUMNS] = {" USA and Mexico   ",
                                              " Brazil           ",
                                              " Australia        ",
                                              " New Zealand      ",
                                              " ETSI             ",
                                              "                  "};
const char ftp_download_menu_title[LCD_COLUMNS] = "Downloading Firmware";
const char ftp_download_menu_text[][LCD_COLUMNS] = {"Type: N/A           ",
                                                     "Version: N/A        ",
                                                     "                    "
};
const char connection_filter_menu_title[LCD_COLUMNS] = "Connection Filter   ";
const char connection_filter_menu_text[][LCD_COLUMNS] = { " Connect to All    ",
                                                  " Gateways Only     ",
                                                  " Repeaters Only    "
};
const char ftp_install_menu_title[LCD_COLUMNS] = "Installing Firmware ";

// GPS Test Options
#define GPS_MENU_MODE_MAX  2
const char gps_menu_modes[GPS_MENU_MODE_MAX+1][LCD_COLUMNS] = {" Mode: On Demand    "," Mode: 10s          "," Mode: 30s          "};
const uint32_t gps_menu_interval[GPS_MENU_MODE_MAX+1] = {UI_TX_ON_DEMAND,10,30};
// Sensor Test Options
#define SENSOR_MENU_MODE_MAX  2
const char sensor_menu_modes[SENSOR_MENU_MODE_MAX+1][LCD_COLUMNS] = {" Mode: On Demand    "," Mode: 10s          "," Mode: 30s          "};
const uint32_t sensor_menu_interval[GPS_MENU_MODE_MAX+1] = {UI_TX_ON_DEMAND,10,30};

const menu_info_t menus[] = {
    {main_menu_title, main_menu_text, 0, 11, 12},
    {gps_menu_title, gps_menu_text, 0, 1, 3},
    {sensor_menu_title, sensor_menu_text, 0, 1, 3},
    {downlink_menu_title, downlink_menu_text, 0, 1, 1},
    {gps_diag_menu_title, gps_diag_menu_text, 0, 0, 0},
    {sensor_diag_menu_title, sensor_diag_menu_text, 0, 0, 0},
    {network_diag_menu_title, network_diag_menu_text, 0, 0, 0},
    {uart_pass_diag_menu_title, uart_pass_diag_menu_text, 0, 0, 0},
    {drive_mode_diag_menu_title, drive_mode_diag_menu_text, 0, 0, 0},
    {ack_mode_diag_menu_title, ack_mode_diag_menu_text, 0, 0, 0},
    {network_token_menu_title, "", 0, 1, 1},
    {set_dl_band_title, set_dl_band_text, 0, 4, 5},
    {connection_filter_menu_title, connection_filter_menu_text, 1, 3, 3},
    {ftp_download_menu_title, ftp_download_menu_text, 0, 0, 0},
    {ftp_install_menu_title, "", 0, 1, 2}
};

// rtos variables
static xTaskHandle      s_btn_task_handle;
static xTaskHandle      s_screen_task_handle;
static xTaskHandle      s_notify_task_handle;
static wdg_handler_t    s_btn_task_wdg_handler;

// menu variables
menu_names_t active_menu;
uint32_t menu_mode = 0;
bool buzz_en = true;
uint32_t update_rate = 0;
static uint32_t menu_pos;
bool tx_trigger = false;
char downlink_msgs[2][LCD_COLUMNS] = {"                    ", "                    "};
uint8_t token_place = 0;
uint32_t temp_token = 0;
bool token_edit_mode = false;
bool dl_band_set = false;
bool dl_band_set_failed = false;
char log [3][LCD_COLUMNS] = {"                    ", "                    ", "                    "};
char log_title[LCD_COLUMNS] = "                    ";
s_is_force_uart_passthru = false;
static char     screen[LCD_ROWS][LCD_COLUMNS] = {"     Link Labs      ",
                                                 "   Network Tester   ",
                                                 VERSION_STRING,
                                                 " www.link-labs.com  "};

// Callbacks
void (*s_wipe_callback) (void) = NULL;
void (*s_uart_pass_set) (bool) = NULL;
bool (*s_uart_pass_get) (void) = NULL;
bool (*s_drive_mode_set) (bool) = NULL;
bool (*s_drive_mode_get) (void) = NULL;
bool (*s_ack_mode_set) (bool) = NULL;
bool (*s_ack_mode_get) (void) = NULL;

static portTASK_FUNCTION_PROTO(btn_task, param);
static portTASK_FUNCTION_PROTO(screen_task, param);
static portTASK_FUNCTION_PROTO(notify_task, param);
static uint8_t init_button_task(void);
static uint8_t init_screen_update_task(void);
static uint8_t init_notify_task(void);
static void ui_load_menu(void);
static void ui_menu_select(void);
static void ui_menu_long_select(void);
static void ui_menu_back(void);
static void ui_print_mac_address_string(char *dest);
static void ui_print_net_token(char *dest);
static void ui_print_net_token_cursor();
static void ui_menu_load_enabled_status(bool is_enabled);

void ui_register_wipe_callback(void (*hook) (void))
{
    s_wipe_callback = hook;
}

void ui_register_uart_pass_callbacks(void (*h_set) (bool), bool (*h_get) (void))
{
    s_uart_pass_set = h_set;
    s_uart_pass_get = h_get;
}

void ui_register_drive_mode_callbacks(bool (*h_set) (bool), bool (*h_get) (void))
{
    s_drive_mode_set = h_set;
    s_drive_mode_get = h_get;
}

void ui_register_ack_mode_callbacks(bool (*h_set) (bool), bool (*h_get) (void))
{
    s_ack_mode_set = h_set;
    s_ack_mode_get = h_get;
}

// Must be called from Main Menu
void ui_uart_passthru_force(void)
{
    s_is_force_uart_passthru = true;
    active_menu = UART_PASS_DIAG_MENU;
    ui_load_menu();
    ui_menu_load_enabled_status(true);
    xTaskNotifyGive(s_screen_task_handle);
}

static void ui_increment_menu_position(void)
{
    // Increment menu position
    switch(active_menu)
    {
        case GPS_DIAG_MENU:
        case SENSOR_DIAG_MENU:
        case NETWORK_DIAG_MENU:
        case UART_PASS_DIAG_MENU:
        case DRIVE_MODE_DIAG_MENU:
        case ACK_MODE_DIAG_MENU:
        case LL_FTP_DOWNLOAD_MENU:
        case LL_FTP_INSTALL_MENU:
            // no cursor in these menus--do nothing
            break;
        case DOWNLINK_MENU:
            // line is displayed on the bottom of the screen
            menu_pos++;
            if(menu_pos > menus[active_menu].cursor_max)
            {
                menu_pos = menus[active_menu].cursor_min;
            }
            if(menu_pos == 0)   // buzzer enable/disable
            {
                if(buzz_en == true)
                {
                    strncpy(screen[MENU_LINE_3],"\x7EOptions: Buzz on   ",LCD_COLUMNS);
                }
                else
                {
                    strncpy(screen[MENU_LINE_3],"\x7EOptions: Buzz off  ",LCD_COLUMNS);
                }
            }
            else
            {
                strncpy(screen[MENU_LINE_3],downlink_menu_text[menu_pos],LCD_COLUMNS);
            }
            xTaskNotifyGive(s_screen_task_handle);
            break;
        case NETWORK_TOKEN_MENU:
            screen[(menu_pos % 3) + 2][0] = ' ';
            menu_pos++;
            if (menu_pos > menus[active_menu].cursor_max)
            {
                menu_pos = menus[active_menu].cursor_min;
            }
            screen[(menu_pos % 3) + 2][0] = CURSOR_GLYPH;
            xTaskNotifyGive(s_screen_task_handle);
            break;
        case MAIN_MENU:
        case GPS_MENU:
        case SENSOR_MODE_MENU:
        default:
            // Increment cursor, load new screen (if necessary)
            screen[(menu_pos%3)+1][0] = ' ';
            menu_pos++;
            if(menu_pos > menus[active_menu].cursor_max)
            {
                menu_pos = menus[active_menu].cursor_min;
            }
            // See if we've rolled onto a new screen
            if(menus[active_menu].num_lines > 3)    // does this menu have more than one screen?
            {
                if((menu_pos%3) == 0)   // yes, we have rolled to a new screen
                {
                    ui_load_menu();     // reload the menu
                }
            }
            screen[(menu_pos%3)+1][0] = CURSOR_GLYPH;
            xTaskNotifyGive(s_screen_task_handle);
            break;
    }
}

// Button Task Entry Point
static portTASK_FUNCTION(btn_task, param)
{
    (void) param;

    // local vars
    uint32_t btn_states = 0;
    uint32_t new_btn_states = 0;
    uint32_t pos_edges = 0;
    uint32_t neg_edges = 0;

    // keeps track of back button long press
    BaseType_t last_back_btn_down_tick = xTaskGetTickCount();
    BaseType_t last_sel_btn_down_tick = xTaskGetTickCount();
    bool was_longpress_back = false, was_longpress_select = false;

    // init button states
    btn_states = BTN_RD();

    // task loop
    while (true)
    {
        vTaskDelay(BUTTON_POLL_PERIOD_MS/portTICK_PERIOD_MS);
        wdg_refresh(s_btn_task_wdg_handler);

        // Read buttons, find rising edges, and save new button valuesR
        new_btn_states = BTN_RD();
        pos_edges = (~btn_states) & new_btn_states;
        neg_edges = btn_states & (~new_btn_states);
        btn_states = new_btn_states;

        // SELECT button up
        if(pos_edges & SEL_BTN_MASK)
        {
            if (!was_longpress_select)
            {
                ui_menu_select();
            }
            was_longpress_select = false;
        }

        // DOWN button up
        if (pos_edges & DOWN_BTN_MASK)
        {
            ui_increment_menu_position();
        }

        // BACK button up
        if(pos_edges & BACK_BTN_MASK)
        {
            // Only execute menu back if not after long press
            if(!was_longpress_back)
            {
                ui_menu_back();
            }
            was_longpress_back = false;
        }

        // SELECT button down
        if (neg_edges & SEL_BTN_MASK)
        {
            last_sel_btn_down_tick = xTaskGetTickCount();
        }

        // SELECT button long press
        if (~btn_states & SEL_BTN_MASK)
        {
            uint32_t seconds_since_last_depress =
                ((uint32_t)(xTaskGetTickCount() - last_sel_btn_down_tick)) * portTICK_PERIOD_MS /
                1000;
            if (seconds_since_last_depress > 1)
            {
                // Refresh time latch
                last_sel_btn_down_tick = xTaskGetTickCount();
                was_longpress_select = true;
                ui_menu_long_select();
            }
        }

        // BACK button down
        if(neg_edges & BACK_BTN_MASK)
        {
            last_back_btn_down_tick = xTaskGetTickCount();
        }

        // BACK button long press
        if(~btn_states & BACK_BTN_MASK)
        {
            uint32_t seconds_since_last_depress = seconds_since_last_depress = ((uint32_t)(xTaskGetTickCount() - last_back_btn_down_tick)) * portTICK_PERIOD_MS/1000;
            if(seconds_since_last_depress > 5)
            {
                // Call wipe
                if(NULL != s_wipe_callback)
                {
                    s_wipe_callback();
                }

                // Refresh time latch
                last_back_btn_down_tick = xTaskGetTickCount();
                was_longpress_back = true;
            }
        }
    }
}

// has one job: updates the screen when notified
static portTASK_FUNCTION(screen_task, param)
{
    (void) param;

    ui_load_menu();
    while (true)
    {
        // wait for someone to ask us to update the screen
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // update the screen
        lcd_write_screen(screen);
    }
}

// notifies user (currently via buzzer, maybe via vibrator motor in the future)
static portTASK_FUNCTION(notify_task, param)
{
    (void) param;

    uint32_t beep_idx;
    uint32_t cycles;

    while (true)
    {
        // wait for someone to request a notification
        //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        xTaskNotifyWait(UINT32_MAX,UINT32_MAX,&beep_idx,portMAX_DELAY);

        cycles = beeps[beep_idx].num_cycles;

        while(cycles--)
        {
            BUZZER_EN(buzz_en);

            vTaskDelay(beeps[beep_idx].beep_on_time/portTICK_PERIOD_MS);

            BUZZER_EN(0);

            vTaskDelay(beeps[beep_idx].beep_off_time/portTICK_PERIOD_MS);
        }
    }
}

static uint8_t init_button_task(void)
{
    if (pdPASS != xTaskCreate(btn_task, (const portCHAR *)"btn_task", BUTTON_TASK_STACK_SIZE, NULL, BUTTON_TASK_PRIORITY, &s_btn_task_handle))
    {
        return EXIT_FAILURE;
    }

    s_btn_task_wdg_handler = wdg_register("BUTTON");
    if (WDG_HANDLER_ERROR == s_btn_task_wdg_handler)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

static uint8_t init_screen_update_task(void)
{
    if (pdPASS != xTaskCreate(screen_task, (const portCHAR *)"screen_task", SCREEN_TASK_STACK_SIZE, NULL, SCREEN_TASK_PRIORITY, &s_screen_task_handle))
    {
        return EXIT_FAILURE;
    }
    // load splash screen (already loaded into screen variable)
    lcd_write_screen(screen);

    return EXIT_SUCCESS;
}

static uint8_t init_notify_task(void)
{
    if (pdPASS != xTaskCreate(notify_task, (const portCHAR *)"notify_task", NOTIFY_TASK_STACK_SIZE, NULL, NOTIFY_TASK_PRIORITY, &s_notify_task_handle))
    {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

static void ui_menu_load_enabled_status(bool is_enabled)
{
    // display state
    if(is_enabled)
    {
        strncpy(screen[MENU_LINE_3],"      ENABLED       ", LCD_COLUMNS);
    }
    else
    {
        strncpy(screen[MENU_LINE_3],"      DISABLED      ", LCD_COLUMNS);
    }
}

static void ui_menu_load_drive_mode_enabled()
{
    LL_ASSERT(NULL != s_drive_mode_get);

    // update menu text
    bool drive_mode_enabled = s_drive_mode_get();
    ui_menu_load_enabled_status(drive_mode_enabled);
}

static void ui_menu_load_ack_mode_enabled()
{
    LL_ASSERT(NULL != s_ack_mode_get);

    // update menu text
    bool ack_mode_enabled = s_ack_mode_get();
    ui_menu_load_enabled_status(ack_mode_enabled);
}

static void ui_load_menu(void)
{
    uint32_t menu_pos_floor;

    switch(active_menu)
    {
        case MAIN_MENU:
            ui_print_mac_address_string(screen[0]);
            menu_pos_floor = (menu_pos/3) * 3;
            strncpy(screen[1], main_menu_text[menu_pos_floor], LCD_COLUMNS);
            strncpy(screen[2], main_menu_text[menu_pos_floor+1], LCD_COLUMNS);
            strncpy(screen[3], main_menu_text[menu_pos_floor+2], LCD_COLUMNS);
            screen[(menu_pos%3)+1][0] = CURSOR_GLYPH;
            xTaskNotifyGive(s_screen_task_handle);
            break;
        case GPS_MENU:
            strncpy(screen[0], gps_menu_title, LCD_COLUMNS);
            strncpy(screen[MENU_LINE_1], gps_menu_modes[menu_mode], LCD_COLUMNS);
            strncpy(screen[2], gps_menu_text[1], LCD_COLUMNS);
            strncpy(screen[3], gps_menu_text[2], LCD_COLUMNS);
            screen[(menu_pos%3)+1][0] = CURSOR_GLYPH;
            xTaskNotifyGive(s_screen_task_handle);
            break;
        case SENSOR_MODE_MENU:
            strncpy(screen[0], sensor_menu_title, LCD_COLUMNS);
            strncpy(screen[MENU_LINE_1], sensor_menu_modes[menu_mode], LCD_COLUMNS);
            strncpy(screen[2], sensor_menu_text[1], LCD_COLUMNS);
            strncpy(screen[3], sensor_menu_text[2], LCD_COLUMNS);
            screen[(menu_pos%3)+1][0] = CURSOR_GLYPH;
            xTaskNotifyGive(s_screen_task_handle);
            break;
        case DOWNLINK_MENU:
            strncpy(screen[0], downlink_menu_title, LCD_COLUMNS);
            strncpy(screen[1], downlink_msgs[0], LCD_COLUMNS);
            strncpy(screen[2], downlink_msgs[1], LCD_COLUMNS);
            if(buzz_en == true)
            {
                strncpy(screen[MENU_LINE_3], "\x7EOptions: Buzz on   ", LCD_COLUMNS);
            }
            else
            {
                strncpy(screen[MENU_LINE_3], "\x7EOptions: Buzz off  ", LCD_COLUMNS);
            }
            xTaskNotifyGive(s_screen_task_handle);
            break;
        case GPS_DIAG_MENU:
            strncpy(screen[0], gps_diag_menu_title, LCD_COLUMNS);
            strncpy(screen[1], gps_diag_menu_text[0], LCD_COLUMNS);
            strncpy(screen[2], gps_diag_menu_text[1], LCD_COLUMNS);
            strncpy(screen[3], gps_diag_menu_text[2], LCD_COLUMNS);
            xTaskNotifyGive(s_screen_task_handle);
            break;
        case SENSOR_DIAG_MENU:
            strncpy(screen[0], sensor_diag_menu_title, LCD_COLUMNS);
            strncpy(screen[1], sensor_diag_menu_text[0], LCD_COLUMNS);
            strncpy(screen[2], sensor_diag_menu_text[1], LCD_COLUMNS);
            strncpy(screen[3], sensor_diag_menu_text[2], LCD_COLUMNS);
            xTaskNotifyGive(s_screen_task_handle);
            break;
        case NETWORK_DIAG_MENU:
            strncpy(screen[0], network_diag_menu_title, LCD_COLUMNS);
            strncpy(screen[1], network_diag_menu_text[0], LCD_COLUMNS);
            strncpy(screen[2], network_diag_menu_text[1], LCD_COLUMNS);
            strncpy(screen[3], network_diag_menu_text[2], LCD_COLUMNS);
            xTaskNotifyGive(s_screen_task_handle);
            break;
        case UART_PASS_DIAG_MENU:
            strncpy(screen[0], uart_pass_diag_menu_title, LCD_COLUMNS);
            strncpy(screen[1], uart_pass_diag_menu_text[0], LCD_COLUMNS);
            strncpy(screen[2], uart_pass_diag_menu_text[1], LCD_COLUMNS);
            strncpy(screen[3], uart_pass_diag_menu_text[2], LCD_COLUMNS);
            xTaskNotifyGive(s_screen_task_handle);
            break;
        case DRIVE_MODE_DIAG_MENU:
            strncpy(screen[0], drive_mode_diag_menu_title, LCD_COLUMNS);
            strncpy(screen[1], drive_mode_diag_menu_text[0], LCD_COLUMNS);
            strncpy(screen[2], drive_mode_diag_menu_text[1], LCD_COLUMNS);
            ui_menu_load_drive_mode_enabled();
            xTaskNotifyGive(s_screen_task_handle);
            break;
        case ACK_MODE_DIAG_MENU:
            strncpy(screen[0], ack_mode_diag_menu_title, LCD_COLUMNS);
            strncpy(screen[1], ack_mode_diag_menu_text[0], LCD_COLUMNS);
            strncpy(screen[2], ack_mode_diag_menu_text[1], LCD_COLUMNS);
            ui_menu_load_ack_mode_enabled();
            xTaskNotifyGive(s_screen_task_handle);
            break;
        case NETWORK_TOKEN_MENU:
            strncpy(screen[0], network_token_menu_title, LCD_COLUMNS);
            strncpy(screen[1], network_token_menu_text[0], LCD_COLUMNS);
            strncpy(screen[2], network_token_menu_text[1], LCD_COLUMNS);
            strncpy(screen[3], network_token_menu_text[2], LCD_COLUMNS);
            ui_print_net_token_cursor();
            ui_print_net_token(screen[2]);
            screen[(menu_pos % 3) + 2][0] = CURSOR_GLYPH;
            xTaskNotifyGive(s_screen_task_handle);
            break;
        case SET_DL_BAND_MENU:
            menu_pos_floor = (menu_pos / 3) * 3;
            llabs_dl_band_cfg_t dl_band_cfg;
            int8_t active_pos;

            strncpy(screen[0], set_dl_band_title, LCD_COLUMNS);
            strncpy(screen[1], set_dl_band_text[menu_pos_floor], LCD_COLUMNS);
            strncpy(screen[2], set_dl_band_text[menu_pos_floor + 1], LCD_COLUMNS);
            strncpy(screen[3], set_dl_band_text[menu_pos_floor + 2], LCD_COLUMNS);
            screen[(menu_pos % 3) + 1][0] = CURSOR_GLYPH;

            if (ll_dl_band_cfg_get(&dl_band_cfg) != 0)
            {
                if (dl_band_cfg.band_edge_lower == DL_BAN_FCC.band_edge_lower)
                {
                    active_pos = CURSOR_LINE_1;
                }
                else if (dl_band_cfg.band_edge_lower == DL_BAN_BRA.band_edge_lower)
                {
                    active_pos = CURSOR_LINE_2;
                }
                else if (dl_band_cfg.band_edge_lower == DL_BAN_AUS.band_edge_lower)
                {
                    active_pos = CURSOR_LINE_3;
                }
                else if (dl_band_cfg.band_edge_lower == DL_BAN_NZL.band_edge_lower)
                {
                    active_pos = CURSOR_LINE_4;
                }
                else if (dl_band_cfg.band_edge_lower == DL_BAN_ETSI.band_edge_lower)
                {
                    active_pos = CURSOR_LINE_5;
                }
                else
                {
                    active_pos = -1;
                }
            }
            else
            {
                screen[0][LCD_COLUMNS - 2] = ERROR_GLYPH;
            }

            if (dl_band_set)
            {
                screen[(menu_pos % 3) + 1][LCD_COLUMNS - 2] =
                    dl_band_set_failed ? ERROR_GLYPH : SMILE_GLYPH;
                dl_band_set = false;
                dl_band_set_failed = false;
            }
            else
            {
                if ((active_pos < 3 && menu_pos <= 2) ||
                    (active_pos >= 3 && menu_pos >= 2)) // render the smile on the right page
                    screen[(active_pos % 3) + 1][LCD_COLUMNS - 2] = SMILE_GLYPH;
            }
            xTaskNotifyGive(s_screen_task_handle);
            break;
        case LL_FTP_DOWNLOAD_MENU:
            strncpy(screen[0], ftp_download_menu_title, LCD_COLUMNS);
            strncpy(screen[1], ftp_download_menu_text[0], LCD_COLUMNS);
            strncpy(screen[2], ftp_download_menu_text[1], LCD_COLUMNS);
            strncpy(screen[3], ftp_download_menu_text[2], LCD_COLUMNS);
            xTaskNotifyGive(s_screen_task_handle);
            break;
        case LL_CONNECTION_FILTER_MENU:
            strncpy(screen[0], connection_filter_menu_title, LCD_COLUMNS);
            strncpy(screen[1], connection_filter_menu_text[0], LCD_COLUMNS);
            strncpy(screen[2], connection_filter_menu_text[1], LCD_COLUMNS);
            strncpy(screen[3], connection_filter_menu_text[2], LCD_COLUMNS);
            screen[(menu_pos % 3) + 1][0] = CURSOR_GLYPH;
            uint8_t current;
            ll_connection_filter_get(&current);
            screen[current + 1][LCD_COLUMNS - 2] = SMILE_GLYPH;
            xTaskNotifyGive(s_screen_task_handle);
            break;
        case LL_FTP_INSTALL_MENU:
            strncpy(screen[MENU_TITLE_LINE], log_title, LCD_COLUMNS);
            strncpy(screen[MENU_LINE_1], log[2], LCD_COLUMNS);
            strncpy(screen[MENU_LINE_2], log[1], LCD_COLUMNS);
            strncpy(screen[MENU_LINE_3], log[0], LCD_COLUMNS);
            xTaskNotifyGive(s_screen_task_handle);
            break;
        default:
            LL_ASSERT(false);
            break;
    }
}

static void ui_menu_select_gps_menu(void)
{
    switch(menu_pos)    // GPS test: change tx interval or trigger tx
    {
        case CURSOR_LINE_1: // update TX interval
            if(++menu_mode > GPS_MENU_MODE_MAX)
            {
                menu_mode = 0;
            }
            strncpy(screen[MENU_LINE_1], gps_menu_modes[menu_mode], LCD_COLUMNS);
            screen[(menu_pos%3)+1][0] = CURSOR_GLYPH;
            xTaskNotifyGive(s_screen_task_handle);
            break;
        case CURSOR_LINE_2:
            tx_trigger = true;  // trigger TX
            break;
        default:
            break;
    }
}

static void ui_menu_select_sensor_menu(void)
{
    switch(menu_pos)
    {
        case CURSOR_LINE_1:
            if(++menu_mode > SENSOR_MENU_MODE_MAX)
            {
                menu_mode = 0;
            }
            strncpy(screen[MENU_LINE_1], sensor_menu_modes[menu_mode], LCD_COLUMNS);
            screen[(menu_pos%3)+1][0] = CURSOR_GLYPH;
            xTaskNotifyGive(s_screen_task_handle);
            break;
        case CURSOR_LINE_2:
            tx_trigger = true;  // trigger TX
            break;
        default:
            break;
    }
}

static void ui_menu_select_downlink_menu(void)
{
    if(menu_pos == DOWNLINK_BUZZ_EN_POS)   // enable/disable buzzer
    {
        buzz_en = !buzz_en;
        if(buzz_en == true)
        {
            strncpy(screen[MENU_LINE_3], "\x7EOptions: Buzz on   ", LCD_COLUMNS);
        }
        else
        {
            strncpy(screen[MENU_LINE_3], "\x7EOptions: Buzz off  ", LCD_COLUMNS);
        }
    }
    else if(menu_pos == DOWNLINK_CLEAR_SCREEN_POS) // clear screen
    {
        strncpy(screen[MENU_LINE_1], "                    ", LCD_COLUMNS);
        strncpy(screen[MENU_LINE_2], "                    ", LCD_COLUMNS);
        strncpy(downlink_msgs[0], "                    ", LCD_COLUMNS);
        strncpy(downlink_msgs[1], "                    ", LCD_COLUMNS);
    }
    xTaskNotifyGive(s_screen_task_handle);
}

static void ui_menu_select_uart_pass_menu(void)
{
    // don't do anything if forcing this mode
    if(s_is_force_uart_passthru)
    {
        return;
    }

    LL_ASSERT(NULL != s_uart_pass_get);
    LL_ASSERT(NULL != s_uart_pass_set);

    // get state
    bool pass_enabled = s_uart_pass_get();

    // Toggle pass-through
    s_uart_pass_set(!pass_enabled);

    while (s_uart_pass_get() == pass_enabled)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS); //check every 5 ms
    }

    // update menu text
    pass_enabled = s_uart_pass_get();
    ui_menu_load_enabled_status(pass_enabled);

    // display
    xTaskNotifyGive(s_screen_task_handle);
}

static void ui_menu_select_drive_mode_menu(void)
{
    LL_ASSERT(NULL != s_drive_mode_get);
    LL_ASSERT(NULL != s_drive_mode_set);

    // get state
    bool drive_mode_enabled = s_drive_mode_get();

    // Toggle pass-through
    bool set_ok = s_drive_mode_set(!drive_mode_enabled);
    if(set_ok)
    {
        while (s_drive_mode_get() == drive_mode_enabled)
        {
            vTaskDelay(50 / portTICK_PERIOD_MS); //check every 50 ms
        }
    }

    // update menu text
    drive_mode_enabled = s_drive_mode_get();
    ui_menu_load_enabled_status(drive_mode_enabled);

    // display
    xTaskNotifyGive(s_screen_task_handle);
}

static void ui_menu_select_ack_mode_menu(void)
{
    LL_ASSERT(NULL != s_ack_mode_get);
    LL_ASSERT(NULL != s_ack_mode_set);

    // get state
    bool ack_mode_enabled = s_ack_mode_get();

    // Toggle pass-through
    bool set_ok = s_ack_mode_set(!ack_mode_enabled);
    if(set_ok)
    {
        while (s_ack_mode_get() == ack_mode_enabled)
        {
            vTaskDelay(50 / portTICK_PERIOD_MS); //check every 50 ms
        }
    }

    // update menu text
    ack_mode_enabled = s_ack_mode_get();
    ui_menu_load_enabled_status(ack_mode_enabled);

    // display
    xTaskNotifyGive(s_screen_task_handle);
}

static void ui_menu_select_net_token_mod(void)
{
    uint8_t app_token[APP_TOKEN_LEN], qos;
    enum ll_downlink_mode dl_mode;

    switch (menu_pos)
    {
    case CURSOR_LINE_1: // Configure Network Token
        if (!token_edit_mode)
        {
            ll_config_get(&temp_token, app_token, &dl_mode, &qos);
            token_edit_mode = true;
        }
        else
        {
            uint32_t mask = 0xFFFFFFFF >> (token_place * 4);
            uint32_t increment = (1 << (4 * (7 - token_place)));

            temp_token = ((((mask & temp_token) + increment) & mask) | (~mask & temp_token));
        }
        break;
    case CURSOR_LINE_2: // Apply Button
        if (token_edit_mode)
        {
            token_edit_mode = false;
            ll_config_set(temp_token, app_token, dl_mode, qos);
        }
        break;
    default:
        LL_ASSERT(0);
        break;
    }
    ui_refresh_display();
    xTaskNotifyGive(s_screen_task_handle);
}

static void ui_menu_long_select_net_token_mod(void)
{
    switch (menu_pos)
    {
    case CURSOR_LINE_1: // Configure Network Token
        token_place = token_place >= 7 ? 0 : token_place + 1;
        ui_refresh_display();
        xTaskNotifyGive(s_screen_task_handle);
        break;
    }
}

static void ui_menu_select_dl_band(void)
{
    llabs_dl_band_cfg_t dl_band_cfg;
    switch (menu_pos)
    {
    case CURSOR_LINE_1:
        dl_band_cfg = DL_BAN_FCC;
        break;
    case CURSOR_LINE_2:
        dl_band_cfg = DL_BAN_BRA;
        break;
    case CURSOR_LINE_3:
        dl_band_cfg = DL_BAN_AUS;
        break;
    case CURSOR_LINE_4:
        dl_band_cfg = DL_BAN_NZL;
        break;
    case CURSOR_LINE_5:
        dl_band_cfg = DL_BAN_ETSI;
        break;
    default:
        break;
    }
    dl_band_set_failed = ll_dl_band_cfg_set(&dl_band_cfg) != 0;
    dl_band_set = true;
    ui_refresh_display();
    xTaskNotifyGive(s_screen_task_handle);
}

static void ui_menu_select_connection_filter_select(void)
{
    uint8_t filter;

    switch (menu_pos)
    {
        case CURSOR_LINE_1: // ALL
            filter = 0;
            break;
        case CURSOR_LINE_2: // Gateway
            filter = 1;
            break;
        case CURSOR_LINE_3: // Repeater
            filter = 2;
            break;
        default:
            filter = 0;
            break;
    }

    ll_connection_filter_set(filter);
    ui_refresh_display();
    xTaskNotifyGive(s_screen_task_handle);
}

static void ui_menu_select_main_menu(void)
{
    switch(menu_pos)    // main menu: set menu variables and go into next menu
    {
        case CURSOR_LINE_1:
            menu_pos = 1;
            menu_mode = 0;
            update_rate = 0;
            active_menu = GPS_MENU;
            ui_load_menu();
            break;
        case CURSOR_LINE_2:
            menu_pos = 1;
            menu_mode = 0;
            update_rate = 0;
            active_menu = SENSOR_MODE_MENU;
            ui_load_menu();
            break;
        case CURSOR_LINE_3:
            menu_pos = 0;
            menu_mode = 0;
            update_rate = 0;
            active_menu = DOWNLINK_MENU;
            ui_load_menu();
            break;
        case CURSOR_LINE_4:
            menu_pos = 0;
            menu_mode = 0;
            update_rate = 0;
            active_menu = GPS_DIAG_MENU;
            ui_load_menu();
            break;
        case CURSOR_LINE_5:
            menu_pos = 0;
            menu_mode = 0;
            update_rate = 0;
            active_menu = SENSOR_DIAG_MENU;
            ui_load_menu();
            break;
        case CURSOR_LINE_6:
            menu_pos = 0;
            menu_mode = 0;
            update_rate = 0;
            active_menu = NETWORK_DIAG_MENU;
            ui_load_menu();
            break;
        case CURSOR_LINE_7:
            menu_pos = 0;
            menu_mode = 0;
            update_rate = 0;
            active_menu = UART_PASS_DIAG_MENU;
            ui_load_menu();
            ui_menu_select_uart_pass_menu();
            break;
        case CURSOR_LINE_8:
            menu_pos = 0;
            menu_mode = 0;
            update_rate = 0;
            active_menu = DRIVE_MODE_DIAG_MENU;
            ui_load_menu();
            break;
        case CURSOR_LINE_9:
            menu_pos = 0;
            menu_mode = 0;
            update_rate = 0;
            active_menu = ACK_MODE_DIAG_MENU;
            ui_load_menu();
            break;
        case CURSOR_LINE_10:
            menu_pos = 0;
            menu_mode = 0;
            update_rate = 0;
            active_menu = NETWORK_TOKEN_MENU;
            ui_load_menu();
            break;
        case CURSOR_LINE_11:
            menu_pos = 0;
            menu_mode = 0;
            update_rate = 0;
            active_menu = SET_DL_BAND_MENU;
            ui_load_menu();
            break;
        case CURSOR_LINE_12:
            menu_pos = 0;
            menu_mode = 0;
            update_rate = 0;
            active_menu = LL_CONNECTION_FILTER_MENU;
            ui_load_menu();
        default:
            break;
    }
}

static void ui_menu_select(void)
{
    switch(active_menu)
    {
        case MAIN_MENU:
            ui_menu_select_main_menu();
            break;
        case GPS_MENU:
            ui_menu_select_gps_menu();
            break;
        case SENSOR_MODE_MENU:
            ui_menu_select_sensor_menu();
            break;
        case DOWNLINK_MENU:
            ui_menu_select_downlink_menu();
            break;
        case UART_PASS_DIAG_MENU:
            ui_menu_select_uart_pass_menu();
            break;
        case DRIVE_MODE_DIAG_MENU:
            ui_menu_select_drive_mode_menu();
            break;
        case ACK_MODE_DIAG_MENU:
            ui_menu_select_ack_mode_menu();
            break;
        case NETWORK_TOKEN_MENU:
            ui_menu_select_net_token_mod();
            break;
        case SET_DL_BAND_MENU:
            ui_menu_select_dl_band();
            break;
        case LL_CONNECTION_FILTER_MENU:
            ui_menu_select_connection_filter_select();
            break;
        case LL_FTP_DOWNLOAD_MENU:
        default:
            break;
        }
}

static void ui_menu_long_select(void)
{
    switch (active_menu)
    {
    case NETWORK_TOKEN_MENU:
        ui_menu_long_select_net_token_mod();
        break;
    case MAIN_MENU:
    case GPS_MENU:
    case SENSOR_MODE_MENU:
    case DOWNLINK_MENU:
    case UART_PASS_DIAG_MENU:
    case DRIVE_MODE_DIAG_MENU:
    case ACK_MODE_DIAG_MENU:
    case SET_DL_BAND_MENU:
    case LL_CONNECTION_FILTER_MENU:
    case LL_FTP_INSTALL_MENU:
    case LL_FTP_DOWNLOAD_MENU:
    default:
        break;
    }
}

static void ui_menu_back_common(void)
{
    menu_pos = active_menu-1;
    active_menu = MAIN_MENU;
    ui_load_menu();
}

static void ui_menu_back(void)
{
    switch(active_menu)
    {
        case UART_PASS_DIAG_MENU:
            s_uart_pass_set(false);
            ui_menu_back_common();
            break;
        case GPS_MENU:
        case SENSOR_MODE_MENU:
        case DOWNLINK_MENU:
        case GPS_DIAG_MENU:
        case SENSOR_DIAG_MENU:
        case NETWORK_DIAG_MENU:
        case DRIVE_MODE_DIAG_MENU:
        case ACK_MODE_DIAG_MENU:
        case LL_CONNECTION_FILTER_MENU:
            ui_menu_back_common();
            break;
        case NETWORK_TOKEN_MENU:
            ui_menu_back_common();
            break;
        case SET_DL_BAND_MENU:
            dl_band_set = false;
            dl_band_set_failed = false;
            ui_menu_back_common();
            break;
        case MAIN_MENU:
        case LL_FTP_DOWNLOAD_MENU:
        case LL_FTP_INSTALL_MENU:
            lcd_bklt_toggle();
            break;
        default:
            LL_ASSERT(false);
            break;
    }
}

// Prints module version/status for the first few
// seconds of start up, then displays the module
// mac address.
static void ui_print_mac_address_string(char* dest)
{
    static uint8_t num_times = 3;
    if (num_times > 0)
    {
        ll_version_t version = {0};
        version = sup_get_version();
        if (version.major == 0)
        {
             sprintf(dest,"Initializing Module");
        }
        else
        {
            sprintf(dest,"Module Version:%i.%i.%i", version.major, version.minor, version.tag);
        }
        num_times--;
    }
    else
    {
        sprintf(dest,"$301$0-0-0-%09X", (unsigned int)sup_get_MAC_address());
    }
}

static void ui_print_net_token(char *dest)
{
    if (token_edit_mode)
    {
        sprintf(dest, " NetToken: %08" PRIx32, (long unsigned int)temp_token);
    }
    else
    {
        uint32_t net_token;
        ll_config_get(&net_token, NULL, NULL, NULL); // short circuit hack
        sprintf(dest, " NetToken: %08X", (unsigned int)net_token);
    }
}

static void ui_print_net_token_cursor()
{
    if (token_edit_mode)
    {
        char x[] = "                    ";
        x[11 + token_place] = '_';
        sprintf(screen[1], x);
        sprintf(screen[3], " Apply     Edit Mode");
    }
    else
    {
         sprintf(screen[3], " Apply              ");
    }
}

uint8_t init_user_interface(void)
{
    if (init_button_task() == EXIT_FAILURE)
    {
        return EXIT_FAILURE;
    }
    if (init_screen_update_task() == EXIT_FAILURE)
    {
        return EXIT_FAILURE;
    }
    if (init_notify_task() == EXIT_FAILURE)
    {
        return EXIT_FAILURE;
    }

    menu_pos = 0;
    active_menu = MAIN_MENU;

    return EXIT_SUCCESS;
}

// returns current menu
menu_names_t ui_get_menu(void)
{
    return(active_menu);
}

void ui_ftp_activate_download(void)
{
    active_menu = LL_FTP_DOWNLOAD_MENU;
    ui_refresh_display();
}

void ui_ftp_activate_apply(void)
{
    active_menu = LL_CONNECTION_FILTER_MENU;
    ui_refresh_display();
}

void ui_activate_main_menu(void)
{
    active_menu = MAIN_MENU;
    ui_refresh_display();
}

bool ui_tx_trigger_rd(void)
{
    if(tx_trigger == true)
    {
        tx_trigger = false;
        return( true );
    }
    return( false );
}

uint32_t ui_get_tx_interval(void)
{
    uint32_t tx_interval = UI_TX_ON_DEMAND;
    switch(active_menu)
    {
        case GPS_MENU:
            tx_interval = gps_menu_interval[menu_mode];
            break;
        case SENSOR_MODE_MENU:
            tx_interval = sensor_menu_interval[menu_mode];
            break;
        default:
            tx_interval = UI_TX_ON_DEMAND;
            break;
    }
    return(tx_interval);
}

static void ui_display_gw_info_dl_strength(llabs_network_info_t* net_info, uint32_t ll_state)
{
    char gw_strength_string[13];
    uint32_t i;
    int16_t rssi_tmp;
    char temp_char;

    if(1 == ll_state) //connected
    {
        // construct strength meter, each square represents 10dB
        rssi_tmp = net_info->rssi + RSSI_METER_MIN;
        for(i=0; i<sizeof(gw_strength_string)-1; i++)
        {
            if(rssi_tmp > 0)
            {
                gw_strength_string[i] = 0xFF;   // black square
            }
            else
            {
                gw_strength_string[i] = ' ';    // blank square
            }
            rssi_tmp -= RSSI_METER_STEP;
        }
        gw_strength_string[sizeof(gw_strength_string)-1] = 0;
        temp_char = screen[MENU_LINE_1][0]; // have to save the first character of next line, since sprintf will put a null there
        sprintf(screen[MENU_TITLE_LINE],"%s:%s%+4d", net_info->is_repeater ? "REP" : "GW", gw_strength_string, net_info->rssi);
        screen[MENU_LINE_1][0] = temp_char;
    }
    else if(3 == ll_state) //initializing
    {
        if(net_info->is_scanning_gateways)
        {
            temp_char = screen[MENU_LINE_1][0]; // have to save the first character of next line, since sprintf will put a null there
            sprintf(screen[MENU_TITLE_LINE],"%s", "Scanning for GW...  ");
            screen[MENU_LINE_1][0] = temp_char;
        }
        else
        {
            temp_char = screen[MENU_LINE_1][0]; // have to save the first character of next line, since sprintf will put a null there
            Debug_Print("Initializing #1\n");
            sprintf(screen[MENU_TITLE_LINE],"%s", "Initializing...  ");
            screen[MENU_LINE_1][0] = temp_char;
        }
    }
    else
    {
        temp_char = screen[MENU_LINE_1][0]; // have to save the first character of next line, since sprintf will put a null there
        sprintf(screen[MENU_TITLE_LINE],"%s", "Disconnected.  ");
        screen[MENU_LINE_1][0] = temp_char;
    }
}

static void ui_display_gw_info_drive_mode(uint32_t ll_state)
{
    if(1 == ll_state) //connected
    {
        strncpy(screen[MENU_LINE_2], drive_mode_diag_menu_text[1], LCD_COLUMNS); // report to screen

    }
    else
    {
        strncpy(screen[MENU_LINE_2], " Waiting for Connect", LCD_COLUMNS); // report to screen
    }
}

void ui_display_gw_info(llabs_network_info_t* net_info, uint32_t ll_state)
{
    switch(active_menu)
    {
        case GPS_MENU:
        case SENSOR_MODE_MENU:
        case DOWNLINK_MENU:
            ui_display_gw_info_dl_strength(net_info, ll_state);
            xTaskNotifyGive(s_screen_task_handle);
            break;
        case DRIVE_MODE_DIAG_MENU:
            ui_display_gw_info_drive_mode(ll_state);
            xTaskNotifyGive(s_screen_task_handle);
            break;
        default:
            break;
    }
}

void ui_display_msg_record(msg_record_t *record)
{
    char msg_str[3][4];
    char temp_string[LCD_COLUMNS+1];
    uint32_t i;

    switch(active_menu)
    {
        case GPS_MENU:
        case SENSOR_MODE_MENU:
            for(i=0;i<3;i++)
            {
                switch(record[i].acked)
                {
                    case MSG_WAITING_FOR_ACK:
                        strcpy(msg_str[i],"***");
                        break;
                    case MSG_ACKED:
                        if (s_ack_mode_get())
                        {
                            strcpy(msg_str[i],"ACK");
                        }
                        else
                        {
                            strcpy(msg_str[i],"OK ");
                        }
                        break;
                    case MSG_ERROR:
                        strcpy(msg_str[i],"ERR");
                        break;
                    default:
                        strcpy(msg_str[i],"   ");
                        break;
                }
            }
            sprintf(temp_string,"%02d-%s %02d-%s %02d-%s",record[0].msg,msg_str[0],record[1].msg,msg_str[1],record[2].msg,msg_str[2]);
            strncpy(screen[MENU_LINE_3],temp_string,LCD_COLUMNS);
            xTaskNotifyGive(s_screen_task_handle);
            break;
        default:
            break;
    }
}

void ui_display_gps_diagnostics(gps_fix_t* fix_ptr)
{
    char temp_string[LCD_COLUMNS+1];
    int32_t temp_whole;
    int32_t temp_dec;

    switch(active_menu)
    {
        case GPS_DIAG_MENU:
            if(fix_ptr->fix_flag)
            {
                temp_whole = fix_ptr->latitude/10000000;
                temp_dec = (fix_ptr->latitude%10000000)/10;
                if(temp_dec < 0)
                {
                    temp_dec *= -1;
                }
                sprintf(temp_string,"Lat: %+3d.%06d\xDF    ", (int)temp_whole, (int)temp_dec);
                strncpy(screen[MENU_LINE_1],temp_string,LCD_COLUMNS);
                temp_whole = fix_ptr->longitude/10000000;
                temp_dec = (fix_ptr->longitude%10000000)/10;
                if(temp_dec < 0)
                {
                    temp_dec *= -1;
                }
                sprintf(temp_string,"Lon:  %+3d.%06d\xDF   ", (int)temp_whole, (int)temp_dec);
                strncpy(screen[MENU_LINE_2],temp_string,LCD_COLUMNS);
                sprintf(temp_string,"Alt: %+5dm        ",fix_ptr->altitude);
                strncpy(screen[MENU_LINE_3],temp_string,LCD_COLUMNS);
            }
            else
            {
                sprintf(temp_string,"Lat: No fix!        ");
                strncpy(screen[MENU_LINE_1],temp_string,LCD_COLUMNS);
                sprintf(temp_string,"Lon: Go outside, may");
                strncpy(screen[MENU_LINE_2],temp_string,LCD_COLUMNS);
                sprintf(temp_string,"Alt: take up to 2min");
                strncpy(screen[MENU_LINE_3],temp_string,LCD_COLUMNS);
            }
            xTaskNotifyGive(s_screen_task_handle);
            break;
        default:
            break;
    }
}

void ui_display_sensor_diagnostics(bme280_data_t* env_ptr, uint16_t lux)
{
    (void) lux; //unused

    char temp_string[LCD_COLUMNS+1];
    int32_t temp_whole;
    int32_t temp_dec;

    switch(active_menu)
    {
        case SENSOR_DIAG_MENU:
            temp_whole = env_ptr->temp/100;
            temp_dec = env_ptr->temp%100;
            if(temp_dec < 0)
            {
                temp_dec *= -1;
            }
            sprintf(temp_string,"Temp: %+3d.%02d C     ", (int)temp_whole, (int)temp_dec);
            strncpy(screen[MENU_LINE_1],temp_string,LCD_COLUMNS);

            temp_whole = env_ptr->humidity>>1;
            if(env_ptr->humidity & 0x01)
            {
                temp_dec = 5;
            }
            else
            {
                temp_dec = 0;
            }
            sprintf(temp_string,"Humidity: %3d.%1d%%    ", (int)temp_whole, (int)temp_dec);
            strncpy(screen[MENU_LINE_2],temp_string,LCD_COLUMNS);

            temp_whole = env_ptr->pressure>>8;
            temp_dec = (((env_ptr->pressure%8)*39)/10);
            sprintf(temp_string,"Press: %6d.%03d Pa", (int)temp_whole, (int)temp_dec);
            strncpy(screen[MENU_LINE_3],temp_string,LCD_COLUMNS);
            xTaskNotifyGive(s_screen_task_handle);
            break;
        default:
            break;
    }
}

void ui_display_network_diagnostics(llabs_network_info_t* net_info, uint32_t ll_state)
{
    char temp_string[LCD_COLUMNS+1];

    if(NETWORK_DIAG_MENU != active_menu)
    {
        // nothing to do
        return;
    }

    if(1 == ll_state) //connected
    {
        sprintf(temp_string,"%s RSSI: %4ddBm    ", net_info->is_repeater ? "REP" : "GW", net_info->rssi);
        strncpy(screen[MENU_LINE_1], temp_string, LCD_COLUMNS);
        sprintf(temp_string,"%s ID: %08X     ", net_info->is_repeater ? "REP" : "GW", (unsigned int) net_info->gateway_id);
        strncpy(screen[MENU_LINE_2], temp_string, LCD_COLUMNS);
        sprintf(temp_string,"Ch/Frq: %02d/%u", net_info->gateway_channel, (unsigned int) net_info->gateway_frequency);
        strncpy(screen[MENU_LINE_3], temp_string, LCD_COLUMNS);
    }
    else if(3 == ll_state) //initializing
    {
        if(net_info->is_scanning_gateways)
        {
            Debug_Print("Scanning...\n");
            sprintf(temp_string,"    Scanning ...    ");
        }
        else
        {
            Debug_Print("Initializing #2\n");
            sprintf(temp_string,"  Initializing ...  ");
        }
        strncpy(screen[MENU_LINE_1], temp_string, LCD_COLUMNS);
        sprintf(temp_string,"                    ");
        strncpy(screen[MENU_LINE_2], temp_string, LCD_COLUMNS);
        strncpy(screen[MENU_LINE_3], temp_string, LCD_COLUMNS);
    }
    else
    {
        sprintf(temp_string,"    Disconnected    ");
        strncpy(screen[MENU_LINE_1], temp_string, LCD_COLUMNS);
        sprintf(temp_string,"                    ");
        strncpy(screen[MENU_LINE_2], temp_string, LCD_COLUMNS);
        strncpy(screen[MENU_LINE_3], temp_string, LCD_COLUMNS);
    }

    // finish
    xTaskNotifyGive(s_screen_task_handle);
}
/*********************************************************************/
void ui_display_dl_msg(uint8_t* msg_buf, uint8_t msg_len)
{
    strncpy(downlink_msgs[1],downlink_msgs[0],LCD_COLUMNS);
    strncpy(downlink_msgs[0],"                    ",LCD_COLUMNS);
    strncpy(downlink_msgs[0], (char*)msg_buf, (msg_len>LCD_COLUMNS) ? LCD_COLUMNS:msg_len);
    switch(active_menu)
    {
        case DOWNLINK_MENU:
            strncpy(screen[MENU_LINE_1],downlink_msgs[0],LCD_COLUMNS);
            strncpy(screen[MENU_LINE_2],downlink_msgs[1],LCD_COLUMNS);
            xTaskNotifyGive(s_screen_task_handle);
            break;
        default:
            break;
    }
}

void ui_display_ftp_download_status(ll_ftp_t *f)
{
    char temp_string[LCD_COLUMNS+1];
    switch (active_menu)
    {
        case LL_FTP_DOWNLOAD_MENU:
            sprintf(temp_string,"Type: %s", (f->file_id == LL_FTP_FIRMWARE_ID_MODULE)
                    ? "RLP Module" : ((f->file_id == LL_FTP_FIRMWARE_ID_NETTEST)
                        ? "Network Tester" : "Unknown"));
            strncpy(screen[MENU_LINE_1], temp_string, LCD_COLUMNS);
            sprintf(temp_string,"Version: %i.%i.%i",
                   (int)(f->file_version >> 24),
                   (int)(f->file_version >> 16) & 255,
                   (int)(f->file_version >> 8)  & 255);
            strncpy(screen[MENU_LINE_2], temp_string, LCD_COLUMNS);
            ui_progress_bar(MENU_LINE_3, f->num_segs - ll_ftp_num_missing_segs_get(f), f->num_segs);
            xTaskNotifyGive(s_screen_task_handle);
            break;
        default:
            break;
    }
}

void ui_progress_bar(uint8_t row, int32_t done, uint32_t total)
{
    // Verify that the done segment is initialized
    if (done < 0)
    {
        strncpy(screen[row], "                   ", LCD_COLUMNS);
        return;
    }

    // Calculate how much of the screen will be blacked out
    uint32_t progress = (uint32_t)((float)(done * LCD_COLUMNS) / (float) total);

    // Create the Bar
    char bar[LCD_COLUMNS];

    // Clear Progress Bar
    strncpy(bar, "                   ", LCD_COLUMNS);
    for (uint8_t i = 0; i < progress; i++)
    {
        bar[i] = LOAD_SEGMENT_GLYPH;
    }

    // Send the bar to the screen
    strncpy(screen[row], bar, LCD_COLUMNS);
    xTaskNotifyGive(s_screen_task_handle);
}

void ui_log(const char* line, ...)
{
    if (active_menu != LL_FTP_INSTALL_MENU)
    {
        active_menu = LL_FTP_INSTALL_MENU;
    }

    char    msg[LCD_COLUMNS];
    va_list args;

    va_start(args, line);
    int32_t ret = vsnprintf(msg, sizeof(msg), line, args);
    if (ret < 0)
    {
        return;
    }
    va_end(args);

    // Update Log
    strncpy(log[2], log[1], LCD_COLUMNS);
    strncpy(log[1], log[0], LCD_COLUMNS);
    strncpy(log[0], msg,   LCD_COLUMNS);

    Debug_Printf("(%i)ms UI LOG: %s\n", xTaskGetTickCount()*portTICK_RATE_MS, msg);

    // Update Screen
    ui_refresh_display();

    xTaskNotifyGive(s_screen_task_handle);
}

void ui_set_log_title(const char* title)
{
    strncpy(log_title, title, LCD_COLUMNS);
}

void ui_refresh_display(void)
{
    ui_load_menu();
}

void ui_notify_user(notifications_t beep_type)
{
    xTaskNotify(s_notify_task_handle,beep_type,eSetValueWithOverwrite);
}
