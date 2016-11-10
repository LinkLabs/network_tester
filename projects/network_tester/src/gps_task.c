#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "em_chip.h"
#include "iomap.h"
#include "osp.h"
#include "gps_task.h"
#include "ui_task.h"
#include "bsp.h"
#include "bsp_uart.h"
#include "bsp_watchdog.h"
#include "bsp_timer.h"
#include "iomap.h"
#include "ll_ifc.h"
#include "supervisor.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "task_mgmt.h"

#define OSP_DATA_READY      (0x01)
#define GPS_FIX_TIMEOUT_S   90      // amount of time to wait for fix before resetting GPS
#define GPS_RESET_DELAY_MS  200     // amount of time to hold GPS in reset

// GPS payload parameters
#define GPS_MSG_TYPE            0

#define GPS_MSG_COUNT_IDX       0
#define GPS_MSG_TYPE_IDX        6
#define GPS_LATITUDE_IDX        8
#define GPS_LONGITUDE_IDX       33
#define GPS_ALTITUDE_IDX        59
#define GPS_RSSI_IDX            72
#define GPS_RESERVED_IDX        79

#define GPS_MSG_COUNT_NUM_BITS  6
#define GPS_MSG_TYPE_NUM_BITS   2
#define GPS_LATITUDE_NUM_BITS   25
#define GPS_LONGITUDE_NUM_BITS  26
#define GPS_ALTITUDE_NUM_BITS   13
#define GPS_RSSI_NUM_BITS       7
#define GPS_RESERVED_NUM_BITS   1

#define GPS_ALTITUDE_OFFSET_M   (-3500)
#define ALT_MAX                 4095
#define ALT_MIN                 (-4096)
#define RSSI_MAX                0
#define RSSI_MIN                (-126)

// rtos variables
static xTaskHandle          s_gps_task_handle;
static wdg_handler_t        s_gps_task_wdg_handler;

// gps variables
static char gps_bytes[64];
static uint32_t gps_idx = 0;
static gps_data_t gps_data;
static gps_fix_t gps_fix;   // gps data + age of fix indicator
static TickType_t last_fix_tick;

static portTASK_FUNCTION_PROTO(gps_task, param);
static void gps_rx_int_callback(char byte);
static void gps_bit_pack(char* dest_bfr,uint32_t bit_idx,uint32_t data, uint32_t num_data_bits);

// decodes gps data, updates gps structure
static portTASK_FUNCTION(gps_task, param)
{
    (void) param;

    uint32_t i;

    last_fix_tick = xTaskGetTickCount();
    while (true)
    {
        // wait for block of OSP data to be received
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        wdg_refresh(s_gps_task_wdg_handler);

        // process data
        for(i=0;i<sizeof(gps_bytes);i++)
        {
            osp(gps_bytes[i]);
        }
        osp_get_latest(&gps_data);

        // copy into gps fix structure
        gps_fix.latitude = gps_data.latitude;
        gps_fix.longitude = gps_data.longitude;
        gps_fix.altitude = gps_data.altitude;
        gps_fix.ehpe = gps_data.ehpe;
        gps_fix.evpe = gps_data.evpe;
        gps_fix.sat_id_list = gps_data.sat_id_list;
        gps_fix.cnt = gps_data.cnt;
        gps_fix.fix_flag = gps_data.sat_id_list != 0;

        // calc age of fix
        if(gps_fix.fix_flag)
        {
            gps_fix.fix_age_s = 0;
            last_fix_tick = xTaskGetTickCount();
        }
        else
        {
            gps_fix.fix_age_s = ((uint32_t)(xTaskGetTickCount() - last_fix_tick))*portTICK_PERIOD_MS/1000;
            if(gps_fix.fix_age_s > GPS_FIX_TIMEOUT_S) // something may be wrong--reset GPS
            {
                GPS_RST(1);
                vTaskDelay(GPS_RESET_DELAY_MS/portTICK_PERIOD_MS);
                GPS_RST(0);
                last_fix_tick = xTaskGetTickCount();        // reset fix tick count (so we don't continually reset GPS)
            }

        }

        // light LED (if we have a fix)
        LED3(gps_fix.fix_flag);

        // if we're in the gps diagnostics menu, display gps data
        ui_display_gps_diagnostics(&gps_fix);
    }
}

static void gps_rx_int_callback(char byte)
{
    BaseType_t xHigherPriorityTaskWoken;

    gps_bytes[gps_idx] = byte;
    if(++gps_idx >= sizeof(gps_bytes))
    {
        gps_idx = 0;
        xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(s_gps_task_handle,&xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}

static void gps_bit_pack(char* dest_bfr,uint32_t bit_idx,uint32_t data, uint32_t num_data_bits)
{
    uint32_t byte_idx, bit_ofst, i;
    uint64_t temp_dest,temp_src,data_mask;

    byte_idx = bit_idx/8;
    bit_ofst = bit_idx%8;

    // load affected bytes into temp buffer
    temp_dest = 0;
    for(i=0;i<sizeof(temp_dest);i++)
    {
        temp_dest<<=8;
        temp_dest |= (uint64_t)(dest_bfr[byte_idx + i]);
    }
    // load data into temp buffer
    temp_src = (uint64_t)(data);

    // shift data to get bits in proper location
    temp_src<<=(64 - bit_ofst - num_data_bits);

    // construct mask
    data_mask = (0xFFFFFFFFFFFFFFFF >> (64-num_data_bits));
    data_mask<<=(64 - bit_ofst - num_data_bits);

    // mask bits in src and dest buffers
    temp_src &= data_mask;
    temp_dest &= (~data_mask);

    // or in data
    temp_dest |= temp_src;

    // load back into destination buffer
    for(i=(sizeof(temp_dest)-1);i != 0xFFFFFFFF;i--)
    {
        dest_bfr[byte_idx+i] = (char)(temp_dest & 255);
        temp_dest>>=8;
    }
}

uint8_t init_gps_task(void)
{
    if (pdPASS != xTaskCreate(gps_task, (const portCHAR *)"gps_task", GPS_TASK_STACK_SIZE, NULL, GPS_TASK_PRIORITY, &s_gps_task_handle))
    {
        return EXIT_FAILURE;
    }

    GPS_RST(1);
    bsp_delay_ms(500);
    GPS_RST(0);

    // config gps uart interrupt callback and init buffer index
    gps_idx = 0;
    register_usart2_rx_callback(gps_rx_int_callback);

    s_gps_task_wdg_handler = wdg_register("GPS");
    if (WDG_HANDLER_ERROR == s_gps_task_wdg_handler)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

void gps_get_latest_fix(gps_fix_t* fix)
{
    *fix = gps_fix;
}

void gps_build_packet(uint8_t* payload_bfr, uint8_t msg_num)
{
    int16_t temp;

    if(gps_fix.fix_flag)
    {
        gps_bit_pack((char*)payload_bfr, GPS_MSG_COUNT_IDX, msg_num,GPS_MSG_COUNT_NUM_BITS);
        gps_bit_pack((char*)payload_bfr, GPS_MSG_TYPE_IDX, GPS_MSG_TYPE,GPS_MSG_TYPE_NUM_BITS);
        gps_bit_pack((char*)payload_bfr, GPS_LATITUDE_IDX, (uint32_t)(gps_fix.latitude/100), GPS_LATITUDE_NUM_BITS);
        gps_bit_pack((char*)payload_bfr, GPS_LONGITUDE_IDX, (uint32_t)(gps_fix.longitude/100), GPS_LONGITUDE_NUM_BITS);
        temp = gps_fix.altitude + GPS_ALTITUDE_OFFSET_M;
        if(temp > ALT_MAX)
        {
            temp = ALT_MAX;
        }
        else if(temp < ALT_MIN)
        {
            temp = ALT_MIN;
        }
        gps_bit_pack((char*)payload_bfr, GPS_ALTITUDE_IDX, (uint32_t)(temp), GPS_ALTITUDE_NUM_BITS);
        sup_get_GW_rssi(&temp);
        if(temp > RSSI_MAX)
        {
            temp = RSSI_MAX;
        }
        else if(temp < RSSI_MIN)
        {
            temp = RSSI_MIN;
        }
        temp *= -1;
        gps_bit_pack((char*)payload_bfr, GPS_RSSI_IDX, (uint32_t)(temp), GPS_RSSI_NUM_BITS);
    }
    else
    {
        gps_bit_pack((char*)payload_bfr, GPS_MSG_COUNT_IDX, msg_num, GPS_MSG_COUNT_NUM_BITS);
        gps_bit_pack((char*)payload_bfr, GPS_MSG_TYPE_IDX, GPS_MSG_TYPE, GPS_MSG_TYPE_NUM_BITS);
        gps_bit_pack((char*)payload_bfr, GPS_LATITUDE_IDX, 0xFFFFFFFF, GPS_LATITUDE_NUM_BITS);
        gps_bit_pack((char*)payload_bfr, GPS_LONGITUDE_IDX, 0xFFFFFFFF, GPS_LONGITUDE_NUM_BITS);
        gps_bit_pack((char*)payload_bfr, GPS_ALTITUDE_IDX, 0xFFFFFFFF, GPS_ALTITUDE_NUM_BITS);
        sup_get_GW_rssi(&temp);
        if(temp > RSSI_MAX)
        {
            temp = RSSI_MAX;
        }
        else if(temp < RSSI_MIN)
        {
            temp = RSSI_MIN;
        }
        temp *= -1;
        gps_bit_pack((char*)payload_bfr, GPS_RSSI_IDX, (uint32_t)(temp), GPS_RSSI_NUM_BITS);
    }
}
