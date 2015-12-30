/*********************************************************************/
/*********************************************************************/
//
// \file    sensor_task.c
// \brief   Environmental and light sensor tasks
// \author  Mark Bloechl
// \version 0.0.1
//
// \copyright LinkLabs, 2015
//
/*********************************************************************/
/*****INCLUDES********************************************************/
//-----Standard Libraries-----//
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
//-----EFM Libraries-----//
#include "em_chip.h"
//-----My Libraries-----//
#include "bsp.h"
#include "bsp_watchdog.h"
#include "iomap.h"
#include "bme280_driver.h"
#include "sensor_task.h"
#include "ui_task.h"
//-----FreeRTOS Libraries-----//
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "task_mgmt.h"
/*********************************************************************/
/*****DEFINES*********************************************************/
#define I2C_BUS_PTR             I2C0
#define SENSOR_POLL_PERIOD_MS   2000

// sensor message payload defs
#define SENSOR_MSG_TYPE            1

#define SENSOR_MSG_COUNT_IDX       0
#define SENSOR_MSG_TYPE_IDX        6
#define SENSOR_MOTION_IDX          8
#define SENSOR_LIGHT_IDX           9
#define SENSOR_HUMIDITY_IDX        24
#define SENSOR_TEMP_IDX            32
#define SENSOR_PRESSURE_IDX        48

#define SENSOR_MSG_COUNT_NUM_BITS  6
#define SENSOR_MSG_TYPE_NUM_BITS   2
#define SENSOR_MOTION_NUM_BITS     1
#define SENSOR_LIGHT_NUM_BITS      15
#define SENSOR_HUMIDITY_NUM_BITS   8
#define SENSOR_TEMP_NUM_BITS       16
#define SENSOR_PRESSURE_NUM_BITS   32
/*********************************************************************/
/*****TYPEDEFS/STRUCTS************************************************/
/*********************************************************************/
/*****CONSTANTS*******************************************************/
const uint8_t BME280_init[] = {};
/*********************************************************************/
/*****VARIABLES*******************************************************/
// rtos variables
static xTaskHandle      s_sensor_task_handle;
static wdg_handler_t    s_sensor_task_wdg_handler;
// sensor variables
bme280_data_t bme280_data;
uint16_t light_data;    // light sensor data, in lux,
/*****PRIVATE FUNCTION PROTOTYPES*************************************/
static portTASK_FUNCTION_PROTO(sensor_task, param);
static void sensor_bit_pack(char* dest_bfr,uint32_t bit_idx,uint32_t data, uint32_t num_data_bits);
/*********************************************************************/
/*****PRIVATE FUNCTIONS***********************************************/
static portTASK_FUNCTION(sensor_task, param)
{
    (void) param;

    while (true)
    {
        vTaskDelay(SENSOR_POLL_PERIOD_MS/portTICK_PERIOD_MS);
        wdg_refresh(s_sensor_task_wdg_handler);
        // poll environmental sensor, update variable
        BME280_poll_sensor(&bme280_data);
        // poll light sensor, update variable
        light_data = TSL4531_poll_sensor();
        // display sensor data (if in diagnostics menu)
        ui_display_sensor_diagnostics(&bme280_data, light_data);
    }
}
/*********************************************************************/
static void sensor_bit_pack(char* dest_bfr,uint32_t bit_idx,uint32_t data, uint32_t num_data_bits)
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
/*********************************************************************/
/*********************************************************************/
/*********************************************************************/
/*****PUBLIC FUNCTIONS************************************************/
/*********************************************************************/
/*********************************************************************/
uint8_t init_sensor_task(void)
{
    if (pdPASS != xTaskCreate(sensor_task, (const portCHAR *)"sensor_task", SENSOR_TASK_STACK_SIZE, NULL, SENSOR_TASK_PRIORITY, &s_sensor_task_handle))
    {
        return EXIT_FAILURE;
    }

    // init sensors
    init_BME280(I2C_BUS_PTR);
    init_TSL4531(I2C_BUS_PTR);
    // init watchdog
    s_sensor_task_wdg_handler = wdg_register("sensor");
    if (WDG_HANDLER_ERROR == s_sensor_task_wdg_handler)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
/*********************************************************************/
void sensor_build_packet(uint8_t* payload_bfr,uint8_t msg_num)
{
    // load msg counter
    sensor_bit_pack(payload_bfr,SENSOR_MSG_COUNT_IDX,(uint32_t)msg_num,SENSOR_MSG_COUNT_NUM_BITS);
    // load msg type
    sensor_bit_pack(payload_bfr,SENSOR_MSG_TYPE_IDX,SENSOR_MSG_TYPE,SENSOR_MSG_TYPE_NUM_BITS);
    // load motion state
    sensor_bit_pack(payload_bfr,SENSOR_MOTION_IDX,0,SENSOR_MOTION_NUM_BITS);
    // load light sensor data
    sensor_bit_pack(payload_bfr,SENSOR_LIGHT_IDX,(uint32_t)(light_data>>1),SENSOR_LIGHT_NUM_BITS);
    // load humidity data
    sensor_bit_pack(payload_bfr,SENSOR_HUMIDITY_IDX,(uint32_t)bme280_data.humidity,SENSOR_HUMIDITY_NUM_BITS);
    // load temperature data
    sensor_bit_pack(payload_bfr,SENSOR_TEMP_IDX,(uint32_t)bme280_data.temp,SENSOR_TEMP_NUM_BITS);
    // load pressure data
    sensor_bit_pack(payload_bfr,SENSOR_PRESSURE_IDX,bme280_data.pressure,SENSOR_PRESSURE_NUM_BITS);
}
/*********************************************************************/
/*********************************************************************/
/*********************************************************************/
