/*********************************************************************/
/*********************************************************************/
//
// \file    main.c
// \brief   network tester main application
// \author  Mark Bloechl
// \author  Adrian Sapio
// \author  Scott Wohler
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
#include "em_device.h"
#include "em_chip.h"
#include "em_usart.h"
//-----My Libraries-----//
#include "bsp.h"
#include "bsp_timer.h"
#include "bsp_trace.h"
#include "bsp_uart.h"
#include "bsp_watchdog.h"
#include "debug_print.h"
#include "iomap.h"
#include "ui_task.h"
#include "gps_task.h"
#include "sensor_task.h"
#include "supervisor.h"
#include "lcd_nhd.h"
#include "ll_ifc.h"
//-----FreeRTOS Libraries-----//
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
/*********************************************************************/
/*****DEFINES*********************************************************/

/*********************************************************************/
/*****TYPEDEFS/STRUCTS************************************************/

/*********************************************************************/
/*****VARIABLES*******************************************************/

/*********************************************************************/
/*****PRIVATE FUNCTION PROTOTYPES*************************************/
static int32_t init_hw(void);
/*********************************************************************/
/*****FUNCTIONS*******************************************************/
int main(void)
{
    // initialize HW
    if(init_hw() != EXIT_SUCCESS)
    {
        EFM_ASSERT(0);
        bsp_trigger_software_reset();
    }
    // Initialize the watchdog module
    if (wdg_init() != EXIT_SUCCESS)
    {
        EFM_ASSERT(0);
        bsp_trigger_software_reset();
    }
    // init screen and button tasks
    if(init_user_interface() != EXIT_SUCCESS)
    {
        EFM_ASSERT(0);
        bsp_trigger_software_reset();
    }
    // init GPS module
    if(init_gps_task() != EXIT_SUCCESS)
    {
        EFM_ASSERT(0);
        bsp_trigger_software_reset();
    }
    // init LL module
    if(init_supervisor_task() != EXIT_SUCCESS)
    {
        EFM_ASSERT(0);
        bsp_trigger_software_reset();
    }
    // init environmental and light sensors
    if(init_sensor_task() != EXIT_SUCCESS)
    {
        EFM_ASSERT(0);
        bsp_trigger_software_reset();
    }
    // Start FreeRTOS Scheduler
    vTaskStartScheduler();

    return 0;
}
/*********************************************************************/
/*********************************************************************/
static int32_t init_hw(void)
{
    /* If first word of user data page is non-zero, enable eA Profiler trace */
    BSP_TraceProfilerSetup();

    /* Chip errata */
    CHIP_Init();

    bsp_system_init();

    Debug_Printf("Booting...\n");
    Debug_Printf("init_hw success\n");

    return(EXIT_SUCCESS);
}
/*********************************************************************/
void vApplicationStackOverflowHook(LL_UNUSED xTaskHandle xTask, signed char *pcTaskName)
{
    Debug_Printf("SOVF %s\n",pcTaskName);
    EFM_ASSERT(0);
}
/*********************************************************************/
/*********************************************************************/

