//
// \file    bsp_watchdog.c
// \brief   Watchdog Module must be controlling that the rest of the modules don't stuck
//          in a permanent loop, or in a blocking state. If a fail in one task is detected system
//          will be rebooted and watchdog should say which task has failed.
//          Update the system hw watchdog.
//
// \copyright LinkLabs, 2015
//
#define WATCHDOG_C_

// Includes ------------------------------------------------------------------
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include "em_chip.h"
#include "em_wdog.h"
#include "em_rmu.h"
#include "em_cmu.h"

#include "FreeRTOS.h"
#include "task.h"
#include "task_mgmt.h"

#include "bsp.h"
#include "bsp_watchdog.h"
#include "debug_print.h"
//#include "main.h"

// \addtogroup Modules
//   @{
//

// \addtogroup Watchdog
//   @{
//

// Uncomment the following line to prevent the watchdog from resetting
// the module.
//#define CONFIG_NO_WATCHDOG_RESET


// Private macros ------------------------------------------------------------
#define WDG_REFRESHED    1u
#define WDG_UNREFRESHED  0u

#ifdef NDEBUG
#define BOOTLOADER_MAGIC_0      (0xa58b2bd9)
#define BOOTLOADER_MAGIC_1      (0x6e3349e6)
#define BOOTLOADER_MAGIC_2      (0xaf06b2bc)
#define BOOTLOADER_MAGIC_3      (0x803026a8)
#endif

#define WDG_MAX_NUM             12

#define WDG_INIT_DEFAULT                                                                                \
  {   .enable     = true,               /* Start watchdog when init done */                             \
      .debugRun   = false,              /* WDOG not counting during debug halt */                       \
      .em2Run     = true,               /* WDOG counting when in EM2 */                                 \
      .em3Run     = true,               /* WDOG counting when in EM3 */                                 \
      .em4Block   = true,               /* EM4 can be entered */                                        \
      .swoscBlock = false,              /* Do not block disabling LFRCO/LFXO in CMU */                  \
      .lock       = true,               /* Lock WDOG configuration (reset needed to unlock) */          \
      .clkSel     = wdogClkSelULFRCO,   /* Select 1kHZ WDOG oscillator */                               \
      .perSel     = wdogPeriod_8k,      /* Set the watchdog period to 8k clock cycles (ie ~8 seconds)*/ \
  }

#define WDG_FUNCTION_RESET() \
    do { \
        NVIC_SystemReset(); \
   } while (0)


// Private types -------------------------------------------------------------
typedef struct wdt_data {
    uint8_t       enabled:1;         //!< Watchdog is enabled
    uint8_t       refreshed:1;       //!< Watchdog has been refreshed WDG_REFRESHED or WDG_UNREFRESHED
    uint16_t      last_refresh_num;  //!< Watchdog last refresh times  in last watchdog cycle
    uint32_t      total_refresh_num; //!< Watchdog average refresh times in all watchdog cycles
    const char    *name_ptr;         //!< Watchdog name assigned by the module on Wdg_Register()
} wdg_data_t;

typedef struct wdt {
    uint8_t      wdg_num;            //!< Number of watchdog registred
    uint8_t      wdg_hw_enabled;     //!< HW watchdog is enabled or not
    wdg_data_t   wdg[WDG_MAX_NUM];   //!< Watchdog data array
} wdg_t;

static xTaskHandle   s_wdog_task_handle;

// Private variables ---------------------------------------------------------
static wdg_t             wdg_table;
static uint32_t          wdg_cycles = 0;
static WDOG_Init_TypeDef s_wdg_hw_config = WDG_INIT_DEFAULT;
static uint32_t          s_wdg_reset_cause = 0;

static bool              s_force_reset = false;

#ifdef NDEBUG
/**
 * Special memory space to hold the magic sentinel value to enter bootloader mode from software
 * The linker will only reserve 16 bytes for this space
 */
uint32_t bootloader_magic[4] __attribute__ ((section (".__data_array__"))) = { 0 };
#endif

// Private function prototypes -----------------------------------------------
static void wdg_hw_enable(void);
static void wdg_hw_disable(void);

static void wdg_task(__attribute__((unused))  void *p_param);

static void wdg_print_info(void);

// Private functions ---------------------------------------------------------
//
// \brief Enable hardware watchdog
//
static void wdg_hw_enable(void)
{
    WDOG_Init(&s_wdg_hw_config);
}

//
// \brief Disable hardware watchdog
//
static void wdg_hw_disable(void)
{
    WDOG_Enable(false);
}

// Exported functions --------------------------------------------------------
//
// \brief   Creates watchdog task and init necessary structs.
//
// \return  true if init is successful, false otherwise.
// \note    This function must be called in main before the init tasks (FreeRTOS).
//
uint8_t wdg_init(void)
{
    uint8_t i;

    Debug_Printf("Initializing watchdog task... \n");

    s_wdg_reset_cause = RMU_ResetCauseGet();
    RMU_ResetCauseClear();

//    wdg_table.wdg_num = 0; //this disables the watchdog
    wdg_table.wdg_hw_enabled = WDG_ENABLED;

    for (i = 0; i < WDG_MAX_NUM; i++) {
        wdg_table.wdg[i].enabled = WDG_DISABLED;
        wdg_table.wdg[i].name_ptr = NULL;
        wdg_table.wdg[i].refreshed = WDG_REFRESHED;
        wdg_table.wdg[i].total_refresh_num = 0;
        wdg_table.wdg[i].last_refresh_num = 0;
    }

    if (pdPASS != xTaskCreate(wdg_task, (const portCHAR *) "Wdg", WDG_TASK_STACK_SIZE, NULL, WDG_TASK_PRIORITY, &s_wdog_task_handle))
    {
        return EXIT_FAILURE;
    }

    wdg_hw_enable();

    Debug_Printf("\nWatchDog Module Init\nRMU Reset cause: 0x%08x\n", s_wdg_reset_cause);

#ifdef CHECK_STACK_USAGE
    // Task init complete. Register task and some metrics with main info structure
    task_info_t info = {
            .task_handle = s_wdog_task_handle,
            .stack_size = WDG_TASK_STACK_SIZE,
            .stack_use_percentage = 0
    };
    register_task(TASK_WDOG, &info);
#endif

    Debug_Printf("wdg_init success\n");

    return EXIT_SUCCESS;
}

//
// \brief Starts the normal behavior of the virtual watchdog handled by the handler argument.
//
// \param handler Handler/Identifier of the virtual watchdog to enable.
//
void wdg_enable(wdg_handler_t handler)
{
    if(handler >= wdg_table.wdg_num)
    {
        return;
    }
    taskENTER_CRITICAL();
    wdg_table.wdg[handler].enabled = WDG_ENABLED;
    taskEXIT_CRITICAL();
}

//
// \brief Stops the normal behavior of the virtual watchdog handled by the handler argument.
//
// \param handler Handler/Identifier of the virtual watchdog to enable.
//
void wdg_disable(wdg_handler_t handler)
{
    if(handler >= wdg_table.wdg_num)
    {
        return;
    }
    taskENTER_CRITICAL();
    wdg_table.wdg[handler].enabled = WDG_DISABLED;
    taskEXIT_CRITICAL();
}

//
// \brief Returns the status of the watchdog specified by the handler parameter.
//
// \param handler Handler/Identifier of the virtual watchdog to enable.
//
// \return Status of watchdog when the handler specifies and existing Watchdog:
//         - WDG_ENABLED
//         - WDG_DISABLED
//         Otherwise:
//         - WDG_STATUS_ERROR
//
uint8_t wdg_get_status(wdg_handler_t handler)
{
    if(handler >= wdg_table.wdg_num)
    {
        return WDG_STATUS_ERROR;
    }

    return wdg_table.wdg[handler].enabled;
}



//
// \brief Return the last cause of reset and the last watchdog failed registered.
//
// \param p_lastResetCause Pointer to return the last reset cause of EFM32.
// \param p_wdg_failed     Pointer to return the pointer of the name of the last task failed registered (See Note 1).
//
// \note  1. p_wdg_failed it's only the last registered watchdog failed. When the last reset cause of EFM32 is a System Request Reset
//        executed by Wdg_Reset() that's mean this watchdog has failed.
//
void wdg_get_last_reset_info(uint32_t *p_lastResetCause, const char **p_wdg_failed)
{
    *p_wdg_failed = NULL;
    *p_lastResetCause = s_wdg_reset_cause;

/* Wdg_LastFailed is never set, so remove this unused code */
#if 0
    if (wdg_last_failed < wdg_table.wdg_num)
    {  /* Note 1 */
        *p_wdg_failed = wdg_table.wdg[wdg_last_failed].name_ptr;
        (void) p_wdg_failed;                  /* Avoid compiler error */
    }
#endif
}

//
// \brief Watchdog Register When a task needs its own Virtualized WatchDog,
//        must call this function to receive the handler related with the
//        virtualized watchdog assigned by the WDG library.
//
// \param p_name Pointer to the name of the module for the logs.
//
// \return Return:
//         - WDG_HANDLER_ERROR if register is not possible.
//         - Handler to manage the watchdog behavior from the modules.
//
// \note  1. By default watchdog is enabled.
//
wdg_handler_t wdg_register(const char *p_name)
{
    wdg_handler_t handler = wdg_table.wdg_num;

    if (WDG_MAX_NUM <= handler) {
        return WDG_HANDLER_ERROR;
    }

    wdg_table.wdg_num++;

    wdg_table.wdg[handler].name_ptr = p_name;
    wdg_table.wdg[handler].enabled = WDG_ENABLED;  /* Note 1 */

    return handler;
}

//
// \brief Is the ping to watchdog, refresh the watchdog.
//
// \param handler Watchdog handler.
//
void wdg_refresh(wdg_handler_t handler)
{

    if(handler >= wdg_table.wdg_num) {
        return;
    }

    taskENTER_CRITICAL();

    wdg_table.wdg[handler].refreshed = WDG_REFRESHED;
    wdg_table.wdg[handler].last_refresh_num++;
    wdg_table.wdg[handler].total_refresh_num++;

    taskEXIT_CRITICAL();
}

//
// \brief Print watchdog information.
//
static void wdg_print_info(void)
{
    uint8_t i = 0;

    Debug_Printf("\nwdg_cycles: %u\n", wdg_cycles);
    for (i = 0; i < wdg_table.wdg_num; i++) {
        Debug_Print(wdg_table.wdg[i].name_ptr);
        if (WDG_ENABLED == wdg_table.wdg[i].enabled) {
            Debug_Print(":Enabled ");
            Debug_Printf("LastRefreshNum: %u ", wdg_table.wdg[i].last_refresh_num);
            Debug_Printf("TotalRefreshNum: %u ", wdg_table.wdg[i].total_refresh_num);
            Debug_Printf("AvgRefreshNum: %u\n", wdg_table.wdg[i].total_refresh_num/wdg_cycles);
        }
        else {
            Debug_Print(":Disabled\n");
        }
    }
}

//
// \brief  Control if all enabled watchdogs are refreshed, if not it will be reset.
//
// \note 1. Is necessary clear the information as LastRefreshNum after the control loop, because the information
//          must be printed when a fail is detected.
//
static void wdg_control(void)
{
    uint8_t i;

    wdg_cycles++;

    for (i = 0; i < wdg_table.wdg_num; i++) {

        if (WDG_ENABLED == wdg_table.wdg[i].enabled && WDG_UNREFRESHED == wdg_table.wdg[i].refreshed) {
            Debug_Print("Watchdog Fail: ");
            Debug_Print(wdg_table.wdg[i].name_ptr);
//            wdg_SavewdgFail(i);

            wdg_print_info();
            wdg_trigger_reset();
        }
    }

    for (i = 0; i < wdg_table.wdg_num; i++) {                     /* Note 1 */
        wdg_table.wdg[i].refreshed = WDG_UNREFRESHED;
        wdg_table.wdg[i].last_refresh_num = 0;
    }

    if(!s_force_reset)
    {
        WDOG_Feed();
    }
    else
    {
#ifdef CONFIG_NO_WATCHDOG_RESET
        WDOG_Feed();
#endif
    }
}

//
// \brief Watchdog Reset Occurred.
//      Check if a watchdog reset is the reason the processor reset the last time.
//
// \param[in]   None.
// \param[out]  None.
// \return      True if Yes, False if no.
//
bool wdg_reset_occured(void)
{
    if (s_wdg_reset_cause == RMU_RSTCAUSE_WDOGRST)
    {
        return(true);
    }
    else
    {
        return(false);
    }
}

//
// \brief Reset the microcontroller by preventing the WDOG from being fed.
//
void wdg_trigger_reset(void)
{
    s_force_reset = true;
}

#ifdef NDEBUG
void wdg_trigger_bootloader(void)
{
    bootloader_magic[0] = BOOTLOADER_MAGIC_0;
    bootloader_magic[1] = BOOTLOADER_MAGIC_1;
    bootloader_magic[2] = BOOTLOADER_MAGIC_2;
    bootloader_magic[3] = BOOTLOADER_MAGIC_3;
    // Wdg_Reset();
    NVIC_SystemReset();
}
#endif

//
// \brief Watchdog Task.
//
// \param p_param Task parameters.
//
#ifdef UNIT_TEST
static void wdg_task(__attribute__((unused))  void *p_param)
{

}
#else
static void wdg_task(__attribute__((unused))  void *p_param)
{
    Debug_Print("WatchDog Task Running...\n");
    WDOG_Feed();

    if(s_wdg_reset_cause == RMU_RSTCAUSE_WDOGRST)
    {
        //set_host_ifc_irq_flags_bit(IRQ_FLAGS_WDOG_RESET);
    }

    for (;;)
    {
        vTaskDelay(WS_WATCHDOG_REC_TOUCH_PERIOD_TICKS);
        wdg_control();
    }
}
#endif

// @} (end addtogroup Watchdog)
// @} (end addtogroup Modules)
