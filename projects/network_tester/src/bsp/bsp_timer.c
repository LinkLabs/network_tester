//
// \file    bsp_timer.c
// \brief   BSP Timer Module
//          Provides configuration, control, and other functions for the timer(s) on the
//          EFM32GG232F1024 mcu.
//
// \copyright LinkLabs, 2015
//
#define BSP_TIMER_C_

// Includes ------------------------------------------------------------------
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_rmu.h"
#include "em_timer.h"

#include "FreeRTOS.h"

#include "bsp_timer.h"
#include "debug_print.h"
//#include "interrupt.h"
#include "iomap.h"

// \addtogroup Modules
//   @{
//

// \addtogroup Timer
//   @{
//

// Private macros ------------------------------------------------------------
// Private types -------------------------------------------------------------
// Private variables ---------------------------------------------------------
static const uint8_t timer_prescale = timerPrescale16;

static TIMER_Init_TypeDef s_timer_init; //!< Timer configuration struct
static uint32_t s_timer_tick = 0;       //!< Timer tick for high resolution ticks

// Private function prototypes -----------------------------------------------
// Private functions ---------------------------------------------------------
// Exported functions --------------------------------------------------------
uint8_t bsp_tick_timer_init(void)
{
    uint8_t ret = EXIT_SUCCESS;
    uint32_t timer_freq;
    uint32_t timer_top_count;

    TIMER_Reset(TIMER0);

    // Configure a system timer for tracking a tick counter.
    CMU_ClockEnable(cmuClock_TIMER0, true);

    // Set the timer configuration
    s_timer_init.enable     = false;                // Enable timer when init complete.
    s_timer_init.debugRun   = true;                 // Stop counter during debug halt.

    s_timer_init.prescale   = timer_prescale;       // Prescale by 16 (2MHz clock)
    s_timer_init.clkSel     = timerClkSelHFPerClk;  // Select HFPER clock.

    s_timer_init.count2x    = false;                // Not 2x count mode.
    s_timer_init.ati        = false;                // No ATI.

    s_timer_init.fallAction = timerInputActionNone; // No action on falling input edge.
    s_timer_init.riseAction = timerInputActionNone; // No action on rising input edge.

    s_timer_init.mode       = timerModeUp;          // Up-counting.
    s_timer_init.dmaClrAct  = false;                // Do not clear DMA requests when DMA channel is active.
    s_timer_init.quadModeX4 = false;                // Select X2 quadrature decode mode (if used).

    s_timer_init.oneShot    = false;                // Disable one shot.
    s_timer_init.sync       = false;                // Not started/stopped/reloaded by other timers.

    // Configure TIMER
    TIMER_Init(TIMER0, &s_timer_init);

    // Set counter top for tick rate
    timer_freq = CMU_ClockFreqGet(cmuClock_TIMER0) / (1 << timer_prescale);
    timer_top_count = (timer_freq / TIMER_TICK_RATE_HZ) - 1;
    TIMER_TopSet(TIMER0, timer_top_count);

    return ret;
}

void bsp_tick_timer_start(void)
{
    // Enable the interrupt and start the timer
    TIMER_IntEnable(TIMER0, TIMER_IF_OF);
    NVIC_SetPriority(TIMER0_IRQn, 1);
    NVIC_EnableIRQ(TIMER0_IRQn);

    TIMER_Enable(TIMER0, true);
}

void bsp_tick_timer_stop(void)
{
    // Disable the interrupt and timer
    TIMER_Enable(TIMER0, false);
    TIMER_IntDisable(TIMER0, TIMER_IF_OF);
    NVIC_DisableIRQ(TIMER0_IRQn);
}

//
// \brief           A blocking delay that uses the systick.
// \param[in]   ms  Number of milliseconds to delay.
// \note            The core clock must be configured before using this function.
//                  The systick rate is configured in bsp.h as SYSTICK_RATE_HZ.
//
void bsp_delay_ms(uint32_t ms)
{
    uint32_t start_value;
    uint32_t delay;
    uint32_t elapsed;

    // Get the current systick counter value
    start_value = bsp_get_timer_tick();

    // Compute the number of ticks in ~1ms
    delay = (TIMER_TICK_COUNT_1MS * ms);

    do
    {
        elapsed = bsp_get_timer_tick() - start_value;
    }
    while (elapsed <= delay);
}

uint32_t bsp_get_timer_tick(void)
{
    return s_timer_tick;
}

void bsp_increment_timer_tick(void)
{
    s_timer_tick++;

#ifdef BSP_TEST_TIMER_TICK
    GPIO_PinOutToggle(MICRO_IO0_PORT, MICRO_IO0_PIN);
#endif
}

#ifdef BSP_TEST_TIMER
void bsp_test_timer(void)
{
    // This will block indefinitely
    while (1)
    {
        bsp_delay_ms(50);
        GPIO_PinOutToggle(MICRO_IO1_PORT, MICRO_IO1_PIN);
    }
}
#else
void bsp_test_timer(void)
{

}
#endif

void TIMER0_IRQHandler(void)
{
    // Clear the interrupt flag
    TIMER_IntClear(TIMER0, TIMER_IF_OF);

    // Increment the timer tick
    bsp_increment_timer_tick();
}

// @} (end addtogroup Timer)
// @} (end addtogroup Modules)
