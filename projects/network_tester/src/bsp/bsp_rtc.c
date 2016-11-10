#define BSP_RTC_C_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include "em_chip.h"
#include "em_cmu.h"
#include "em_rtc.h"
#include "bsp_rtc.h"
#include "debug_print.h"

// \addtogroup Modules
//   @{
//

// \addtogroup RTC
//   @{
//

static RTC_Init_TypeDef s_rtc_init;   //!< RTC configuration struct

uint8_t bsp_rtc_init(void)
{
    uint8_t ret = EXIT_SUCCESS;

    // The LF external oscillator should already enabled and selected as the
    // LFA source.
    CMU_ClockEnable(cmuClock_RTC, true);

    // Configure the RTC
    s_rtc_init.enable = true;       // Enable the RTC on init
    s_rtc_init.comp0Top = false;    // No counter compare
    s_rtc_init.debugRun = true;     // RTC runs while debugging

    RTC_Init(&s_rtc_init);

    return ret;
}

void bsp_rtc_start(void)
{
    RTC_Enable(true);
}

void bsp_rtc_stop(void)
{
    RTC_Enable(false);
}

// @} (end addtogroup RTC)
// @} (end addtogroup Modules)
