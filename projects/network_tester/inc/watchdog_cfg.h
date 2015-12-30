/**
  * \file    watchdog_cfg.h
  * \brief   Watchdog Config Header File: Defines for watchdog.c  
  * \author  Worldsensing  
  *
  * \section License
  *          (C) Copyright 2014 Worldsensing, http://www.worldsensing.com
  */

#ifndef  __WATCHDOG_C__

#error  This header can only be included from watchdog.c

#endif

#include "FreeRTOS.h"

/** \addtogroup Watchdog
  *   @{
  */

#define WDG_TASK_PRIORITY    (tskIDLE_PRIORITY + configMAX_PRIORITIES - 1) 
#define WDG_TASK_STACK_SIZE  100u

#define WDG_MAX_NUM           10

#define WDG_INIT_DEFAULT                                                                         \
  { false,              /* Start watchdog when init done */                                      \
    false,              /* WDOG not counting during debug halt */                                \
    true,               /* WDOG not counting when in EM2 */                                      \
    true,               /* WDOG not counting when in EM3 */                                      \
    false,              /* EM4 can be entered */                                                 \
    true,               /* Do not block disabling LFRCO/LFXO in CMU */                           \
    false,              /* Do not lock WDOG configuration (if locked, reset needed to unlock) */ \
    wdogClkSelULFRCO,   /* Select 1kHZ WDOG oscillator */                                        \
    wdogPeriod_8k       /* Set longest possible timeout period */                                \
  }

#define WDG_FUNCTION_RESET() \
    do { \
        NVIC_SystemReset(); \
   } while (0)

/** @} (end addtogroup Watchdog)   */
