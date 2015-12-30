//
// \file    bsp.h
// \brief   BSP Module Header.
//          System initialization and high-level peripheral control.
//
// \copyright LinkLabs, 2015
//
#ifndef __BSP_H__
#define __BSP_H__

#include "em_assert.h"

#include "iomap.h"

// Includes ------------------------------------------------------------------
// Exported macros -----------------------------------------------------------
#define LL_UNUSED           __attribute__((unused))

#define LL_ASSERT           EFM_ASSERT
#define HFCLK_FREQ          (32000000UL)                //!< System HF oscillator frequency

// Exported function prototypes ----------------------------------------------
uint8_t bsp_system_init(void);

void bsp_trigger_software_reset(void);

#endif // __BSP_H__
