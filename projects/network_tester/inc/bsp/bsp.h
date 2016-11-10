#ifndef __BSP_H__
#define __BSP_H__

#include "em_assert.h"

#include "iomap.h"

#define LL_UNUSED           __attribute__((unused))
#define LL_RAM_FUNC         __attribute__ ((section(".ram")))

#define LL_ASSERT           EFM_ASSERT
#define HFCLK_FREQ          (32000000UL)                //!< System HF oscillator frequency

uint8_t bsp_system_init(void);

void bsp_trigger_software_reset(void);

#endif // __BSP_H__
