#ifndef __BSP_RTC_H__
#define __BSP_RTC_H__

#include <stdbool.h>
#include <stdint.h>

#define RTC_RATE_HZ             (32768u)                        //!< RTC tick frequency
#define RTC_TICK_COUNT_1MS      (RTC_RATE_HZ / 1000u)           //!< Number of counts to achieve 1ms delay

uint8_t bsp_rtc_init(void);
void bsp_rtc_start(void);
void bsp_rtc_stop(void);

#endif  // __BSP_RTC_H__
