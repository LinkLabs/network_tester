#ifndef __BSP_TIMER_H__
#define __BSP_TIMER_H__

#include <stdbool.h>
#include <stdint.h>


#define TIMER_TICK_RATE_HZ      (20000u)                        //!< Timer tick frequency (currently 50us)

#define TIMER_TICK_COUNT_1MS    (TIMER_TICK_RATE_HZ / 1000u)    //!< Number of counts to achieve 1ms delay

uint8_t bsp_tick_timer_init(void);
void bsp_tick_timer_start(void);
void bsp_tick_timer_stop(void);

void bsp_delay_ms(uint32_t ms);
uint32_t bsp_get_timer_tick(void);
void bsp_increment_timer_tick(void);

void bsp_test_timer(void);

void TIMER0_IRQHandler(void);

#endif  // __BSP_TIMER_H__
