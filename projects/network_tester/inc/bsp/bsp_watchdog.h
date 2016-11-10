#ifndef __WATCHDOG_H__
#define __WATCHDOG_H__

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"

#define WDG_EXT

#define WDG_DISABLED        0u
#define WDG_ENABLED         1u
#define WDG_STATUS_ERROR    255u

#define WDG_HANDLER_ERROR   255u

#define WS_WATCHDOG_REC_TOUCH_PERIOD_MS        2000u
#define WS_WATCHDOG_REC_TOUCH_PERIOD_TICKS     (WS_WATCHDOG_REC_TOUCH_PERIOD_MS / portTICK_RATE_MS)

typedef uint8_t wdg_handler_t;

uint8_t wdg_init(void);

void wdg_enable(wdg_handler_t handler);
void wdg_disable(wdg_handler_t handler);
uint8_t wdg_get_status(wdg_handler_t handler);

void wdg_get_last_reset_info(uint32_t *p_lastResetCause, const char **p_wdg_failed);
wdg_handler_t wdg_register(const char *p_name);
void wdg_refresh(wdg_handler_t handler);

bool wdg_reset_occured(void);
void wdg_trigger_reset(void);
void wdg_trigger_bootloader(void);

#endif  // __WATCHDOG_H__
