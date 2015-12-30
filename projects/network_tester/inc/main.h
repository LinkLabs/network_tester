#ifndef __MAIN_H_
#define __MAIN_H_

#include "bsp.h"

// Uncomment the following line to include the hard fault handler.
// Useful for troubleshooting memory access and overflow errors.
//#define INCLUDE_HARDFAULT_HANDLER


#define EFM32_HFXO_FREQ     (HFCLK_FREQ)

typedef enum {
    REP_APP_MODE_RUN    = 0,        //!< Standard operating mode of REP
    REP_APP_MODE_TEST,              //!< Test mode of REP
    NUM_REP_APP_MODES
} rep_app_mode_t;

void rep_app_mode_set(rep_app_mode_t mode, uint8_t reboot_after_change);
void rep_app_mode_get(rep_app_mode_t* mode);
void rep_new_app_reboot(void);

#endif /* __MAIN_H_ */
