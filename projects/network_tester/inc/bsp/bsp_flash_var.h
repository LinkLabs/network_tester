#ifndef __BSP_FLASH_VAR_H__
#define __BSP_FLASH_VAR_H__

#include <stdbool.h>
#include <stdint.h>

typedef void (*flash_var_done_callback_t)(void);

/**
 * @brief
 *   Initializes the flash_var task.  This also checks the variable format
 *   version, and performs an update if necessary.
 * 
 * @return EXIT_SUCCESS/EXIT_FAILURE
 */
int32_t init_flash_var(void);

/**
 * @brief
 *   This functions sets a flag, so that the current state is saved to flash
 *   on the next call to flash_var_task()
 *
 * @note
 *   WARNING: When a variable has been updated and must be written to flash
 *   interrupts must be disabled.
 */
void flash_var_request_store(flash_var_done_callback_t callback);

/**
 * @brief
 *   This functions sets a flag, so that the current state is loaded from flash
 *   on the next call to flash_var_task()
 */
void flash_var_request_load(void);

/**
 * @brief
 *   This functions sets a flag, so that the all flash variables are removed.
 *
 * @note
 *   WARNING: When a variable has been updated and must be written to flash
 *   interrupts must be disabled.
 */
void flash_var_request_delete_all(void);

#endif /* __BSP_FLASH_VAR_H__ */
