#ifndef __LL_IFC_H
#define __LL_IFC_H

#include <stdint.h>
#include "ll_ifc_consts.h"
#include "ifc_struct_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

    /**
     * @addtogroup Link_Labs_Interface_Library
     * @{
     */

    /**
     * @addtogroup Module_Interface
     * @brief
     * @{
     */

    /**
     * The following transport function must be defined in order to use this library.
     * This function is usually a simple UART wrapper.
     *   0 - success, negative otherwise
     */
    int32_t transport_write(uint8_t *buff, uint16_t len);

    /**
     * The following transport function must be defined in order to use this library.
     * This function is usually a simple UART wrapper.
     *   0 - success, negative otherwise
     */
    int32_t transport_read(uint8_t *buff, uint16_t len);

    /**
     * @brief
     *   Get the module firmware type
     *
     * @param[out] t
     *   pointer to a firmware type struct
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_firmware_type_get(ll_firmware_type_t *t);

    /**
     * @brief
     *   Get the module hardware type
     *
     * @param[out] t
     *   pointer to a hardware type enum
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_hardware_type_get(ll_hardware_type_t *t);

    /**
     * @brief
     *   Get the host interface version number
     *
     * @param[out] version
     *   pointer to a version struct
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_interface_version_get(ll_version_t *version);

    /**
     * @brief
     *   Get the module firmware version number
     *
     * @param[out] version
     *   pointer to a version struct
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_version_get(ll_version_t *version);

    /**
	 * @brief
	 *   Block any sleep mode.
	 *
     * @details
     *   This function blocks any sleep mode.  ll_sleep_unblock() allows the
     *   module to enter into the sleep mode again.
     *
	 * @return
	 *   0 - success, negative otherwise
	 */
	int32_t ll_sleep_block(void);

    /**
	 * @brief
	 *   Unblock sleep modes
	 *
     * @details
     *   This function unblocks sleep mode regardless of how many times
     *   ll_sleep_block() has been called.
     *
	 * @return
	 *   0 - success, negative otherwise
	 */
	int32_t ll_sleep_unblock(void);

	/**
     * @brief
     *   Set the MAC Mode
     *
     * @param[in] mac_mode
     *   0 = No MAC (Pass through mode)
     *   1 = LoRaMAC (European version)
     *   2 = LoRaMAC (North American version)
     *   3 = Link Labs MAC v1
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_mac_mode_set(ll_mac_type_t mac_mode);

    /**
     * @brief
     *   Get the MAC Mode
     *
     * @param[out] mac_mode
     *   0 = No MAC (Pass through mode)
     *   1 = LoRaMAC (European version)
     *   2 = LoRaMAC (North American version)
     *   3 = Link Labs MAC v1
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_mac_mode_get(ll_mac_type_t *mac_mode);

    /**
     * @brief
     *   Get the module unique identifier
     *
     * @param[out] unique_id
     *   pointer to a unsigned 64-bit integer
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_unique_id_get(uint64_t *unique_id);

    /**
     * @brief
     *   Get the antenna configuration
     *
     * @param[out] ant
     *   Antenna configuration.
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_antenna_get(uint8_t *ant);

    /**
     * @brief
     *   Set the antenna configuration
     *
     * @param[in] ant
     *   Antenna configuration. (1=>U.FL, 2=>trace)
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_antenna_set(uint8_t ant);

     /**
     * @brief
     *   Store the current radio parameters to flash:
     *     - mode
     *     - frequency
     *     - bandwidth
     *     - spreading factor
     *     - coding rate
     *     - low_rate_opt
     *     - tx power
     *
     * @details
     *   This function stores settings to flash.
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_settings_store(void);

    /**
     * @brief
     *   Delete all settings from flash storage.
     *
     * @details
     *   This function stores settings to flash.
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_settings_delete(void);

    /**
     * @brief
     *   Restore the default radio parameters:
     *     - frequency
     *     - bandwidth
     *     - spreading factor
     *     - coding rate
     *     - low_rate_opt
     *     - tx power
     *
     * @details
     *   This function stores settings to flash.
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_restore_defaults(void);

    /**
     * @brief
     *   Force the module to enter sleep.
     *
     * @details
     *   This function puts the module into the sleep mode.  The wakeup
     *   signal is issued with a logic high pulse of at least 60 us duration.
     *   Alternatively, the module will enter the idle state in response to a received
     *   byte on the host UART.
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_sleep();

    /**
     * @brief
     *   Force the module to reset (takes a few seconds).
     *
     * @details
     *   This function forces the module to reset.
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_reset_mcu();

    /**
     * @brief
     *   Force the module to reset and enter bootloader mode (takes a few seconds).
     *
     * @details
     *   This function forces the module to reset and enter bootloader mode.
     *
     * @return
     *   0 - success, negative otherwise
     */
    int32_t ll_bootloader_mode();

     /**
      * @brief
      *   Read (and optionally clear) IRQ flags in module
      *
      * @details
      *   This function allows the host processor to check whether an event has occured in the
      *   module that has latched a bit in the "IRQ Flags" vector.
      *
      * @param[in] flags_to_clear
      *   A uint32_t bit vector containing flags that should be cleared if they are set. This can be
      *   0 if the host interface just wants to read without clearing. If a bit is set, this function
      *   performs a clear-on-read of the irq_flags bits passed in.
      *
      * @param[out] flags
      *   A uint32_t bit vector - the value of the irq_flags in the module. Note that if the flags_to_clear
      *   argument is non-zero, this argument is the value of the flags before the clear operation.
      *
      * @return
      *   0 - success, negative otherwise
      */
     int32_t ll_irq_flags(uint32_t flags_to_clear, uint32_t *flags);

#if 0
     /**
      * @brief
      *   Set IRQ flags in module
      *
      * @details
      *
      * @param[in] none
      *
      * @param[out] flags mask
      *
      * @return
      *   0 - success, negative otherwise
      */
     int8_t ll_irq_flags_mask_get(uint32_t flags_to_clear, uint32_t *flags);

     /**
      * @brief
      *   Set IRQ flags in module
      *
      * @details
      *
      * @param[in] none
      *
      * @param[out] flags mask
      *
      * @return
      *   0 - success, negative otherwise
      */
     int8_t ll_irq_flags_mask_set(uint32_t flags_to_clear, uint32_t *flags);
#endif
    /** @} (end addtogroup Module_Interface) */
    /** @} (end addtogroup Link_Labs_Interface_Library) */

#ifdef __cplusplus
}
#endif

#endif /* __LL_IFC_H */
