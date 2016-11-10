#ifndef __LL_IFC_XMODEM_H
#define __LL_IFC_XMODEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ll_ifc.h"

#include <string.h>
#include <stdbool.h>

/**
 * @addtogroup Link_Labs_Interface_Library
 * @{
 */

/**
 * @defgroup XMODEM_Interface XModem File Transfer Protocol
 *
 * @breif XModem File Transfer Protocol.
 * An implementation of the XMODEM File Transfer Protocol using the Hardware Abstraction Layer.
 *
 * We use XModem a lot of times to communicate with the bootloaders of our modules and
 * other embedded devices to bootload the device with new firmware.
 *
 * The entire xmodem transfer utilizes the Hardware Abstraction Layer (HAL)
 * which means that as long as the user has properly defined the HAL, this
 * protocol should work on any platform.
 *
 * @{
 */

/**
*  @breif Gets called every time a packet gets sent.
*
*  @param[in] sent
*    The amount of bytes sent.
*
*  @param[in] total
*    The amount of bytes to send.
*
*  @return
*    0 - on success, negative otherwise.
*/
typedef int32_t (*ll_xmodem_progress_update_t)(uint32_t sent, uint32_t total);

typedef struct ll_xmodem_callbacks
{
    ll_xmodem_progress_update_t progress;
} ll_xmodem_callbacks_t;

/**
 * @breif
 *   Prepares the module to recieve firmware via XModem.
 *
 * @details
 *   * Puts the module in bootloader mode.
 *   * Waits 2 seconds.
 *   * Sends the bootloader the Upload Request.
 *   * Returns when the Module starts waiting for an XModem Request.
 *
 * @param[in] is_host_ifc_active
 *   True if the host interface is active.  False otherwise, indicating that the
 *   module is already running the bootloader.
 *
 * @return
 *   0 - success, negative otherwise.
 */
int32_t ll_xmodem_prepare_module(bool is_host_ifc_active);

/**
 * @breif
 *   Sends data through the Hardware Abstraction Layer (HAL) utilizing the
 *   XMODEM File Transfer Protocol.
 *
 * @details
 *   Make sure you have already implemented the HAL functions like transport_read,
 *   transport_write, and gettime for your target platform. This function will do
 *   a complete XModem Transfer and Verification. After the firmware is verified
 *   and activated the protocol will reboot the module.
 *
 * @param[in] cb
 *   The XModem callbacks (should be initialized).
 *
 * @param[in] payload
 *   Pointer to the buffer where the bytes will be stored.
 *
 * @param[in] len
 *    Number of bytes to read.
 *
 *  @return
 *    *  0 - success, negative otherwise.
 *    * -2 : Timeout.
 *    * -3 : Canceled.
 *    * -4 : Maxed out Attempts.
 *    * -5 : Callbacks aren't initialized.
 *    * -6 : Firmware did not download
 *    * -7 : Firmware did not verify
 *    * -8 : Firmware did not activate
 *    Other negative values are ifc errors.
 */
int32_t ll_xmodem_send(ll_xmodem_callbacks_t *cb, uint8_t* payload, size_t len);

/** @} (end defgroup XMODEM_Interface) */

/** @} (end addtogroup Link_Labs_Interface_Library) */


#ifdef __cplusplus
}
#endif

#endif // __LL_IFC_XMODEM_H
