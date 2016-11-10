#ifndef __LL_IFC_TRANSPORT_PC_H
#define __LL_IFC_TRANSPORT_PC_H

/**
 * @file
 *
 * @brief Enable serial port access to LinkLabs modules on PCs including
 *		Windows, Linux and OS X.
 */

 #ifdef __cplusplus
extern "C" {
#endif



#define LL_TTY_DEFAULT_BAUDRATE 115200

#ifdef __APPLE__
/* On OS X, Opening /dev/cu* doesn't require DCD to be asserted and succeeds immediately. */
#define LL_TTY_DEFAULT_DEVICE				"/dev/cu.SLAB_USBtoUART"
#else
    #if defined(WIN32) || defined (__MINGW32__)
        #define LL_TTY_DEFAULT_DEVICE      "\\\\.\\COM3"
    #else
        #define LL_TTY_DEFAULT_DEVICE      "/dev/ttyUSB0"
    #endif
#endif

/**
 * @brief Open the serial port to the LinkLabs module.
 * 
 * @param dev_name The device name string.  NULL uses the default.
 * @param baudrate The desired baud rate.  0 uses the default and is
 *      recommended for normal use.
 * @return 0 or error code.
 */
int ll_tty_open(const char * dev_name, int baudrate);

/**
 * @brief Close the serial port to the LinkLabs module.
 *
 * @return 0 or error code.
 */
int ll_tty_close(void);


#ifdef __cplusplus
}
#endif

#endif // __LL_IFC_TRANSPORT_PC_H
