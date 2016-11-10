#ifndef __LL_IFC_UTIL_H
#define __LL_IFC_UTIL_H

#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include "ll_ifc_utils.h"
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
 * @defgroup UTILS_Interface Symphony Utility Functions
 *
 * @brief Symphony Utility Functions.
 * These are probably useful.
 *
 * @{
 */

/**
* @brief The structure used to store time.
*/
struct time
{
    long tv_sec;
	long tv_nsec;
};

/**
 * @brief
 *   Calculate a 16-bit CRC Checksum.
 *
 * @param[in] ptr
 *   The data you want to calculate the checksum for.
 *
 * @param[in] count
 *   The number of bytes to included the CRC calculation.
 *
 * @return
 *   The 16-bit CRC Checksum.
 *
 * source: Amulet Technologies, LLC (MIT)
 */
uint16_t crc16(char *ptr, int count);

/**
 * @brief
 *   Calculate a 32-bit CRC Checksum.
 *
 * @param[in] crc
 *   Initial value for CRC Calculation. Commonly 0.
 *
 * @param[in] buf
 *   The data you want to calculate the checksum for.
 *
 * @param[in] len
 *   The number of bytes included in the CRC calculation.
 *
 * @return
 *   The 32-bit CRC Checksum.
 *
 * source: Zlib
 */
uint32_t crc32(uint32_t crc, uint8_t *buf, size_t len);

/**
 * @brief
 *   Get the diffrence  between two times in seconds.
 *
 * @param[in] a
 *   The start time.
 *
 * @param[out] b
 *   The end time.
 *
 * @return
 *   The floor of the number of seconds (b-a).
 */
int32_t ll_difftime(struct time *a, struct time *b);

/**
 * @breif
 *   Get the diffrence from a start time and when the functions called.
 *
 * @param[in] a
 *   The start time.
 *
 * @return
 *   The floor of the number of seconds (now-a).
 */
int32_t ll_difftime_from_now(struct time *a);

/** @} (end defgroup Symphony_Interface) */

/** @} (end addtogroup Link_Labs_Interface_Library) */

#ifdef __cplusplus
}
#endif

#endif //__LL_IFC_UTIL_H
