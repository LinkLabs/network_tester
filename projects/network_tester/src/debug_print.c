/**
  * \file    debug.c
  * \brief   Debug library provide functions to use in develompent process.
  * \author  Worldsensing  
  *
  * \section License
  *          (C) Copyright 2013 Worldsensing, http://www.worldsensing.com
  */


#define __DEBUG_C__

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "em_chip.h"

#include "debug_print.h"

/** \addtogroup Modules
  *   @{
  */

/** \addtogroup Debug
  *   @{
  */
#define DEBUG_PRINT_BUF_SIZE  70u
/************************************ Local Var **********************************************/

static char Debug_PrintBuf[DEBUG_PRINT_BUF_SIZE];

/***************************** Function implementation ***************************************/

/**
 * \brief Print a string using SWO Cortex M4 ITM0
 *
 * \param p_string Pointer to string to print
 *
 * \todo  Mutex must to be implement ?????
 */
void Debug_PrintFunc(const char *p_string) {
    while (*p_string) {
        ITM_SendChar(*p_string++);    
    }
}

/**
 * \brief Printf function using SWO Cortex M4 ITM0
 *
 * \param format  Printf format string
 * \param ...     Printf arguments
 */
void Debug_PrintfFunc(const char * format, ...) {
    va_list arg;
    va_start(arg, format);
    vsnprintf(Debug_PrintBuf, DEBUG_PRINT_BUF_SIZE, (const char *) format, arg);
    va_end(arg);                        
    Debug_PrintFunc(Debug_PrintBuf);
}


/** @} (end addtogroup Debug)   */
/** @} (end addtogroup Modules) */

