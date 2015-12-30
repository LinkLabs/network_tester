/**
  * \file    debug.h
  * \brief   Debug Module
  * \author  Worldsensing
  *
  * \section License
  *          (C) Copyright 2013 Worldsensing, http://www.worldsensing.com
  */
#ifndef __DEBUG_H__
#define __DEBUG_H__


#ifdef __DEBUG_C__
#define DEBUG_EXT
#else
#define DEBUG_EXT extern
#endif


#ifdef DEBUG
    #define DEBUG_PRINT_ENABLED  1
#else
    #define DEBUG_PRINT_ENABLED  0
#endif



/**
  Debug_Printf("OnlyText\n"); Is not suportted is necessary do the following trick
  GCC support putting ## in front of __VA_ARGS__
*/
#ifdef DEBUG
    #define Debug_Printf(fmt, ...)                  \
        do {                                        \
            if (DEBUG_PRINT_ENABLED) {              \
                Debug_PrintfFunc(fmt, ##__VA_ARGS__); \
            }                                       \
        } while(0)
#else
    #define Debug_Printf(fmt, ...)
#endif

#ifdef DEBUG
    #define Debug_Print(S)                          \
        do {                                        \
            if (DEBUG_PRINT_ENABLED) {              \
                Debug_PrintFunc(S);                 \
            }                                       \
        } while(0)
#else
    #define Debug_Print(S)
#endif

void Debug_Init (void);
void Debug_PrintFunc(const char *p_string);
void Debug_PrintfFunc(const char * format, ...);


#endif
