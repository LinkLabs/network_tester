//
// \file    bsp_io.h
// \brief   BSP IO Module Header.
//          Device GPIO
//
// \copyright LinkLabs, 2015
//
#ifndef __BSP_IO_H__
#define __BSP_IO_H__

// Includes
#include <stdbool.h>
#include <stdint.h>

// Types
typedef enum
{
    HW_REV3,
    HW_REV4,
    NUM_HW_REVS,
    HW_REV_UNKNOWN,
} bsp_hw_rev_t;


// Interface
int8_t bsp_io_init(void);
bsp_hw_rev_t bsp_hw_rev_get(void);

uint8_t lcd_data_rd(void);
void lcd_data(uint8_t val);
void lcd_dbx(uint8_t pin, uint8_t val);
void lcd_e(uint8_t val);
void lcd_rnw(uint8_t val);
void lcd_rs(uint8_t val);
void lcd_bklt_en(uint8_t val);
void lcd_bklt_toggle(void);

#endif  // __BSP_IO_H__
