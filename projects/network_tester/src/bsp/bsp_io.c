//Includes
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"

#include "iomap.h"
#include "bsp_io.h"
#include "debug_print.h"



// Static Globals (file scope)
static bsp_hw_rev_t s_hw_rev = HW_REV_UNKNOWN;

// Determines hardware revision subsequently does pin assignments
// returns 0 if ok, -1 otherwise
int8_t bsp_io_init(void)
{
    // Set Pin F5 to input with internal pullup
    GPIO_PinModeSet(gpioPortF, 5, gpioModeInputPull, 1); //TODO set to pull UP not down?

    // Check Level
    switch ((uint8_t) GPIO_PinInGet(gpioPortF, 5))
    {
        case 0:
            s_hw_rev = HW_REV4;
            break;
        case 1:
            s_hw_rev = HW_REV3;
            break;
        default:
            s_hw_rev = HW_REV_UNKNOWN;
            EFM_ASSERT(false);
            return EXIT_FAILURE;
    }

    // Set pin to HI-Z
    GPIO_PinModeSet(gpioPortF, 5, gpioModeDisabled, 0);

    return EXIT_SUCCESS;
}

bsp_hw_rev_t bsp_hw_rev_get(void)
{
    return s_hw_rev;
}

uint8_t lcd_data_rd(void)
{
    uint8_t ret = 0;
    if(HW_REV3 == s_hw_rev)
    {
        ret = LCD_DATA_RD_r3();
    }
    else if(HW_REV4 == s_hw_rev)
    {
        ret = LCD_DATA_RD_r4();
    }
    else
    {
        EFM_ASSERT(0);
    }

    return ret;
}

void lcd_data(uint8_t val)
{
    if(HW_REV3 == s_hw_rev)
    {
        LCD_DATA_r3(val);
    }
    else if(HW_REV4 == s_hw_rev)
    {
        LCD_DATA_r4(val);
    }
    else
    {
        EFM_ASSERT(0);
    }
}


void lcd_dbx(uint8_t pin, uint8_t val)
{
    EFM_ASSERT(pin < 8);

    //todo
    if(HW_REV3 == s_hw_rev)
    {
        if(0 == pin)
        {
            LCD_DB0_r3(val);
        }
        else if(1 == pin)
        {
            LCD_DB1_r3(val);
        }
        else if(2 == pin)
        {
            LCD_DB2_r3(val);
        }
        else if(3 == pin)
        {
            LCD_DB3_r3(val);
        }
        else if(4 == pin)
        {
            LCD_DB4_r3(val);
        }
        else if(5 == pin)
        {
            LCD_DB5_r3(val);
        }
        else if(6 == pin)
        {
            LCD_DB6_r3(val);
        }
        else if(7 == pin)
        {
            LCD_DB7_r3(val);
        }
    }
    else if(HW_REV4 == s_hw_rev)
    {
        if(0 == pin)
        {
            LCD_DB0_r4(val);
        }
        else if(1 == pin)
        {
            LCD_DB1_r4(val);
        }
        else if(2 == pin)
        {
            LCD_DB2_r4(val);
        }
        else if(3 == pin)
        {
            LCD_DB3_r4(val);
        }
        else if(4 == pin)
        {
            LCD_DB4_r4(val);
        }
        else if(5 == pin)
        {
            LCD_DB5_r4(val);
        }
        else if(6 == pin)
        {
            LCD_DB6_r4(val);
        }
        else if(7 == pin)
        {
            LCD_DB7_r4(val);
        }
    }
    else
    {
        EFM_ASSERT(0);
    }
}


void lcd_e(uint8_t val)
{
    if(HW_REV3 == s_hw_rev)
    {
        LCD_E_r3(val);
    }
    else if(HW_REV4 == s_hw_rev)
    {
        LCD_E_r4(val);
    }
    else
    {
        EFM_ASSERT(0);
    }
}


void lcd_rnw(uint8_t val)
{
    if(HW_REV3 == s_hw_rev)
    {
        LCD_RnW_r3(val);
    }
    else if(HW_REV4 == s_hw_rev)
    {
        LCD_RnW_r4(val);
    }
    else
    {
        EFM_ASSERT(0);
    }
}


void lcd_rs(uint8_t val)
{
    if(HW_REV3 == s_hw_rev)
    {
        LCD_RS_r3(val);
    }
    else if(HW_REV4 == s_hw_rev)
    {
        LCD_RS_r4(val);
    }
    else
    {
        EFM_ASSERT(0);
    }
}


void lcd_bklt_en(uint8_t val)
{
    if(HW_REV3 == s_hw_rev)
    {
        LCD_BKLT_EN_r3(val);
    }
    else if(HW_REV4 == s_hw_rev)
    {
        LCD_BKLT_EN_r4(val);
    }
    else
    {
        EFM_ASSERT(0);
    }
}


void lcd_bklt_toggle(void)
{
    if(HW_REV3 == s_hw_rev)
    {
        LCD_BKLT_TOGGLE_r3();
    }
    else if(HW_REV4 == s_hw_rev)
    {
        LCD_BKLT_TOGGLE_r4();
    }
    else
    {
        EFM_ASSERT(0);
    }
}


