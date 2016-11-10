#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "em_chip.h"
#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_i2c.h"
#include "bsp.h"
#include "bsp_i2c.h"
#include "iomap.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"

uint8_t bsp_i2c_init(I2C_TypeDef *i2c,uint8_t location)
{
    I2C_Init_TypeDef i2c_init;
    uint8_t ret = EXIT_SUCCESS;

    CMU_ClockEnable(cmuClock_I2C0, true);

    i2c->ROUTE = I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN | (location << _I2C_ROUTE_LOCATION_SHIFT);
    i2c_init.enable = true;
    i2c_init.master = true;
    i2c_init.refFreq = 0;
    i2c_init.freq = I2C_BUS_FREQ;
    I2C_Init(i2c,&i2c_init);

    return ret;
}
