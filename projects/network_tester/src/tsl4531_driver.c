#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "em_chip.h"
#include "em_i2c.h"
#include "bsp.h"
#include "bsp_i2c.h"
#include "iomap.h"
#include "tsl4531_driver.h"

#define TSL4531_ADDR            0x52//0x29
// register addresses
#define TSL4531_CMD_BIT         0x80
#define TSL4531_CTRL_ADDR       0x00
#define TSL4531_CFG_ADDR        0x01
#define TSL4531_DLOW_ADDR       0x04
#define TSL4531_DHIGH_ADDR      0x05
#define TSL4531_ID_ADDR         0x0A
// control field options
#define TSL4531_CRTL_POWDOWN    0x00
#define TSL4531_CRTL_ONESHOT    0x02
#define TSL4531_CRTL_NORMALOP   0x03
// configuration field options
#define TSL4531_CFG_PSAVESKIP   0x08
#define TSL4531_CFG_TCNTRL_1X   0x00
#define TSL4531_CFG_TCNTRL_2X   0x01
#define TSL4531_CFG_TCNTRL_4X   0x02

const uint8_t TSL4531_config[] = {TSL4531_CMD_BIT|TSL4531_CTRL_ADDR,
                                  TSL4531_CRTL_NORMALOP,
                                  TSL4531_CFG_PSAVESKIP|TSL4531_CFG_TCNTRL_1X};

uint16_t light_level = 0;
I2C_TypeDef* TSL4531_i2c;

uint16_t TSL4531_poll_sensor(void)
{
    I2C_TransferSeq_TypeDef TSL_transfer;
    uint8_t data[2];
    uint8_t reg_addr[1];
    I2C_TransferReturn_TypeDef stat;

    // get block of data
    reg_addr[0] = TSL4531_CMD_BIT|TSL4531_DLOW_ADDR;
    TSL_transfer.addr = TSL4531_ADDR;
    TSL_transfer.flags = I2C_FLAG_WRITE_READ;
    TSL_transfer.buf[0].data = reg_addr;
    TSL_transfer.buf[0].len = 1;
    TSL_transfer.buf[1].data = data;
    TSL_transfer.buf[1].len = 2;
    stat = I2C_TransferInit(TSL4531_i2c,&TSL_transfer);
    while(stat == i2cTransferInProgress)
    {
        stat = I2C_Transfer(TSL4531_i2c);
    }

    light_level = data[1];
    light_level<<=8;
    light_level |= data[0];

    return(light_level);

}

uint32_t init_TSL4531(I2C_TypeDef *i2c)
{
    I2C_TransferSeq_TypeDef TSL_transfer;
    I2C_TransferReturn_TypeDef stat;

    TSL4531_i2c = i2c;
    // init registers
    TSL_transfer.addr = TSL4531_ADDR;
    TSL_transfer.flags = I2C_FLAG_WRITE;
    TSL_transfer.buf[0].data = (uint8_t*) TSL4531_config;
    TSL_transfer.buf[0].len = sizeof(TSL4531_config);
    TSL_transfer.buf[1].data = 0;
    TSL_transfer.buf[1].len = 0;
    stat = I2C_TransferInit(i2c,&TSL_transfer);
    while(stat == i2cTransferInProgress)
    {
        stat = I2C_Transfer(i2c);
    }

    return(EXIT_SUCCESS);
}

uint16_t TSL4531_get_data(void)
{
    return(light_level);
}
