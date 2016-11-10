#ifndef BSP_I2C_H_INCLUDED
#define BSP_I2C_H_INCLUDED

#define I2C_BUS_FREQ        200000

uint8_t bsp_i2c_init(I2C_TypeDef *i2c,uint8_t location);

#endif /* BSP_I2C_H_INCLUDED */
