#ifndef TSL4531_DRIVER_H_INCLUDED
#define TSL4531_DRIVER_H_INCLUDED
#include "em_i2c.h"

uint16_t TSL4531_poll_sensor(void);
uint32_t init_TSL4531(I2C_TypeDef *i2c);
uint16_t TSL4531_get_data(void);

#endif /* TSL4531_DRIVER_H_INCLUDED */
