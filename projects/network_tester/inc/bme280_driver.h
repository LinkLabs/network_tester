#ifndef BME280_DRIVER_H_INCLUDED
#define BME280_DRIVER_H_INCLUDED

#include "em_i2c.h"

typedef struct
{
    uint8_t  humidity;
    int16_t  temp;
    uint32_t pressure;
} bme280_data_t;

uint32_t init_BME280(I2C_TypeDef *i2c);
void BME280_poll_sensor(bme280_data_t *data_ptr);
void BME280_get_data(bme280_data_t *data_ptr);

#endif /* BME280_DRIVER_H_INCLUDED */
