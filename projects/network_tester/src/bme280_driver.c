/*********************************************************************/
// compensation code for BME280, copied from datasheet
/*********************************************************************/
/*****INCLUDES********************************************************/
//-----Standard Libraries-----//
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
//-----EFM Libraries-----//
#include "em_chip.h"
#include "em_i2c.h"
//-----My Libraries-----//
#include "bsp.h"
#include "bsp_i2c.h"
#include "iomap.h"
#include "bme280_driver.h"
/*********************************************************************/
/*****DEFINES*********************************************************/
#define BME280_ADDR             0xEC
#define BME280_CAL0_START_ADDR  0x88
#define BME280_CAL1_START_ADDR  0xE1
#define BME280_CTRL_HUM_ADDR    0xF2
#define BME280_CTRL_MEAS_ADDR   0xF4
#define BME280_DATA_START_ADDR  0xF7
/*********************************************************************/
/*****TYPEDEFS/STRUCTS************************************************/
/*********************************************************************/
/*****CONSTANTS*******************************************************/
const uint8_t BME280_config[] = {BME280_CTRL_HUM_ADDR,
                                 0x01,  // 1x humidity oversampling
                                 BME280_CTRL_MEAS_ADDR,
                                 0x27};  // 1x temp/pressure oversampling, normal mode
/*********************************************************************/
/*****VARIABLES*******************************************************/
I2C_TypeDef* BME280_i2c;

bme280_data_t bme280_data;

int32_t t_fine;
// compensation values
uint16_t dig_T1;
int16_t  dig_T2;
int16_t  dig_T3;
uint16_t dig_P1;
int16_t  dig_P2;
int16_t  dig_P3;
int16_t  dig_P4;
int16_t  dig_P5;
int16_t  dig_P6;
int16_t  dig_P7;
int16_t  dig_P8;
int16_t  dig_P9;
uint8_t  dig_H1;
int16_t  dig_H2;
uint8_t  dig_H3;
int16_t  dig_H4;
int16_t  dig_H5;
int8_t   dig_H6;
/*********************************************************************/
/*****PRIVATE FUNCTION PROTOTYPES*************************************/
static int32_t BME280_compensate_T_int32(int32_t adc_T);
static uint32_t BME280_compensate_P_int64(int32_t adc_P);
static uint32_t bme280_compensate_H_int32(int32_t adc_H);
/*********************************************************************/
/*****PRIVATE FUNCTIONS***********************************************/
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
static int32_t BME280_compensate_T_int32(int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T>>3) - ((int32_t)(dig_T1<<1)))*((int32_t)(dig_T2)) >> 11));
    var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;

    return(T);
}
/*********************************************************************/
// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
static uint32_t BME280_compensate_P_int64(int32_t adc_P)
{
    int64_t var1, var2, p;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
    var2 = var2 + (((int64_t)dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);

    return (uint32_t)p;
}
/*********************************************************************/
// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
static uint32_t bme280_compensate_H_int32(int32_t adc_H)
{
    int32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) +
                ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r *
                ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                ((int32_t)dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (uint32_t)(v_x1_u32r>>12);
}
/*********************************************************************/
/*********************************************************************/
/*********************************************************************/
/*********************************************************************/
/*****PUBLIC FUNCTIONS************************************************/
void BME280_poll_sensor(bme280_data_t *data_ptr)
{
    (void) data_ptr; //unused

    I2C_TransferSeq_TypeDef BME_transfer;
    uint8_t data[26];
    uint8_t reg_addr[1];
    I2C_TransferReturn_TypeDef stat;
    uint32_t sens_tmp;
    int32_t processed_T;
    uint32_t processed_P;
    uint32_t processed_H;

    // get block of data
    reg_addr[0] = BME280_DATA_START_ADDR;
    BME_transfer.addr = BME280_ADDR;
    BME_transfer.flags = I2C_FLAG_WRITE_READ;
    BME_transfer.buf[0].data = reg_addr;
    BME_transfer.buf[0].len = 1;
    BME_transfer.buf[1].data = data;
    BME_transfer.buf[1].len = 8;
    stat = I2C_TransferInit(BME280_i2c,&BME_transfer);
    while(stat == i2cTransferInProgress)
    {
        stat = I2C_Transfer(BME280_i2c);
    }
    // process data
    sens_tmp = data[3];
    sens_tmp<<=8;
    sens_tmp |= data[4];
    sens_tmp<<=4;
    sens_tmp |= (data[5]>>4);
    processed_T = BME280_compensate_T_int32((int32_t)(sens_tmp));
    sens_tmp = data[0];
    sens_tmp<<=8;
    sens_tmp |= data[1];
    sens_tmp<<=4;
    sens_tmp |= (data[2]>>4);
    processed_P = BME280_compensate_P_int64((int32_t)(sens_tmp));
    sens_tmp = data[6];
    sens_tmp<<=8;
    sens_tmp |= data[7];
    processed_H = bme280_compensate_H_int32((int32_t)(sens_tmp));
    // store data in structure
    bme280_data.temp = (int16_t)(processed_T);
    bme280_data.pressure = (uint32_t)(processed_P);
    bme280_data.humidity = (uint8_t)((processed_H>>9) & 255);
}
/*********************************************************************/
uint32_t init_BME280(I2C_TypeDef *i2c)
{
    I2C_TransferSeq_TypeDef BME_transfer;
    uint8_t comp_data[26];
    uint8_t reg_addr[1];
    I2C_TransferReturn_TypeDef stat;

    BME280_i2c = i2c;
    // get first block of compensation values
    reg_addr[0] = BME280_CAL0_START_ADDR;
    BME_transfer.addr = BME280_ADDR;
    BME_transfer.flags = I2C_FLAG_WRITE_READ;
    BME_transfer.buf[0].data = reg_addr;
    BME_transfer.buf[0].len = 1;
    BME_transfer.buf[1].data = comp_data;
    BME_transfer.buf[1].len = 26;
    stat = I2C_TransferInit(i2c,&BME_transfer);
    while(stat == i2cTransferInProgress)
    {
        stat = I2C_Transfer(i2c);
    }
    // store compensation values
    dig_T1 = comp_data[1];
    dig_T1<<=8;
    dig_T1 |= comp_data[0];

    dig_T2 = comp_data[3];
    dig_T2<<=8;
    dig_T2 |= comp_data[2];

    dig_T3 = comp_data[5];
    dig_T3<<=8;
    dig_T3 |= comp_data[4];

    dig_P1 = comp_data[7];
    dig_P1<<=8;
    dig_P1 |= comp_data[6];

    dig_P2 = comp_data[9];
    dig_P2<<=8;
    dig_P2 |= comp_data[8];

    dig_P3 = comp_data[11];
    dig_P3<<=8;
    dig_P3 |= comp_data[10];

    dig_P4 = comp_data[13];
    dig_P4<<=8;
    dig_P4 |= comp_data[12];

    dig_P5 = comp_data[15];
    dig_P5<<=8;
    dig_P5 |= comp_data[14];

    dig_P6 = comp_data[17];
    dig_P6<<=8;
    dig_P6 |= comp_data[16];

    dig_P7 = comp_data[19];
    dig_P7<<=8;
    dig_P7 |= comp_data[18];

    dig_P8 = comp_data[21];
    dig_P8<<=8;
    dig_P8 |= comp_data[20];

    dig_P9 = comp_data[23];
    dig_P9<<=8;
    dig_P9 |= comp_data[22];

    dig_H1 = comp_data[25];
    // get second block of compensation values
    reg_addr[0] = BME280_CAL1_START_ADDR;
    BME_transfer.addr = BME280_ADDR;
    BME_transfer.flags = I2C_FLAG_WRITE_READ;
    BME_transfer.buf[0].data = reg_addr;
    BME_transfer.buf[0].len = 1;
    BME_transfer.buf[1].data = comp_data;
    BME_transfer.buf[1].len = 7;
    stat = I2C_TransferInit(i2c,&BME_transfer);
    while(stat == i2cTransferInProgress)
    {
        stat = I2C_Transfer(i2c);
    }
    // store compensation values
    dig_H2 = comp_data[1];
    dig_H2<<=8;
    dig_H2 |= comp_data[0];

    dig_H3 = comp_data[2];

    dig_H4 = comp_data[3];
    dig_H4<<=4;
    dig_H4 |= (comp_data[4] & 0x0F);

    dig_H5 = comp_data[5];
    dig_H5<<=4;
    dig_H5 |= (comp_data[4] >> 4);

    dig_H6 = comp_data[6];

    // init control registers
    BME_transfer.flags = I2C_FLAG_WRITE;
    BME_transfer.buf[0].data = (uint8_t*) BME280_config;
    BME_transfer.buf[0].len = sizeof(BME280_config);
    stat = I2C_TransferInit(i2c,&BME_transfer);
    while(stat == i2cTransferInProgress)
    {
        stat = I2C_Transfer(i2c);
    }

    return(EXIT_SUCCESS);
}
/*********************************************************************/
void BME280_get_data(bme280_data_t *data_ptr)
{
    *data_ptr = bme280_data;
}
/*********************************************************************/
/*********************************************************************/
/*********************************************************************/
/*********************************************************************/
