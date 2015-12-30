#ifndef __OSP_H
#define __OSP_H

/************/
/* Includes */
/************/
#include <stdint.h>

/*********/
/* Types */
/*********/
typedef struct{
    int32_t latitude;
    int32_t longitude;
    int16_t altitude;
    uint32_t ehpe;
    uint32_t evpe;
    uint32_t sat_id_list;
    uint8_t cnt;
} gps_data_t;

/***********************/
/* Function Prototypes */
/***********************/
//int  osp_init(void);
//void osp_task(void);
void osp(uint8_t byte);
void osp_get_latest(gps_data_t *gps_data);
void osp_sw_version_poll();
void osp_tracker_config(void);
uint8_t osp_get_num_sats(gps_data_t* data);
uint16_t osp_get_fix_valid(void);	// returns zero if fix not valid

#endif  // #define __OSP_H
