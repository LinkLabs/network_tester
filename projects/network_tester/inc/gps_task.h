#ifndef GPS_TASK_H_INCLUDED
#define GPS_TASK_H_INCLUDED

typedef struct
{
    int32_t  latitude;
    int32_t  longitude;
    int16_t  altitude;
    uint32_t ehpe;
    uint32_t evpe;
    uint32_t sat_id_list;
    uint8_t  cnt;
    uint8_t  fix_flag;
    uint32_t fix_age_s;
} gps_fix_t;

uint8_t init_gps_task(void);
void gps_get_latest_fix(gps_fix_t* fix);
void gps_build_packet(uint8_t* payload_bfr,uint8_t msg_num);

#endif /* GPS_TASK_H_INCLUDED */
