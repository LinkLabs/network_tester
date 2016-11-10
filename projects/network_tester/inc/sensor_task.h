#ifndef SENSOR_TASK_H_INCLUDED
#define SENSOR_TASK_H_INCLUDED

uint8_t init_sensor_task(void);
void sensor_build_packet(uint8_t* payload_bfr,uint8_t msg_num);

#endif /* SENSOR_TASK_H_INCLUDED */
