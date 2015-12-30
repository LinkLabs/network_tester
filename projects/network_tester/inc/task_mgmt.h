#ifndef __TASK_MGMT_H__
#define __TASK_MGMT_H__

#define REP_TASK_PRIORITY               (tskIDLE_PRIORITY + 3)
#define BUTTON_TASK_PRIORITY            (tskIDLE_PRIORITY + 2)
#define SCREEN_TASK_PRIORITY            (tskIDLE_PRIORITY + 2)
#define NOTIFY_TASK_PRIORITY            (tskIDLE_PRIORITY + 2)
#define GPS_TASK_PRIORITY               (tskIDLE_PRIORITY + 3)
#define SENSOR_TASK_PRIORITY            (tskIDLE_PRIORITY + 3)
#define MODULE_TASK_PRIORITY            (tskIDLE_PRIORITY + 4)
#define SWITCH_TASK_PRIORITY            (tskIDLE_PRIORITY + 2)
#define FLASH_VAR_TASK_PRIORITY         (tskIDLE_PRIORITY + 1)
#define WDG_TASK_PRIORITY               ((tskIDLE_PRIORITY + configMAX_PRIORITIES) - 1)

//NOTE: configTIMER_TASK_PRIORITY       defined in FreeRTOS file FreeRTOSConfig.h

#define REP_TASK_STACK_SIZE             (configMINIMAL_STACK_SIZE + 200)
#define BUTTON_TASK_STACK_SIZE          (configMINIMAL_STACK_SIZE + 60)
#define SCREEN_TASK_STACK_SIZE          (configMINIMAL_STACK_SIZE + 60)
#define NOTIFY_TASK_STACK_SIZE          (configMINIMAL_STACK_SIZE + 60)
#define GPS_TASK_STACK_SIZE             (configMINIMAL_STACK_SIZE + 60)
#define SENSOR_TASK_STACK_SIZE          (configMINIMAL_STACK_SIZE + 60)
#define MODULE_TASK_STACK_SIZE          (configMINIMAL_STACK_SIZE + 60)
#define SWITCH_TASK_STACK_SIZE          (configMINIMAL_STACK_SIZE + 60)
#define FLASH_VAR_TASK_STACK_SIZE       (configMINIMAL_STACK_SIZE)
#define WDG_TASK_STACK_SIZE             (configMINIMAL_STACK_SIZE + 50)

//NOTE: configTIMER_TASK_STACK_DEPTH    defined in FreeRTOS file FreeRTOSCONFIG.h
//NOTE: tskIDLE_STACK_SIZE              defined in FreeRTOS file tasks.c



#endif //__TASK_MGMT_H__
