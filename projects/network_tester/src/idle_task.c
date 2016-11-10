#include "debug_print.h"

/**************************************************************************//**
 * @brief vApplicationIdleHook
 * Override the default definition of vApplicationIdleHook()
 *****************************************************************************/
void vApplicationIdleHook(void)
{
#ifdef CHECK_STACK_USAGE
    static bool b_tasks_registered = false;

    if (!b_tasks_registered)
    {

        // Register Idle & Timer Task with stack check code
        TaskHandle_t task;
        task = xTaskGetIdleTaskHandle();

        // Task init complete. Register task and some metrics with main info structure
        task_info_t info;
        info.task_handle = task;
        info.stack_size = configMINIMAL_STACK_SIZE;
        info.stack_use_percentage = 0;

        register_task(TASK_IDLE, &info);

        task = xTimerGetTimerDaemonTaskHandle();
        info.task_handle = task;
        info.stack_size = configTIMER_TASK_STACK_DEPTH;
        info.stack_use_percentage = 0;

        register_task(TASK_TIMER, &info);

        b_tasks_registered = true;
    }
#endif
}

