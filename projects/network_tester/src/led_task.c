//
// \file    led_task.c
// \brief   LED Routines for Liferaft Repeater Module.
//          Provides configuration, control, and other functions for the LEDs on the
//          Liferaft Repeater Module. using the EFM32GG232F1024 mcu.
//
// \copyright LinkLabs, 2015
//

// Includes ------------------------------------------------------------------
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "task_mgmt.h"

#include "bsp.h"
#include "bsp_watchdog.h"
#include "debug_print.h"
#include "iomap.h"

#include "led_task.h"

//
// \addtogroup LLABS_LIBS
//  @{
//

// \defgroup LED
//  @{
//

// Exported constants --------------------------------------------------------
// \defgroup LED_Exported_Constants LED Exported Constants
//  @{
//

//
//  @}
//

// Private macros ------------------------------------------------------------
#define LED_TASK_RATE_MS                (50u)
#define LED_TASK_PERIOD_TICKS           (LED_TASK_RATE_MS / portTICK_RATE_MS)

#define LED_TASK_QUEUE_SIZE             (4u)



// Private variables ---------------------------------------------------------
// \defgroup LED_Private_Variables LED API Library Private Variables
//  @{
//
static xTaskHandle      s_led_task_handle;
static xQueueHandle     s_led_task_queue;
static wdg_handler_t    s_led_task_wdg_handler;

// Exported functions --------------------------------------------------------
// \defgroup LED_Exported_Functions LED API Library Exported Functions
//  @{
//

//
// \brief Turns the specified LED on.
// \param led: LED to be turned on.
//
void led_on(led_t led)
{
    led_task_queue_msg_t msg;

    msg.cmd = LED_TASK_LED_ON;
    msg.ctrl.led = led;
    msg.ctrl.on_time = 0;
    msg.ctrl.off_time = 0;
    msg.ctrl.repeat_count = 0;

    msg.data_size = sizeof(led_control_t);

    // Send the command to the queue
    if (pdTRUE != xQueueSend(s_led_task_queue, &msg, 0))
    {
        Debug_Print("Send LED ON command to queue failed.\n");
        LL_ASSERT(false);
    }
}

//
// \brief Turns the specified LED off.
// \param led: LED to be turned off.
//
void led_off(led_t led)
{
    led_task_queue_msg_t msg;

    msg.cmd = LED_TASK_LED_OFF;
    msg.ctrl.led = led;
    msg.ctrl.on_time = 0;
    msg.ctrl.off_time = 0;
    msg.ctrl.repeat_count = 0;

    msg.data_size = sizeof(led_control_t);

    // Send the command to the queue
    if (pdTRUE != xQueueSend(s_led_task_queue, &msg, 0))
    {
        Debug_Print("Send LED OFF command to queue failed.\n");
        LL_ASSERT(false);
    }
}

//
// \brief Toggles the specified LED.
// \param led: LED to be toggled.
//
void led_toggle(led_t led)
{
    led_task_queue_msg_t msg;

    msg.cmd = LED_TASK_LED_TOGGLE;
    msg.ctrl.led = led;
    msg.ctrl.on_time = 0;
    msg.ctrl.off_time = 0;
    msg.ctrl.repeat_count = 0;

    msg.data_size = sizeof(led_control_t);

    // Send the command to the queue
    if (pdTRUE != xQueueSend(s_led_task_queue, &msg, 0))
    {
        Debug_Print("Send LED Toggle command to queue failed.\n");
        LL_ASSERT(false);
    }
}

//
// \brief Turns the specified LED on for a period of time.
// \param led: LED to be turned on.
// \param flashTime: Time to turn on LED using the 2ms (500Hz) timebase macros.
//
void led_flash(led_t led, uint16_t on_time, uint16_t off_time, uint8_t repeat_count)
{
    led_task_queue_msg_t msg;

    msg.cmd = LED_TASK_LED_FLASH;
    msg.ctrl.led = led;
    msg.ctrl.on_time = on_time;
    msg.ctrl.off_time = off_time;
    msg.ctrl.repeat_count = repeat_count;

    msg.data_size = sizeof(led_control_t);

    // Send the command to the queue
    if (pdTRUE != xQueueSend(s_led_task_queue, &msg, 0))
    {
        Debug_Print("Send LED Flash command to queue failed.\n");
        LL_ASSERT(false);
    }
}

//
// \brief The main LED service state machine.
// \note Here is where we check the config of the LEDs, set the timers for on
//  or off, turn the LEDs on or off, set the next state, count down repeats, etc.
//  This task will compare to the systick to determine if it is time to be serviced.
//
static portTASK_FUNCTION_PROTO(led_task, param);
static portTASK_FUNCTION(led_task, param)
{
    (void) param;

    led_task_queue_msg_t msg;
    led_t led_num;

    while (true)
    {
        if (xQueueReceive(s_led_task_queue, &msg, LED_TASK_PERIOD_TICKS))
        {
#ifdef DEBUG
            Debug_Printf("(%d ms) led_task CMD: %s\n", (xTaskGetTickCount() * portTICK_RATE_MS), \
                         led_task_cmd_strings[msg.cmd]);
#endif

            // TODO: Perform checking on msg control struct and pass to set function.
            if (led_set(&msg) == -1)
            {
                LL_ASSERT(false);
            }
        }
        else
        {
            // LED state machine handler.
            wdg_refresh(s_led_task_wdg_handler);

            for (led_num = LED0; led_num < NUM_LEDS; led_num++)
            {
                led_config_t *led = &leds[led_num];

                switch (led->curr_state)
                {
                    case LED_STATE_OFF:
                        // First turn the LED off, and check if we are repeating. If we are,
                        // use the timerOff count.
                        if (led->flags.led_active)
                        {
                            // LED is on, turn it off.
                            bsp_led_set(led_num, false);

                            // Turn off the LED active bit
                            led->flags.led_active = false;
                        }
                        else
                        {
                            // LED is already off, check the timer to repeat.
                            if (led->repeat_count != 0)
                            {
                                // We are repeating, we need to check the timer.
                                if (led->flags.timer_off)
                                {
                                    if (led->off_time == 0)
                                    {
                                        // We're finished with the off count
                                        led->next_state = LED_STATE_ON;
                                        led->flags.timer_off = false;
                                    }
                                    else
                                    {
                                        led->off_time--;
                                    }
                                }
                                else
                                {
                                    // Enable the timer off, set the reload value.
                                    led->flags.timer_off = true;
                                    led->off_time = led->off_time_reload;
                                }
                            }
                            else
                            {
                                // The LED is off and we're not repeating any more.

                            }
                        }

                        break;

                    case LED_STATE_ON:
                        // Check if the LED timer is active, and decrement the count, unless 0 then
                        // go to OFF state. If we are repeating, reload the timer.
                        if (led->flags.timer_on)
                        {
                            if (led->on_time == 0)
                            {
                                led->next_state = LED_STATE_OFF;
                                led->flags.timer_on = false;

                                // Only decrement the repeat counter if not set to infinite flash
                                if (led->repeat_count != LED_REPEAT_COUNT_INF)
                                {
                                    led->repeat_count--;
                                }

                                if (led->repeat_count != 0)
                                {
                                    // We still have repeats, reload the timer.
                                    led->on_time = led->on_time_reload;
                                }
                            }
                            else
                            {
                                led->on_time--;
                            }
                        }
                        else
                        {
                            // If the LED isn't on, turn it on.
                            if (!(led->flags.led_active))
                            {

                                bsp_led_set(led_num, true);

                                // Turn on the LED active bit
                                led->flags.led_active = true;

                                // If the LED is only on for a period, set the timerOn flag.
                                if (led->on_time != LED_COUNT_ON)
                                {
                                    led->flags.timer_on = true;
                                }
                            }
                        }

                        break;

                    default:
                        LL_ASSERT(false);
                        break;
                }

                led_state_advance(led);
            }
        }
    }
}

//
// \brief Initializes the LED interface.
// \param led: LED to be initialized.
// \return True if init successful, otherwise false.
//
uint8_t init_led(void)
{
    Debug_Printf("Initializing LED Task... \n");

    // Create the LED task queue
    s_led_task_queue = xQueueCreate(LED_TASK_QUEUE_SIZE, sizeof(led_task_queue_msg_t));

    if (NULL == s_led_task_queue)
    {
        return EXIT_FAILURE;
    }

    if (pdPASS != xTaskCreate(led_task, (const portCHAR *)"LED Task", LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, &s_led_task_handle))
    {
        return EXIT_FAILURE;
    }

    // Set LEDs off.
    led0(0);
    led1(0);

    s_led_task_wdg_handler = wdg_register("LED");
    if (WDG_HANDLER_ERROR == s_led_task_wdg_handler)
    {
        return EXIT_FAILURE;
    }

#ifdef CHECK_STACK_USAGE
    // Task init complete. Register task and some metrics with main info structure
    task_info_t info = {
            .task_handle = s_led_task_handle,
            .stack_size = LED_TASK_STACK_SIZE,
            .stack_use_percentage = 0
    };
    register_task(TASK_LED, &info);
#endif

    Debug_Printf("init_led success\n");

    return EXIT_SUCCESS;
}

//
// @}
//

//
// @}
//

//
// @}
//
