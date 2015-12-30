//
// \file    lora_module_task.c
// \brief   LoRa Module Interface Routines for Liferaft Repeater Module.
//          Provides configuration, control, and other functions for the Module IFC on the
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

#include "em_gpio.h"
#include "gpiointerrupt.h"

#include "bsp.h"
#include "bsp_timer.h"
#include "bsp_uart.h"
#include "bsp_watchdog.h"
#include "debug_print.h"
#include "iomap.h"
#include "repeater_task.h"
#include "lora_module_task.h"
#include "radio_params.h"
#include "packet_queue.h"

#include "ll_ifc.h"

//
// \addtogroup LLABS_LIBS
//  @{
//

// \defgroup Module_IFC
//  @{
//

// Exported constants --------------------------------------------------------
// Private macros ------------------------------------------------------------
#define MODULE_TASK_QUEUE_SIZE              (6u)
#define MODULE_QUEUE_TIMEOUT_TICKS          (1000 / portTICK_RATE_MS)

#define MODULE_RX_TIMEOUT                   (1000u)

// #define DEBUG_PRINT_EVERY_BYTE_TX_RX
#define MODULE_RESET_TIME_S                 (5u)
#define MODULE_RETRY_INIT_TIME              (3000u / portTICK_RATE_MS)
#define MODULE_IDLE_WAIT_PERIOD             (5000u / portTICK_RATE_MS)

#define DEFAULT_MODULE_BW                   (3u)  // 3=500kHz
#define DEFAULT_MODULE_PWR                  (26u) // +26.5dBm on RXRv3

#define MODULE_RX_BUFFER_SIZE               (256u)  //!< RX Buffer for incoming packets
#define MODULE_TX_BUFFER_SIZE               (256u)  //!< TX Buffer for outgoing packets

// Private types -------------------------------------------------------------
//!
// \brief   Commands accepted by the module task.
//
typedef enum
{
    MODULE_TASK_CMD_START_RX        = 0,    //!< Start Module RX Mode
    MODULE_TASK_CMD_STOP_RX,                //!< Stop Module RX Mode
    MODULE_TASK_CMD_START_TX,               //!< Start Module TX Mode
    MODULE_TASK_CMD_STOP_TX,                //!< Stop Module TX Mode
    MODULE_TASK_CMD_IO0_NOTIFY,             //!< Host IO0 Notification
    NUM_MODULE_TASK_COMMANDS
} module_task_command_t;

#ifdef DEBUG
// For debug - printing state transitions
static const char* module_task_cmd_strings[NUM_MODULE_TASK_COMMANDS] = {
        "Module Start RX",
        "Module Stop RX",
        "Module Start TX",
        "Module Stop TX",
        "Host IO0 Notify"
};
#endif

//
// \brief  Module state enumeration
//
typedef enum
{
    MODULE_STATE_INIT = 0,              //!< Module Init state
    MODULE_STATE_IDLE,                  //!< Module Idle state
    MODULE_STATE_RX,                    //!< Module RX state
    MODULE_STATE_TX,                    //!< Module TX state
    NUM_MODULE_STATES
} module_state_t;

#ifdef DEBUG
// For debug - printing state transitions
static const char* module_state_strings[NUM_MODULE_STATES] = {
        "INIT",
        "IDLE",
        "RX",
        "TX"
};
#endif

//
// \brief  Module task queue message definition.
//
typedef struct module_task_queue_msg
{
    module_task_command_t   cmd;
    uint8_t                 data;
    uint8_t                 *data_ptr;
    uint8_t                 data_size;
    struct radio_params     *params;
    struct packet_queue     *queue;
} module_task_queue_msg_t;

// Private variables ---------------------------------------------------------
// \defgroup Module_IFC_Private_Variables Module IFC API Library Private Variables
//  @{
//
static xTaskHandle   s_module_task_handle;
static xQueueHandle  s_module_task_queue;

static wdg_handler_t s_module_task_wdg_handler;

static module_state_t s_module_state = MODULE_STATE_INIT;

static packet_queue_t *s_rx_queue = NULL;
static packet_queue_t *s_tx_queue = NULL;
static radio_params_t *s_radio_params = NULL;

static radio_params_t s_local_rx_radio_params;
static radio_params_t s_local_tx_radio_params;

static uint8_t s_rx_buffer[MODULE_RX_BUFFER_SIZE];
static uint8_t s_tx_buffer[MODULE_TX_BUFFER_SIZE];

static packet_info_t s_tx_packet_info;

#if configUSE_TRACE_FACILITY
static traceLabel s_module_trace_event;
static bool trace_started = false;
#endif

//
// @}
//

// Private function prototypes -----------------------------------------------
static void module_state_set(module_state_t state);
static module_state_t module_state_get(void);

static void module_init_state(void);
static void module_idle_state(void);
static void module_rx_state(void);
static void module_tx_state(void);

static void module_attach_interface(radio_params_t *p, packet_queue_t *q, module_state_t state);
static void module_release_interface(module_state_t state);

static bool module_radio_params_updated(radio_params_t *old_params, radio_params_t *new_params);

static void host_io_interrupt_configure(void);
static void host_io0_interrupt_enable(void);
static void host_io1_interrupt_enable(void);
static void host_io0_interrupt_disable(void);
static void host_io1_interrupt_disable(void);

static void host_io0_isr(void);
static void host_io1_isr(void);

static int32_t get_irq_flags(uint32_t* irq_flags);
static int32_t get_stats_from_rx_packet(uint8_t* payload, uint8_t len, int16_t* rssi, int8_t* snr);

// Private functions ---------------------------------------------------------
// \defgroup Module_IFC_Private_Functions Module IFC API Library Private Functions
//  @{
//

static void module_state_set(module_state_t state)
{
#ifdef DEBUG
#if 0
    Debug_Printf("\n(%d ms) module State Change %s => %s\n\n", xTaskGetTickCount()*portTICK_RATE_MS, \
                 module_state_strings[s_module_state], module_state_strings[state]);
#endif
#endif

#ifdef DEBUG

    // Make sure we can enumerate all state transitions
    switch(s_module_state)
    {
        case MODULE_STATE_INIT:
            // From this state to the following is legal:
            LL_ASSERT(state == MODULE_STATE_IDLE);
            break;
        case MODULE_STATE_IDLE:
            // From this state to the following is legal:
            LL_ASSERT((state == MODULE_STATE_RX) || (state == MODULE_STATE_TX));
            break;
        case MODULE_STATE_RX:
            // From this state to the following is legal:
            LL_ASSERT((state == MODULE_STATE_IDLE) || (state == MODULE_STATE_RX) || (state == MODULE_STATE_TX));
            break;
        case MODULE_STATE_TX:
            // From this state to the following is legal:
            LL_ASSERT((state == MODULE_STATE_IDLE) || (state == MODULE_STATE_TX) || (state == MODULE_STATE_RX));
            break;
        default:
            break;
    }
#endif

    s_module_state = state;
}

static module_state_t module_state_get(void)
{
    return s_module_state;
}

static void module_init_state(void)
{
    bool b_exit_state = false;

    while(!b_exit_state)
    {
        wdg_refresh(s_module_task_wdg_handler);

        // In this state, we simply initialize the module and jump to the idle state.
        module_state_set(MODULE_STATE_IDLE);
        break;
    }
}

static void module_idle_state(void)
{
    module_task_queue_msg_t msg;
    bool b_exit_state = false;

    while(!b_exit_state)
    {
        if (xQueueReceive(s_module_task_queue, &msg, MODULE_QUEUE_TIMEOUT_TICKS))
        {
#ifdef DEBUG
            Debug_Printf("(%d ms) module_task CMD: %s in state %s\n", (xTaskGetTickCount() * portTICK_RATE_MS), \
                         module_task_cmd_strings[msg.cmd], module_state_strings[module_state_get()]);
#endif
            // Check command type and handle
            if (MODULE_TASK_CMD_START_RX == msg.cmd)
            {
                // Configure the module for RX state
                module_attach_interface(msg.params, msg.queue, MODULE_STATE_RX);

                // Put the task into the RX state.
                module_state_set(MODULE_STATE_RX);

                break;
            }
            else if (MODULE_TASK_CMD_START_TX == msg.cmd)
            {
                // Configure the module for RX state
                module_attach_interface(msg.params, msg.queue, MODULE_STATE_TX);

                // Put the task into the RX state.
                module_state_set(MODULE_STATE_TX);

                break;
            }
            else
            {
                // We shouldn't be getting any other commands in idle.
                LL_ASSERT(false);
            }

            wdg_refresh(s_module_task_wdg_handler);
        }
        else
        {
            wdg_refresh(s_module_task_wdg_handler);
        }
    }
}

static void module_rx_state(void)
{
    module_task_queue_msg_t msg;
    bool b_exit_state = false;
    bool start_trace = false;

    packet_info_t pkt_info;
    uint8_t bytes_received;
    uint32_t irq_flags;
    int16_t rssi;
    int8_t snr;

    // Check that the queue pointer has been assigned
    if (s_rx_queue == NULL)
    {
        LL_ASSERT(false);
    }

    // First put the radio in RX continuous and wait for packets until we get a stop command.
    ll_packet_recv_cont(s_rx_buffer, MODULE_RX_BUFFER_SIZE, &bytes_received);

    // TODO: Check the received bytes here. We are just putting the radio in RX, so
    // we shouldn't get anything here.

    while(!b_exit_state)
    {
        if (xQueueReceive(s_module_task_queue, &msg, (1000 / portTICK_RATE_MS)))
        {
#if 1
#ifdef DEBUG
            Debug_Printf("(%d ms) module_task CMD: %s in state %s\n", (xTaskGetTickCount() * portTICK_RATE_MS), \
                         module_task_cmd_strings[msg.cmd], module_state_strings[module_state_get()]);
#endif
#endif
            // Check command type and handle
            if (MODULE_TASK_CMD_STOP_RX == msg.cmd)
            {
                // Release the interface
                module_release_interface(MODULE_STATE_RX);

                // Put the task back into the idle state.
                module_state_set(MODULE_STATE_IDLE);

                break;
            }
            else if (MODULE_TASK_CMD_START_RX == msg.cmd)
            {
                // We are already in the RX state and getting the command to RX again.
                // We should not have to reconfigure the radio every time, only if the params
                // are different.

                // Check that the radio params are set locally
                LL_ASSERT(s_radio_params != NULL);

                // Check if the radio params are equal
                if (module_radio_params_updated(&s_local_rx_radio_params, msg.params))
                {
                    // Get radio params and configure radio
                    set_radio_params(msg.params);
                    s_radio_params = msg.params;
                    s_local_rx_radio_params = *s_radio_params;
                }

                // NOTE: Don't need to re-enable the Host IO0 interrupt

                // Get the RX packet queue
                s_rx_queue = msg.queue;
            }
            else if (MODULE_TASK_CMD_START_TX == msg.cmd)
            {
                // We received the command to enter the TX state. DL_TX or UL_TX.
                // Stop the RX and exit the state.

                // Release the interface
                module_release_interface(MODULE_STATE_RX);

                // Reconfigure for the TX interface
                module_attach_interface(msg.params, msg.queue, MODULE_STATE_TX);

                // Set the TX state
                module_state_set(MODULE_STATE_TX);

                break;
            }
            else if (MODULE_TASK_CMD_IO0_NOTIFY == msg.cmd)
            {
                // Debug_Print("Host IO0 Notify in RX state\n");

                // We received a Host IO0 interrupt, get the flags and clear them.
                // irq_flags will contain the flags before they were cleared. The
                // Host IO will notify us if another packet is received.
                if (get_irq_flags(&irq_flags) != -1)
                {
                    // Check for RX_DONE
                    if (irq_flags & IRQ_FLAGS_RX_DONE)
                    {
                        // We received RX_DONE flag, go get the packet
                        ll_packet_recv_cont(s_rx_buffer, MODULE_RX_BUFFER_SIZE, &bytes_received);

                        // Get the radio params and populate the tx_packet_desc
                        if (get_stats_from_rx_packet(s_rx_buffer, bytes_received, &rssi, &snr) != -1)
                        {
                            // Assemble the tx_packet_desc
                            pkt_info.timestamp = bsp_get_timer_tick();
                            pkt_info.rx_info.rssi = rssi;
                            pkt_info.rx_info.snr = snr;
                            pkt_info.length = (bytes_received - 3);

                            // Debug_Printf("RX %d byte packet, RSSI= %d, SNR= %d\n", (bytes_received - 3), rssi, snr);

                            // Update the packet queue if we received a packet
                            if (packet_queue_push_back(s_rx_queue, &pkt_info, &s_rx_buffer[3]) != 0)
                            {
                                // Problem pushing to the packet queue
                                Debug_Print("Error pushing to packet queue in RX state\n");
                            }
                            else
                            {
                                // Kick start repeater task so it can take a look at the packet pushed on to
                                // the s_rx_queue right away
                                notify_repeater_task(NOTIFY_REPEATER_RX);
                            }
                        }
                        else
                        {
                            // The packet was garbage, don't do anything.
                            Debug_Print("RX Packet Invalid\n");
                        }
                    }
                    else
                    {
                        // We didn't get the RX_DONE flag, check if we got other flags and
                        // respond accordingly.

                    }
                }
                else
                {
                    // There was a problem getting the flags, HAL UART comms, etc.
                    LL_ASSERT(false);
                }
            }
            else
            {
                // Illegal command in RX state
                LL_ASSERT(false);
            }

            wdg_refresh(s_module_task_wdg_handler);
        }
        else
        {
            wdg_refresh(s_module_task_wdg_handler);
        }
    }
}

static void module_tx_state(void)
{
    module_task_queue_msg_t msg;
    bool b_exit_state = false;

    uint32_t irq_flags;
    int32_t num_tx_packets;

    // Check that the queue pointer has been assigned
    if (s_tx_queue == NULL)
    {
        LL_ASSERT(false);
    }

    num_tx_packets = (int32_t) s_tx_queue->num_packets;

    while(!b_exit_state)
    {
        // Here we need to push the packets to the radio queue until there are no more to push
        // While doing this, we need to check for task queue messages - TX_STOP or HOST_IO0
        // Once we are done stuffing packets, we still need to wait on the queue.

        // Pop the packets off the queue and send them out
        while (packet_queue_pop_front(s_tx_queue, &s_tx_packet_info, s_tx_buffer) == 0)
        {
            uint8_t param_flags = 0;
            int32_t packet_queued = 0;
#if 0
            uint8_t i;
            for (i=0; i<s_incoming_packet_desc.length; i++)
            {
                Debug_Printf("0x%02X ", s_incoming_bytes[i]);
            }
            Debug_Print("\n");
#endif
            // Check that the packet info matches current configuration
            if (s_tx_packet_info.tx_info.spreading_factor != s_radio_params->spreading_factor)
            {
                param_flags |= RADIO_PARAM_FLAGS_SF;
                s_radio_params->spreading_factor = s_tx_packet_info.tx_info.spreading_factor;
            }

            if (s_tx_packet_info.tx_info.programmed_preamble_syms != s_radio_params->programmed_preamble_syms)
            {
                param_flags |= RADIO_PARAM_FLAGS_PREAMBLE;
                s_radio_params->programmed_preamble_syms = s_tx_packet_info.tx_info.programmed_preamble_syms;
            }

            // Parameters don't match, reconfigure the radio
            if (param_flags != 0)
            {
                set_radio_params(s_radio_params);
            }

            // Stuff the packet.
            // TODO: We may get trapped here.
            while (packet_queued == 0)
            {
                packet_queued = ll_packet_send_queue(s_tx_buffer, s_tx_packet_info.length);

                // If the response was an error, handle it
                if (packet_queued == -1)
                {
                    LL_ASSERT(false);
                }
            }

            num_tx_packets--;
            LL_ASSERT(num_tx_packets >= 0);

            wdg_refresh(s_module_task_wdg_handler);
        }

        // We have popped all the packets from the queue, we need to wait on the task queue
        if (xQueueReceive(s_module_task_queue, &msg, (1000 / portTICK_RATE_MS)))
        {
#ifdef DEBUG
            Debug_Printf("(%d ms) module_task CMD: %s in state %s\n", (xTaskGetTickCount() * portTICK_RATE_MS), \
                         module_task_cmd_strings[msg.cmd], module_state_strings[module_state_get()]);
#endif

            // TODO: Handle the RX command in this state

            // Check command type and handle
            if (MODULE_TASK_CMD_STOP_TX == msg.cmd)
            {
                // Release the interface
                module_release_interface(MODULE_STATE_TX);

                // Put the task back into the idle state.
                module_state_set(MODULE_STATE_IDLE);

                // Exit the state
                break;
            }
            else if (MODULE_TASK_CMD_START_TX == msg.cmd)
            {

                // Check that the radio params are set locally
                LL_ASSERT(s_radio_params != NULL);

                // Check if the radio params are equal
                if (module_radio_params_updated(&s_local_tx_radio_params, msg.params))
                {
                    // Get radio params and configure radio
                    set_radio_params(msg.params);
                    s_radio_params = msg.params;
                    s_local_tx_radio_params = *s_radio_params;
                }

                // NOTE: Don't need to re-enable the Host IO0 interrupt

                // Get the RX packet queue
                s_tx_queue = msg.queue;
            }
            else if (MODULE_TASK_CMD_START_RX == msg.cmd)
            {
                // We received the command to enter the RX state. DL_RX or UL_RX.
                // Stop the TX and exit the state.

                // Release the interface
                module_release_interface(MODULE_STATE_TX);

                // Reconfigure for the TX interface
                module_attach_interface(msg.params, msg.queue, MODULE_STATE_RX);

                // Set the TX state
                module_state_set(MODULE_STATE_RX);

                // Break out of the state
                break;
            }
            else if (MODULE_TASK_CMD_IO0_NOTIFY == msg.cmd)
            {
#ifdef DEBUG
                Debug_Print("Host IO0 Notify in TX state\n");
#endif

                // We received a Host IO0 interrupt, get the flags and clear them.
                // irq_flags will contain the flags before they were cleared. The
                // Host IO will notify us if another packet is received.
                if (get_irq_flags(&irq_flags) != -1)
                {
                    // Check for TX_QUEUE_EMPTY
                    if (irq_flags & IRQ_FLAGS_TX_QUEUE_EMPTY)
                    {
                        // We sent the packets, notify the repeater task
                        notify_repeater_task(NOTIFY_REPEATER_TX);
                    }
                    else
                    {
                        // We didn't get the TX_QUEUE_EMPTY flag, check if we got other flags and
                        // respond accordingly.

                    }
                }
                else
                {
                    // There was a problem getting the flags, HAL UART comms, etc.
                    LL_ASSERT(false);
                }
            }
            else
            {
                // Illegal command in TX state
                LL_ASSERT(false);
            }

            wdg_refresh(s_module_task_wdg_handler);
        }
        else
        {
            wdg_refresh(s_module_task_wdg_handler);
        }
    }
}

static void module_attach_interface(radio_params_t *p, packet_queue_t *q, module_state_t state)
{
    uint32_t irq_flags;

    // Get radio params and configure radio
    set_radio_params(p);
    s_radio_params = p;

    // Enable the Host IO0 interrupt
    host_io0_interrupt_enable();

    // Clear the IRQ flags
    get_irq_flags(&irq_flags);

    // Get the packet queue
    if (MODULE_STATE_RX == state)
    {
        s_rx_queue = q;
        s_local_rx_radio_params = *s_radio_params;
    }
    else if (MODULE_STATE_TX == state)
    {
        s_tx_queue = q;
        s_local_tx_radio_params = *s_radio_params;
    }
    else
    {
        // Incorrect state passed to function
        LL_ASSERT(false);
    }
}

static void module_release_interface(module_state_t state)
{
    // Disable the Host IO0 interrupt
    host_io0_interrupt_disable();

    // Clear the pointer to the radio params and RX packet queue
    s_radio_params = NULL;

    // Get the packet queue
    if (MODULE_STATE_RX == state)
    {
        s_rx_queue = NULL;
    }
    else if (MODULE_STATE_TX == state)
    {
        s_tx_queue = NULL;
    }
    else
    {
        // Incorrect state passed to function
        LL_ASSERT(false);
    }
}

static bool module_radio_params_updated(radio_params_t *old_params, radio_params_t *new_params)
{
    bool b_ret = false;

    // Verify the radio params
    if (new_params->bandwidth != old_params->bandwidth)
    {
        b_ret = true;
    }

    if (new_params->channel_type != old_params->channel_type)
    {
        b_ret = true;
    }

    if (new_params->coding_rate != old_params->coding_rate)
    {
        b_ret = true;
    }

    if (new_params->frequency != old_params->frequency)
    {
        b_ret = true;
    }

    if (new_params->programmed_preamble_syms != old_params->programmed_preamble_syms)
    {
        b_ret = true;
    }

    if (new_params->spreading_factor != old_params->spreading_factor)
    {
        b_ret = true;
    }

    return b_ret;
}

//
// \brief Configures the Host IO interrupts.
//
static void host_io_interrupt_configure(void)
{
    // Make sure GPIO is configured correctly
    GPIO_PinModeSet(RXR_IO0_PORT, RXR_IO0_PIN, gpioModeInput, 0);
    //GPIO_PinModeSet(RXR_IO1_PORT, RXR_IO1_PIN, gpioModeInput, 0);

    // Set the callback function for the interrupt
    GPIOINT_CallbackRegister(RXR_IO0_PIN, host_io0_isr);
    //GPIOINT_CallbackRegister(RXR_IO1_PIN, host_io1_isr);
}

//
// \brief Enables the Host IO0 interrupt.
//
static void host_io0_interrupt_enable(void)
{
    // Make sure the interrupt flag is cleared and disabled
    GPIO_IntDisable(1 << RXR_IO0_PIN);
    GPIO_IntClear(1 << RXR_IO0_PIN);

    // Enable the even GPIO int.
    NVIC_SetPriority(GPIO_ODD_IRQn, configKERNEL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(GPIO_ODD_IRQn);

    GPIO_IntConfig(RXR_IO0_PORT, RXR_IO0_PIN, true, false, true);
}

//
// \brief Enables the Host IO1 interrupt.
//
static void host_io1_interrupt_enable(void)
{
    // Make sure the interrupt flag is cleared and disabled
    //GPIO_IntDisable(1 << RXR_IO1_PIN);
    //GPIO_IntClear(1 << RXR_IO1_PIN);

    // Enable the even GPIO int.
    //NVIC_SetPriority(GPIO_EVEN_IRQn, configKERNEL_INTERRUPT_PRIORITY);
    //NVIC_EnableIRQ(GPIO_EVEN_IRQn);

    //GPIO_IntConfig(RXR_IO1_PORT, RXR_IO1_PIN, true, false, true);
}

//
// \brief Disables the Host IO0 interrupt.
//
static void host_io0_interrupt_disable(void)
{
    // Make sure the interrupt flag is cleared and disabled
    GPIO_IntDisable(1 << RXR_IO0_PIN);
    GPIO_IntClear(1 << RXR_IO0_PIN);

    GPIO_IntConfig(RXR_IO0_PORT, RXR_IO0_PIN, true, false, false);
}

//
// \brief Disables the Host IO1 interrupt.
//
static void host_io1_interrupt_disable(void)
{
    // Make sure the interrupt flag is cleared and disabled
    //GPIO_IntDisable(1 << RXR_IO1_PIN);
    //GPIO_IntClear(1 << RXR_IO1_PIN);

    //GPIO_IntConfig(RXR_IO1_PORT, RXR_IO1_PIN, true, false, false);
}


static void host_io0_isr(void)
{
    // Send a signal to the task queue from ISR
    signed portBASE_TYPE TaskWoken;
    module_task_queue_msg_t msg;

    // Host IO0 ISR Routine - clear the flag
    GPIO_IntClear(1 << RXR_IO0_PIN);

    msg.cmd = MODULE_TASK_CMD_IO0_NOTIFY;
    msg.data_size = 0;
    msg.data = 0;

    // Send the command to the queue
    if (pdTRUE != xQueueSendFromISR(s_module_task_queue, &msg, &TaskWoken))
    {
        Debug_Print("Module Host IO0 ISR to queue failed.\n");
        LL_ASSERT(false);
    }

    portYIELD_FROM_ISR(TaskWoken);
}

static void host_io1_isr(void)
{

}

/**
 * @brief
 *  Get the IRQ flags from the module and handle their conditions.
 *
 * @details
 *  This function will clear the flags after getting their current value.
 *
 * @param[out] uint32_t* irq_flags
 *  This pointer will be written with the current IRQ flags before clearing them.
 *
 * @retval:
 *   0 = successfully got IRQ flags from the module and no unexpected flags.
 *   -1 = Failed to get IRQ flags.
 *   -2 = Unexpected flag.
 *   -3 = Got an assert or watchdog flag.
 */
static int32_t get_irq_flags(uint32_t* irq_flags)
{
    int32_t i32_ret = 0;
    module_state_t curr_state = module_state_get();

    // Get the flags and check that they are valid in the current state.
    if (ll_irq_flags(0xFFFFFFFF, irq_flags) == -1)
    {
        // There was a problem getting the IRQ flags
        i32_ret = -1;
    }
    else
    {
        // First check for any flags that indicate an error
        if (*irq_flags & IRQ_FLAGS_WDOG_RESET)
        {
            // We received a watchdog reset from the module, handle it.
            // TODO: Handle this condition. Just returning -1 for now.
            i32_ret = -3;
        }

        if (*irq_flags & IRQ_FLAGS_ASSERT)
        {
            // We received an assert flag from the module, handle it.
            // TODO: Handle this condition. Just returning -1 for now.
            i32_ret = -3;
        }

        switch (curr_state)
        {
            case MODULE_STATE_IDLE:
                // What flags can we get in IDLE?

                break;
            case MODULE_STATE_RX:
                // In RX mode, valid flags we could get that aren't already handled:
                // RX_DONE
                if (*irq_flags & IRQ_FLAGS_RX_DONE)
                {
                    i32_ret = 0;
                }
                else
                {
                    i32_ret = -2;
                }

                break;
            case MODULE_STATE_TX:
                // In TX mode, valid flags we could get that aren't already handled:
                // TX_QUEUE_EMPTY
                if (*irq_flags & IRQ_FLAGS_TX_QUEUE_EMPTY)
                {
                    i32_ret = 0;
                }
                else
                {
                    i32_ret = -2;
                }

                break;
            default:
                // This could only be the INIT state. We should get a RESET flag here.
                if (*irq_flags & IRQ_FLAGS_RESET)
                {
                    i32_ret = 0;
                }
                else
                {
                    i32_ret = -2;
                }

                break;
        }
    }

    return i32_ret;
}

/**
 * @brief
 *  Get the RSSI and SNR from the RX packet.
 *
 * @details
 *
 * @param[in] uint8_t* payload
 *  Pointer to the packet payload.
 *
 * @param[in] uint8_t len
 *  Length of the packet payload.
 *
 * @param[out] int16_t* rssi
 *  This pointer will be written with the received RSSI value.
 *
 * @param[out] int8_t* snr
 *  This pointer will be written with the received SNR value.
 *
 * @retval:
 *   0 = Successfully got radio stats.
 *   -1 = Packet failure.
 */
static int32_t get_stats_from_rx_packet(uint8_t* payload, uint8_t len, int16_t* rssi, int8_t* snr)
{
    int32_t i32_ret = 0;

    // Check that the length is valid
    if (len < 4)
    {
        // Packet is empty
        i32_ret = -1;
    }

    // Get the RSSI value
    *rssi = *((int16_t *) &payload[0]);

    // Get the SNR value
    *snr = (int8_t) payload[2] / 4;

    return i32_ret;
}

//
// @}
//

// Exported functions --------------------------------------------------------
// \defgroup Module_IFC_Exported_Functions Module IFC API Library Exported Functions
//  @{
//

// TODO: Module task needs to handle comms to the UART

/*
 * Returns:
 *  EXIT_SUCCESS if we can proceed with booting
 *  EXIT_FAILURE if we should reboot
 *
 *  Assumes UART has already been set up, but not enabled
 */
int32_t init_lora_module(void)
{
    // Enable serial port ?

    // Flush serial port ?

    // Reset module & wait for it to reboot
    Debug_Printf("Resetting Lora Module... \n");

    MODULE_RST(0);
    bsp_delay_ms(1);
    MODULE_RST(1);
    bsp_delay_ms((uint32_t)MODULE_RESET_TIME_S * 1000);

    int32_t i32_ret;

    ll_firmware_type_t fwt;
    i32_ret = ll_firmware_type_get(&fwt);
    if(i32_ret == FIRMWARE_TYPE_LEN)
    {
        Debug_Printf("Link Labs Firmware Type: %04d.%04d\n", fwt.cpu_code, fwt.functionality_code);

        uint16_t expected_cpu_code;
        uint16_t expected_functionality_code = GATEWAY_TX_ONLY;

        expected_cpu_code = CPU_R5F51115ADNE; // RXR Rev 3 or Rev 4
        expected_functionality_code = MODULE_END_NODE;

        if ((fwt.cpu_code != expected_cpu_code) || (fwt.functionality_code != expected_functionality_code))
        {
            Debug_Printf("Wrong lora module firmware type. Expected %04d.%04d\n", expected_cpu_code, expected_functionality_code);
            return(EXIT_FAILURE);
        }
    }
    else
    {
        Debug_Printf("Unable to get Link Labs Firmware Type\n");
        return(EXIT_FAILURE);
    }

    ll_version_t ver;
    i32_ret = ll_interface_version_get(&ver);
    if(i32_ret == VERSION_LEN)
    {
        Debug_Printf("Module Host Interface version: v%d.%d.%d\n", ver.major, ver.minor, ver.tag);

        uint8_t MINIMUM_IFC_SUPPORTED_MAJOR;
        uint8_t MINIMUM_IFC_SUPPORTED_MINOR;
        uint16_t MINIMUM_IFC_SUPPORTED_TAG;

        MINIMUM_IFC_SUPPORTED_MAJOR = 0;
        MINIMUM_IFC_SUPPORTED_MINOR = 0;
        MINIMUM_IFC_SUPPORTED_TAG = 0;

        uint32_t total_version_minimum = (MINIMUM_IFC_SUPPORTED_MAJOR<<24) | (MINIMUM_IFC_SUPPORTED_MINOR<<16) | MINIMUM_IFC_SUPPORTED_TAG;

        uint32_t total_version_actual = (ver.major<<24) | (ver.minor<<16) | ver.tag;

        if(total_version_actual < total_version_minimum)
        {
            Debug_Printf("FATAL: Module host_ifc version must be newer than v%d.%d.%d\n", MINIMUM_IFC_SUPPORTED_MAJOR, MINIMUM_IFC_SUPPORTED_MINOR, MINIMUM_IFC_SUPPORTED_TAG);
            return(EXIT_FAILURE);
        }
    }
    else
    {
        Debug_Printf("Unable to get Link Labs version. \n");
        return(EXIT_FAILURE);
    }

    // Set RF output to be UFL cable
    i32_ret =  ll_antenna_set(1);
    if(i32_ret != 0)
    {
        Debug_Printf("Unable to set module antenna output.\n");
        return(EXIT_FAILURE);
    }

    i32_ret = ll_version_get(&ver);
    if(i32_ret == VERSION_LEN)
    {
        Debug_Printf("Link Labs Lora Module version: v%d.%d.%d\n", ver.major, ver.minor, ver.tag);

        uint8_t MINIMUM_SUPPORTED_MAJOR;
        uint8_t MINIMUM_SUPPORTED_MINOR;
        uint16_t MINIMUM_SUPPORTED_TAG;

        MINIMUM_SUPPORTED_MAJOR = 0;
        MINIMUM_SUPPORTED_MINOR = 7;
        MINIMUM_SUPPORTED_TAG = 8;

        uint32_t total_version_minimum = (MINIMUM_SUPPORTED_MAJOR<<24) | (MINIMUM_SUPPORTED_MINOR<<16) | MINIMUM_SUPPORTED_TAG;

        uint32_t total_version_actual = (ver.major<<24) | (ver.minor<<16) | ver.tag;

        if(total_version_actual < total_version_minimum)
        {
            Debug_Printf("FATAL: Firmware must be newer than v%d.%d.%d\n", MINIMUM_SUPPORTED_MAJOR, MINIMUM_SUPPORTED_MINOR, MINIMUM_SUPPORTED_TAG);
            return(EXIT_FAILURE);
        }
    }
    else
    {
        Debug_Printf("Unable to get Link Labs version. \n");
        return(EXIT_FAILURE);
    }

    uint8_t mac_get;
    i32_ret = ll_mac_mode_get(&mac_get);
    if(i32_ret < 0)
    {
        Debug_Printf("Error getting MAC mode\n");
        return (EXIT_FAILURE);
    }
    if (mac_get != 0)
    {
        Debug_Printf("Radio module not in correct MAC mode. Setting MAC mode...\n");
        i32_ret = ll_mac_mode_set(0);
        if (i32_ret < 0)
        {
            Debug_Printf("Error setting MAC mode\n");
            return (EXIT_FAILURE);
        }
        Debug_Printf("Waiting %d seconds for module to reboot\n", MODULE_RESET_TIME_S);

        // sleep(MODULE_RESET_TIME_S);
        bsp_delay_ms((uint32_t)MODULE_RESET_TIME_S * 1000);

        i32_ret = ll_mac_mode_get(&mac_get);
        if (i32_ret < 0 || mac_get != 0)
        {
            Debug_Printf("MAC mode still not right. Exiting...\n");
            return (EXIT_FAILURE);
        }
    }

    i32_ret = ll_sleep_block();
    if(i32_ret<0)
    {
        Debug_Printf("Error blocking sleep() : %d\n", i32_ret);
        return(EXIT_FAILURE);
    }


    i32_ret = ll_radio_params_set(RADIO_PARAM_FLAGS_BW, 0, 0, DEFAULT_MODULE_BW, 0, 0, 0, 0 ,0);
    if(i32_ret<0)
    {
        Debug_Printf("Error setting Tx radio parameters() : %d\n", i32_ret);
        return(EXIT_FAILURE);
    }

    Debug_Printf("Setting Tx Power to %+d dBm\n", DEFAULT_MODULE_PWR);

    i32_ret = ll_tx_power_set(DEFAULT_MODULE_PWR);
    if (i32_ret<0)
    {
        Debug_Printf("Error calling ll_tx_power_set() : %d\n", i32_ret);
        return(EXIT_FAILURE);
    }

    // Clear reset irq_flag, make sure its zero
    uint32_t irq_flags_read = 0;
    i32_ret = ll_irq_flags(0xFFFFFFFF, &irq_flags_read);

    // ret is positive if good, negative if error
    if (i32_ret<0)
    {
        // Don't interpret irq_flags_read if a host_ifc error occured
        Debug_Printf("Error calling ll_irq_flags() #1: %d\n", i32_ret);
    }
    else
    {
        Debug_Printf("irq_flags = 0x%08X\n", irq_flags_read);
    }

    // Clear reset irq_flag, make sure its zero
    i32_ret = ll_irq_flags(0, &irq_flags_read);

    // ret is positive if good, negative if error
    if (i32_ret<0)
    {
        // Don't interpret irq_flags_read if a host_ifc error occured
        Debug_Printf("Error calling ll_irq_flags() #2: %d\n", i32_ret);
    }
    else
    {
        Debug_Printf("irq_flags = 0x%08X\n", irq_flags_read);
    }

    Debug_Printf("init_lora_module success\n");

    return(EXIT_SUCCESS);
}

int32_t module_rx_start(struct radio_params *p, struct packet_queue *q)
{
    module_task_queue_msg_t msg;
    msg.cmd = MODULE_TASK_CMD_START_RX;
    msg.params = p;
    msg.queue = q;
    msg.data = 0;
    msg.data_size = 0;
    msg.data_ptr = NULL;

    if (pdTRUE != xQueueSend(s_module_task_queue, &msg, MODULE_QUEUE_TIMEOUT_TICKS))
    {
        Debug_Print("RX Start xQueueSend failed\n");
        LL_ASSERT(false);
    }

    // TODO: Handle different return states, i.e. if the module can't be put in RX mode.

    return(0);
}

int32_t module_rx_stop(void)
{
    module_task_queue_msg_t msg;
    msg.cmd = MODULE_TASK_CMD_STOP_RX;
    msg.params = NULL;
    msg.queue = NULL;
    msg.data = 0;
    msg.data_size = 0;
    msg.data_ptr = NULL;

    if (pdTRUE != xQueueSend(s_module_task_queue, &msg, MODULE_QUEUE_TIMEOUT_TICKS))
    {
        Debug_Print("RX Stop xQueueSend failed\n");
        LL_ASSERT(false);
    }

    // TODO: Handle different return states, i.e. if the module is busy.

    return(0);
}

int32_t module_tx_start(struct radio_params *p, struct packet_queue *q)
{
    module_task_queue_msg_t msg;
    msg.cmd = MODULE_TASK_CMD_START_TX;
    msg.params = p;
    msg.queue = q;
    msg.data = 0;
    msg.data_size = 0;
    msg.data_ptr = NULL;

    if (pdTRUE != xQueueSend(s_module_task_queue, &msg, MODULE_QUEUE_TIMEOUT_TICKS))
    {
        Debug_Print("TX Start xQueueSend failed\n");
        LL_ASSERT(false);
    }

    // TODO: Handle different return states, i.e. if the module can't be put in TX mode.

    return(0);
}

int32_t module_tx_stop(void)
{
    module_task_queue_msg_t msg;
    msg.cmd = MODULE_TASK_CMD_STOP_TX;
    msg.params = NULL;
    msg.queue = NULL;
    msg.data = 0;
    msg.data_size = 0;
    msg.data_ptr = NULL;

    if (pdTRUE != xQueueSend(s_module_task_queue, &msg, MODULE_QUEUE_TIMEOUT_TICKS))
    {
        Debug_Print("TX Stop xQueueSend failed\n");
        LL_ASSERT(false);
    }

    // TODO: Handle different return states, i.e. if the module is busy.

    return(0);
}

/**
 * @brief
 *  Sets the radio parameters from the config structure.
 *
 * @details
 *
 * @param[in] radio_params *p
 *  Pointer to a radio parameter structure containing config.
 *
 * @retval:
 *   0 = Radio successfully configured
 *   -1 = There was an error configuring the radio
 */
int32_t set_radio_params(struct radio_params *p)
{
    // Configure the radio
    return (ll_radio_params_set(0xFF, p->spreading_factor, p->coding_rate, p->bandwidth, p->frequency, \
                            p->programmed_preamble_syms, true, true, false));
}

/*
 * Return:
 *  0 if bytes were written
 *  -1 otherwise
 */
int32_t transport_write(uint8_t *buff, uint16_t len)
{
    int32_t i32_ret;
    i32_ret = bsp_uart_module_tx(buff, len);
    if (i32_ret < 0)
    {
        Debug_Printf("Error writing tty\n");
        return -1;
    }

#ifdef DEBUG_PRINT_EVERY_BYTE_TX_RX
    int i;
    for (i = 0; i < len; i++)
    {
        Debug_Printf("W: 0x%02x\n", buff[i]);
    }
#endif

    return 0;
}

/*
 * Return:
 *  0 if len bytes were read before the timeout time,
 *  -1 otherwise
 */
int32_t transport_read(uint8_t *buff, uint16_t len)
{
    int32_t i32_ret_inner;

    uint8_t rx_byte;
    uint16_t bytes_received = 0;

    uint32_t start_tick = bsp_get_timer_tick();
    uint32_t timeout_val = (500 * TIMER_TICK_COUNT_1MS);

    while((bsp_get_timer_tick() - start_tick) < timeout_val)
    {
        i32_ret_inner = bsp_uart_module_rx(&rx_byte);

        if (i32_ret_inner == 0)
        {
            // We received a byte
            buff[bytes_received++] = rx_byte;

            if (bytes_received == len)
            {
                // We received all of the requested bytes

#ifdef DEBUG_PRINT_EVERY_BYTE_TX_RX
//            Debug_Printf("\tR: 0x%02x\n", rx_byte);
#endif

                return(0);
            }
        }
    }

    return(-1);
}

//
// \brief Services the switch state machine.
//
static portTASK_FUNCTION_PROTO(module_task, param);
static portTASK_FUNCTION(module_task, param)
{
    (void) param;

    while(true)
    {
        module_state_t curr_state = module_state_get();
        switch (curr_state)
        {
            case MODULE_STATE_INIT:
                module_init_state();
                break;
            case MODULE_STATE_IDLE:
                module_idle_state();
                break;
            case MODULE_STATE_RX:
                module_rx_state();
                break;
            case MODULE_STATE_TX:
                module_tx_state();
                break;
            default:
                LL_ASSERT(false);
                break;
        }

        // Delay temporarily until this task can yield.
        wdg_refresh(s_module_task_wdg_handler);
    }
}

//
// \brief Initializes the module task.
// \return True if init successful, otherwise false.
//
uint8_t init_module(void)
{
    Debug_Printf("Initializing module task... \n");

    // Create the task queue
    s_module_task_queue = xQueueCreate(MODULE_TASK_QUEUE_SIZE, sizeof(module_task_queue_msg_t));

    if (NULL == s_module_task_queue)
    {
        return EXIT_FAILURE;
    }

    if (pdPASS != xTaskCreate(module_task, (const portCHAR *)"Module Task", MODULE_TASK_STACK_SIZE, NULL, MODULE_TASK_PRIORITY, &s_module_task_handle))
    {
        return EXIT_FAILURE;
    }

    if (EXIT_SUCCESS != init_lora_module())
    {
        return EXIT_FAILURE;
    }

    // Configure the Host IO interrupts, do not enable.
    host_io_interrupt_configure();

    //TODO: Handle this in the init state, using RTOS-safe UART functions.

    s_module_task_wdg_handler = wdg_register("MODULE");
    if (WDG_HANDLER_ERROR == s_module_task_wdg_handler)
    {
        return EXIT_FAILURE;
    }

#ifdef CHECK_STACK_USAGE
    // Task init complete. Register task and some metrics with main info structure
    task_info_t info = {
            .task_handle = s_module_task_handle,
            .stack_size = MODULE_TASK_STACK_SIZE,
            .stack_use_percentage = 0
    };
    register_task(TASK_MODULE, &info);
#endif

#if configUSE_TRACE_FACILITY
s_module_trace_event = xTraceOpenLabel("MOD");
#endif

    Debug_Printf("init_module success\n");

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





