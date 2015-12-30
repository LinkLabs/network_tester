//
// \file    bsp_uart.c
// \brief   USART module high level functions and interfaces.
//
// \copyright LinkLabs, 2015
//
#define UART_C_

// Includes
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include "em_chip.h"
#include "em_cmu.h"
#include "em_usart.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "bsp.h"
#include "bsp_uart.h"
#include "bsp_timer.h"
#include "debug_print.h"
#include "iomap.h"
#include "bsp_io.h"
#include "byte_fifo.h"


// Types
typedef void (*usart_irq_callback_t)(char);

// Static Global Variables
usart_irq_callback_t s_usart_rx_callbacks[3] = {NULL};

//static USART_TypeDef *module_uart = IOMAP_MODULE_UART;
static USART_TypeDef *gps_uart = IOMAP_GPS_UART;

byte_fifo_t s_modfifo_rx; //!< Buffer used to temporarily store bytes incoming from uart ifc, take modified only at main level, put modified only at ISR level
byte_fifo_t s_modfifo_tx; //!< Buffer used to temporarily store bytes incoming from uart ifc, take modified only at main level, put modified only at ISR level
byte_fifo_t s_userfifo_rx; //!< Buffer used to temporarily store bytes incoming from uart ifc, take modified only at main level, put modified only at ISR level
byte_fifo_t s_userfifo_tx; //!< Buffer used to temporarily store bytes incoming from uart ifc, take modified only at main level, put modified only at ISR level

static bool s_is_ll_bypass = false;                         //!< Variables used for bypassing net tester UART to module
static usart_irq_callback_t s_ll_bypass_callback = NULL;    //!< Variables used for bypassing net tester UART to module

// Static Forward Declarations
static uint32_t irq_handler_bsp_uart_module_tx(uint32_t in);
static uint32_t irq_handler_bsp_uart_module_notify(uint32_t in);






// NOTE that this is a ram function, since it can be called when the flash variables are being
// written to. Thus, all functions called from here have to either be ram variables also or
// be inlined.
static uint32_t irq_handler_bsp_uart_module_rx(uint32_t in) __attribute__ ((section(".ram")));
static uint32_t irq_handler_bsp_uart_module_rx(uint32_t in)
{
    // Clear interrupt flag
    uint32_t flags = USART_IntGet(IOMAP_MODULE_UART);
    USART_IntClear(IOMAP_MODULE_UART, flags);

    // Get byte
    uint8_t data = USART_Rx(IOMAP_MODULE_UART);

    // Push onto FIFO
    fifo_push(&s_modfifo_rx, &data, 1);

    // TODO: Use this interrupt??
    NVIC_SetPendingIRQ(DAC0_IRQn);

    return(in);
}

static uint32_t irq_handler_bsp_uart_module_tx(uint32_t in)
{
    USART_TypeDef *module_uart = IOMAP_MODULE_UART;

    // Clear interrupt flags by reading them
    USART_IntGet(IOMAP_MODULE_UART);

    // Check TX buffer level status
    if (module_uart->STATUS & USART_STATUS_TXBL)
    {
        uint8_t queued_byte;
        if (fifo_get(&s_modfifo_tx, &queued_byte))
        {
            USART_Tx(module_uart, queued_byte);
        }
        else
        {
            USART_IntDisable(module_uart, USART_IF_TXBL);
        }
    }

    return(in);
}

static uint32_t irq_handler_bsp_uart_module_notify(uint32_t in)
{
//    portBASE_TYPE higher_priority_task_woken = pdFALSE;

    NVIC_ClearPendingIRQ(DAC0_IRQn);

//    // Notify the host_ifc task that new bytes are available on s_rx_fifo
//    if (pdTRUE != xTaskNotifyFromISR(s_module_task_handle, 0, eNoAction, &higher_priority_task_woken))
//    {
//        Debug_Printf("ASSERT: Unable to notify host_ifc task\n");
//        LL_ASSERT(false);
//    }
//
//    // If we woke any tasks we may require a context switch.
//    portYIELD_FROM_ISR(higher_priority_task_woken);

    return(in);
}
/*********************************************************************/
/*****PUBLIC FUNCTIONS************************************************/
int32_t bsp_uart_module_tx(uint8_t *buff, uint16_t len)
{
    int32_t i32_ret = 0;

    // Load the TX FIFO
    fifo_push(&s_modfifo_tx, buff, len);

    // Trigger interrupt flag to start UART transmission
    USART_IntEnable(IOMAP_MODULE_UART, USART_IF_TXBL);

    return i32_ret;
}
/*********************************************************************/
/**
 * @brief
 *  Get a byte from the RX queue, timing out if we exceed the timeout period.
 *
 * @details
 *
 * @param[out] uint8_t* rx_byte
 *  This pointer will be written with the byte to write if return value = 1
 * @param[in] uint32_t timeout_ms
 *  Timeout period value in ms.
 *
 * @retval
 *   0 = Successfully received a byte from the UART.
 *   -1 = Operation timeout.
 */
int32_t bsp_uart_module_rx(uint8_t *rx_byte)
{
    if (fifo_get(&s_modfifo_rx, rx_byte))
    {
        return 0;
    }

    return -1;
}

int32_t bsp_uart_user_tx(uint8_t *buff, uint16_t len)
{
    int32_t i32_ret = 0;

    // Load the TX FIFO
    fifo_push(&s_userfifo_tx, buff, len);

    // Trigger interrupt flag to start UART transmission
    USART_IntEnable(IOMAP_USER_UART, USART_IF_TXBL);

    return i32_ret;
}

int32_t bsp_uart_user_rx(uint8_t *rx_byte)
{
    if (fifo_get(&s_userfifo_rx, rx_byte))
    {
        return 0;
    }

    return -1;
}

/*********************************************************************/
uint8_t bsp_uart_init(bsp_uart_t uart)
{
    uint8_t ret = EXIT_SUCCESS;

    // Initialize byte fifos
    fifo_init(&s_modfifo_rx);
    fifo_init(&s_modfifo_tx);
    fifo_init(&s_userfifo_rx);
    fifo_init(&s_userfifo_tx);

    USART_InitAsync_TypeDef uart_async_init = USART_INITASYNC_DEFAULT;

    uart_async_init.enable       = usartDisable;   /* Don't enable UART upon intialization */
    uart_async_init.refFreq      = 0;              /* Provide information on reference frequency. When set to 0, the reference frequency is */
    uart_async_init.baudrate     = 115200;         /* Baud rate */
    uart_async_init.oversampling = usartOVS16;     /* Oversampling. Range is 4x, 6x, 8x or 16x */
    uart_async_init.databits     = usartDatabits8; /* Number of data bits. Range is 4 to 10 */
    uart_async_init.parity       = usartNoParity;  /* Parity mode */
    uart_async_init.stopbits     = usartStopbits1; /* Number of stop bits. Range is 0 to 2 */
    uart_async_init.mvdis        = false;          /* Disable majority voting */
    uart_async_init.prsRxEnable  = false;          /* Enable USART Rx via Peripheral Reflex System */

    if (BSP_MODULE_UART == uart)
    {
        CMU_ClockEnable(cmuClock_USART1, true);

        GPIO_PinModeSet(gpioPortD, 0, gpioModePushPull, 1); // UART TX to module
        GPIO_PinModeSet(gpioPortD, 1, gpioModeInput, 0);    // UART RX from module

        USART_InitAsync(USART1, &uart_async_init);

        USART1->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | USART_ROUTE_LOCATION_LOC1;

        // Prepare UART Rx and Tx interrupts
        NVIC_SetPriority(USART1_RX_IRQn, 0x80);
        NVIC_SetPriority(USART1_TX_IRQn, 0x80);
        USART_IntClear(USART1, _USART_IF_MASK);
        USART_IntEnable(USART1, USART_IF_RXDATAV);
        NVIC_ClearPendingIRQ(USART1_RX_IRQn);
        NVIC_ClearPendingIRQ(USART1_TX_IRQn);
        NVIC_EnableIRQ(USART1_RX_IRQn);
        NVIC_EnableIRQ(USART1_TX_IRQn);
        // enable UART
        USART_Enable(USART1, usartEnable);
    }
    else if (BSP_GPS_UART == uart)
    {
        bsp_hw_rev_t hw_rev = bsp_hw_rev_get();
        if(HW_REV3 == hw_rev)
        {
            CMU_ClockEnable(cmuClock_USART0, true);

            USART_InitAsync(USART0, &uart_async_init);

            USART0->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | USART_ROUTE_LOCATION_LOC0;

            // Prepare UART Rx and Tx interrupts
            USART_IntClear(USART0, _USART_IF_MASK);
            USART_IntEnable(USART0, USART_IF_RXDATAV);
            NVIC_ClearPendingIRQ(USART0_RX_IRQn);
            NVIC_SetPriority(USART0_RX_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY);
            NVIC_EnableIRQ(USART0_RX_IRQn);
            // enable UART
            USART_Enable(USART0, usartEnable);
        }
        else if(HW_REV4 == hw_rev)
        {
            CMU_ClockEnable(cmuClock_USART2, true);
            GPIO_PinModeSet(gpioPortC, 2, gpioModePushPull, 1); // UART TX to GPS
            GPIO_PinModeSet(gpioPortC, 3, gpioModeInput, 0);    // UART RX from GPS

            USART_InitAsync(USART2, &uart_async_init);

            USART2->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | USART_ROUTE_LOCATION_LOC0;

            // Prepare UART Rx and Tx interrupts
            USART_IntClear(USART2, _USART_IF_MASK);
            USART_IntEnable(USART2, USART_IF_RXDATAV);
            NVIC_ClearPendingIRQ(USART2_RX_IRQn);
            NVIC_SetPriority(USART2_RX_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY);
            NVIC_EnableIRQ(USART2_RX_IRQn);
            // enable UART
            USART_Enable(USART2, usartEnable);
        }
        else
        {
            EFM_ASSERT(false);
        }
    }
    else if (BSP_USER_UART == uart)
    {
        bsp_hw_rev_t hw_rev = bsp_hw_rev_get();
        if(HW_REV4 == hw_rev)
        {
            CMU_ClockEnable(cmuClock_USART0, true);

            USART_InitAsync(USART0, &uart_async_init);

            USART0->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | USART_ROUTE_LOCATION_LOC0;

            // Prepare UART Rx and Tx interrupts
            NVIC_SetPriority(USART0_RX_IRQn, 0x80); //?
            NVIC_SetPriority(USART0_TX_IRQn, 0x80); //?
            USART_IntClear(USART0, _USART_IF_MASK);
            USART_IntEnable(USART0, USART_IF_RXDATAV);
            NVIC_ClearPendingIRQ(USART0_RX_IRQn);
            //NVIC_SetPriority(USART0_RX_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY);
            NVIC_EnableIRQ(USART0_RX_IRQn);
            NVIC_EnableIRQ(USART0_TX_IRQn);
            // enable UART
            USART_Enable(USART0, usartEnable);
        }
        else
        {
            //ignore
        }
    }
    else
    {
        EFM_ASSERT(0);
    }

    return ret;
}

void register_usart0_rx_callback(usart_irq_callback_t hook)
{
    s_usart_rx_callbacks[0] = hook;
}

void register_usart1_rx_callback(usart_irq_callback_t hook)
{
    s_usart_rx_callbacks[1] = hook;
}

void register_usart2_rx_callback(usart_irq_callback_t hook)
{
    s_usart_rx_callbacks[2] = hook;
}

void bsp_module_bypass_enable(bool enable, void(hook)(char))
{
    s_is_ll_bypass = enable;
    s_ll_bypass_callback = hook;
}
/*********************************************************************/
int32_t transport_write(uint8_t *buff, uint16_t len)
{
    int32_t i32_ret;
    i32_ret = bsp_uart_module_tx(buff, len);
    if (i32_ret < 0)
    {
        Debug_Printf("Error writing tty\n");
        return -1;
    }

    return 0;
}
/*********************************************************************/
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
                return(0);
            }
        }
    }

    return(-1);
}

/*********************************************************************/
/*****INTERRUPT HANDLERS**********************************************/
void USART0_RX_IRQHandler(void)
{
    char data = (char) (USART0->RXDATA);
    USART_IntClear(USART0, USART_IF_RXDATAV);
    if(s_usart_rx_callbacks[0] != 0)
    {
        s_usart_rx_callbacks[0](data);
    }
}

void USART1_RX_IRQHandler(void)
{
    if(s_is_ll_bypass)
    {
        char data = (char) (IOMAP_MODULE_UART->RXDATA);
        USART_IntClear(IOMAP_MODULE_UART, USART_IF_RXDATAV);
        if(s_ll_bypass_callback != 0)
        {
            s_ll_bypass_callback(data);
        }
    }
    else
    {
        irq_handler_bsp_uart_module_rx(0);
    }
}

void USART2_RX_IRQHandler(void)
{
    char temp;

    temp = (char) (USART2->RXDATA);
    USART_IntClear(USART2,USART_IF_RXDATAV);
    if(s_usart_rx_callbacks[2] != 0)
    {
        s_usart_rx_callbacks[2](temp);
    }
}
/*********************************************************************/
void USART1_TX_IRQHandler(void)
{
    irq_handler_bsp_uart_module_tx(0);
}

void USART0_TX_IRQHandler(void)
{
    USART_TypeDef *user_uart = IOMAP_USER_UART;

    // Clear interrupt flags by reading them
    USART_IntGet(user_uart);

    // Check TX buffer level status
    if (user_uart->STATUS & USART_STATUS_TXBL)
    {
        uint8_t queued_byte;
        if (fifo_get(&s_userfifo_tx, &queued_byte))
        {
            USART_Tx(user_uart, queued_byte);
        }
        else
        {
            USART_IntDisable(user_uart, USART_IF_TXBL);
        }
    }
}

/*********************************************************************/
// @} (end addtogroup UART)
// @} (end addtogroup Modules)
