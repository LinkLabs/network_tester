//
// \file    bsp_uart.h
// \brief   BSP USART Module Header.
//          Device UART initialization and high-level functions.
//
// \copyright LinkLabs, 2015
//
#ifndef __UART_H__
#define __UART_H__

// Includes ------------------------------------------------------------------
#include <stdbool.h>
#include <stdint.h>

#include "em_usart.h"

// Exported macros -----------------------------------------------------------
// Exported types ------------------------------------------------------------
typedef enum {
    BSP_MODULE_UART     = 0,    //!< RXR Module Host IFC UART
    BSP_GPS_UART        = 1,    //!< GPS UART
    BSP_USER_UART       = 2     //!< User UART
} bsp_uart_t;

// Exported function prototypes ----------------------------------------------
uint8_t bsp_uart_init(bsp_uart_t uart);

int32_t bsp_uart_module_tx(uint8_t *buff, uint16_t len);
int32_t bsp_uart_module_rx(uint8_t *rx_byte);
int32_t bsp_uart_user_tx(uint8_t *buff, uint16_t len);
int32_t bsp_uart_user_rx(uint8_t *rx_byte);

int32_t transport_read(uint8_t *buff, uint16_t len);
int32_t transport_write(uint8_t *buff, uint16_t len);

void register_usart0_rx_callback(void (*rx_func)(char));
void register_usart1_rx_callback(void (*rx_func)(char));
void register_usart2_rx_callback(void (*rx_func)(char));

void bsp_module_bypass_enable(bool enable, void(hook)(char));

//void USART0_RX_IRQHandler(void);
//void USART1_TX_IRQHandler(void);
//void USART1_RX_IRQHandler(void);

#endif  // __UART_H__
