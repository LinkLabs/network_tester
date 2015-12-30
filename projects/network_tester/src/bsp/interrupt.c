#include <stdint.h>
#include <stdio.h>

#include "em_timer.h"

#include "FreeRTOS.h"

#include "bsp.h"
#include "bsp_timer.h"
#include "bsp_uart.h"
#include "interrupt.h"
#include "main.h"

static irq_handler_callback_t s_handlers[NUM_IRQ_HANDLERS] = {0};

void reset_irq_handlers(void)
{
    uint32_t i;
    for(i=0; i<NUM_IRQ_HANDLERS; i++)
    {
        s_handlers[i] = NULL;
    }
}

void register_irq_handler(irq_handler_t handler, irq_handler_callback_t callback)
{
    s_handlers[handler] = callback;
}


void USART0_TX_IRQHandler(void)
{
    irq_handler_t h = IRQ_HANDLER_USART0_TX;

    if(s_handlers[h] != NULL)
    {
        ((irq_handler_callback_t)s_handlers[h])(0);
    }
}

#if 0
void USART0_RX_IRQHandler(void)
{
    irq_handler_t h = IRQ_HANDLER_USART0_RX;

    if(s_handlers[h] != NULL)
    {
        ((irq_handler_callback_t)s_handlers[h])(0);
    }
}
#endif

void USART1_TX_IRQHandler(void)
{
    irq_handler_t h = IRQ_HANDLER_USART1_TX;

    if(s_handlers[h] != NULL)
    {
        ((irq_handler_callback_t)s_handlers[h])(0);
    }
}

void USART1_RX_IRQHandler(void)
{
    irq_handler_t h = IRQ_HANDLER_USART1_RX;

    if(s_handlers[h] != NULL)
    {
        ((irq_handler_callback_t)s_handlers[h])(0);
    }
}

void DAC0_IRQHandler(void)
{
    irq_handler_t h = IRQ_HANDLER_DAC0;

    if(s_handlers[h] != NULL)
    {
        ((irq_handler_callback_t)s_handlers[h])(0);
    }
}

void TIMER0_IRQHandler(void)
{
    // Clear the interrupt flag
    TIMER_IntClear(TIMER0, TIMER_IF_OF);

    // Increment the timer tick
    bsp_increment_timer_tick();
}

#ifdef INCLUDE_HARDFAULT_HANDLER
/* The fault handler implementation calls a function called
prvGetRegistersFromStack(). */
void HardFault_Handler(void)
{
    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler2_address_const                            \n"
        " bx r2                                                     \n"
        " handler2_address_const: .word prvGetRegistersFromStack    \n"
    );
}

void prvGetRegistersFromStack( uint32_t *pulFaultStackAddress )
{
/* These are volatile to try and prevent the compiler/linker optimising them
away as the variables never actually get used.  If the debugger won't show the
values of the variables, make them global my moving their declaration outside
of this function. */
volatile uint32_t r0;
volatile uint32_t r1;
volatile uint32_t r2;
volatile uint32_t r3;
volatile uint32_t r12;
volatile uint32_t lr; /* Link register. */
volatile uint32_t pc; /* Program counter. */
volatile uint32_t psr;/* Program status register. */

    r0 = pulFaultStackAddress[ 0 ];
    r1 = pulFaultStackAddress[ 1 ];
    r2 = pulFaultStackAddress[ 2 ];
    r3 = pulFaultStackAddress[ 3 ];

    r12 = pulFaultStackAddress[ 4 ];
    lr = pulFaultStackAddress[ 5 ];
    pc = pulFaultStackAddress[ 6 ];
    psr = pulFaultStackAddress[ 7 ];

    /* When the following line is hit, the variables contain the register values. */
    for( ;; );
}
#endif

#if 0
/**************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter
 *****************************************************************************/
void SysTick_Handler(void)
{
    bsp_increment_systick();
}

#endif
