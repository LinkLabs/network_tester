#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_i2c.h"
#include "em_usart.h"
#include "em_dac.h"
#include "debug_print.h"
#include "iomap.h"
#include "lcd_nhd.h"
#include "bsp.h"
#include "bsp_io.h"
#include "bsp_rtc.h"
#include "bsp_uart.h"
#include "bsp_timer.h"
#include "bsp_watchdog.h"
#include "bsp_i2c.h"
// \addtogroup Modules
//   @{
//

// \addtogroup BSP
//   @{
//

static uint8_t bsp_clock_init(void);
static uint8_t bsp_gpio_init(void);
static uint8_t bsp_module_uart_init(void);
static uint8_t bsp_gps_uart_init(void);
static uint8_t bsp_user_uart_init(void);
static uint8_t bsp_dac_init(void);
static uint8_t bsp_lcd_init(void);

//
// \brief   Initialize the microcontroller gpio, peripherals, etc.
// \return  Returns true if initialization was successful, otherwise false.
//
uint8_t bsp_system_init(void)
{
    uint8_t ret = EXIT_SUCCESS;

    // Make sure all priority bits are group for FreeRTOS.
    NVIC_SetPriorityGrouping(0);


    // Initialize the system clocks and configure the systick to 1ms.
    if (bsp_clock_init() != EXIT_SUCCESS)
    {
        // There was an error with clock initialization, handle it.
        ret = EXIT_FAILURE;
    }

    // Initialize the tick timer
    if (bsp_tick_timer_init() != EXIT_SUCCESS)
    {
        ret = EXIT_FAILURE;
    }
    else
    {
        bsp_tick_timer_start();
    }

    // Initialize the chip GPIO
    if (bsp_gpio_init() != EXIT_SUCCESS)
    {
        // There was an error with GPIO initialization, handle it.
        ret = EXIT_FAILURE;
    }

    // Initialize the module UART
    if (bsp_module_uart_init() != EXIT_SUCCESS)
    {
        // There was an error with UART initialization, handle it.
        ret = EXIT_FAILURE;
    }

    // Initialize the GPS UART
    if (bsp_gps_uart_init() != EXIT_SUCCESS)
    {
        // There was an error with UART initialization, handle it.
        ret = EXIT_FAILURE;
    }

    // Initialize the USER UART
    if (bsp_user_uart_init() != EXIT_SUCCESS)
    {
        // There was an error with UART initialization, handle it.
        ret = EXIT_FAILURE;
    }

    // Initialize the DAC
    if (bsp_dac_init() != EXIT_SUCCESS)
    {
        // There was an error with UART initialization, handle it.
        ret = EXIT_FAILURE;
    }

    // Initialize the lcd
    if (bsp_lcd_init() != EXIT_SUCCESS)
    {
        // There was an error with UART initialization, handle it.
        ret = EXIT_FAILURE;
    }

    // Initialize I2C bus
    if (bsp_i2c_init(I2C0,6) != EXIT_SUCCESS)
    {
        // There was an error with UART initialization, handle it.
        ret = EXIT_FAILURE;
    }

    return ret;
}

void bsp_trigger_software_reset(void)
{
    NVIC_SystemReset();
}

//
// \brief   Initialize the microcontroller clocks.
// \return  Returns true if initialization was successful, otherwise false.
// \note
//
//
static uint8_t bsp_clock_init(void)
{
    uint32_t freq;

    // Configure the external HF oscillator and select as main clock source
    CMU_OscillatorEnable(cmuOsc_HFXO, true, true);
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
    CMU_ClockDivSet(cmuClock_CORE, cmuClkDiv_1);
    CMU_ClockDivSet(cmuClock_HFPER, cmuClkDiv_1);

    SystemHFXOClockSet(HFCLK_FREQ);

    // Configure the external LF oscillator
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

    // Get system clock and compare to expected frequency value
    freq = CMU_ClockFreqGet(cmuClock_CORE);
    if (freq != HFCLK_FREQ)
    {
        // The CORE clock frequency is not equal to the expected value.
        return EXIT_FAILURE;
    }

    // Used for watchdog and RTC
    CMU_ClockEnable(cmuClock_CORELE, true);

    // Configure and start the RTC
    if (bsp_rtc_init() != EXIT_SUCCESS)
    {
        return EXIT_FAILURE;
    }

    // FreeRTOS configures the systick to use the RTC clock.

    return EXIT_SUCCESS;
}

//
// \brief   Initialize the microcontroller gpio.
// \return  Returns true if initialization was successful, otherwise false.
// \note
static uint8_t bsp_gpio_init(void)
{
    uint8_t ret = EXIT_SUCCESS;
    uint8_t i;

    // Enable GPIO clock
    CMU_ClockEnable(cmuClock_GPIO, true);
    GPIO_Unlock();

    // disable buzzer ASAP
    BUZZER_EN(0);

    // init GPIO based on hardware type
    ret |= bsp_io_init();

    // config inputs
    GPIO_PinModeSet(gpioPortA, 0, gpioModeInputPull, 1);   // PIR input
    GPIO_PinModeSet(gpioPortA, 1, gpioModeInput, 1);       // Accelerometer interrupt
    GPIO_PinModeSet(gpioPortA, 3, gpioModeInputPull, 1);   // 'select' button
    GPIO_PinModeSet(gpioPortA, 4, gpioModeInputPull, 1);   // 'up' button
    GPIO_PinModeSet(gpioPortA, 5, gpioModeInputPull, 1);   // 'down' button
    GPIO_PinModeSet(gpioPortD, 0, gpioModePushPull, 1); // UART TX to module
    GPIO_PinModeSet(gpioPortD, 1, gpioModeInput, 0);    // UART RX from module
    GPIO_PinModeSet(gpioPortD, 2, gpioModeInput,1);     // Module interrupt
    GPIO_PinModeSet(gpioPortE, 10, gpioModePushPull, 1);// UART TX to GPS
    GPIO_PinModeSet(gpioPortE, 11, gpioModeInput, 0);   // UART RX from GPS
    GPIO_PinModeSet(gpioPortE, 12, gpioModeWiredAndPullUpFilter, 1);    // i2c
    GPIO_PinModeSet(gpioPortE, 13, gpioModeWiredAndPullUpFilter, 1);    // i2c
    GPIO_PinModeSet(gpioPortE, 14, gpioModeInput, 1);      // gps power on indicator direction

    // config outputs (macros in iomap.h)
    // lcd lines (note: after this, use LCD_DATA(x) macro to write to LCD)
    lcd_rnw(0);
    for(i=0; i<8; i++)
    {
        lcd_dbx(i, 0);
    }
    lcd_e(1);
    lcd_rs(0);
    lcd_bklt_en(1);

    // LEDs
    LED1(0);
    LED2(0);
    LED3(0);

    // module
    MODULE_BOOT(0);
    MODULE_RST(0);

    //-----GPS init-----//
    GPS_RST(0);
    bsp_delay_ms(250);

    return ret;
}

static uint8_t bsp_module_uart_init(void)
{
    uint8_t ret = EXIT_SUCCESS;

    if (bsp_uart_init(BSP_MODULE_UART) != EXIT_SUCCESS)
    {
        ret = EXIT_FAILURE;
    }

    return ret;
}

static uint8_t bsp_gps_uart_init(void)
{
    uint8_t ret = EXIT_SUCCESS;

    if (bsp_uart_init(BSP_GPS_UART) != EXIT_SUCCESS)
    {
        ret = EXIT_FAILURE;
    }

    return ret;
}

static uint8_t bsp_user_uart_init(void)
{
    uint8_t ret = EXIT_SUCCESS;

    if (bsp_uart_init(BSP_USER_UART) != EXIT_SUCCESS)
    {
        ret = EXIT_FAILURE;
    }

    return ret;
}

static uint8_t bsp_dac_init(void)
{
    uint8_t ret = EXIT_SUCCESS;

    /* Use default settings */
    DAC_Init_TypeDef        init        = DAC_INIT_DEFAULT;
    DAC_InitChannel_TypeDef initChannel = DAC_INITCHANNEL_DEFAULT;

    /* Enable the DAC clock */
    CMU_ClockEnable(cmuClock_DAC0, true);

    /* Calculate the DAC clock prescaler value that will result in a DAC clock
    * close to 500kHz. Second parameter is zero, if the HFPERCLK value is 0, the
    * function will check what the current value actually is. */
    init.prescale = DAC_PrescaleCalc(500000, 0);

    /* Set reference voltage to 1.25V */
    init.reference = dacRef1V25;

    /* Initialize the DAC and DAC channel. */
    DAC_Init(DAC0, &init);
    DAC_InitChannel(DAC0, &initChannel, 0);

    // enable DAC and write value
    DAC_Enable(DAC0, 0, true);
    uint32_t DAC_Value = (uint32_t)((0.2 * 4096) / 1.25);
    DAC0->CH0DATA = DAC_Value;
    return ret;
}

static uint8_t bsp_lcd_init(void)
{
    uint8_t ret = EXIT_SUCCESS;

    lcd_init();

    return ret;
}

/** @} (end addtogroup BSP)   */
/** @} (end addtogroup Modules) */
