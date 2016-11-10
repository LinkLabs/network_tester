#ifndef __IOMAP_H__
#define __IOMAP_H__

#include "em_gpio.h"

#define ON  (1)
#define OFF (0)

#define IOMAP_MODULE_UART   USART1
#define IOMAP_USER_UART     USART0
#define IOMAP_GPS_UART      USART2

// port A
#define PIR_RD()            (GPIO_PinInGet(gpioPortA,0))
#define ACCEL_INT_RD()      (GPIO_PinInGet(gpioPortA,1))
#define SEL_BTN_MASK        (1<<3)
#define BACK_BTN_MASK       (1<<4)
#define DOWN_BTN_MASK       (1<<5)
#define BTN_RD()            (GPIO_PortInGet(gpioPortA) & (SEL_BTN_MASK|BACK_BTN_MASK|DOWN_BTN_MASK))
#define BUZZER_EN(x)        GPIO_PinModeSet(gpioPortA,8,gpioModePushPull,x)

// port B
#define LCD_CONTRAST(x)     GPIO_PinModeSet(gpioPortB,11,gpioModePushPull,x)

// port C [rev3]
#define LCD_PORT_r3        gpioPortC
#define LCD_PIN0_r3        0
#define LCD_DATA_RD_r3()   (GPIO_PortInGet(gpioPortC) & 255)
#define LCD_DATA_r3(x)     GPIO_PortOutSetVal(gpioPortC,x,0x000000FF)
#define LCD_DB0_r3(x)      GPIO_PinModeSet(gpioPortC,0,gpioModePushPull,x)
#define LCD_DB1_r3(x)      GPIO_PinModeSet(gpioPortC,1,gpioModePushPull,x)
#define LCD_DB2_r3(x)      GPIO_PinModeSet(gpioPortC,2,gpioModePushPull,x)
#define LCD_DB3_r3(x)      GPIO_PinModeSet(gpioPortC,3,gpioModePushPull,x)
#define LCD_DB4_r3(x)      GPIO_PinModeSet(gpioPortC,4,gpioModePushPull,x)
#define LCD_DB5_r3(x)      GPIO_PinModeSet(gpioPortC,5,gpioModePushPull,x)
#define LCD_DB6_r3(x)      GPIO_PinModeSet(gpioPortC,6,gpioModePushPull,x)
#define LCD_DB7_r3(x)      GPIO_PinModeSet(gpioPortC,7,gpioModePushPull,x)
#define LCD_E_r3(x)        GPIO_PinModeSet(gpioPortC,8,gpioModePushPull,x)
#define LCD_RnW_r3(x)      GPIO_PinModeSet(gpioPortC,9,gpioModePushPull,x)
#define LCD_RS_r3(x)       GPIO_PinModeSet(gpioPortC,10,gpioModePushPull,x)
#define LCD_BKLT_EN_r3(x)  GPIO_PinModeSet(gpioPortC,11,gpioModePushPull,x)
#define LCD_BKLT_TOGGLE_r3()   GPIO_PinOutToggle(gpioPortC,11)

//port C [rev4]
#define LCD_PORT_r4        gpioPortC
#define LCD_PIN0_r4        4
#define LCD_DATA_RD_r4()   ((GPIO_PortInGet(gpioPortC)>>4) & 255)
#define LCD_DATA_r4(x)     GPIO_PortOutSetVal(gpioPortC,x<<4,0x000000FF<<4)
#define LCD_DB0_r4(x)      GPIO_PinModeSet(gpioPortC,4,gpioModePushPull,x)
#define LCD_DB1_r4(x)      GPIO_PinModeSet(gpioPortC,5,gpioModePushPull,x)
#define LCD_DB2_r4(x)      GPIO_PinModeSet(gpioPortC,6,gpioModePushPull,x)
#define LCD_DB3_r4(x)      GPIO_PinModeSet(gpioPortC,7,gpioModePushPull,x)
#define LCD_DB4_r4(x)      GPIO_PinModeSet(gpioPortC,8,gpioModePushPull,x)
#define LCD_DB5_r4(x)      GPIO_PinModeSet(gpioPortC,9,gpioModePushPull,x)
#define LCD_DB6_r4(x)      GPIO_PinModeSet(gpioPortC,10,gpioModePushPull,x)
#define LCD_DB7_r4(x)      GPIO_PinModeSet(gpioPortC,11,gpioModePushPull,x)
#define LCD_E_r4(x)        GPIO_PinModeSet(gpioPortC,12,gpioModePushPull,x)
#define LCD_RnW_r4(x)      GPIO_PinModeSet(gpioPortC,13,gpioModePushPull,x)
#define LCD_RS_r4(x)       GPIO_PinModeSet(gpioPortC,14,gpioModePushPull,x)
#define LCD_BKLT_EN_r4(x)  GPIO_PinModeSet(gpioPortC,15,gpioModePushPull,x)
#define LCD_BKLT_TOGGLE_r4()   GPIO_PinOutToggle(gpioPortC,15)

// port D
#define MODULE_BOOT(x)         GPIO_PinModeSet(gpioPortD,3,gpioModePushPull,x)
#define MODULE_RST(x)          GPIO_PinModeSet(gpioPortD,4,gpioModePushPull,x)
#define LED1(x)                GPIO_PinModeSet(gpioPortD,7,gpioModePushPull,x)
#define LED1_TOGGLE()          GPIO_PinOutToggle(gpioPortD,7)
#define GW_CONNECTED_LED(x)    LED1(x)
#define GW_CONNECTED_TOGGLE(x) LED1_TOGGLE(x)
#define LED2(x)                GPIO_PinModeSet(gpioPortD,6,gpioModePushPull,x)
#define LED2_TOGGLE()          GPIO_PinOutToggle(gpioPortD,6)
#define MESSAGE_SENT_LED(x)    LED2(x)
#define LED3(x)                GPIO_PinModeSet(gpioPortD,8,gpioModePushPull,x)
#define LED3_TOGGLE()          GPIO_PinOutToggle(gpioPortD,8)
#define GPS_FIX_LED(x)         LED3(x)
#define RXR_IO0_PORT           gpioPortD
#define RXR_IO0_PIN            2
#define MOD_INT_RD()           (GPIO_PinInGet(RXR_IO0_PORT,RXR_IO0_PIN))

// port E
#define GPS_RST(x)      GPIO_PinModeSet(gpioPortE,8,gpioModePushPull,x)
#define GPS_ON_RD()     (GPIO_PinInGet(gpioPortE,14))

#endif // __IOMAP_H__
