#include "em_device.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "iomap.h"
#include "bsp_io.h"
#include "lcd_nhd.h"

#define LCD_CLEAR_CMD           0x01
#define LCD_HOME_CMD            0x02
#define LCD_SET_CURSOR_CMD      0x80
#define LCD_BUSY_FLAG           0x80
#define LCD_CG_COUNTER_CMD      0x40

static void lcd_delay(uint32_t dly_ticks);
static void lcd_write_cmd(uint8_t data);
static void lcd_write_data(uint8_t data);

void lcd_init(void)
{
    lcd_delay(20000);      // Wait >15 msec after power is applied
    lcd_write_cmd(0x30);    // 0x30 = Wake up
    lcd_delay(7000);       // must wait 5ms, busy flag not available
    lcd_write_cmd(0x30);    // 0x30 = Wake up #2
    lcd_delay(7000);        // must wait 160us, busy flag not available
    lcd_write_cmd(0x30);    // 0x30 = Wake up #3
    lcd_delay(7000);        // must wait 160us, busy flag not available
    lcd_write_cmd(0x38);    // Function set: 8-bit/2-line
    lcd_delay(7000);
    lcd_write_cmd(0x08);    // display off
    lcd_delay(7000);
    lcd_write_cmd(0x06);    // Entry mode set
    lcd_delay(7000);
    lcd_write_cmd(0x0C);    // Display ON; Cursor ON and blinking
}

void lcd_write_line(char* text,uint8_t line)
{
    uint32_t i;
    uint8_t start_addr;

    switch(line)
    {
        case 1:
            start_addr = 0x40;
            break;
        case 2:
            start_addr = 0x14;
            break;
        case 3:
            start_addr = 0x54;
            break;
        default:
            start_addr = 0x00;
            break;
    }
    lcd_write_cmd(LCD_SET_CURSOR_CMD | start_addr);
    for(i=0;i<LCD_COLUMNS;i++)
    {
        if(text[i])
        {
            lcd_write_data(text[i]);
        }
        else    // clear rest of line after null
        {
            while(i<LCD_COLUMNS)
            {
                lcd_write_data(' ');
                i++;
            }
        }
    }
}

void lcd_write_screen(char screen[][20])
{
    uint32_t i;
    for(i=0;i<LCD_ROWS;i++)
    {
        lcd_write_line(screen[i],i);
    }
}

void lcd_write_character(char character,uint8_t line,uint8_t column)
{
    uint8_t start_addr;

    switch(line)
    {
        case 1:
            start_addr = 0x40;
            break;
        case 2:
            start_addr = 0x14;
            break;
        case 3:
            start_addr = 0x54;
            break;
        default:
            start_addr = 0x00;
            break;
    }
    start_addr += column;
    lcd_write_cmd(LCD_SET_CURSOR_CMD | start_addr);
    lcd_write_data(character);
}

static void lcd_delay(uint32_t dly_ticks)
{
    while(dly_ticks--);
}

static void lcd_write_cmd(uint8_t data)
{
    lcd_data(data);    //put data on output Port
    lcd_rs(0);          //RS=LOW : send instruction
    lcd_rnw(0);         //R/W=LOW : Write
    lcd_e(1);
    lcd_delay(80);       //enable pulse width >= 300ns
    lcd_e(0);          //Clock enable: falling edge
}

static void lcd_write_data(uint8_t data)
{
    lcd_data(data);    //put data on output Port
    lcd_rs(1);          //RS=1 : send data
    lcd_rnw(0);         //R/W=LOW : Write
    lcd_e(1);
    lcd_delay(80);       //enable pulse width >= 300ns
    lcd_e(0);          //Clock enable: falling edge
}
