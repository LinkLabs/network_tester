#ifndef LCD_NHD_H_INCLUDED
#define LCD_NHD_H_INCLUDED

#define LCD_ROWS                4
#define LCD_COLUMNS             20

void lcd_init(void);
void lcd_write_line(char* text,uint8_t line);
void lcd_write_screen(char screen[][20]);
void lcd_write_character(char character,uint8_t line,uint8_t column);

#endif /* LCD_NHD_H_INCLUDED */
