#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "hardware/i2c.h"

// minimal LiquadCrystal cpp arduino library port

#define LCD_DEFAULT_I2C_ADDRESS 0x27
#define LCD_DEFAULT_COLS 16
#define LCD_DEFAULT_ROWS 2

struct LCD {
    i2c_inst_t* i2c;
    uint8_t address;
    uint8_t cols;
    uint8_t rows;
    uint8_t display_control;
    uint8_t backlight;
};

void lcd_i2c_init_with_i2c_uninitialized(struct LCD* lcd, i2c_inst_t* i2c,
                                         uint scl_pin, uint sda_pin,
                                         uint8_t address, uint8_t cols,
                                         uint8_t rows);

void lcd_i2c_init(struct LCD* lcd, i2c_inst_t* i2c, uint8_t address,
                  uint8_t cols, uint8_t rows);

void lcd_i2c_clear(struct LCD* lcd);
void lcd_i2c_clear_row_fast(struct LCD* lcd, char clear_char, uint8_t row);
void lcd_i2c_home(struct LCD* lcd);
void lcd_i2c_set_cursor(struct LCD* lcd, uint8_t col, uint8_t row);

void lcd_i2c_putc(struct LCD* lcd, char c);
void lcd_i2c_puts(struct LCD* lcd, const char* text);

void lcd_i2c_display(struct LCD* lcd, bool enabled);
void lcd_i2c_cursor(struct LCD* lcd, bool enabled);
void lcd_i2c_blink(struct LCD* lcd, bool enabled);
void lcd_i2c_backlight(struct LCD* lcd, bool enabled);

void lcd_i2c_create_char(struct LCD* lcd, uint8_t location,
                         const uint8_t bitmap[8]);
