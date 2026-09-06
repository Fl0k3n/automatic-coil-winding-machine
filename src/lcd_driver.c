#include "lcd_driver.h"

#include "pico/stdlib.h"

#define LCD_CLEAR_DISPLAY 0x01
#define LCD_RETURN_HOME 0x02
#define LCD_ENTRY_MODE_SET 0x04
#define LCD_DISPLAY_CONTROL 0x08
#define LCD_FUNCTION_SET 0x20
#define LCD_SET_CGRAM_ADDR 0x40
#define LCD_SET_DDRAM_ADDR 0x80

#define LCD_ENTRY_LEFT 0x02
#define LCD_DISPLAY_ON 0x04
#define LCD_CURSOR_ON 0x02
#define LCD_BLINK_ON 0x01
#define LCD_2_LINE 0x08

#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE 0x04
#define LCD_RS 0x01

static void expander_write(struct LCD* lcd, uint8_t value) {
    uint8_t data = value | lcd->backlight;
    i2c_write_blocking(lcd->i2c, lcd->address, &data, 1, false);
}

static void pulse_enable(struct LCD* lcd, uint8_t value) {
    expander_write(lcd, value | LCD_ENABLE);
    sleep_us(1);
    expander_write(lcd, value & (uint8_t)~LCD_ENABLE);
    sleep_us(50);
}

static void write4(struct LCD* lcd, uint8_t value) {
    expander_write(lcd, value);
    pulse_enable(lcd, value);
}

static void send(struct LCD* lcd, uint8_t value, uint8_t mode) {
    write4(lcd, (value & 0xF0) | mode);
    write4(lcd, ((value << 4) & 0xF0) | mode);
}

static void command(struct LCD* lcd, uint8_t value) { send(lcd, value, 0); }

static void update_display_control(struct LCD* lcd) {
    command(lcd, LCD_DISPLAY_CONTROL | lcd->display_control);
}

void lcd_i2c_init(struct LCD* lcd, i2c_inst_t* i2c, uint8_t address,
                  uint8_t cols, uint8_t rows) {
    lcd->i2c = i2c;
    lcd->address = address;
    lcd->cols = cols;
    lcd->rows = rows;
    lcd->display_control = LCD_DISPLAY_ON;
    lcd->backlight = LCD_BACKLIGHT;

    sleep_ms(50);
    expander_write(lcd, 0);

    write4(lcd, 0x30);
    sleep_us(4500);
    write4(lcd, 0x30);
    sleep_us(4500);
    write4(lcd, 0x30);
    sleep_us(150);
    write4(lcd, 0x20);

    command(lcd, LCD_FUNCTION_SET | (rows > 1 ? LCD_2_LINE : 0));
    update_display_control(lcd);
    lcd_i2c_clear(lcd);
    command(lcd, LCD_ENTRY_MODE_SET | LCD_ENTRY_LEFT);
    lcd_i2c_home(lcd);
}

void lcd_i2c_init_with_i2c_uninitialized(struct LCD* lcd, i2c_inst_t* i2c,
                                         uint scl_pin, uint sda_pin,
                                         uint8_t address, uint8_t cols,
                                         uint8_t rows) {
    i2c_init(i2c, 100000);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_pull_up(scl_pin);
    gpio_pull_up(sda_pin);
    lcd_i2c_init(lcd, i2c, address, cols, rows);
}

void lcd_i2c_clear_row_fast(struct LCD* lcd, char clear_char, uint8_t row) {
    lcd_i2c_set_cursor(lcd, 0, row);
    for (size_t i = 0; i < lcd->cols; i++) {
        lcd_i2c_putc(lcd, clear_char);
    }
}

void lcd_i2c_clear(struct LCD* lcd) {
    command(lcd, LCD_CLEAR_DISPLAY);
    sleep_us(2000);
}

void lcd_i2c_home(struct LCD* lcd) {
    command(lcd, LCD_RETURN_HOME);
    sleep_us(2000);
}

void lcd_i2c_set_cursor(struct LCD* lcd, uint8_t col, uint8_t row) {
    static const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};

    if (lcd->rows == 0) {
        return;
    }

    if (row >= lcd->rows) {
        row = lcd->rows - 1;
    }

    if (row > 3) {
        row = 3;
    }

    command(lcd, LCD_SET_DDRAM_ADDR | (uint8_t)(col + row_offsets[row]));
}

void lcd_i2c_putc(struct LCD* lcd, char c) { send(lcd, (uint8_t)c, LCD_RS); }

void lcd_i2c_puts(struct LCD* lcd, const char* text) {
    while (*text) {
        lcd_i2c_putc(lcd, *text++);
    }
}

void lcd_i2c_display(struct LCD* lcd, bool enabled) {
    if (enabled) {
        lcd->display_control |= LCD_DISPLAY_ON;
    } else {
        lcd->display_control &= (uint8_t)~LCD_DISPLAY_ON;
    }

    update_display_control(lcd);
}

void lcd_i2c_cursor(struct LCD* lcd, bool enabled) {
    if (enabled) {
        lcd->display_control |= LCD_CURSOR_ON;
    } else {
        lcd->display_control &= (uint8_t)~LCD_CURSOR_ON;
    }

    update_display_control(lcd);
}

void lcd_i2c_blink(struct LCD* lcd, bool enabled) {
    if (enabled) {
        lcd->display_control |= LCD_BLINK_ON;
    } else {
        lcd->display_control &= (uint8_t)~LCD_BLINK_ON;
    }

    update_display_control(lcd);
}

void lcd_i2c_backlight(struct LCD* lcd, bool enabled) {
    lcd->backlight = enabled ? LCD_BACKLIGHT : 0;
    expander_write(lcd, 0);
}

void lcd_i2c_create_char(struct LCD* lcd, uint8_t location,
                         const uint8_t bitmap[8]) {
    location &= 0x07;
    command(lcd, LCD_SET_CGRAM_ADDR | (location << 3));

    for (int i = 0; i < 8; ++i) {
        lcd_i2c_putc(lcd, (char)bitmap[i]);
    }
}
