#include <stdio.h>

#include "controller.h"
#include "hardware/uart.h"
#include "io.h"
#include "lcd_driver.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "rotary_encoder_driver.h"
#include "tmc2209_driver.h"

#define LED1_PIN 6
#define LED2_PIN 7

#define BUTTON1_PIN 10
#define BUTTON2_PIN 11
#define BUTTON3_PIN 12
#define BUTTON4_PIN 13

#define LCD_I2C_SCL_PIN 1
#define LCD_I2C_SDA_PIN 0
#define LCD_I2C i2c0

#define ENCODER_S1_PIN 2
#define ENCODER_S2_PIN 3
#define ENCODER_KEY_PIN 4

#define SCREW_MOTOR_STEP_PIN 19
#define SCREW_MOTOR_DIR_PIN 18
#define SCREW_MOTOR_ENABLE_PIN 22

#define COIL_MOTOR_STEP_PIN 21
#define COIL_MOTOR_DIR_PIN 20
#define COIL_MOTOR_ENABLE_PIN 28

#define MOTOR_DRIVER_UART uart0
#define MOTOR_DRIVER_UART_BAUD_RATE 115200
#define COIL_MOTOR_UART_TX_PIN 16
#define COIL_MOTOR_UART_RX_PIN 17

#define HOMING_BUTTON_PIN 15

#define RPM_TO_TIME_BETWEEN_STEPS_US(rpm, steps_per_rev) \
    ((60 * 1000000) / ((rpm) * (steps_per_rev)))

static struct LCD lcd;
static struct RotaryEncoder rotary_encoder;

static struct ControllerConfig controller_config = {
    .coil_motor_enable_pin = COIL_MOTOR_ENABLE_PIN,
    .coil_motor_wind_button_pin = BUTTON1_PIN,
    .coil_motor_unwind_button_pin = BUTTON2_PIN,
    .screw_motor_enable_pin = SCREW_MOTOR_ENABLE_PIN,
    .screw_motor_left_button_pin = BUTTON4_PIN,
    .screw_motor_righ_button_pin = BUTTON3_PIN,
    .homing_button_pin = HOMING_BUTTON_PIN,
    .coil_motor_step_pin = COIL_MOTOR_STEP_PIN,
    .coil_motor_dir_pin = COIL_MOTOR_DIR_PIN,
    .screw_motor_step_pin = SCREW_MOTOR_STEP_PIN,
    .screw_motor_dir_pin = SCREW_MOTOR_DIR_PIN,
    .min_time_between_coil_motor_steps_us =
        RPM_TO_TIME_BETWEEN_STEPS_US(300, 200),
    .min_time_between_screw_motor_steps_us =
        RPM_TO_TIME_BETWEEN_STEPS_US(300, 200),
    .screw_pitch_microns = 2000,
    .active_indicator_led_pin = LED1_PIN,
    .paused_or_error_led_pin = LED2_PIN,
    .coil_motor_wind_dir_value = true,
    .screw_motor_right_dir_value = true,
    .async_coil_motor_step_interval_us = 3000,
    .async_screw_motor_step_interval_us = 3000,
    .full_rotation_num_steps_coil_motor = 200,
    .full_rotation_num_steps_screw_motor = 200,
    .homing_button_press_range_microns = 3000,
};

static inline void configure_motor_drivers() {
    gpio_put(COIL_MOTOR_ENABLE_PIN, false);
    gpio_put(SCREW_MOTOR_ENABLE_PIN, false);
    sleep_ms(50);
    tmc2209_uart_init(MOTOR_DRIVER_UART, COIL_MOTOR_UART_TX_PIN,
                      COIL_MOTOR_UART_RX_PIN, MOTOR_DRIVER_UART_BAUD_RATE);
    sleep_ms(10);
    tmc2209_set_fullstep(MOTOR_DRIVER_UART, 1);  // coil motor
    tmc2209_set_fullstep(MOTOR_DRIVER_UART, 0);  // screw motor
    sleep_ms(10);
}

static void gpio_isr(uint gpio, uint32_t events) {
    rotary_encoder_handle_gpio_irq(&rotary_encoder, gpio, events);
    if (gpio == ENCODER_KEY_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
        controller_handle_rotary_encoder_button_pressed_irq();
    }
    if (gpio == HOMING_BUTTON_PIN) {
        if (events & GPIO_IRQ_EDGE_FALL) {
            controller_handle_homing_button_pressed_irq();
        } else if (events & GPIO_IRQ_EDGE_RISE) {
            controller_handle_homing_button_released_irq();
        }
    }
}

int main() {
    stdio_init_all();

    sleep_ms(500);
    printf("Coil winder initializing...\n");

    gpio_init(LED1_PIN);
    gpio_set_dir(LED1_PIN, true);
    gpio_put(LED1_PIN, false);
    gpio_init(LED2_PIN);
    gpio_set_dir(LED2_PIN, true);
    gpio_put(LED2_PIN, false);

    gpio_init(BUTTON1_PIN);
    gpio_set_dir(BUTTON1_PIN, false);
    gpio_pull_up(BUTTON1_PIN);

    gpio_init(BUTTON2_PIN);
    gpio_set_dir(BUTTON2_PIN, false);
    gpio_pull_up(BUTTON2_PIN);

    gpio_init(BUTTON3_PIN);
    gpio_set_dir(BUTTON3_PIN, false);
    gpio_pull_up(BUTTON3_PIN);

    gpio_init(BUTTON4_PIN);
    gpio_set_dir(BUTTON4_PIN, false);
    gpio_pull_up(BUTTON4_PIN);

    gpio_init(SCREW_MOTOR_ENABLE_PIN);
    gpio_put(SCREW_MOTOR_ENABLE_PIN, true);
    gpio_set_dir(SCREW_MOTOR_ENABLE_PIN, true);

    gpio_init(SCREW_MOTOR_STEP_PIN);
    gpio_set_dir(SCREW_MOTOR_STEP_PIN, true);
    gpio_put(SCREW_MOTOR_STEP_PIN, 0);

    gpio_init(SCREW_MOTOR_DIR_PIN);
    gpio_set_dir(SCREW_MOTOR_DIR_PIN, true);
    gpio_put(SCREW_MOTOR_DIR_PIN, false);

    gpio_init(COIL_MOTOR_ENABLE_PIN);
    gpio_put(COIL_MOTOR_ENABLE_PIN, true);
    gpio_set_dir(COIL_MOTOR_ENABLE_PIN, true);

    gpio_init(COIL_MOTOR_STEP_PIN);
    gpio_set_dir(COIL_MOTOR_STEP_PIN, true);
    gpio_put(COIL_MOTOR_STEP_PIN, 0);

    gpio_init(COIL_MOTOR_DIR_PIN);
    gpio_set_dir(COIL_MOTOR_DIR_PIN, true);
    gpio_put(COIL_MOTOR_DIR_PIN, false);

    gpio_init(HOMING_BUTTON_PIN);
    gpio_set_dir(HOMING_BUTTON_PIN, false);
    gpio_pull_up(HOMING_BUTTON_PIN);

    lcd_i2c_init_with_i2c_uninitialized(
        &lcd, LCD_I2C, LCD_I2C_SCL_PIN, LCD_I2C_SDA_PIN,
        LCD_DEFAULT_I2C_ADDRESS, LCD_DEFAULT_COLS, LCD_DEFAULT_ROWS);

    lcd_i2c_puts(&lcd, "initializing...");

    configure_motor_drivers();

    rotary_encoder_init(&rotary_encoder, ENCODER_S1_PIN, ENCODER_S2_PIN,
                        ENCODER_KEY_PIN);

    gpio_set_irq_enabled_with_callback(ENCODER_S1_PIN,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true, gpio_isr);
    gpio_set_irq_enabled(ENCODER_S2_PIN,
                         GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENCODER_KEY_PIN,
                         GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(HOMING_BUTTON_PIN,
                         GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    menu_init(&lcd, &rotary_encoder);

    controller_init(controller_config);

    multicore_launch_core1(controller_core1_main);

    controller_run();
}
