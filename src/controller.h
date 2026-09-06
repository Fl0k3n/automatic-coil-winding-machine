#pragma once
#include "pico/stdlib.h"

struct ControllerConfig {
    uint coil_motor_enable_pin;
    uint coil_motor_wind_button_pin;
    uint coil_motor_unwind_button_pin;
    uint screw_motor_enable_pin;
    uint screw_motor_left_button_pin;
    uint screw_motor_righ_button_pin;
    uint homing_button_pin;

    uint coil_motor_step_pin;
    uint coil_motor_dir_pin;
    uint screw_motor_step_pin;
    uint screw_motor_dir_pin;

    bool coil_motor_wind_dir_value;
    bool screw_motor_right_dir_value;
    uint screw_pitch_microns;

    uint min_time_between_screw_motor_steps_us;
    uint min_time_between_coil_motor_steps_us;

    uint async_coil_motor_step_interval_us;
    uint async_screw_motor_step_interval_us;

    uint full_rotation_num_steps_coil_motor;
    uint full_rotation_num_steps_screw_motor;

    uint paused_or_error_led_pin;
    uint active_indicator_led_pin;

    uint homing_button_press_range_microns;
};

void controller_handle_rotary_encoder_button_pressed_irq(void);
void controller_handle_homing_button_pressed_irq(void);
void controller_handle_homing_button_released_irq(void);

void controller_init(struct ControllerConfig cfg);
void controller_core1_main(void);

void controller_run(void);
