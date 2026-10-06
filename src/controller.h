#pragma once
#include "hardware/pio.h"
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

    // the lower the harder motor has to work, returns true if read
    // successful
    bool (*blocking_read_coil_motor_load)(uint16_t* result);

    // constant parasitic C should be included
    double L_meter_LC_tank_capacitance;
    double L_meter_LC_tank_parasitic_series_inductance;
    bool L_meter_power_on_value;
    uint L_meter_comparator_output_pin;
    uint L_meter_power_switch_pin;
    pio_hw_t* L_meter_pio;
    uint L_meter_pio_sm;
    uint L_meter_pio_program_offset;
};

void controller_handle_rotary_encoder_button_pressed_irq(void);
void controller_handle_rotary_encoder_delta_changed_irq(int incremental_delta);
void controller_handle_homing_button_pressed_irq(void);
void controller_handle_homing_button_released_irq(void);

void controller_init(struct ControllerConfig cfg);
void controller_core1_main(void);

void controller_run(void);
