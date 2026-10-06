#pragma once

#include <stddef.h>

#include "fixedpoint.h"
#include "pico/stdlib.h"
#include "state.h"

struct WinderHardwareConfig {
    uint screw_motor_num_steps_per_revolution,
        coil_motor_num_steps_per_revolution;
    uint screw_motor_step_pin, coil_motor_step_pin;
    uint screw_motor_dir_pin, coil_motor_dir_pin;
    bool screw_motor_right_dir_value, coil_motor_wind_dir_value;
    uint homing_button_press_range_microns;
    uint homing_button_pin;
    uint screw_pitch_microns;
    uint min_time_between_screw_motor_steps_us;
    uint min_time_between_coil_motor_steps_us;
};

struct MotorSpeedController {
    uint steps_per_revolution;
    uint min_time_between_steps_us;

    uint initial_speed_mili_revolutions_per_sec;
    uint current_speed_mili_revolutions_per_sec;
    uint target_speed_mili_revolutions_per_sec;

    uint64_t next_step_timestamp_us_fix16;
    uint64_t time_us_between_steps_at_target_speed_fix16;

    uint64_t acceleration_start_timestamp_us;
    uint64_t acceleration_end_timestamp_us;
    uint64_t acceleration_time_delta_us;
    int64_t acceleration_speed_delta;
};

struct CoupledMotorSpeedController {
    struct MotorSpeedController* master;
    uint min_time_between_steps_us;

    uint64_t speed_ratio_numerator;
    uint64_t speed_ratio_denominator;

    uint64_t speed_ratio_numerator_delta_from_step_change;

    uint64_t time_us_between_steps_at_master_target_speed_fix16;

    uint64_t min_speed_ratio_numerator;
    uint64_t max_speed_ratio_numerator;

    uint64_t next_step_timestamp_us_fix16;
};

enum WinderWindingState {
    WINDER_STATE_INITIALIZED = 0,
    WINDER_STATE_JOGGING,
    WINDER_STATE_JOG_INTERRUPTED,
    WINDER_STATE_WINDING,
    WINDER_STATE_WINDING_INTERRUPTED,
    WINDER_STATE_WINDING_FINISHED,
    WINDER_STATE_WINDING_ERROR,
};

struct Winder {
    const struct WinderHardwareConfig* hw_config;
    struct WindingConfig winding_config;

    volatile enum WinderWindingState winding_state;
    volatile enum WindingErrorCode error_code;

    // coil motor dictates speed, screw motor moves as fast as possible to stay
    // in sync (it's up to the user to configure it so that screw motor is
    // physically capable of keeping up, which is guaranteed with current
    // hardware)
    struct MotorSpeedController msc;
    struct CoupledMotorSpeedController cmsc;
    uint desired_full_coil_motor_speed_mili_revolutions_per_sec;
    bool moving_right;

    // with 2000microns screw, at full (200) step delta is integer,
    // with 8x stepping it's 1.25 and 0.625 at 16x stepping. All should be fine
    // with this resolution fixed point. Range is also larger than 1m and
    // we need <30cm.
    fix8 guide_step_position_microns_delta;
    fix8 guide_position_microns;

    size_t num_current_coil_motor_steps;
    size_t num_total_required_coil_motor_steps;

    // if homing button is pressed/not pressed and guide position is offset from
    // the expected 0 position by more than this value then winder reports an
    // error and stops
    uint homing_button_press_inconsistency_tolerance;

    volatile bool* can_continue;

    volatile int incremental_guide_speed_delta;
    volatile int total_acked_guide_speed_delta;
};

void winder_init(struct Winder* winder,
                 const struct WinderHardwareConfig* winder_hardware_config,
                 struct WindingConfig winding_config,
                 volatile bool* can_continue);

int winder_get_guide_position_microns(struct Winder* winder);
uint winder_get_num_current_turns(struct Winder* winder);

enum WinderWindingState winder_wind(struct Winder* winder);

enum WinderWindingState winder_resume(struct Winder* winder);

void winder_add_extra_turns(struct Winder* winder, uint num_extra_turns);

void winder_stop_with_error(struct Winder* winder,
                            enum WindingErrorCode error_code);

uint64_t winder_get_acceleration_end_timestamp_us(struct Winder* winder);

enum WinderJogState {
    JOG_MOVING_RIGHT_FROM_INITIALLY_PRESSED_BUTTON = 0,
    JOG_MOVING_LEFT_TOWARDS_PRESSED_BUTTON,
    JOG_FINAL_CALIBRATIONS,
    JOG_FINISHED,
};

struct Jogger {
    uint screw_motor_dir_pin;
    uint screw_motor_step_pin;
    bool screw_motor_right_dir_value;
    uint screw_pitch_microns;
    uint steps_per_revolution;
    uint min_time_between_screw_motor_steps_us;

    uint max_mrps;
    uint homing_button_pin;
    uint homing_button_press_range_microns;
    int target_delta_from_homing_button_contact_microns;

    enum WinderJogState jog_state;
    volatile bool* can_continue;
};

void jogger_init(struct Jogger* jogger, uint screw_motor_dir_pin,
                 uint screw_motor_step_pin, bool screw_motor_right_dir_value,
                 uint screw_pitch_microns, uint steps_per_revolution,
                 uint homing_button_pin, uint homing_button_press_range_microns,
                 uint min_time_between_screw_motor_steps_us,
                 int target_delta_from_homing_button_contact_microns,
                 volatile bool* can_continue);

static inline void jogger_init_from_winder_hardware_config(
    struct Jogger* jogger, const struct WinderHardwareConfig* hw_conf,
    int target_delta_from_homing_button_contact_microns,
    volatile bool* can_continue) {
    jogger_init(
        jogger, hw_conf->screw_motor_dir_pin, hw_conf->screw_motor_step_pin,
        hw_conf->screw_motor_right_dir_value, hw_conf->screw_pitch_microns,
        hw_conf->screw_motor_num_steps_per_revolution,
        hw_conf->homing_button_pin, hw_conf->homing_button_press_range_microns,
        hw_conf->min_time_between_screw_motor_steps_us,
        target_delta_from_homing_button_contact_microns, can_continue);
}

bool jogger_jog(struct Jogger* jogger);
