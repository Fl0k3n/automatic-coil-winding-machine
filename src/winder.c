#include "winder.h"

#include <stdlib.h>

#include "hardware/sync.h"
#include "pico/stdlib.h"
#include "state.h"

#define MIN_RUNNING_MILI_REVOLUTIONS_PER_SEC 50
#define HOMING_BUTTON_POSITION_ERROR_MULTIPLIER 4
#define RUNTIME_GUIDE_SPEED_PER_ENCODER_STEP_CHANGE_PERCENT 5
#define USER_100_PERCENT_SPEED_COIL_MOTOR_MRPS 1500
// #define DEBUG_MOTOR_STEPS

volatile int _debug_motor_lateness_counter = 0;

static uint screw_microns_to_mili_revolutions(uint microns,
                                              uint screw_pitch_microns) {
    return (uint)((float)(microns) * 1000.0f / (float)(screw_pitch_microns));
}

static uint screw_microns_per_sec_to_mili_revolutions_per_sec(
    uint microns_per_sec, uint screw_pitch_microns) {
    return screw_microns_to_mili_revolutions(microns_per_sec,
                                             screw_pitch_microns);
}

static inline uint64_t get_time_us_between_steps(uint steps_per_revolution,
                                                 uint speed_mrps) {
    return 1000000000ULL / ((uint64_t)speed_mrps * steps_per_revolution);
}

static inline uint64_t get_time_us_between_steps_fix16(
    uint steps_per_revolution, uint speed_mrps) {
    return (1000000000ULL << 16) /
           ((uint64_t)speed_mrps * steps_per_revolution);
}

static inline uint motor_speed_controller_compute_speed_at(
    struct MotorSpeedController* msc, uint64_t time_us) {
    if (msc->acceleration_time_delta_us == 0) {
        return msc->target_speed_mili_revolutions_per_sec;
    }
    int64_t delta_t = (int64_t)(time_us - msc->acceleration_start_timestamp_us);

    int64_t res = (int64_t)msc->initial_speed_mili_revolutions_per_sec +
                  msc->acceleration_speed_delta * delta_t /
                      (int64_t)msc->acceleration_time_delta_us;
    if (res < 0) {
        return 0;
    } else if (res > msc->target_speed_mili_revolutions_per_sec) {
        return msc->target_speed_mili_revolutions_per_sec;
    } else {
        return (uint)res;
    }
}

static uint64_t motor_speed_controller_compute_step_period_at_fix16(
    struct MotorSpeedController* msc, uint64_t time_us) {
    uint speed_mrps;

    if (time_us >= msc->acceleration_end_timestamp_us) {
        speed_mrps = msc->target_speed_mili_revolutions_per_sec;
    } else {
        speed_mrps = motor_speed_controller_compute_speed_at(msc, time_us);

        if (msc->target_speed_mili_revolutions_per_sec >
            msc->initial_speed_mili_revolutions_per_sec) {
            speed_mrps = MAX(speed_mrps, MIN_RUNNING_MILI_REVOLUTIONS_PER_SEC);
        } else if (speed_mrps < MIN_RUNNING_MILI_REVOLUTIONS_PER_SEC) {
            return 0;
        }
    }

    if (speed_mrps == 0) {
        return 0;
    }

    return get_time_us_between_steps_fix16(msc->steps_per_revolution,
                                           speed_mrps);
}

static void motor_speed_controller_init(struct MotorSpeedController* msc,
                                        uint steps_per_revolution,
                                        uint initial_speed_mrps,
                                        uint target_speed_mrps,
                                        uint acceleration_duration_ms,
                                        uint min_time_between_steps_us) {
    msc->next_step_timestamp_us_fix16 = 0;
    msc->min_time_between_steps_us = min_time_between_steps_us;
    msc->steps_per_revolution = steps_per_revolution;
    msc->initial_speed_mili_revolutions_per_sec = initial_speed_mrps;
    msc->current_speed_mili_revolutions_per_sec = initial_speed_mrps;
    msc->target_speed_mili_revolutions_per_sec = target_speed_mrps;

    if (target_speed_mrps != 0) {
        msc->target_speed_mili_revolutions_per_sec =
            MAX(msc->target_speed_mili_revolutions_per_sec,
                MIN_RUNNING_MILI_REVOLUTIONS_PER_SEC);

        msc->time_us_between_steps_at_target_speed_fix16 =
            get_time_us_between_steps_fix16(
                msc->steps_per_revolution,
                msc->target_speed_mili_revolutions_per_sec);
    } else {
        msc->time_us_between_steps_at_target_speed_fix16 = 0;
    }

    // will be offset when the controller starts
    msc->acceleration_start_timestamp_us = 0;
    msc->acceleration_end_timestamp_us = acceleration_duration_ms * 1000;

    msc->acceleration_time_delta_us = msc->acceleration_end_timestamp_us;
    msc->acceleration_speed_delta =
        (int64_t)msc->target_speed_mili_revolutions_per_sec -
        (int64_t)msc->initial_speed_mili_revolutions_per_sec;
}

static uint64_t motor_speed_controller_compute_next_step_period_at_fix16(
    struct MotorSpeedController* msc, uint64_t step_timestamp_us) {
    if (step_timestamp_us >= msc->acceleration_end_timestamp_us) {
        return msc->time_us_between_steps_at_target_speed_fix16;
    }
    uint64_t period_at_current_speed_fix16 =
        motor_speed_controller_compute_step_period_at_fix16(msc,
                                                            step_timestamp_us);
    if (period_at_current_speed_fix16 == 0) {
        return 0;
    }
    uint64_t period_at_next_speed_fix16 =
        motor_speed_controller_compute_step_period_at_fix16(
            msc, step_timestamp_us + (period_at_current_speed_fix16 >> 16));
    if (period_at_next_speed_fix16 == 0) {
        return 0;
    }
    return (period_at_current_speed_fix16 + period_at_next_speed_fix16) / 2;
}

static inline void motor_speed_controller_track_step(
    struct MotorSpeedController* msc) {
    uint64_t step_timestamp_us = msc->next_step_timestamp_us_fix16 >> 16;

    uint64_t next_period_us_fix16 =
        motor_speed_controller_compute_next_step_period_at_fix16(
            msc, step_timestamp_us);
    if (next_period_us_fix16 == 0) {
        msc->next_step_timestamp_us_fix16 = 0;
        return;
    }
    msc->next_step_timestamp_us_fix16 += next_period_us_fix16;

    uint64_t now_us = to_us_since_boot(get_absolute_time());
    if ((msc->next_step_timestamp_us_fix16 >> 16) <=
        now_us + msc->min_time_between_steps_us) {
        _debug_motor_lateness_counter++;
        msc->next_step_timestamp_us_fix16 =
            (now_us + msc->min_time_between_steps_us) << 16;
    }
}

static inline void motor_speed_controller_ensure_started(
    struct MotorSpeedController* msc, uint64_t now_us) {
    if (msc->acceleration_start_timestamp_us == 0) {
        msc->acceleration_start_timestamp_us += now_us;
        msc->acceleration_end_timestamp_us += now_us;
    }
}

static uint64_t motor_speed_controller_get_next_step_timestamp(
    struct MotorSpeedController* msc) {
    uint64_t now_us = to_us_since_boot(get_absolute_time());

    motor_speed_controller_ensure_started(msc, now_us);

    if (msc->next_step_timestamp_us_fix16 == 0) {
        uint64_t start_timestamp_us = msc->acceleration_start_timestamp_us;

        uint64_t first_period_us_fix16 =
            motor_speed_controller_compute_next_step_period_at_fix16(
                msc, start_timestamp_us);

        if (first_period_us_fix16 == 0) {
            return 0;
        }

        msc->next_step_timestamp_us_fix16 =
            (start_timestamp_us << 16) + first_period_us_fix16;
    }

    return msc->next_step_timestamp_us_fix16 >> 16;
}

static uint compute_gcd(uint a, uint b) {
    while (b != 0) {
        uint tmp = b;
        b = a % b;
        a = tmp;
    }
    return a;
}

static inline void cmsc_update_time_us_between_steps_at_master_target_speed(
    struct CoupledMotorSpeedController* cmsc) {
    uint64_t master_period_us_fix16 = get_time_us_between_steps_fix16(
        cmsc->master->steps_per_revolution,
        cmsc->master->target_speed_mili_revolutions_per_sec);

    // inverted since period is inversely proportional to speed
    cmsc->time_us_between_steps_at_master_target_speed_fix16 =
        (master_period_us_fix16 * cmsc->speed_ratio_denominator) /
        cmsc->speed_ratio_numerator;
}

static inline void cmsc_update_max_speed_ratio_numerator(
    struct CoupledMotorSpeedController* cmsc, uint min_time_between_steps_us) {
    uint64_t master_period_us =
        cmsc->master->time_us_between_steps_at_target_speed_fix16 >> 16;
    cmsc->max_speed_ratio_numerator = master_period_us *
                                      cmsc->speed_ratio_denominator /
                                      (uint64_t)min_time_between_steps_us;
    cmsc->max_speed_ratio_numerator = MAX(cmsc->max_speed_ratio_numerator, 1);
}

static void coupled_motor_speed_controller_init(
    struct CoupledMotorSpeedController* cmsc,
    struct MotorSpeedController* master, uint speed_ratio_numerator,
    uint speed_ratio_denominator, uint step_to_speed_ratio_change_percent,
    uint min_time_between_steps_us) {
    cmsc->next_step_timestamp_us_fix16 = 0;
    cmsc->master = master;
    cmsc->min_time_between_steps_us = min_time_between_steps_us;

    uint gcd = compute_gcd(speed_ratio_numerator, speed_ratio_denominator);
    speed_ratio_numerator /= gcd;
    speed_ratio_denominator /= gcd;

    // *100 to make the step change precision better
    cmsc->speed_ratio_numerator = (uint64_t)speed_ratio_numerator * 100;
    cmsc->speed_ratio_denominator = (uint64_t)speed_ratio_denominator * 100;

    cmsc_update_max_speed_ratio_numerator(cmsc, min_time_between_steps_us);
    assert(cmsc->speed_ratio_numerator <= cmsc->max_speed_ratio_numerator);

    cmsc->speed_ratio_numerator_delta_from_step_change =
        (uint64_t)speed_ratio_numerator * step_to_speed_ratio_change_percent;

    cmsc_update_time_us_between_steps_at_master_target_speed(cmsc);
}

static uint64_t coupled_motor_speed_controller_compute_step_period_at_fix16(
    struct CoupledMotorSpeedController* cmsc, uint64_t time_us) {
    uint master_speed_mrps;

    if (time_us >= cmsc->master->acceleration_end_timestamp_us) {
        master_speed_mrps = cmsc->master->target_speed_mili_revolutions_per_sec;
    } else {
        master_speed_mrps =
            motor_speed_controller_compute_speed_at(cmsc->master, time_us);
        master_speed_mrps =
            MAX(master_speed_mrps, MIN_RUNNING_MILI_REVOLUTIONS_PER_SEC);
    }

    uint64_t master_step_period_us_fix16 = get_time_us_between_steps_fix16(
        cmsc->master->steps_per_revolution, master_speed_mrps);

    return (master_step_period_us_fix16 * cmsc->speed_ratio_denominator) /
           cmsc->speed_ratio_numerator;
}

static uint64_t
coupled_motor_speed_controller_compute_next_step_period_at_fix16(
    struct CoupledMotorSpeedController* cmsc, uint64_t step_timestamp_us) {
    if (step_timestamp_us >= cmsc->master->acceleration_end_timestamp_us) {
        return cmsc->time_us_between_steps_at_master_target_speed_fix16;
    }

    uint64_t period_at_current_speed_fix16 =
        coupled_motor_speed_controller_compute_step_period_at_fix16(
            cmsc, step_timestamp_us);

    uint64_t period_at_next_speed_fix16 =
        coupled_motor_speed_controller_compute_step_period_at_fix16(
            cmsc, step_timestamp_us + (period_at_current_speed_fix16 >> 16));

    return (period_at_current_speed_fix16 + period_at_next_speed_fix16) / 2;
}

static inline void coupled_motor_speed_controller_track_step(
    struct CoupledMotorSpeedController* cmsc) {
    uint64_t step_timestamp_us = cmsc->next_step_timestamp_us_fix16 >> 16;

    cmsc->next_step_timestamp_us_fix16 +=
        coupled_motor_speed_controller_compute_next_step_period_at_fix16(
            cmsc, step_timestamp_us);

    uint64_t now_us = to_us_since_boot(get_absolute_time());
    if ((cmsc->next_step_timestamp_us_fix16 >> 16) <=
        now_us + cmsc->min_time_between_steps_us) {
        _debug_motor_lateness_counter++;
        cmsc->next_step_timestamp_us_fix16 =
            (now_us + cmsc->min_time_between_steps_us) << 16;
    }
}

static uint64_t coupled_motor_speed_controller_get_next_step_timestamp(
    struct CoupledMotorSpeedController* cmsc) {
    uint64_t now_us = to_us_since_boot(get_absolute_time());

    motor_speed_controller_ensure_started(cmsc->master, now_us);

    if (cmsc->next_step_timestamp_us_fix16 == 0) {
        uint64_t start_timestamp_us =
            cmsc->master->acceleration_start_timestamp_us;

        uint64_t first_period_us_fix16 =
            coupled_motor_speed_controller_compute_next_step_period_at_fix16(
                cmsc, start_timestamp_us);

        cmsc->next_step_timestamp_us_fix16 =
            (start_timestamp_us << 16) + first_period_us_fix16;
    }

    return cmsc->next_step_timestamp_us_fix16 >> 16;
}

static void coupled_motor_speed_controller_change_speed(
    struct CoupledMotorSpeedController* cmsc, int delta_steps) {
    uint64_t numerator_delta =
        cmsc->speed_ratio_numerator_delta_from_step_change *
        (uint)abs(delta_steps);

    if (delta_steps > 0) {
        cmsc->speed_ratio_numerator =
            MIN(cmsc->speed_ratio_numerator + numerator_delta,
                cmsc->max_speed_ratio_numerator);
    } else if (delta_steps < 0) {
        uint64_t min_numerator =
            cmsc->speed_ratio_numerator_delta_from_step_change;
        if (numerator_delta >= cmsc->speed_ratio_numerator - min_numerator) {
            cmsc->speed_ratio_numerator = min_numerator;
        } else {
            cmsc->speed_ratio_numerator -= numerator_delta;
        }
    }

    cmsc_update_time_us_between_steps_at_master_target_speed(cmsc);
}

static inline bool homing_button_is_pressed(uint pin) { return !gpio_get(pin); }

static void set_direction(struct Winder* winder, bool right) {
    bool v1 = winder->hw_config->screw_motor_right_dir_value;
    if (!right) {
        v1 = !v1;
    }
    winder->moving_right = right;
    gpio_put(winder->hw_config->screw_motor_dir_pin, v1);
}

static inline void winder_init_motor_speed_controllers_to_start_from_idle(
    struct Winder* winder) {
    motor_speed_controller_init(
        &winder->msc, winder->hw_config->coil_motor_num_steps_per_revolution, 0,
        winder->desired_full_coil_motor_speed_mili_revolutions_per_sec, 2000,
        winder->hw_config->min_time_between_coil_motor_steps_us);

    uint speed_ratio_numerator =
        winder->winding_config.wire_pitch_microns *
        winder->hw_config->screw_motor_num_steps_per_revolution;
    uint speed_ratio_denominator =
        winder->hw_config->screw_pitch_microns *
        winder->hw_config->coil_motor_num_steps_per_revolution;
    coupled_motor_speed_controller_init(
        &winder->cmsc, &winder->msc, speed_ratio_numerator,
        speed_ratio_denominator,
        RUNTIME_GUIDE_SPEED_PER_ENCODER_STEP_CHANGE_PERCENT,
        winder->hw_config->min_time_between_screw_motor_steps_us);
    winder->total_acked_guide_speed_delta = 0;
}

void winder_init(struct Winder* winder,
                 const struct WinderHardwareConfig* winder_hardware_config,
                 struct WindingConfig winding_config,
                 volatile bool* can_continue) {
    winder->hw_config = winder_hardware_config;
    winder->winding_config = winding_config;
    winder->moving_right = true;
    winder->can_continue = can_continue;
    winder->winding_state = WINDER_STATE_INITIALIZED;
    winder->incremental_guide_speed_delta = 0;
    winder->total_acked_guide_speed_delta = 0;

    uint64_t max_coil_motor_speed_mrps =
        1000000000ULL /
        ((uint64_t)winder->hw_config->min_time_between_coil_motor_steps_us *
         winder->hw_config->coil_motor_num_steps_per_revolution);
    uint64_t coil_motor_speed_mrps_100percent_speed_adjustment =
        MIN(max_coil_motor_speed_mrps, USER_100_PERCENT_SPEED_COIL_MOTOR_MRPS);

    winder->desired_full_coil_motor_speed_mili_revolutions_per_sec =
        (uint)(coil_motor_speed_mrps_100percent_speed_adjustment *
               winder->winding_config.speed_percent / 100);

    winder_init_motor_speed_controllers_to_start_from_idle(winder);
    winder->total_acked_guide_speed_delta = 0;
    winder->guide_position_microns =
        int_to_fix8((int)winding_config.winding_start_offset_microns);
    winder->guide_step_position_microns_delta = float_to_fix8(
        (float)winder->hw_config->screw_pitch_microns /
        (float)winder->hw_config->screw_motor_num_steps_per_revolution);
    winder->num_current_coil_motor_steps = 0;
    winder->num_total_required_coil_motor_steps =
        (winding_config.number_of_turns - 1) *
        winder->hw_config->coil_motor_num_steps_per_revolution;

    winder->homing_button_press_inconsistency_tolerance =
        HOMING_BUTTON_POSITION_ERROR_MULTIPLIER *
        winder->hw_config->homing_button_press_range_microns;
}

#ifdef DEBUG_MOTOR_STEPS
#define STEP_TIMING_DEBUG_BUFFER_SIZE 128
volatile uint32_t step_timing_debug_us[STEP_TIMING_DEBUG_BUFFER_SIZE];

static void _debug_track_screw_motor_step(void) {
    static volatile uint32_t step_timing_debug_index = 0;
    static volatile uint32_t step_timing_debug_wraps = 0;
    static uint64_t previous_step_timestamp_us = 0;

    uint64_t actual_step_timestamp_us = to_us_since_boot(get_absolute_time());

    if (previous_step_timestamp_us != 0) {
        step_timing_debug_us[step_timing_debug_index] =
            (uint32_t)(actual_step_timestamp_us - previous_step_timestamp_us);

        step_timing_debug_index++;

        if (step_timing_debug_index == STEP_TIMING_DEBUG_BUFFER_SIZE) {
            step_timing_debug_index = 0;
            step_timing_debug_wraps++;
        }
    }
    previous_step_timestamp_us = actual_step_timestamp_us;
}
#endif  // DEBUG_MOTOR_STEPS

static void winder_reverse_direction(struct Winder* winder) {
    set_direction(winder, !winder->moving_right);
    winder->guide_step_position_microns_delta =
        multfix8(winder->guide_step_position_microns_delta, int_to_fix8(-1));
}

static bool winder_step_and_maybe_reverse_screw_motor(struct Winder* winder) {
    gpio_put(winder->hw_config->screw_motor_step_pin, 1);
    sleep_us(1);
    gpio_put(winder->hw_config->screw_motor_step_pin, 0);
    coupled_motor_speed_controller_track_step(&winder->cmsc);

#ifdef DEBUG_MOTOR_STEPS
    _debug_track_screw_motor_step();
#endif

    winder->guide_position_microns += winder->guide_step_position_microns_delta;

    int guide_pos_microns = fix8_to_int(winder->guide_position_microns);
    bool homing_button_pressed =
        homing_button_is_pressed(winder->hw_config->homing_button_pin);

    bool cant_be_pressed =
        guide_pos_microns >=
        (int)winder->homing_button_press_inconsistency_tolerance;
    bool must_be_pressed =
        guide_pos_microns < 0 &&
        (uint)(-guide_pos_microns) >
            winder->homing_button_press_inconsistency_tolerance;
    if ((cant_be_pressed && homing_button_pressed) ||
        (must_be_pressed && !homing_button_pressed)) {
        return false;
    }

    if (winder->moving_right) {
        if (guide_pos_microns >= (int)winder->winding_config.length_microns) {
            winder_reverse_direction(winder);
        }
    } else if (homing_button_pressed) {
        winder_reverse_direction(winder);
        winder->guide_position_microns = 0;
    }
    return true;
}

static void winder_step_coil_motor(struct Winder* winder) {
    gpio_put(winder->hw_config->coil_motor_step_pin, 1);
    sleep_us(1);
    gpio_put(winder->hw_config->coil_motor_step_pin, 0);
    motor_speed_controller_track_step(&winder->msc);
    winder->num_current_coil_motor_steps++;
}

static inline void winder_maybe_update_guide_speed_based_on_runtime_user_input(
    struct Winder* winder) {
    if (winder->incremental_guide_speed_delta == 0) {
        return;
    }
    uint32_t irq_save = save_and_disable_interrupts();
    int actual_delta = winder->incremental_guide_speed_delta;
    winder->incremental_guide_speed_delta = 0;
    restore_interrupts(irq_save);
    if (actual_delta == 0) {
        return;
    }
    winder->total_acked_guide_speed_delta += actual_delta;
    coupled_motor_speed_controller_change_speed(&winder->cmsc, actual_delta);
}

static void winder_continue_winding(struct Winder* winder) {
    uint64_t next_coil_motor_step_timestamp_us =
        motor_speed_controller_get_next_step_timestamp(&winder->msc);
    uint64_t next_screw_motor_step_timestamp_us =
        coupled_motor_speed_controller_get_next_step_timestamp(&winder->cmsc);

    while (winder->num_current_coil_motor_steps <
               winder->num_total_required_coil_motor_steps &&
           *winder->can_continue &&
           winder->winding_state == WINDER_STATE_WINDING) {
        uint64_t now_us = to_us_since_boot(get_absolute_time());
        winder_maybe_update_guide_speed_based_on_runtime_user_input(winder);

        if (next_screw_motor_step_timestamp_us <=
            next_coil_motor_step_timestamp_us) {
            if (now_us < next_screw_motor_step_timestamp_us) {
                continue;
            }
            bool ok = winder_step_and_maybe_reverse_screw_motor(winder);
            if (!ok) {
                winder_stop_with_error(winder,
                                       WINDING_ERROR_GUIDE_POSITION_MISMATCH);
                break;
            }
            next_screw_motor_step_timestamp_us =
                coupled_motor_speed_controller_get_next_step_timestamp(
                    &winder->cmsc);
            continue;
        }

        if (now_us < next_coil_motor_step_timestamp_us) {
            continue;
        }

        winder_step_coil_motor(winder);
        next_coil_motor_step_timestamp_us =
            motor_speed_controller_get_next_step_timestamp(&winder->msc);
    }

    if (winder->winding_state != WINDER_STATE_WINDING_ERROR) {
        if (winder->num_current_coil_motor_steps >=
            winder->num_total_required_coil_motor_steps) {
            winder->winding_state = WINDER_STATE_WINDING_FINISHED;
        } else {
            winder->winding_state = WINDER_STATE_WINDING_INTERRUPTED;
        }
    }
}

enum WinderWindingState winder_wind(struct Winder* winder) {
    struct Jogger jogger;
    jogger_init_from_winder_hardware_config(
        &jogger, winder->hw_config, winder_get_guide_position_microns(winder),
        winder->can_continue);
    winder->winding_state = WINDER_STATE_JOGGING;
    bool jog_finished = jogger_jog(&jogger);
    if (!jog_finished) {
        winder->winding_state = WINDER_STATE_JOG_INTERRUPTED;
        return winder->winding_state;
    }
    winder->winding_state = WINDER_STATE_WINDING;

    // just for visual inspection to see where it stopped
    sleep_ms(250);

    set_direction(winder, true);
    gpio_put(winder->hw_config->coil_motor_dir_pin,
             winder->hw_config->coil_motor_wind_dir_value);
    sleep_us(50);
    winder_continue_winding(winder);
    return winder->winding_state;
}

enum WinderWindingState winder_resume(struct Winder* winder) {
    assert(winder->winding_state == WINDER_STATE_WINDING_INTERRUPTED ||
           winder->winding_state == WINDER_STATE_WINDING_FINISHED ||
           winder->winding_state == WINDER_STATE_WINDING_ERROR);
    winder->winding_state = WINDER_STATE_WINDING;
    gpio_put(winder->hw_config->coil_motor_dir_pin,
             winder->hw_config->coil_motor_wind_dir_value);
    gpio_put(winder->hw_config->screw_motor_dir_pin,
             winder->moving_right
                 ? winder->hw_config->screw_motor_right_dir_value
                 : !winder->hw_config->screw_motor_right_dir_value);
    sleep_us(50);
    // we may lose a part of screw motor step but that shouldn't matter in
    // practice
    winder_init_motor_speed_controllers_to_start_from_idle(winder);
    winder_continue_winding(winder);
    return winder->winding_state;
}

void winder_add_extra_turns(struct Winder* winder, uint num_extra_turns) {
    winder->winding_config.number_of_turns += num_extra_turns;
    winder->num_total_required_coil_motor_steps +=
        num_extra_turns *
        winder->hw_config->coil_motor_num_steps_per_revolution;
}

void winder_stop_with_error(struct Winder* winder,
                            enum WindingErrorCode error_code) {
    // no need to bother with locking here, race for this state change is
    // harmless
    winder->error_code = error_code;
    winder->winding_state = WINDER_STATE_WINDING_ERROR;
}

uint64_t winder_get_acceleration_end_timestamp_us(struct Winder* winder) {
    if (winder->winding_state != WINDER_STATE_WINDING) {
        return 0;
    }
    return winder->msc.acceleration_end_timestamp_us;
}

int winder_get_guide_position_microns(struct Winder* winder) {
    return fix8_to_int(winder->guide_position_microns);
}

uint winder_get_num_current_turns(struct Winder* winder) {
    // + 1 for the initial user-made turn
    return winder->num_current_coil_motor_steps /
               winder->hw_config->coil_motor_num_steps_per_revolution +
           1;
}

void jogger_init(struct Jogger* jogger, uint screw_motor_dir_pin,
                 uint screw_motor_step_pin, bool screw_motor_right_dir_value,
                 uint screw_pitch_microns, uint steps_per_revolution,
                 uint homing_button_pin, uint homing_button_press_range_microns,
                 uint min_time_between_screw_motor_steps_us,
                 int target_delta_from_homing_button_contact_microns,
                 volatile bool* can_continue) {
    jogger->screw_motor_step_pin = screw_motor_step_pin;
    jogger->screw_motor_dir_pin = screw_motor_dir_pin;
    jogger->screw_motor_right_dir_value = screw_motor_right_dir_value;
    jogger->screw_pitch_microns = screw_pitch_microns;
    jogger->steps_per_revolution = steps_per_revolution;
    jogger->min_time_between_screw_motor_steps_us =
        min_time_between_screw_motor_steps_us;

    jogger->homing_button_pin = homing_button_pin;
    jogger->homing_button_press_range_microns =
        homing_button_press_range_microns;
    jogger->target_delta_from_homing_button_contact_microns =
        target_delta_from_homing_button_contact_microns;
    if (jogger->target_delta_from_homing_button_contact_microns < 0 &&
        (uint)(-jogger->target_delta_from_homing_button_contact_microns) >
            homing_button_press_range_microns) {
        jogger->target_delta_from_homing_button_contact_microns =
            -(int)homing_button_press_range_microns;
    }

    jogger->max_mrps =
        (uint)(((1000.0 * 1e6) /
                (double)(jogger->min_time_between_screw_motor_steps_us *
                         jogger->steps_per_revolution)));

    jogger->can_continue = can_continue;
    // debounce homing button just in case
    sleep_ms(1);
    if (homing_button_is_pressed(jogger->homing_button_pin)) {
        jogger->jog_state = JOG_MOVING_RIGHT_FROM_INITIALLY_PRESSED_BUTTON;
    } else {
        jogger->jog_state = JOG_MOVING_LEFT_TOWARDS_PRESSED_BUTTON;
    }
}

static void jogger_step(struct Jogger* jogger,
                        struct MotorSpeedController* msc) {
    gpio_put(jogger->screw_motor_step_pin, 1);
    sleep_us(1);
    gpio_put(jogger->screw_motor_step_pin, 0);
    motor_speed_controller_track_step(msc);
}

#define JOGGER_FULL_MOTOR_SPEED 0
static void jogger_configure_motor_speed_controller(
    struct Jogger* jogger, struct MotorSpeedController* msc,
    uint target_microns_per_sec, bool right) {
    uint target_mrps =
        target_microns_per_sec == JOGGER_FULL_MOTOR_SPEED
            ? jogger->max_mrps
            : screw_microns_per_sec_to_mili_revolutions_per_sec(
                  target_microns_per_sec, jogger->screw_pitch_microns);
    motor_speed_controller_init(msc, jogger->steps_per_revolution, 0,
                                target_mrps, 2000,
                                jogger->min_time_between_screw_motor_steps_us);
    gpio_put(jogger->screw_motor_dir_pin,
             right ? jogger->screw_motor_right_dir_value
                   : !jogger->screw_motor_right_dir_value);
}

static inline void jogger_configure_motor_speed_controller_for_right_direction(
    struct Jogger* jogger, struct MotorSpeedController* msc,
    uint target_microns_per_sec) {
    jogger_configure_motor_speed_controller(jogger, msc, target_microns_per_sec,
                                            true);
}

static void jogger_configure_motor_speed_controller_for_left_direction(
    struct Jogger* jogger, struct MotorSpeedController* msc,
    uint target_microns_per_sec) {
    jogger_configure_motor_speed_controller(jogger, msc, target_microns_per_sec,
                                            false);
}

bool jogger_jog(struct Jogger* jogger) {
    struct MotorSpeedController msc;
    if (jogger->jog_state == JOG_MOVING_LEFT_TOWARDS_PRESSED_BUTTON) {
        jogger_configure_motor_speed_controller_for_left_direction(
            jogger, &msc, JOGGER_FULL_MOTOR_SPEED);
    } else {
        jogger_configure_motor_speed_controller_for_right_direction(jogger,
                                                                    &msc, 1000);
    }

    uint64_t next_step_timestamp =
        motor_speed_controller_get_next_step_timestamp(&msc);
    uint finalizing_stage_remaining_steps = 0;

    // the goal is to be at almost not pressed position (a few steps right
    // will cause state change) then in final calibrations stage move
    // target_delta_from_homing_button_contact_microns

    while (jogger->jog_state != JOG_FINISHED && *jogger->can_continue) {
        uint64_t now_us = to_us_since_boot(get_absolute_time());
        if (now_us < next_step_timestamp) {
            continue;
        }
        if (jogger->jog_state ==
            JOG_MOVING_RIGHT_FROM_INITIALLY_PRESSED_BUTTON) {
            jogger_step(jogger, &msc);
            if (!homing_button_is_pressed(jogger->homing_button_pin)) {
                sleep_ms(1);
                if (!homing_button_is_pressed(jogger->homing_button_pin)) {
                    jogger->jog_state = JOG_MOVING_LEFT_TOWARDS_PRESSED_BUTTON;
                    jogger_configure_motor_speed_controller_for_left_direction(
                        jogger, &msc, 500);
                }
            }
        } else if (jogger->jog_state ==
                   JOG_MOVING_LEFT_TOWARDS_PRESSED_BUTTON) {
            jogger_step(jogger, &msc);
            if (homing_button_is_pressed(jogger->homing_button_pin)) {
                sleep_ms(1);
                if (homing_button_is_pressed(jogger->homing_button_pin)) {
                    jogger->jog_state = JOG_FINAL_CALIBRATIONS;
                    if (jogger
                            ->target_delta_from_homing_button_contact_microns >
                        0) {
                        jogger_configure_motor_speed_controller_for_right_direction(
                            jogger, &msc, 1000);
                    } else {
                        jogger_configure_motor_speed_controller_for_left_direction(
                            jogger, &msc, 1000);
                    }
                    sleep_ms(5);
                    finalizing_stage_remaining_steps =
                        screw_microns_to_mili_revolutions(
                            (uint)abs(
                                jogger
                                    ->target_delta_from_homing_button_contact_microns),
                            jogger->screw_pitch_microns) *
                        jogger->steps_per_revolution / 1000;
                }
            }
        } else {
            if (finalizing_stage_remaining_steps == 0) {
                jogger->jog_state = JOG_FINISHED;
                break;
            }
            finalizing_stage_remaining_steps--;
            jogger_step(jogger, &msc);
        }
        next_step_timestamp =
            motor_speed_controller_get_next_step_timestamp(&msc);
    }

    return jogger->jog_state == JOG_FINISHED;
}
