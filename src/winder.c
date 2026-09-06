#include "winder.h"

#include <stdlib.h>

#include "state.h"

#define MIN_RUNNING_MILI_REVOLUTIONS_PER_SEC 50
#define HOMING_BUTTON_POSITION_ERROR_MULTIPLIER 4

static uint screw_microns_to_mili_revolutions(uint microns,
                                              uint screw_pitch_microns) {
    return (uint)((float)(microns) * 1000.0f / (float)(screw_pitch_microns));
}

static uint screw_microns_per_sec_to_mili_revolutions_per_sec(
    uint microns_per_sec, uint screw_pitch_microns) {
    return screw_microns_to_mili_revolutions(microns_per_sec,
                                             screw_pitch_microns);
}

static inline uint get_time_us_between_steps(uint steps_per_revolution,
                                             uint speed_mrps) {
    return 1000 * (1000000 / (speed_mrps * steps_per_revolution));
}

static inline uint motor_speed_controller_compute_speed_at(
    struct MotorSpeedController* msc, uint64_t time_us) {
    if (msc->acceleration_time_delta_us == 0) {
        return msc->target_speed_mili_revolutions_per_sec;
    }
    uint64_t delta_t = time_us - msc->acceleration_start_timestamp_us;
    int64_t res = (int64_t)msc->initial_speed_mili_revolutions_per_sec +
                  (msc->acceleration_speed_delta * delta_t) /
                      msc->acceleration_time_delta_us;
    if (res < 0) {
        return 0;
    } else if (res > msc->target_speed_mili_revolutions_per_sec) {
        return msc->target_speed_mili_revolutions_per_sec;
    } else {
        return (uint)res;
    }
}

static void motor_speed_controller_init(struct MotorSpeedController* msc,
                                        uint steps_per_revolution,
                                        uint initial_speed_mrps,
                                        uint target_speed_mrps,
                                        uint acceleration_duration_ms) {
    msc->last_step_timestamp_us = 0;
    msc->steps_per_revolution = steps_per_revolution;
    msc->initial_speed_mili_revolutions_per_sec = initial_speed_mrps;
    msc->current_speed_mili_revolutions_per_sec = initial_speed_mrps;
    msc->target_speed_mili_revolutions_per_sec = target_speed_mrps;
    if (target_speed_mrps != 0) {
        msc->target_speed_mili_revolutions_per_sec =
            MAX(msc->target_speed_mili_revolutions_per_sec,
                MIN_RUNNING_MILI_REVOLUTIONS_PER_SEC);
    }

    if (target_speed_mrps == 0) {
        msc->time_us_between_steps_at_target_speed = 0;
    } else {
        msc->time_us_between_steps_at_target_speed = get_time_us_between_steps(
            msc->steps_per_revolution,
            msc->target_speed_mili_revolutions_per_sec);
    }

    // will be offset at first step
    msc->acceleration_start_timestamp_us = 0;
    msc->acceleration_end_timestamp_us = acceleration_duration_ms * 1000;

    msc->acceleration_time_delta_us = msc->acceleration_end_timestamp_us;
    msc->acceleration_speed_delta =
        (int64_t)msc->target_speed_mili_revolutions_per_sec -
        (int64_t)msc->initial_speed_mili_revolutions_per_sec;
}

static void motor_speed_controller_update_target_speed(
    struct MotorSpeedController* msc, uint new_target_speed_mrps,
    uint acceleration_duration_ms) {
    msc->initial_speed_mili_revolutions_per_sec =
        msc->current_speed_mili_revolutions_per_sec;
    msc->target_speed_mili_revolutions_per_sec = new_target_speed_mrps;
    if (new_target_speed_mrps != 0) {
        msc->target_speed_mili_revolutions_per_sec =
            MAX(msc->target_speed_mili_revolutions_per_sec,
                MIN_RUNNING_MILI_REVOLUTIONS_PER_SEC);
    }
    msc->last_step_timestamp_us = to_us_since_boot(get_absolute_time());

    if (new_target_speed_mrps == 0) {
        msc->time_us_between_steps_at_target_speed = 0;
    } else {
        msc->time_us_between_steps_at_target_speed = get_time_us_between_steps(
            msc->steps_per_revolution,
            msc->target_speed_mili_revolutions_per_sec);
    }

    msc->acceleration_time_delta_us = acceleration_duration_ms * 1000;
    msc->acceleration_start_timestamp_us = msc->last_step_timestamp_us;
    msc->acceleration_end_timestamp_us =
        msc->acceleration_start_timestamp_us + msc->acceleration_time_delta_us;

    msc->acceleration_speed_delta =
        (int64_t)msc->target_speed_mili_revolutions_per_sec -
        (int64_t)msc->initial_speed_mili_revolutions_per_sec;
}

static inline void motor_speed_controller_track_step(
    struct MotorSpeedController* msc) {
    msc->last_step_timestamp_us = to_us_since_boot(get_absolute_time());
}

static uint64_t motor_speed_controller_get_next_step_timestamp(
    struct MotorSpeedController* msc) {
    uint64_t now_us = to_us_since_boot(get_absolute_time());

    bool is_first_step_after_init = msc->acceleration_start_timestamp_us == 0;
    if (is_first_step_after_init) {
        msc->acceleration_start_timestamp_us += now_us;
        msc->acceleration_end_timestamp_us += now_us;
    }

    if (now_us <= msc->acceleration_end_timestamp_us) {
        msc->current_speed_mili_revolutions_per_sec =
            motor_speed_controller_compute_speed_at(msc, now_us);
        if (msc->target_speed_mili_revolutions_per_sec >
            msc->initial_speed_mili_revolutions_per_sec) {
            msc->current_speed_mili_revolutions_per_sec =
                MAX(msc->current_speed_mili_revolutions_per_sec,
                    MIN_RUNNING_MILI_REVOLUTIONS_PER_SEC);
        } else if (msc->current_speed_mili_revolutions_per_sec <
                   MIN_RUNNING_MILI_REVOLUTIONS_PER_SEC) {
            // stop condition
            return 0;
        }
        uint next_step_us_at_current_speed =
            msc->last_step_timestamp_us +
            get_time_us_between_steps(
                msc->steps_per_revolution,
                msc->current_speed_mili_revolutions_per_sec);
        uint speed_at_next_step = motor_speed_controller_compute_speed_at(
            msc, next_step_us_at_current_speed);
        speed_at_next_step =
            MAX(speed_at_next_step, MIN_RUNNING_MILI_REVOLUTIONS_PER_SEC);
        uint next_step_us_at_next_speed =
            msc->last_step_timestamp_us +
            get_time_us_between_steps(msc->steps_per_revolution,
                                      speed_at_next_step);
        return (next_step_us_at_current_speed + next_step_us_at_next_speed) / 2;
    }
    return msc->last_step_timestamp_us +
           msc->time_us_between_steps_at_target_speed;
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

static inline void winder_init_motor_speed_controller_to_start_from_idle(
    struct Winder* winder) {
    motor_speed_controller_init(
        &winder->msc, winder->hw_config->coil_motor_num_steps_per_revolution, 0,
        winder->desired_full_coil_motor_speed_mili_revoluions_per_sec, 2000);
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

    float min_time_between_steps =
        (float)winder->hw_config->min_time_between_screw_motor_steps_us /
        1000.0f;
    float max_possible_rps = 1 / min_time_between_steps;
    float desired_rps = ((float)winder->winding_config.speed_percent / 100.0f) *
                        max_possible_rps;
    winder->desired_full_coil_motor_speed_mili_revoluions_per_sec =
        (uint)(desired_rps * 1000.0f);

    winder_init_motor_speed_controller_to_start_from_idle(winder);
    winder->next_screw_motor_step_timestamp_us = 0;
    winder->guide_position_microns =
        int_to_fix8((int)winding_config.winding_start_offset_microns);
    winder->guide_step_position_microns_delta = float_to_fix8(
        (float)winder->hw_config->screw_pitch_microns /
        (float)winder->hw_config->screw_motor_num_steps_per_revolution);
    winder->num_current_coil_motor_steps = 0;
    winder->num_total_required_coil_motor_steps =
        (winding_config.number_of_turns - 1) *
        winder->hw_config->coil_motor_num_steps_per_revolution;

    winder->guide_motor_step_ratio_dda.accumulator = 0;
    winder->guide_motor_step_ratio_dda.numerator =
        winding_config.wire_pitch_microns *
        winder->hw_config->screw_motor_num_steps_per_revolution;
    winder->guide_motor_step_ratio_dda.denominator =
        winder->hw_config->screw_pitch_microns *
        winder->hw_config->coil_motor_num_steps_per_revolution;

    winder->homing_button_press_inconsistency_tolerance =
        HOMING_BUTTON_POSITION_ERROR_MULTIPLIER *
        winder->hw_config->homing_button_press_range_microns;
}

static bool winder_step_screw_motor_if_possible(struct Winder* winder,
                                                uint64_t now_us) {
    if (now_us < winder->next_screw_motor_step_timestamp_us ||
        !dda_has_integer_part(&winder->guide_motor_step_ratio_dda)) {
        return false;
    }
    dda_subtract_integer_part(&winder->guide_motor_step_ratio_dda);
    gpio_put(winder->hw_config->screw_motor_step_pin, 1);
    sleep_us(1);
    gpio_put(winder->hw_config->screw_motor_step_pin, 0);
    winder->next_screw_motor_step_timestamp_us +=
        winder->hw_config->min_time_between_screw_motor_steps_us;
    winder->guide_position_microns += winder->guide_step_position_microns_delta;
    return true;
}

static void winder_reverse_direction(struct Winder* winder) {
    set_direction(winder, !winder->moving_right);
    winder->guide_step_position_microns_delta =
        multfix8(winder->guide_step_position_microns_delta, int_to_fix8(-1));
    // dda accumulator must have 0 integral part at this point
    // some remainder would be possible only if coil motor was moving faster
    // than screw motor is allowed to move. Input validation must reject such
    // speed values.
}

static void winder_continue_winding(struct Winder* winder) {
    uint next_coil_motor_step_timestamp_us =
        motor_speed_controller_get_next_step_timestamp(&winder->msc);

    while (winder->num_current_coil_motor_steps <
               winder->num_total_required_coil_motor_steps &&
           *winder->can_continue) {
        uint now_us = to_us_since_boot(get_absolute_time());
        if (now_us <= next_coil_motor_step_timestamp_us) {
            winder_step_screw_motor_if_possible(winder, now_us);
            continue;
        }

        winder->num_current_coil_motor_steps++;
        dda_increment(&winder->guide_motor_step_ratio_dda);

        gpio_put(winder->hw_config->coil_motor_step_pin, 1);
        if (!winder_step_screw_motor_if_possible(winder, now_us)) {
            sleep_us(1);
        }
        gpio_put(winder->hw_config->coil_motor_step_pin, 0);
        motor_speed_controller_track_step(&winder->msc);
        next_coil_motor_step_timestamp_us =
            motor_speed_controller_get_next_step_timestamp(&winder->msc);

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
            winder->winding_state = WINDER_STATE_WINDING_ERROR;
            return;
        }

        if (winder->moving_right) {
            if (guide_pos_microns >=
                (int)winder->winding_config.length_microns) {
                winder_reverse_direction(winder);
            }
        } else {
            if (homing_button_pressed) {
                winder_reverse_direction(winder);
                winder->guide_position_microns = 0;
            }
        }
    }
    if (winder->num_current_coil_motor_steps >=
        winder->num_total_required_coil_motor_steps) {
        winder->winding_state = WINDER_STATE_WINDING_FINISHED;
    } else {
        winder->winding_state = WINDER_STATE_WINDING_INTERRUPTED;
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
           winder->winding_state == WINDER_STATE_WINDING_FINISHED);
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
    winder_init_motor_speed_controller_to_start_from_idle(winder);
    winder_continue_winding(winder);
    return winder->winding_state;
}

void winder_add_extra_turns(struct Winder* winder, uint num_extra_turns) {
    winder->winding_config.number_of_turns += num_extra_turns;
    winder->num_total_required_coil_motor_steps +=
        num_extra_turns *
        winder->hw_config->coil_motor_num_steps_per_revolution;
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
                 int target_delta_from_homing_button_contact_microns,
                 volatile bool* can_continue) {
    jogger->screw_motor_step_pin = screw_motor_step_pin;
    jogger->screw_motor_dir_pin = screw_motor_dir_pin;
    jogger->screw_motor_right_dir_value = screw_motor_right_dir_value;
    jogger->screw_pitch_microns = screw_pitch_microns;
    jogger->steps_per_revolution = steps_per_revolution;

    jogger->homing_button_pin = homing_button_pin;
    jogger->homing_button_press_range_microns =
        homing_button_press_range_microns;
    jogger->target_delta_from_homing_button_contact_microns =
        target_delta_from_homing_button_contact_microns;
    if (jogger->target_delta_from_homing_button_contact_microns < 0 &&
        -jogger->target_delta_from_homing_button_contact_microns >
            homing_button_press_range_microns) {
        jogger->target_delta_from_homing_button_contact_microns =
            -(int)homing_button_press_range_microns;
    }

    jogger->can_continue = can_continue;
    // debounce homing button just in case
    sleep_ms(10);
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

static void jogger_configure_motor_speed_controller_for_right_direction(
    struct Jogger* jogger, struct MotorSpeedController* msc,
    uint target_microns_per_sec) {
    motor_speed_controller_init(
        msc, jogger->steps_per_revolution, 0,
        screw_microns_per_sec_to_mili_revolutions_per_sec(
            target_microns_per_sec, jogger->screw_pitch_microns),
        2000);
    gpio_put(jogger->screw_motor_dir_pin, jogger->screw_motor_right_dir_value);
}

static void jogger_configure_motor_speed_controller_for_left_direction(
    struct Jogger* jogger, struct MotorSpeedController* msc,
    uint target_microns_per_sec) {
    motor_speed_controller_init(
        msc, jogger->steps_per_revolution, 0,
        screw_microns_per_sec_to_mili_revolutions_per_sec(
            target_microns_per_sec, jogger->screw_pitch_microns),
        2000);
    gpio_put(jogger->screw_motor_dir_pin, !jogger->screw_motor_right_dir_value);
}

bool jogger_jog(struct Jogger* jogger) {
    struct MotorSpeedController msc;
    if (jogger->jog_state == JOG_MOVING_LEFT_TOWARDS_PRESSED_BUTTON) {
        jogger_configure_motor_speed_controller_for_left_direction(jogger, &msc,
                                                                   2000);
    } else {
        jogger_configure_motor_speed_controller_for_right_direction(jogger,
                                                                    &msc, 500);
    }

    uint next_step_timestamp =
        motor_speed_controller_get_next_step_timestamp(&msc);
    uint finalizing_stage_remaining_steps = 0;

    // the goal is to be at almost not pressed position (a few steps right will
    // cause state change) then in final calibrations stage move
    // target_delta_from_homing_button_contact_microns

    while (jogger->jog_state != JOG_FINISHED && *jogger->can_continue) {
        uint now_us = to_us_since_boot(get_absolute_time());
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
                            abs(jogger
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
