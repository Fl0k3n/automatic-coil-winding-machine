#include "controller.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "inductance_meter.h"
#include "io.h"
#include "persistent_storage.h"
#include "pico/flash.h"
#include "pico/mutex.h"
#include "pico/util/queue.h"
#include "state.h"
#include "winder.h"

#define WINDING_PROGRESS_UPDATE_PERIOD_MS 100
#define MOTOR_LOAD_CHECK_PERIOD_US 20000

enum BackgroundThreadCommand {
    CMD_BEGIN_UPDATING_PROGRESS,
    CMD_STOP_UPDATING_PROGRESS
};

struct ControllerState {
    uint motor_button_pins[4];
    bool hold_motors_while_idle;
    volatile bool stall_detection_enabled;
    struct WindingConfig current_winding_config;
    struct InductanceMeterMeasurementConfig
        current_inductance_measurement_config;

    mutex_t io_mutex;
    // queue[enum BackgroundThreadCommand]
    queue_t background_thread_command_queue;
};

static struct Winder winder;
static struct Jogger jogger;
static struct ControllerConfig cfg;
static struct WinderHardwareConfig winder_hardware_config;
static struct ControllerState controller_state;

static struct InductanceMeter inductance_meter;
static struct InductanceMeterHardwareConfig inductance_meter_hardware_config;

volatile enum MachineState machine_state = MACHINE_STATE_IDLE;
volatile enum MachineState previous_machine_state = MACHINE_STATE_IDLE;
volatile bool can_continue_winding;
volatile bool can_continue_jog;
volatile int guide_speed_increment;

static void switch_state(enum MachineState new_state) {
    previous_machine_state = machine_state;
    machine_state = new_state;
}

static void switch_to_previous_state(void) {
    assert(machine_state == MACHINE_STATE_SERVE_ASYNC_COMMAND);
    switch_state(previous_machine_state);
}

void state_switch_to_serve_async_command(void) {
    if (machine_state != MACHINE_STATE_IDLE &&
        machine_state != MACHINE_STATE_SERVE_ASYNC_COMMAND &&
        machine_state != MACHINE_STATE_WINDING_PAUSED) {
        return;
    }
    switch_state(MACHINE_STATE_SERVE_ASYNC_COMMAND);
}

void state_switch_to_winding(struct WindingConfig config) {
    controller_state.current_winding_config = config;
    switch_state(MACHINE_STATE_WINDING);
}

void state_switch_to_winding_paused(void) {
    switch_state(MACHINE_STATE_WINDING_PAUSED);
}

void state_switch_to_winding_finished(void) {
    switch_state(MACHINE_STATE_WIDING_FINISHED);
}
void state_switch_to_jog_finished(void) {
    switch_state(MACHINE_STATE_JOG_FINISHED);
}

void state_switch_to_jog(void) { switch_state(MACHINE_STATE_JOG); }

void state_switch_to_idle(void) { switch_state(MACHINE_STATE_IDLE); }

void state_switch_to_winding_error(void) {
    switch_state(MACHINE_STATE_WINDING_ERROR);
}

void state_switch_to_winding_resumed(void) {
    switch_state(MACHINE_STATE_WINDING_RESUMED);
}

void state_toggle_hold_motors_while_idle(void) {
    controller_state.hold_motors_while_idle =
        !controller_state.hold_motors_while_idle;
}

void state_switch_to_continue_winding_after_finished(uint num_extra_turns) {
    winder_add_extra_turns(&winder, num_extra_turns);
    controller_state.current_winding_config.number_of_turns += num_extra_turns;
    switch_state(MACHINE_STATE_WINDING_RESUMED);
}

void state_switch_to_measuring_inductance(
    struct InductanceMeterMeasurementConfig config) {
    controller_state.current_inductance_measurement_config = config;
    switch_state(MACHINE_STATE_MEASURING_INDUCTANCE);
}

void state_switch_to_measuring_inductance_finished(void) {
    switch_state(MACHINE_STATE_MEASURING_INDUCTANCE_FINISHED);
}

void state_disable_stall_detection(void) {
    controller_state.stall_detection_enabled = false;
}

void controller_init(struct ControllerConfig config) {
    cfg = config;

    winder_hardware_config = (struct WinderHardwareConfig){
        .coil_motor_dir_pin = config.coil_motor_dir_pin,
        .coil_motor_step_pin = config.coil_motor_step_pin,
        .coil_motor_num_steps_per_revolution =
            config.full_rotation_num_steps_coil_motor,
        .coil_motor_wind_dir_value = config.coil_motor_wind_dir_value,
        .screw_motor_dir_pin = config.screw_motor_dir_pin,
        .screw_motor_step_pin = config.screw_motor_step_pin,
        .screw_motor_num_steps_per_revolution =
            config.full_rotation_num_steps_screw_motor,
        .screw_motor_right_dir_value = config.screw_motor_right_dir_value,
        .homing_button_pin = config.homing_button_pin,
        .homing_button_press_range_microns =
            config.homing_button_press_range_microns,
        .min_time_between_screw_motor_steps_us =
            config.min_time_between_screw_motor_steps_us,
        .min_time_between_coil_motor_steps_us =
            config.min_time_between_coil_motor_steps_us,
        .screw_pitch_microns = config.screw_pitch_microns};

    inductance_meter_hardware_config = (struct InductanceMeterHardwareConfig){
        .LC_tank_capacitance = config.L_meter_LC_tank_capacitance,
        .LC_tank_parasitic_series_inductance =
            config.L_meter_LC_tank_parasitic_series_inductance,
        .comparator_output_pin = config.L_meter_comparator_output_pin,
        .power_switch_pin = config.L_meter_power_switch_pin,
        .power_on_value = config.L_meter_power_on_value,
        .freq_meter_pio = config.L_meter_pio,
        .freq_meter_pio_sm = config.L_meter_pio_sm,
        .freq_meter_pio_program_offset = config.L_meter_pio_program_offset,
    };

    controller_state.hold_motors_while_idle = false;
    controller_state.stall_detection_enabled = true;

    controller_state.motor_button_pins[0] = cfg.coil_motor_wind_button_pin;
    controller_state.motor_button_pins[1] = cfg.coil_motor_unwind_button_pin;
    controller_state.motor_button_pins[2] = cfg.screw_motor_left_button_pin;
    controller_state.motor_button_pins[3] = cfg.screw_motor_righ_button_pin;

    mutex_init(&controller_state.io_mutex);
    queue_init(&controller_state.background_thread_command_queue,
               sizeof(enum BackgroundThreadCommand), 1);
    mutex_enter_blocking(&controller_state.io_mutex);
}

static bool check_for_async_motor_commands(void) {
    for (size_t i = 0; i < sizeof(controller_state.motor_button_pins) /
                               sizeof(controller_state.motor_button_pins[0]);
         i++) {
        if (!gpio_get(controller_state.motor_button_pins[i])) {
            return true;
        }
    }
    return false;
}

static void motor_step(uint pin) {
    gpio_put(pin, 1);
    sleep_us(1);
    gpio_put(pin, 0);
}

static void motor_set_dir(uint pin, bool dir) {
    gpio_put(pin, dir);
    sleep_us(5);
}

static bool serve_async_command(void) {
    static uint64_t last_coil_motor_move_us = 0;
    static uint64_t last_screw_motor_move_us = 0;

    bool move_coil_motor = false;
    bool coil_motor_dir_wind = false;
    bool move_screw_motor = false;
    bool screw_motor_dir_right = false;

    bool coil_wind = !gpio_get(cfg.coil_motor_wind_button_pin);
    bool coil_unwind = !gpio_get(cfg.coil_motor_unwind_button_pin);
    if (coil_wind ^ coil_unwind) {
        move_coil_motor = true;
        coil_motor_dir_wind = coil_wind;
    }

    bool screw_right = !gpio_get(cfg.screw_motor_righ_button_pin);
    bool screw_left = !gpio_get(cfg.screw_motor_left_button_pin);
    if (screw_right ^ screw_left) {
        move_screw_motor = true;
        screw_motor_dir_right = screw_right;
    }

    if (!move_coil_motor && !move_screw_motor) {
        return true;
    }

    uint64_t now_us = to_us_since_boot(get_absolute_time());
    if (move_coil_motor && now_us - last_coil_motor_move_us >
                               cfg.async_coil_motor_step_interval_us) {
        last_coil_motor_move_us = now_us;
        motor_set_dir(cfg.coil_motor_dir_pin,
                      coil_motor_dir_wind ? cfg.coil_motor_wind_dir_value
                                          : !cfg.coil_motor_wind_dir_value);
        motor_step(cfg.coil_motor_step_pin);
    }

    if (move_screw_motor && now_us - last_screw_motor_move_us >
                                cfg.async_screw_motor_step_interval_us) {
        last_screw_motor_move_us = now_us;
        motor_set_dir(cfg.screw_motor_dir_pin,
                      screw_motor_dir_right ? cfg.screw_motor_right_dir_value
                                            : !cfg.screw_motor_right_dir_value);
        motor_step(cfg.screw_motor_step_pin);
    }
    return false;
}

void controller_handle_rotary_encoder_button_pressed_irq(void) {
    if (machine_state == MACHINE_STATE_WINDING ||
        machine_state == MACHINE_STATE_WINDING_RESUMED) {
        can_continue_winding = false;
    } else if (machine_state == MACHINE_STATE_JOG) {
        can_continue_jog = false;
    }
}

void controller_handle_rotary_encoder_delta_changed_irq(int incremental_delta) {
    if (machine_state == MACHINE_STATE_WINDING ||
        machine_state == MACHINE_STATE_WINDING_RESUMED) {
        winder.incremental_guide_speed_delta += incremental_delta;
    }
}

void controller_handle_homing_button_pressed_irq(void) {
    // gpio_put(cfg.paused_or_error_led_pin, true);
}

void controller_handle_homing_button_released_irq(void) {
    // gpio_put(cfg.paused_or_error_led_pin, false);
}

static inline void update_leds(void) {
    bool winding_or_movement =
        machine_state == MACHINE_STATE_WINDING ||
        machine_state == MACHINE_STATE_WINDING_RESUMED ||
        machine_state == MACHINE_STATE_SERVE_ASYNC_COMMAND ||
        machine_state == MACHINE_STATE_JOG ||
        controller_state.hold_motors_while_idle;
    gpio_put(cfg.active_indicator_led_pin, winding_or_movement);
    bool error_or_paused = machine_state == MACHINE_STATE_WINDING_ERROR ||
                           machine_state == MACHINE_STATE_WINDING_PAUSED;
    gpio_put(cfg.paused_or_error_led_pin, error_or_paused);
}

static void set_motors_enabled(bool enabled) {
    gpio_put(cfg.coil_motor_enable_pin, !enabled);
    gpio_put(cfg.screw_motor_enable_pin, !enabled);
}

static void handle_winding_result(
    enum WinderWindingState winding_result_state) {
    if (winding_result_state == WINDER_STATE_WINDING_FINISHED) {
        state_switch_to_winding_finished();
        menu_show_winding_finished_screen();
    } else if (winding_result_state == WINDER_STATE_JOG_INTERRUPTED) {
        state_switch_to_idle();
    } else if (winding_result_state == WINDER_STATE_WINDING_INTERRUPTED) {
        menu_show_winding_paused_screen();
        state_switch_to_winding_paused();
    } else {
        menu_show_winding_error_screen(winder.error_code);
        state_switch_to_winding_error();
    }
}

static void handover_menu_control_to_core1(void) {
    mutex_exit(&controller_state.io_mutex);
    enum BackgroundThreadCommand cmd = CMD_BEGIN_UPDATING_PROGRESS;
    queue_add_blocking(&controller_state.background_thread_command_queue, &cmd);
}

static void take_menu_control_from_core1(void) {
    enum BackgroundThreadCommand cmd = CMD_STOP_UPDATING_PROGRESS;
    queue_add_blocking(&controller_state.background_thread_command_queue, &cmd);
    mutex_enter_blocking(&controller_state.io_mutex);
}

void controller_run(void) {
    for (;;) {
        update_leds();

        if (machine_state == MACHINE_STATE_IDLE) {
            set_motors_enabled(controller_state.hold_motors_while_idle);
            menu_next_frame();
            if (check_for_async_motor_commands()) {
                state_switch_to_serve_async_command();
            }
        } else if (machine_state == MACHINE_STATE_SERVE_ASYNC_COMMAND) {
            set_motors_enabled(true);
            bool done = serve_async_command();
            if (done) {
                switch_to_previous_state();
            }
        } else if (machine_state == MACHINE_STATE_JOG) {
            set_motors_enabled(true);
            can_continue_jog = true;
            jogger_init(
                &jogger, cfg.screw_motor_dir_pin, cfg.screw_motor_step_pin,
                cfg.screw_motor_right_dir_value, cfg.screw_pitch_microns,
                cfg.full_rotation_num_steps_screw_motor, cfg.homing_button_pin,
                cfg.homing_button_press_range_microns,
                cfg.min_time_between_screw_motor_steps_us, 0,
                &can_continue_jog);
            bool jog_finished = jogger_jog(&jogger);
            if (jog_finished) {
                state_switch_to_jog_finished();
            } else {
                menu_jog_paused();
                state_switch_to_idle();
            }
        } else if (machine_state == MACHINE_STATE_JOG_FINISHED) {
            menu_show_jog_finished_and_wait_for_confirm();
        } else if (machine_state == MACHINE_STATE_WINDING) {
            set_motors_enabled(true);
            can_continue_winding = true;
            winder_init(&winder, &winder_hardware_config,
                        controller_state.current_winding_config,
                        &can_continue_winding);
            handover_menu_control_to_core1();
            enum WinderWindingState winding_result_state = winder_wind(&winder);
            take_menu_control_from_core1();
            handle_winding_result(winding_result_state);
        } else if (machine_state == MACHINE_STATE_WINDING_PAUSED ||
                   machine_state == MACHINE_STATE_WINDING_ERROR) {
            set_motors_enabled(true);  // keep wire tensioned
            menu_next_frame();
            if (check_for_async_motor_commands()) {
                state_switch_to_serve_async_command();
            }
        } else if (machine_state == MACHINE_STATE_WINDING_RESUMED) {
            set_motors_enabled(true);
            can_continue_winding = true;
            handover_menu_control_to_core1();
            enum WinderWindingState winding_result_state =
                winder_resume(&winder);
            take_menu_control_from_core1();
            handle_winding_result(winding_result_state);
        } else if (machine_state == MACHINE_STATE_WIDING_FINISHED) {
            set_motors_enabled(true);
            menu_next_frame();
        } else if (machine_state == MACHINE_STATE_MEASURING_INDUCTANCE) {
            set_motors_enabled(false);
            inductance_meter_init(
                &inductance_meter, &inductance_meter_hardware_config,
                controller_state.current_inductance_measurement_config);
            struct InductanceMeasurementResults results =
                inductance_meter_measure(&inductance_meter);
            menu_show_inductance_measurement_finished_screen(results);
            state_switch_to_measuring_inductance_finished();
        } else if (machine_state ==
                   MACHINE_STATE_MEASURING_INDUCTANCE_FINISHED) {
            menu_next_frame();
        }
    }
}

// ~3s with checks at 20ms period
// stall detection assumes that stall right after winding start is unlikely, and
// if it occurs, it will be noticed by user who will still be near the machine
// so real stall detection kicks off after about 5s
#define STALL_DETECTION_NUM_CALIBRATION_MEASUREMENTS 150
#define STALL_DETECTION_CALIBRATION_WAIT_TIME_AFTER_ACCELERATION_FINISHED_US \
    500 * 1000
#define STALL_DETECTION_RUNNING_MEASUREMENTS_SIZE 5

struct StallDetectionState {
    uint64_t last_load_check_timestamp_us;

    bool calibrating;
    size_t num_done_calibration_measurements;
    uint16_t
        calibration_measurements[STALL_DETECTION_NUM_CALIBRATION_MEASUREMENTS];

    // these values can't really be hardcoded since they depend on a ton of
    // factors, including winding speed, wire diameter, spool friction...
    float stall_threshold_coeff;
    bool callibrated;
    uint16_t median_motor_load;
    uint16_t stall_threshold;
    uint16_t running_measurements[STALL_DETECTION_RUNNING_MEASUREMENTS_SIZE];

    uint16_t current_running_median;
    bool stall_requested;
};

static struct StallDetectionState stall_detection_state;

static void reset_stall_detection_state(void) {
    stall_detection_state.last_load_check_timestamp_us = 0;
    stall_detection_state.calibrating = false;
    stall_detection_state.callibrated = false;
    stall_detection_state.stall_requested = false;
    stall_detection_state.num_done_calibration_measurements = 0;
    stall_detection_state.current_running_median = 0;
    struct StoredSettings settings;
    persistent_storage_get_stored_settings_or_defaults(&settings);
    stall_detection_state.stall_threshold_coeff =
        (float)settings.stall_threshold_percent / 100.0f;
}

static int cmp_u16(const void* a, const void* b) {
    uint16_t x = *(const uint16_t*)a;
    uint16_t y = *(const uint16_t*)b;
    return (x > y) - (x < y);
}

static uint16_t stall_detection_state_put_load_measurement_and_get_median(
    uint16_t motor_load) {
    uint16_t copied_measurements[STALL_DETECTION_RUNNING_MEASUREMENTS_SIZE];
    for (size_t i = 0; i < STALL_DETECTION_RUNNING_MEASUREMENTS_SIZE - 1; i++) {
        stall_detection_state.running_measurements[i] =
            stall_detection_state.running_measurements[i + 1];
    }
    stall_detection_state
        .running_measurements[STALL_DETECTION_RUNNING_MEASUREMENTS_SIZE - 1] =
        motor_load;

    memcpy(copied_measurements, stall_detection_state.running_measurements,
           sizeof(copied_measurements[0]) *
               STALL_DETECTION_RUNNING_MEASUREMENTS_SIZE);
    qsort(copied_measurements, STALL_DETECTION_RUNNING_MEASUREMENTS_SIZE,
          sizeof(copied_measurements[0]), cmp_u16);
    return copied_measurements[STALL_DETECTION_RUNNING_MEASUREMENTS_SIZE / 2];
}

static void core1_maybe_run_stall_check(void) {
    uint64_t now_us = to_us_since_boot(get_absolute_time());
    if (!controller_state.stall_detection_enabled ||
        stall_detection_state.stall_requested ||
        now_us - stall_detection_state.last_load_check_timestamp_us <
            MOTOR_LOAD_CHECK_PERIOD_US) {
        return;
    }
    uint64_t acceleration_end_timestamp_us =
        winder_get_acceleration_end_timestamp_us(&winder);
    if (acceleration_end_timestamp_us == 0 ||
        now_us <
            acceleration_end_timestamp_us +
                STALL_DETECTION_CALIBRATION_WAIT_TIME_AFTER_ACCELERATION_FINISHED_US) {
        return;
    }
    if (!stall_detection_state.callibrated &&
        !stall_detection_state.calibrating) {
        stall_detection_state.calibrating = true;
    }

    uint16_t motor_load;
    bool ok = cfg.blocking_read_coil_motor_load(&motor_load);
    if (!ok) {
        return;
    }
    if (stall_detection_state.calibrating) {
        stall_detection_state.calibration_measurements
            [stall_detection_state.num_done_calibration_measurements] =
            motor_load;
        stall_detection_state.num_done_calibration_measurements++;
        if (stall_detection_state.num_done_calibration_measurements ==
            STALL_DETECTION_NUM_CALIBRATION_MEASUREMENTS) {
            stall_detection_state.calibrating = false;
            stall_detection_state.callibrated = true;

            for (size_t i = 0; i < STALL_DETECTION_RUNNING_MEASUREMENTS_SIZE;
                 i++) {
                stall_detection_state.running_measurements[i] =
                    stall_detection_state.calibration_measurements
                        [STALL_DETECTION_NUM_CALIBRATION_MEASUREMENTS -
                         STALL_DETECTION_RUNNING_MEASUREMENTS_SIZE - 1 - i];
            }
            qsort(stall_detection_state.calibration_measurements,
                  STALL_DETECTION_NUM_CALIBRATION_MEASUREMENTS,
                  sizeof(stall_detection_state.calibration_measurements[0]),
                  cmp_u16);
            stall_detection_state.median_motor_load =
                stall_detection_state.calibration_measurements
                    [STALL_DETECTION_NUM_CALIBRATION_MEASUREMENTS / 2];

            stall_detection_state.stall_threshold =
                (uint16_t)(stall_detection_state.median_motor_load *
                           stall_detection_state.stall_threshold_coeff);
        }
        return;
    }

    // calibrated and motor load valid
    uint16_t running_median =
        stall_detection_state_put_load_measurement_and_get_median(motor_load);
    if (running_median < stall_detection_state.stall_threshold) {
        winder_stop_with_error(&winder, WINDING_ERROR_MOTOR_STALLED);
        stall_detection_state.stall_requested = true;
    }
    stall_detection_state.current_running_median = running_median;
}

void controller_core1_main(void) {
    bool updating_progress = false;
    uint last_progress_update_ms = 0;

    // this has some side effects concerning hardware fifos, could be
    // worked-around if needed, e.g. by defering core1 init and allowing flash
    // changes only before it was initialized. With the current code state it
    // doesn't matter
    bool flash_safe_execute_init_ok = flash_safe_execute_core_init();
    hard_assert(flash_safe_execute_init_ok);

    while (true) {
        enum BackgroundThreadCommand cmd;
        uint now_ms = to_ms_since_boot(get_absolute_time());

        if (updating_progress) {
            bool got_command = queue_try_remove(
                &controller_state.background_thread_command_queue, &cmd);
            if (got_command && cmd == CMD_STOP_UPDATING_PROGRESS) {
                updating_progress = false;
                mutex_exit(&controller_state.io_mutex);
                continue;
            }

            core1_maybe_run_stall_check();

            if (now_ms - last_progress_update_ms >=
                WINDING_PROGRESS_UPDATE_PERIOD_MS) {
                last_progress_update_ms = now_ms;

                int guide_position_microns =
                    winder_get_guide_position_microns(&winder);
                uint num_current_turns = winder_get_num_current_turns(&winder);
                // just to be sure this doesn't get stale without making that
                // explicitly volatile in config struct
                uint total_turns = *(volatile uint*)&controller_state
                                        .current_winding_config.number_of_turns;
                menu_show_winding_progress(
                    guide_position_microns, num_current_turns, total_turns,
                    winder.total_acked_guide_speed_delta);
                // debug for stall detection
                // menu_show_winding_progress(
                //     0, stall_detection_state.stall_threshold,
                //     stall_detection_state.current_running_median, 0);
            }
        } else {
            queue_remove_blocking(
                &controller_state.background_thread_command_queue, &cmd);
            if (cmd == CMD_BEGIN_UPDATING_PROGRESS) {
                mutex_enter_blocking(&controller_state.io_mutex);
                reset_stall_detection_state();
                updating_progress = true;
            }
        }
    }
}
