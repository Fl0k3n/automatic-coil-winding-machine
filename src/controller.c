#include "controller.h"

#include "io.h"
#include "pico/mutex.h"
#include "pico/util/queue.h"
#include "state.h"
#include "winder.h"

#define WINDING_PROGRESS_UPDATE_PERIOD_MS 100

enum BackgroundThreadCommand {
    CMD_BEGIN_UPDATING_PROGRESS,
    CMD_STOP_UPDATING_PROGRESS
};

struct ControllerState {
    uint motor_button_pins[4];
    bool hold_motors_while_idle;
    struct WindingConfig current_winding_config;

    mutex_t io_mutex;
    // queue[enum BackgroundThreadCommand]
    queue_t background_thread_command_queue;
};

static struct Winder winder;
static struct ControllerConfig cfg;
static struct WinderHardwareConfig winder_hardware_config;
static struct ControllerState controller_state;

volatile enum MachineState machine_state = MACHINE_STATE_IDLE;
volatile enum MachineState previous_machine_state = MACHINE_STATE_IDLE;
volatile bool can_continue_winding;
volatile bool can_continue_jog;

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
        .screw_pitch_microns = config.screw_pitch_microns};

    controller_state.hold_motors_while_idle = false;

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
            struct Jogger jogger;
            can_continue_jog = true;
            jogger_init(
                &jogger, cfg.screw_motor_dir_pin, cfg.screw_motor_step_pin,
                cfg.screw_motor_right_dir_value, cfg.screw_pitch_microns,
                cfg.full_rotation_num_steps_screw_motor, cfg.homing_button_pin,
                cfg.homing_button_press_range_microns, 0, &can_continue_jog);
            bool jog_finished = jogger_jog(&jogger);
            if (jog_finished == JOG_FINISHED) {
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
        } else if (machine_state == MACHINE_STATE_WINDING_PAUSED) {
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
        }
    }
}

void controller_core1_main(void) {
    bool updating_progress = false;
    uint last_progress_update_ms = 0;

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
                menu_show_winding_progress(guide_position_microns,
                                           num_current_turns, total_turns);
            }
        } else {
            queue_remove_blocking(
                &controller_state.background_thread_command_queue, &cmd);
            if (cmd == CMD_BEGIN_UPDATING_PROGRESS) {
                mutex_enter_blocking(&controller_state.io_mutex);
                updating_progress = true;
            }
        }
    }
}
