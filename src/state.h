#pragma once
#include "pico/stdlib.h"

enum MachineState {
    MACHINE_STATE_IDLE,
    MACHINE_STATE_SERVE_ASYNC_COMMAND,
    MACHINE_STATE_JOG,
    MACHINE_STATE_JOG_FINISHED,
    MACHINE_STATE_WINDING,
    MACHINE_STATE_WINDING_PAUSED,
    MACHINE_STATE_WINDING_RESUMED,
    MACHINE_STATE_WIDING_FINISHED,
    MACHINE_STATE_WINDING_ERROR
};

struct WindingConfig {
    uint length_microns;
    uint number_of_turns;
    uint wire_diameter_microns;
    uint wire_pitch_microns;
    // adjusted to physical limits of motors, 100 is full speed 1 is basically
    // stalled
    uint speed_percent;
    // guide will initially position at this extra offset from homing button
    // defaults to 1 wire pitch
    uint winding_start_offset_microns;
};

enum AsyncCommand {
    ASYNC_COMMAND_COIL_MOTOR_WIND = 1,
    ASYNC_COMMAND_COIL_MOTOR_UNWIND = 2,
    ASYNC_COMMAND_SCREW_MOTOR_LEFT = 4,
    ASYNC_COMMAND_SCREW_MOTOR_RIGHT = 8,
};

extern volatile enum MachineState machine_state;

void state_switch_to_serve_async_command(void);

void state_switch_to_winding(struct WindingConfig config);

void state_switch_to_winding_paused(void);

void state_switch_to_winding_finished(void);

void state_switch_to_continue_winding_after_finished(uint num_extra_turns);

void state_switch_to_idle(void);

void state_switch_to_jog_finished(void);

void state_switch_to_jog(void);

void state_switch_to_winding_error(void);

void state_switch_to_winding_resumed(void);

void state_toggle_hold_motors_while_idle(void);
