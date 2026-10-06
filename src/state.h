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
    MACHINE_STATE_WINDING_ERROR,
    MACHINE_STATE_MEASURING_INDUCTANCE,
    MACHINE_STATE_MEASURING_INDUCTANCE_FINISHED
};

enum WindingErrorCode {
    WINDING_ERROR_GUIDE_POSITION_MISMATCH = 1,
    WINDING_ERROR_MOTOR_STALLED
};

struct WindingConfig {
    uint length_microns;
    uint number_of_turns;
    uint wire_diameter_microns;
    uint wire_pitch_microns;
    // adjusted to physical limits of motors, 100 is full "clean" speed 1 is
    // basically stalled, doesn't have to be clipped to 100
    uint speed_percent;
    // guide will initially position at this extra offset from homing button
    // defaults to 1 wire pitch
    uint winding_start_offset_microns;
};

struct InductanceMeterMeasurementConfig {
    bool auto_mode;

    // relevant only if auto_mode = false
    uint num_desired_measurements;
    uint pulse_duration_us;
};

struct InductanceMeasurementResultStats {
    uint num_total_measurements;
    uint num_total_cycles;
    uint avg_cycle_count;
    uint cycle_count_std;
    uint L_nH;
    uint L_nh_std;
    // low -> stable period, high especially common with nonlinear inductors
    uint period_drift_percent;
    // weighted by number of cycles obtained with such duration
    uint avg_pulse_duration;
};

struct InductanceMeasurementResults {
    bool success;
    bool show_filtered_by_default;

    uint total_attempted_cycle_measurements;
    struct InductanceMeasurementResultStats stats_before_filtering;
    struct InductanceMeasurementResultStats stats;
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

void state_switch_to_measuring_inductance(
    struct InductanceMeterMeasurementConfig config);

void state_switch_to_measuring_inductance_finished(void);

void state_disable_stall_detection(void);
