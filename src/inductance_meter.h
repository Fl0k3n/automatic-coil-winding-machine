#pragma once
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "state.h"

struct InductanceMeterHardwareConfig {
    double LC_tank_capacitance;
    double LC_tank_parasitic_series_inductance;  // parasitic C should be
                                                 // included above
    uint comparator_output_pin;
    uint power_switch_pin;
    bool power_on_value;
    pio_hw_t* freq_meter_pio;
    uint freq_meter_pio_sm;
    uint freq_meter_pio_program_offset;
};

#define MAX_TRACKED_LC_RINGING_CYCLES 16
#define MAX_INDUNCATANCE_SUB_MEASUREMENTS 16

struct InductanceSubMeasurementResult {
    uint pulse_duration_us;
    uint num_initially_failed;
    uint num_recorded_ringing_cycles;
    uint32_t cycle_counts[MAX_TRACKED_LC_RINGING_CYCLES];
    bool keep_for_final_computation;
};

struct SubMeasurementStats {
    uint num_points;
    uint avg_cycle_count;
    uint cycle_count_std;
};

struct InductanceMeter {
    const struct InductanceMeterHardwareConfig* hw_config;
    struct InductanceMeterMeasurementConfig measurement_config;

    double L_nh_from_cycle_count_square_coeff;
    uint parasitic_series_L_nH;

    uint last_sub_measurement_timestamp_ms;
    uint num_sub_measurements;
    struct InductanceSubMeasurementResult
        sub_measurement_results[MAX_INDUNCATANCE_SUB_MEASUREMENTS];
    struct SubMeasurementStats
        sub_measurement_stats[MAX_INDUNCATANCE_SUB_MEASUREMENTS];
};

void inductance_meter_init(
    struct InductanceMeter* meter,
    const struct InductanceMeterHardwareConfig* hw_config,
    struct InductanceMeterMeasurementConfig measurement_config);

struct InductanceMeasurementResults inductance_meter_measure(
    struct InductanceMeter* meter);
