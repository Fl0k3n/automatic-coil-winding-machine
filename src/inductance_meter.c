#include "inductance_meter.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "freq_meter.pio.h"
#include "hardware/sync.h"
#include "hardware/timer.h"

// pmos takes about 2us to properly turn off, with short pulses LC tank will be
// ringing already after the starting pulse, also counting initialization takes
// a bit so delay should be picked experimentally
#define MEASUREMENT_DELAY_AFTER_POWER_SWITCH_OFF_US 1

// ringing period ~= 2pi * sqrt(LC), with C = 100nF this gives ~2us for 1uH and
// ~600us for 100mH, 1000 is more than enough
#define MAX_CYCLE_WAITING_TIME_US 1000

#define MIN_CYCLE_COUNT_TOLERACE 10
#define MIN_DEAD_TIME_MS_BETWEEN_SUBMEASUREMENTS 5

#define M_PI 3.14159265358979323846

static inline uint nanos_to_cycles(uint64_t nanos) {
    uint64_t sys_freq = (uint64_t)clock_get_hz(clk_sys);
    return (uint)((nanos * sys_freq) / 1000000000ull);
}

static inline uint uint_abs_diff(uint a, uint b) {
    return a > b ? a - b : b - a;
}

void inductance_meter_init(
    struct InductanceMeter* meter,
    const struct InductanceMeterHardwareConfig* hw_config,
    struct InductanceMeterMeasurementConfig measurement_config) {
    meter->hw_config = hw_config;
    meter->measurement_config = measurement_config;
    meter->num_sub_measurements = 0;
    meter->last_sub_measurement_timestamp_ms = 0;
    meter->parasitic_series_L_nH =
        (uint)(hw_config->LC_tank_parasitic_series_inductance * 1e9);
    uint64_t sys_freq = (uint64_t)clock_get_hz(clk_sys);
    uint64_t sys_freq_squared = sys_freq * sys_freq;
    // n - cycle count -> T = n*f0
    // L = T^2 / (4 pi^2 * C) = n^2 / (4 pi^2 * C * f0^2)
    // in nano henries L = n^2 * 1e9 / (4pi^2*C*f0^2) = n^2 * const
    meter->L_nh_from_cycle_count_square_coeff =
        1e9 / (4. * M_PI * M_PI * hw_config->LC_tank_capacitance *
               (double)sys_freq_squared);
}

static void run_sub_measurement(struct InductanceMeter* meter,
                                struct InductanceSubMeasurementResult* result,
                                uint pulse_duration_us,
                                uint cycle_waiting_timeout_us,
                                uint max_cycles_to_measure,
                                uint max_allowed_start_failures) {
    result->num_recorded_ringing_cycles = 0;
    result->num_initially_failed = 0;
    result->pulse_duration_us = pulse_duration_us;
    result->keep_for_final_computation = true;

    uint now_ms = to_ms_since_boot(get_absolute_time());
    if (now_ms - meter->last_sub_measurement_timestamp_ms <
        MIN_DEAD_TIME_MS_BETWEEN_SUBMEASUREMENTS) {
        sleep_ms(now_ms - meter->last_sub_measurement_timestamp_ms);
    }

    // inductance meter runs when rest of the machine is effectively expected to
    // halt, so masking irqs is allowed here
    uint32_t irq_state = save_and_disable_interrupts();

    gpio_put(meter->hw_config->power_switch_pin,
             meter->hw_config->power_on_value);
    busy_wait_us_32(pulse_duration_us);
    gpio_put(meter->hw_config->power_switch_pin,
             !meter->hw_config->power_on_value);

    busy_wait_us_32(MEASUREMENT_DELAY_AFTER_POWER_SWITCH_OFF_US);

    bool first_cycle = true;
    freq_meter_program_reset(meter->hw_config->freq_meter_pio,
                             meter->hw_config->freq_meter_pio_sm,
                             meter->hw_config->freq_meter_pio_program_offset,
                             meter->hw_config->comparator_output_pin);

    restore_interrupts_from_disabled(irq_state);

    for (uint i = 0; i < max_cycles_to_measure;) {
        bool success = freq_meter_get_cycle_count_timeout(
            meter->hw_config->freq_meter_pio,
            meter->hw_config->freq_meter_pio_sm, cycle_waiting_timeout_us,
            &result->cycle_counts[i], first_cycle);
        first_cycle = false;
        if (success) {
            result->num_recorded_ringing_cycles++;
            i++;
        } else {
            if (i > 0) {
                break;
            } else {
                result->num_initially_failed++;
                if (result->num_initially_failed > max_allowed_start_failures) {
                    break;
                }
            }
        }
    }

    meter->last_sub_measurement_timestamp_ms =
        to_ms_since_boot(get_absolute_time());
}

static void filter_cycle_counts(
    struct InductanceSubMeasurementResult* sub_measurement_result) {
    if (sub_measurement_result->num_recorded_ringing_cycles <= 2) {
        return;
    }
    bool keep_measurement[MAX_TRACKED_LC_RINGING_CYCLES];
    for (size_t i = 0; i < sub_measurement_result->num_recorded_ringing_cycles;
         i++) {
        keep_measurement[i] = true;
    }
    if (sub_measurement_result->num_recorded_ringing_cycles == 3) {
        // first is the least certain, discard it if it's different than next 2
        int second_to_third_delta =
            (int)sub_measurement_result->cycle_counts[2] -
            (int)sub_measurement_result->cycle_counts[1];
        int first_to_second_delta =
            (int)sub_measurement_result->cycle_counts[1] -
            (int)sub_measurement_result->cycle_counts[0];
        if (abs(first_to_second_delta) >=
                MAX(abs(second_to_third_delta) * 2, MIN_CYCLE_COUNT_TOLERACE) ||
            abs(first_to_second_delta) > nanos_to_cycles(500)) {
            keep_measurement[0] = false;
        }
    } else {
        // discard values far from the most likely value
        size_t start = 0;
        if (sub_measurement_result->num_recorded_ringing_cycles > 5) {
            // we have enough to always discard first
            keep_measurement[0] = false;
            start++;
        }
        size_t centriod_idx = start;
        uint best_total_delta = UINT32_MAX;
        for (size_t i = start;
             i < sub_measurement_result->num_recorded_ringing_cycles; i++) {
            uint total_delta = 0;
            for (size_t j = start;
                 j < sub_measurement_result->num_recorded_ringing_cycles; j++) {
                total_delta +=
                    uint_abs_diff(sub_measurement_result->cycle_counts[j],
                                  sub_measurement_result->cycle_counts[i]);
            }
            if (total_delta < best_total_delta) {
                centriod_idx = i;
                best_total_delta = total_delta;
            }
        }
        uint centroid_cycle_count =
            sub_measurement_result->cycle_counts[centriod_idx];
        uint closest_delta = UINT32_MAX;
        for (size_t i = start;
             i < sub_measurement_result->num_recorded_ringing_cycles; i++) {
            if (i == centriod_idx) continue;
            uint delta = uint_abs_diff(sub_measurement_result->cycle_counts[i],
                                       centroid_cycle_count);
            if (delta < closest_delta) {
                closest_delta = delta;
            }
        }

        uint delta_tolerance =
            MAX(MAX(3 * closest_delta / 2, centroid_cycle_count / 10), 10);
        for (size_t i = start;
             i < sub_measurement_result->num_recorded_ringing_cycles; i++) {
            uint delta = uint_abs_diff(sub_measurement_result->cycle_counts[i],
                                       centroid_cycle_count);
            if (delta > delta_tolerance) {
                keep_measurement[i] = false;
            }
        }
    }
    size_t kept_value_idx = 0;
    for (size_t i = 0; i < sub_measurement_result->num_recorded_ringing_cycles;
         i++) {
        if (keep_measurement[i]) {
            sub_measurement_result->cycle_counts[kept_value_idx++] =
                sub_measurement_result->cycle_counts[i];
        }
    }
    sub_measurement_result->num_recorded_ringing_cycles = kept_value_idx;
}

static void compute_sub_measurement_stats(struct InductanceMeter* meter) {
    for (size_t i = 0; i < meter->num_sub_measurements; i++) {
        struct InductanceSubMeasurementResult* sub_measurement =
            &meter->sub_measurement_results[i];
        if (sub_measurement->num_recorded_ringing_cycles == 0 ||
            !sub_measurement->keep_for_final_computation) {
            sub_measurement->keep_for_final_computation = false;
            continue;
        }
        struct SubMeasurementStats* stats = &meter->sub_measurement_stats[i];
        stats->num_points = sub_measurement->num_recorded_ringing_cycles;
        if (sub_measurement->num_recorded_ringing_cycles == 1) {
            stats->avg_cycle_count = sub_measurement->cycle_counts[0];
            stats->cycle_count_std = 0;
        } else {
            assert(sub_measurement->num_recorded_ringing_cycles > 0);
            uint total_count = 0;
            for (size_t j = 0; j < sub_measurement->num_recorded_ringing_cycles;
                 j++) {
                total_count += sub_measurement->cycle_counts[j];
            }
            stats->avg_cycle_count =
                total_count / sub_measurement->num_recorded_ringing_cycles;

            uint total_diff_squared = 0;
            for (size_t j = 0; j < sub_measurement->num_recorded_ringing_cycles;
                 j++) {
                uint diff = uint_abs_diff(sub_measurement->cycle_counts[j],
                                          stats->avg_cycle_count);
                total_diff_squared += diff * diff;
            }
            stats->cycle_count_std = (uint)sqrtf(
                (float)total_diff_squared /
                (float)(sub_measurement->num_recorded_ringing_cycles - 1));
        }
    }
}

static void filter_sub_measurements(struct InductanceMeter* meter) {
    // similar strategy as for filtering in individual sub measurements, but
    // here mostly accounting for averaged counts
    size_t initially_valid_measurements = 0;
    size_t valid_result_idxs[MAX_INDUNCATANCE_SUB_MEASUREMENTS];
    for (size_t i = 0; i < meter->num_sub_measurements; i++) {
        if (meter->sub_measurement_results[i].num_recorded_ringing_cycles ==
            0) {
            meter->sub_measurement_results[i].keep_for_final_computation =
                false;
        }
        if (meter->sub_measurement_results[i].keep_for_final_computation) {
            valid_result_idxs[initially_valid_measurements++] = i;
        }
    }
    if (initially_valid_measurements <= 1) {
        return;
    }
    if (initially_valid_measurements == 2) {
        bool first_has_ok_num_cycles =
            meter->sub_measurement_results[valid_result_idxs[0]]
                .num_recorded_ringing_cycles >= 3;
        bool second_has_ok_num_cycles =
            meter->sub_measurement_results[valid_result_idxs[1]]
                .num_recorded_ringing_cycles >= 3;
        if ((first_has_ok_num_cycles && second_has_ok_num_cycles) ||
            (!first_has_ok_num_cycles && !second_has_ok_num_cycles)) {
            return;
        }
        // at least one has >= 3, reject the lower-count one if it's far from
        // the better one
        size_t better_one = valid_result_idxs[0],
               worse_one = valid_result_idxs[1];
        if (second_has_ok_num_cycles) {
            better_one = valid_result_idxs[1];
            worse_one = valid_result_idxs[0];
        }
        if (uint_abs_diff(
                meter->sub_measurement_stats[better_one].avg_cycle_count,
                meter->sub_measurement_stats[worse_one].avg_cycle_count) >
            meter->sub_measurement_stats[better_one].cycle_count_std) {
            meter->sub_measurement_results[worse_one]
                .keep_for_final_computation = false;
        }
        return;
    }
    // find centroid and reject measurements with avgs far from it
    size_t centroid_idx = 0;
    uint best_delta = UINT32_MAX;
    for (size_t i = 0; i < initially_valid_measurements; i++) {
        struct SubMeasurementStats* stats =
            &meter->sub_measurement_stats[valid_result_idxs[i]];
        uint total_delta = 0;
        for (size_t j = 0; j < initially_valid_measurements; j++) {
            if (i == j) continue;
            total_delta +=
                uint_abs_diff(stats->avg_cycle_count,
                              meter->sub_measurement_stats[valid_result_idxs[j]]
                                  .avg_cycle_count);
        }
        if (total_delta < best_delta) {
            centroid_idx = i;
            best_delta = total_delta;
        }
    }
    uint centroid_avg_cycle_count =
        meter->sub_measurement_stats[valid_result_idxs[centroid_idx]]
            .avg_cycle_count;
    uint avg_cycle_count_diff_tolerance = 0;
    if (meter->sub_measurement_results[valid_result_idxs[centroid_idx]]
            .num_recorded_ringing_cycles == 1) {
        size_t closest_idx = 0;
        best_delta = UINT32_MAX;
        for (size_t i = 0; i < initially_valid_measurements; i++) {
            if (i == centroid_idx) continue;
            uint delta =
                uint_abs_diff(centroid_avg_cycle_count,
                              meter->sub_measurement_stats[valid_result_idxs[i]]
                                  .avg_cycle_count);
            if (delta < best_delta) {
                best_delta = delta;
                closest_idx = i;
            }
        }
        if (meter->sub_measurement_results[valid_result_idxs[closest_idx]]
                .num_recorded_ringing_cycles == 1) {
            avg_cycle_count_diff_tolerance =
                MAX(MAX(best_delta * 3 / 2,
                        meter->sub_measurement_stats
                                [valid_result_idxs[centroid_idx]]
                                    .avg_cycle_count /
                            10),
                    MIN_CYCLE_COUNT_TOLERACE);
        } else {
            avg_cycle_count_diff_tolerance = MAX(
                MAX(best_delta * 3 / 2,
                    meter->sub_measurement_stats[valid_result_idxs[closest_idx]]
                            .cycle_count_std *
                        2),
                MIN_CYCLE_COUNT_TOLERACE);
        }
    } else {
        avg_cycle_count_diff_tolerance =
            MAX(meter->sub_measurement_stats[valid_result_idxs[centroid_idx]]
                        .cycle_count_std *
                    2,
                MIN_CYCLE_COUNT_TOLERACE);
    }

    for (size_t i = 0; i < initially_valid_measurements; i++) {
        struct InductanceSubMeasurementResult* smr =
            &meter->sub_measurement_results[valid_result_idxs[i]];
        struct SubMeasurementStats* stats =
            &meter->sub_measurement_stats[valid_result_idxs[i]];
        if (uint_abs_diff(centroid_avg_cycle_count, stats->avg_cycle_count) >
            avg_cycle_count_diff_tolerance) {
            smr->keep_for_final_computation = false;
        }
    }
}

static inline uint64_t compute_L_nH(struct InductanceMeter* meter,
                                    uint cycle_count) {
    double n = (double)cycle_count;
    double L_nH = n * n * meter->L_nh_from_cycle_count_square_coeff;
    return (uint64_t)L_nH;
}

static uint compute_period_drift_percent(struct InductanceMeter* meter,
                                         bool filtered) {
    // metric suggested by chat gpt
    double total_drift_percent = 0.0;
    uint num_usable_sub_measurements = 0;

    for (size_t m = 0; m < meter->num_sub_measurements; m++) {
        struct InductanceSubMeasurementResult* r =
            &meter->sub_measurement_results[m];

        if (filtered && !r->keep_for_final_computation) {
            continue;
        }

        uint n = r->num_recorded_ringing_cycles;
        if (n < 2) {
            continue;
        }

        // linear regression:
        // period(i) = a + b*i
        // b tells us how many PIO clock cycles the measured period changes per
        // ringing cycle
        double mean_x = (double)(n - 1) / 2.0;

        double mean_y = 0.0;
        for (size_t i = 0; i < n; i++) {
            mean_y += r->cycle_counts[i];
        }
        mean_y /= n;

        double covariance = 0.0;
        double x_variance = 0.0;

        for (size_t i = 0; i < n; i++) {
            double dx = (double)i - mean_x;
            double dy = (double)r->cycle_counts[i] - mean_y;

            covariance += dx * dy;
            x_variance += dx * dx;
        }

        if (x_variance == 0.0 || mean_y == 0.0) {
            continue;
        }

        // least squares slope
        double slope = covariance / x_variance;

        // predicted total period change from the first to the last recorded
        // ringing cycle
        double total_period_change = fabs(slope) * (double)(n - 1);

        double drift_percent = 100.0 * total_period_change / mean_y;

        total_drift_percent += drift_percent;
        num_usable_sub_measurements++;
    }

    if (num_usable_sub_measurements == 0) {
        return 100;
    }

    return (uint)lround(total_drift_percent / num_usable_sub_measurements);
}

static void compute_inductance_measurement_stats(
    struct InductanceMeter* meter, bool filtered,
    struct InductanceMeasurementResultStats* stats) {
    stats->num_total_cycles = 0;
    uint total_cycle_count = 0;
    stats->num_total_measurements = 0;
    uint64_t L_nH_aggregated = 0;
    uint weighted_total_pulse_duration = 0;

    for (size_t i = 0; i < meter->num_sub_measurements; i++) {
        if (filtered &&
            !meter->sub_measurement_results[i].keep_for_final_computation) {
            continue;
        }
        stats->num_total_measurements++;
        weighted_total_pulse_duration +=
            meter->sub_measurement_results[i].num_recorded_ringing_cycles *
            meter->sub_measurement_results[i].pulse_duration_us;
        for (size_t j = 0;
             j < meter->sub_measurement_results[i].num_recorded_ringing_cycles;
             j++) {
            total_cycle_count +=
                meter->sub_measurement_results[i].cycle_counts[j];
            stats->num_total_cycles++;
            L_nH_aggregated += compute_L_nH(
                meter, meter->sub_measurement_results[i].cycle_counts[j]);
        }
    }
    assert(stats->num_total_cycles > 0);
    stats->avg_cycle_count = total_cycle_count / stats->num_total_cycles;
    stats->L_nH = (uint)(L_nH_aggregated / (uint64_t)stats->num_total_cycles);

    uint64_t total_cycle_count_diff_squared = 0;
    uint64_t total_L_nh_diff_squared = 0;
    for (size_t i = 0; i < meter->num_sub_measurements; i++) {
        if (filtered &&
            !meter->sub_measurement_results[i].keep_for_final_computation) {
            continue;
        }
        for (size_t j = 0;
             j < meter->sub_measurement_results[i].num_recorded_ringing_cycles;
             j++) {
            uint64_t delta = (uint64_t)uint_abs_diff(
                meter->sub_measurement_results[i].cycle_counts[j],
                stats->avg_cycle_count);
            total_cycle_count_diff_squared += delta * delta;
            uint64_t L = compute_L_nH(
                meter, meter->sub_measurement_results[i].cycle_counts[j]);
            int64_t L_delta = (int64_t)L - (int64_t)stats->L_nH;
            total_L_nh_diff_squared += (uint64_t)(L_delta * L_delta);
        }
    }
    if (stats->num_total_cycles == 1) {
        stats->cycle_count_std = 0;
        stats->L_nh_std = 0;
    } else {
        stats->cycle_count_std =
            (uint)sqrtf((float)total_cycle_count_diff_squared /
                        (float)(stats->num_total_cycles - 1));
        stats->L_nh_std = (uint)sqrtf((float)total_L_nh_diff_squared /
                                      (float)(stats->num_total_cycles - 1));
    }

    if (stats->L_nH <= meter->parasitic_series_L_nH) {
        stats->L_nH = 0;
    } else {
        stats->L_nH -= meter->parasitic_series_L_nH;
    }
    stats->period_drift_percent = compute_period_drift_percent(meter, filtered);
    stats->avg_pulse_duration =
        weighted_total_pulse_duration / stats->num_total_cycles;
}

#ifdef IMF_DEBUG
static void _debug_print_detailed_measurement_results(
    struct InductanceMeter* meter, bool before_filtering) {
    printf("Debugging N=%u measurements %s filtering\n",
           meter->num_sub_measurements, before_filtering ? "before" : "after");
    for (size_t i = 0; i < meter->num_sub_measurements; i++) {
        struct InductanceSubMeasurementResult* r =
            &meter->sub_measurement_results[i];
        printf(
            "%u. num recorded cycles=%u, num initial failures=%u, kept for "
            "final measurement=%s\n",
            i, r->num_recorded_ringing_cycles, r->num_initially_failed,
            r->keep_for_final_computation ? "yes" : "no");
        for (size_t j = 0; j < r->num_recorded_ringing_cycles; j++) {
            printf("%u%s", r->cycle_counts[j],
                   j < r->num_recorded_ringing_cycles - 1 ? ", " : "\n");
        }
    }
}
#endif

static void compute_inductance_with_stats(
    struct InductanceMeter* meter, struct InductanceMeasurementResults* res) {
    if (meter->num_sub_measurements == 0) {
        res->success = false;
        return;
    }
    // at least 1 cycle must've been recorded
    res->success = true;

#ifdef IMF_DEBUG
    _debug_print_detailed_measurement_results(meter, true);
#endif

    compute_inductance_measurement_stats(meter, false,
                                         &res->stats_before_filtering);

    // first handle individual measurements so they have reasonable series
    for (size_t i = 0; i < meter->num_sub_measurements; i++) {
        filter_cycle_counts(&meter->sub_measurement_results[i]);
    }

    compute_sub_measurement_stats(meter);

    filter_sub_measurements(meter);

    compute_inductance_measurement_stats(meter, true, &res->stats);

#ifdef IMF_DEBUG
    _debug_print_detailed_measurement_results(meter, false);
#endif
}

static bool run_auto_measurements(struct InductanceMeter* meter,
                                  struct InductanceMeasurementResults* res) {
    // this auto strategy of averaging different pulse durations is not ideal
    // for nonlinear core inductors where L heavily depends on current, huge
    // stds are exposed by it. the std is, however, visible on UI and info
    // exposes period drift which for such iron/ferrite core inductors is large
    // (>20%), switch to manual mode should handle such issues. Otherwise, this
    // strategy is accurate from 1uH to 10mH, and even handled big 100mH
    // inductor with 1mH error.
    // Maybe, when needed, the filtering/averaging strategy could be adjusted to
    // find one "nice" pulse duration and average only that duration.

    struct InductanceSubMeasurementResult* first_sub_measurement_res =
        &meter->sub_measurement_results[0];
    run_sub_measurement(meter, first_sub_measurement_res, 2,
                        MAX_CYCLE_WAITING_TIME_US,
                        MAX_TRACKED_LC_RINGING_CYCLES, 4);

    if (first_sub_measurement_res->num_recorded_ringing_cycles == 0) {
        uint pulse_duration_schedule[] = {2, 4, 8, 16, 32};
        for (size_t i = 0; i < sizeof(pulse_duration_schedule) /
                                   sizeof(pulse_duration_schedule[0]);
             i++) {
            uint pulse_duration_us = pulse_duration_schedule[i];
            run_sub_measurement(meter, first_sub_measurement_res,
                                pulse_duration_us, MAX_CYCLE_WAITING_TIME_US,
                                MAX_TRACKED_LC_RINGING_CYCLES, 4);
            if (first_sub_measurement_res->num_recorded_ringing_cycles > 0) {
                break;
            }
        }
    }
    if (first_sub_measurement_res->num_recorded_ringing_cycles == 0) {
        res->success = false;
        return false;
    }

    res->total_attempted_cycle_measurements += MAX_TRACKED_LC_RINGING_CYCLES;
    meter->num_sub_measurements = 1;

    if (first_sub_measurement_res->num_recorded_ringing_cycles > 5) {
        // high confidence, do some retries with same period and double the
        // period and compute best result
        uint desired_cycle_count =
            MIN(8, first_sub_measurement_res->num_recorded_ringing_cycles);
        for (; meter->num_sub_measurements < 5; meter->num_sub_measurements++) {
            run_sub_measurement(
                meter,
                &meter->sub_measurement_results[meter->num_sub_measurements],
                first_sub_measurement_res->pulse_duration_us,
                MAX_CYCLE_WAITING_TIME_US, desired_cycle_count, 1);
            res->total_attempted_cycle_measurements += desired_cycle_count;
        }
        for (; meter->num_sub_measurements < 9; meter->num_sub_measurements++) {
            run_sub_measurement(
                meter,
                &meter->sub_measurement_results[meter->num_sub_measurements],
                first_sub_measurement_res->pulse_duration_us * 2,
                MAX_CYCLE_WAITING_TIME_US, desired_cycle_count, 1);
            res->total_attempted_cycle_measurements += desired_cycle_count;
        }
    } else {
        // low confidence, do a broader pulse duration scan with more trials
        uint max_pulse_duration_us = 128;
        uint cur_pulse_duration = first_sub_measurement_res->pulse_duration_us;
        uint desired_cycle_count = 8;

        uint most_ringing_sub_measurement_idx = 0;
        uint max_cycle_count =
            first_sub_measurement_res->num_recorded_ringing_cycles;
        while (meter->num_sub_measurements <
                   MAX_INDUNCATANCE_SUB_MEASUREMENTS &&
               cur_pulse_duration <= max_pulse_duration_us) {
            run_sub_measurement(
                meter,
                &meter->sub_measurement_results[meter->num_sub_measurements],
                cur_pulse_duration, MAX_CYCLE_WAITING_TIME_US,
                desired_cycle_count, 2);
            res->total_attempted_cycle_measurements += desired_cycle_count;
            if (meter->sub_measurement_results[meter->num_sub_measurements]
                    .num_recorded_ringing_cycles > max_cycle_count) {
                most_ringing_sub_measurement_idx = meter->num_sub_measurements;
                max_cycle_count =
                    meter->sub_measurement_results[meter->num_sub_measurements]
                        .num_recorded_ringing_cycles;
            }
            meter->num_sub_measurements++;
            cur_pulse_duration *= 2;
        }

        uint pulse_duration_for_more_retries =
            meter->sub_measurement_results[most_ringing_sub_measurement_idx]
                .pulse_duration_us;

        for (; meter->num_sub_measurements < MAX_INDUNCATANCE_SUB_MEASUREMENTS;
             meter->num_sub_measurements++) {
            run_sub_measurement(
                meter,
                &meter->sub_measurement_results[meter->num_sub_measurements],
                pulse_duration_for_more_retries, MAX_CYCLE_WAITING_TIME_US,
                desired_cycle_count, 2);
            res->total_attempted_cycle_measurements += desired_cycle_count;
        }
    }
    return true;
}

static bool run_manual_measurements(struct InductanceMeter* meter,
                                    struct InductanceMeasurementResults* res) {
    size_t num_desired_measurements =
        meter->measurement_config.num_desired_measurements;
    if (num_desired_measurements == 0) {
        num_desired_measurements = MAX_INDUNCATANCE_SUB_MEASUREMENTS;
    }
    num_desired_measurements =
        MIN(num_desired_measurements, MAX_INDUNCATANCE_SUB_MEASUREMENTS);
    if (meter->measurement_config.pulse_duration_us < 1) {
        return false;
    }
    for (size_t i = 0; i < num_desired_measurements; i++) {
        run_sub_measurement(
            meter, &meter->sub_measurement_results[meter->num_sub_measurements],
            meter->measurement_config.pulse_duration_us,
            MAX_CYCLE_WAITING_TIME_US, MAX_TRACKED_LC_RINGING_CYCLES, 3);
        if (meter->sub_measurement_results[meter->num_sub_measurements]
                    .num_recorded_ringing_cycles > 0 ||
            meter->num_sub_measurements > 0) {
            // make sure first is valid, if possible
            meter->num_sub_measurements++;
            res->total_attempted_cycle_measurements +=
                MAX_TRACKED_LC_RINGING_CYCLES;
        }
    }
    return meter->num_sub_measurements > 0;
}

static void set_should_use_filtered_results_by_default(
    struct InductanceMeter* meter, struct InductanceMeasurementResults* res) {
    uint raw_L = res->stats_before_filtering.L_nH;
    uint filtered_L = res->stats.L_nH;

    uint64_t L_change_percent =
        raw_L > 0 ? (uint64_t)uint_abs_diff(raw_L, filtered_L) * 100 / raw_L
                  : 100;

    uint cycles_kept_percent =
        res->stats_before_filtering.num_total_cycles > 0
            ? (uint)((uint64_t)res->stats.num_total_cycles * 100 /
                     res->stats_before_filtering.num_total_cycles)
            : 0;

    uint raw_std_percent =
        raw_L > 0 ? (uint)((uint64_t)res->stats_before_filtering.L_nh_std *
                           100 / raw_L)
                  : 100;

    // the goal is to not hide shitty measurement (specifically one for iron
    // core coils) by filtering, and use filtering only when it really helped
    // cleanup the measurements
    res->show_filtered_by_default =
        cycles_kept_percent >= 70 && L_change_percent <= 10 &&
        raw_std_percent <= 10 &&
        res->stats_before_filtering.period_drift_percent <= 10;
}

struct InductanceMeasurementResults inductance_meter_measure(
    struct InductanceMeter* meter) {
    struct InductanceMeasurementResults res = {
        .success = false,
        .show_filtered_by_default = true,
        .total_attempted_cycle_measurements = 0};

    bool measurements_ok = false;
    if (meter->measurement_config.auto_mode) {
        measurements_ok = run_auto_measurements(meter, &res);
    } else {
        measurements_ok = run_manual_measurements(meter, &res);
    }
    if (!measurements_ok) {
        return res;
    }
    compute_inductance_with_stats(meter, &res);
    set_should_use_filtered_results_by_default(meter, &res);
    return res;
}
