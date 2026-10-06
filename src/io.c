#include "io.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lcd_driver.h"
#include "persistent_storage.h"
#include "pico/stdlib.h"
#include "rotary_encoder_driver.h"
#include "state.h"

#define MENU_REFRESH_PERIOD_MS 30

#define NUM_ROWS 2
#define NUM_COLS 16
#define SCREEN_BUFF_SIZE (NUM_ROWS * NUM_COLS)
#define ENCODER_PRESS_MIN_HOLD_DURATION_MS 1500
#define ENCODER_PRESS_DEBOUNCE_MS 50
#define SELECTED_OPTION_CHAR '>'
#define MIN_WINDINGS 10

#define MM_TO_MICRONS(mm) ((mm) * 1000)

#define MIN_WIRE_DIAMETER_MICRONS 100   // 0.1mm
#define MAX_WIRE_DIAMETER_MICRONS 2000  // 2mm
#define WIRE_DIAMETER_STEP_MICRONS 50   // 0.05mm
#define WIRE_PITCH_STEP_MICRONS 5       // 0.005mm

#define MIN_OBJECT_LENGTH_MICRONS MM_TO_MICRONS(5)
#define MAX_OBJECT_LENGTH_MICRONS MM_TO_MICRONS(200)
#define BASE_OBJECT_LENGTH_STEP_MICRONS 500  // 0.5mm

#define MAX_START_OFFSET_MICRONS MM_TO_MICRONS(100)
#define MAX_SPEED_PERCENT 200

static inline uint default_wire_pitch_from_wire_diameter(
    uint diameter_microns) {
    if (diameter_microns == 500) {
        return 600;
    } else if (diameter_microns == 250) {
        return 350;
    }
    return diameter_microns + 2 * (diameter_microns / 10);
}

static char prev_screen_buff[SCREEN_BUFF_SIZE];
static char screen_buff[SCREEN_BUFF_SIZE];

static struct LCD* output_screen = NULL;
static struct RotaryEncoder* rotary_encoder = NULL;

static bool is_encoder_click_disabled = false;
static uint encoder_click_disabled_end_timestamp_ms = 0;

static void menu_disable_encoder_click(uint duration_ms) {
    is_encoder_click_disabled = true;
    encoder_click_disabled_end_timestamp_ms =
        to_ms_since_boot(get_absolute_time()) + duration_ms;
}

struct MenuScreen {
    struct MenuScreen* prev;
    void* state;

    void (*render)(struct MenuScreen* screen, char* screen_mem);
    void (*handle_click)(struct MenuScreen* screen);
    void (*handle_rotation)(struct MenuScreen* screen, int delta);

    // optional
    void (*handle_hold)(struct MenuScreen* screen);
};

static struct MenuScreen* current_menu_screen = NULL;

static void switch_to_previous_menu_screen(void);

static struct WindingConfig last_winding_config;
static void menu_begin_winding(struct WindingConfig config);

static struct InductanceMeterMeasurementConfig
    last_inductance_measurement_config;

static void output_screen_set_row(uint row, const char* text) {
    assert(row < NUM_ROWS);
    lcd_i2c_set_cursor(output_screen, 0, (uint8_t)row);
    size_t i = 0;
    for (; *text != '\0' && i < NUM_COLS; text++, i++) {
        lcd_i2c_putc(output_screen, *text);
        prev_screen_buff[row * NUM_COLS + i] = *text;
    }
    for (; i < NUM_COLS; i++) {
        lcd_i2c_putc(output_screen, ' ');
        prev_screen_buff[row * NUM_COLS + i] = ' ';
    }
}

// Wind Menu
// -------------------------------------------------------------------------------------------------

static const char* wind_menu_titles[] = {
    "object length:", "number of turns:", "wire diameter:",
    "wire pitch:",    "speed:",           "start offset:"};
static const char* wind_menu_confirm_short_labels[] = {"L", "N", "D",
                                                       "P", "S", "O"};

enum WindMenuStage {
    WIND_MENU_LENGTH = 0,
    WIND_MENU_NUMBER_OF_TURNS,
    WIND_MENU_WIRE_DIAMETER,
    WIND_MENU_WIRE_PITCH,
    WIND_MENU_SPEED,
    WIND_MENU_START_OFFSET,
    WIND_MENU_CONFIRM
};

struct WindMenuState {
    enum WindMenuStage current_stage;
    struct WindingConfig config;

    enum WindMenuStage current_confirm_substage;

    uint current_step;
    uint value_snapshot;
    uint value_snapshot_timestamp_ms;
    int last_rotation_direction;
};
static const uint wind_menu_base_step_values[] = {
    BASE_OBJECT_LENGTH_STEP_MICRONS, 1, WIRE_DIAMETER_STEP_MICRONS,
    WIRE_PITCH_STEP_MICRONS,         1, WIRE_DIAMETER_STEP_MICRONS};
static const uint wind_menu_max_step[] = {
    MM_TO_MICRONS(10),          100, WIRE_DIAMETER_STEP_MICRONS,
    WIRE_DIAMETER_STEP_MICRONS, 5,   MM_TO_MICRONS(1)};
static const uint wind_menu_value_offsets[] = {
    offsetof(struct WindingConfig, length_microns),
    offsetof(struct WindingConfig, number_of_turns),
    offsetof(struct WindingConfig, wire_diameter_microns),
    offsetof(struct WindingConfig, wire_pitch_microns),
    offsetof(struct WindingConfig, speed_percent),
    offsetof(struct WindingConfig, winding_start_offset_microns)};

static size_t pretty_render_microns_with_unit(char* target, size_t target_size,
                                              uint val, bool compact) {
    uint val_mm = val / 1000;
    uint val_um = val % 1000;
    char number_buff[NUM_COLS];
    int sz = 0;
    const char* unit_str = compact ? "" : " mm";
    if (val_um == 0) {
        sz = snprintf(number_buff, target_size, "%u%s", val_mm, unit_str);
    } else {
        sz = snprintf(number_buff, target_size, "%u.%u%s", val_mm, val_um,
                      unit_str);
    }
    assert(sz >= 0);
    memcpy(target, number_buff, (size_t)sz);
    return (size_t)sz;
}

static size_t render_menu_options(char* target, const char** options,
                                  size_t num_options, size_t selected_option) {
    size_t pos = 0;
    for (size_t i = 0; i < num_options; i++) {
        if (selected_option == i) {
            target[pos++] = SELECTED_OPTION_CHAR;
        }
        size_t option_size = strlen(options[i]);
        memcpy(&target[pos], options[i], option_size);
        pos += option_size;
        if (i < num_options - 1) {
            target[pos++] = ' ';
        }
    }
    return pos;
}

static void wind_menu_render_value(const struct WindingConfig* cfg,
                                   char* target, size_t target_size,
                                   enum WindMenuStage stage) {
    memset(target, ' ', target_size);
    if (stage == WIND_MENU_NUMBER_OF_TURNS) {
        snprintf(target, target_size, "%u", cfg->number_of_turns);
    } else if (stage == WIND_MENU_SPEED) {
        snprintf(target, target_size, "%u%%", cfg->speed_percent);
    } else {
        uint val =
            *(const uint*)((const void*)cfg + wind_menu_value_offsets[stage]);
        pretty_render_microns_with_unit(target, target_size, val, false);
    }
}

static void wind_menu_reset_snapshot_state(struct WindMenuState* state) {
    if (state->current_stage == WIND_MENU_CONFIRM) {
        return;
    }
    state->value_snapshot =
        *(const uint*)((const void*)&state->config +
                       wind_menu_value_offsets[state->current_stage]);
    state->value_snapshot_timestamp_ms = to_ms_since_boot(get_absolute_time());
    state->current_step = wind_menu_base_step_values[state->current_stage];
    state->last_rotation_direction = 0;
}

static void wind_menu_render(struct MenuScreen* screen, char* screen_mem) {
    struct WindMenuState* state = screen->state;
    if (state->current_stage == WIND_MENU_CONFIRM) {
        strcpy(screen_mem, "ok?");
        if (state->current_confirm_substage == WIND_MENU_CONFIRM) {
            strcpy(&screen_mem[NUM_COLS], "click to begin");
        } else {
            memcpy(
                &screen_mem[NUM_COLS],
                wind_menu_confirm_short_labels[state->current_confirm_substage],
                1);
            screen_mem[NUM_COLS + 1] = '=';
            wind_menu_render_value(&state->config, &screen_mem[NUM_COLS + 2],
                                   NUM_COLS - 2,
                                   state->current_confirm_substage);
        }
    } else {
        memcpy(screen_mem, wind_menu_titles[state->current_stage],
               strlen(wind_menu_titles[state->current_stage]));
        wind_menu_render_value(&state->config, &screen_mem[NUM_COLS], NUM_COLS,
                               state->current_stage);
    }
}

static void wind_menu_handle_click(struct MenuScreen* screen) {
    struct WindMenuState* state = screen->state;
    if (state->current_stage == WIND_MENU_CONFIRM) {
        state->current_stage = 0;
        wind_menu_reset_snapshot_state(state);
        menu_begin_winding(state->config);
    } else {
        state->current_stage++;
        wind_menu_reset_snapshot_state(state);
        if (state->current_stage == WIND_MENU_CONFIRM) {
            state->current_confirm_substage = 0;
        } else if (state->current_stage == WIND_MENU_WIRE_PITCH) {
            state->config.wire_pitch_microns =
                default_wire_pitch_from_wire_diameter(
                    state->config.wire_diameter_microns);
        } else if (state->current_stage == WIND_MENU_START_OFFSET) {
            state->config.winding_start_offset_microns =
                2 * state->config.wire_diameter_microns;
        }
    }
}

static void wind_menu_handle_hold(struct MenuScreen* screen) {
    struct WindMenuState* state = screen->state;
    state->current_stage = 0;
    current_menu_screen = screen->prev;
}

static uint rotation_clip_uint(uint val, int delta, uint min_val,
                               uint max_val) {
    if (delta < 0 && val <= min_val + (uint)(-delta)) {
        return min_val;
    } else if (delta < 0) {
        return val - (uint)(-delta);
    } else if (max_val != 0 && val + (uint)delta >= max_val) {
        return max_val;
    } else {
        return val + (uint)delta;
    }
}

#define SNAPSHOT_PERIOD_MS 500
static void wind_menu_handle_rotation(struct MenuScreen* screen, int delta) {
    struct WindMenuState* state = screen->state;
    if (state->current_stage == WIND_MENU_CONFIRM) {
        if (delta < 0 && state->current_confirm_substage == 0) {
            // go back to start but don't reset any values
            state->current_stage = 0;
            wind_menu_reset_snapshot_state(state);
        } else if (delta < 0) {
            state->current_confirm_substage--;
        } else if (state->current_confirm_substage < WIND_MENU_CONFIRM) {
            state->current_confirm_substage++;
        }
    } else {
        int direction = (delta > 0) - (delta < 0);
        if (direction != 0 && state->last_rotation_direction != 0 &&
            direction != state->last_rotation_direction) {
            wind_menu_reset_snapshot_state(state);
        }
        if (direction != 0) {
            state->last_rotation_direction = direction;
        }

        uint now_ms = to_ms_since_boot(get_absolute_time());

        if (now_ms - state->value_snapshot_timestamp_ms > SNAPSHOT_PERIOD_MS) {
            int cur_snapshot_delta =
                (int)(*(uint*)((void*)&state->config +
                               wind_menu_value_offsets[state->current_stage])) -
                (int)state->value_snapshot;
            if (cur_snapshot_delta == 0 ||
                now_ms - state->value_snapshot_timestamp_ms >
                    2 * SNAPSHOT_PERIOD_MS) {
                state->current_step =
                    wind_menu_base_step_values[state->current_stage];
            } else if (abs(cur_snapshot_delta) > 4 * state->current_step) {
                if (state->current_step >
                    wind_menu_base_step_values[state->current_stage] * 10) {
                    state->current_step *= 2;
                } else {
                    state->current_step *= 10;
                }
                state->current_step =
                    MIN(state->current_step,
                        wind_menu_max_step[state->current_stage]);
            }
        }
        int actual_delta = delta * (int)state->current_step;

        if (state->current_stage == WIND_MENU_NUMBER_OF_TURNS) {
            state->config.number_of_turns = rotation_clip_uint(
                state->config.number_of_turns, actual_delta, MIN_WINDINGS, 0);
        } else if (state->current_stage == WIND_MENU_WIRE_DIAMETER) {
            state->config.wire_diameter_microns = rotation_clip_uint(
                state->config.wire_diameter_microns, actual_delta,
                MIN_WIRE_DIAMETER_MICRONS, MAX_WIRE_DIAMETER_MICRONS);
        } else if (state->current_stage == WIND_MENU_WIRE_PITCH) {
            state->config.wire_pitch_microns = rotation_clip_uint(
                state->config.wire_pitch_microns, actual_delta, 0,
                state->config.wire_diameter_microns * 2);
        } else if (state->current_stage == WIND_MENU_LENGTH) {
            state->config.length_microns = rotation_clip_uint(
                state->config.length_microns, actual_delta,
                MIN_OBJECT_LENGTH_MICRONS, MAX_OBJECT_LENGTH_MICRONS);
        } else if (state->current_stage == WIND_MENU_SPEED) {
            state->config.speed_percent =
                rotation_clip_uint(state->config.speed_percent, actual_delta, 1,
                                   MAX_SPEED_PERCENT);
        } else if (state->current_stage == WIND_MENU_START_OFFSET) {
            state->config.winding_start_offset_microns =
                rotation_clip_uint(state->config.winding_start_offset_microns,
                                   actual_delta, 0, MAX_START_OFFSET_MICRONS);
        }
        if (now_ms - state->value_snapshot_timestamp_ms > SNAPSHOT_PERIOD_MS) {
            state->value_snapshot =
                *(uint*)((void*)&state->config +
                         wind_menu_value_offsets[state->current_stage]);
            state->value_snapshot_timestamp_ms = now_ms;
        }
    }
}

#define DEFAULT_WIND_MENU_LENGTH 18000
static struct WindMenuState wind_menu_state = {
    .current_confirm_substage = 0,
    .current_stage = 0,
    .current_step = BASE_OBJECT_LENGTH_STEP_MICRONS,
    .config = {.length_microns = DEFAULT_WIND_MENU_LENGTH,
               .number_of_turns = 100,
               .wire_diameter_microns = 500,  // 0.5mm
               .wire_pitch_microns = 600,
               .speed_percent = 80,
               .winding_start_offset_microns = 0},
    .value_snapshot = DEFAULT_WIND_MENU_LENGTH,
    .value_snapshot_timestamp_ms = 0,
};
static struct MenuScreen wind_menu_screen = {
    .state = &wind_menu_state,
    .prev = NULL,
    .render = wind_menu_render,
    .handle_click = wind_menu_handle_click,
    .handle_hold = wind_menu_handle_hold,
    .handle_rotation = wind_menu_handle_rotation};

// -------------------------------------------------------------------------------------------------

// Wind Preset Menu
// -------------------------------------------------------------------------------------------------

struct WindingPreset {
    const char* name;
    struct WindingConfig config;
};

static const struct WindingPreset presets[] = {
    {.name = "18_120_050",
     .config = {.length_microns = MM_TO_MICRONS(17) + 500,
                .number_of_turns = 120,
                .wire_diameter_microns = 500,
                .wire_pitch_microns = 600,
                .speed_percent = 80,
                .winding_start_offset_microns = 1000}},
    {.name = "18_120_025",
     .config = {.length_microns = MM_TO_MICRONS(17) + 500,
                .number_of_turns = 120,
                .wire_diameter_microns = 250,
                .wire_pitch_microns = 350,
                .speed_percent = 80,
                .winding_start_offset_microns = 500}}};

#define NUM_PRESETS (sizeof(presets) / sizeof(presets[0]))

struct WindPresetMenuState {
    uint current_preset;
    bool preset_selected;
    enum WindMenuStage show_preset_stage;
};

static void wind_preset_menu_render(struct MenuScreen* screen,
                                    char* screen_mem) {
    struct WindPresetMenuState* state = screen->state;
    const char* name = presets[state->current_preset].name;
    if (state->preset_selected) {
        size_t name_len = strlen(name);
        memcpy(screen_mem, name, name_len);
        screen_mem[name_len] = ':';
        memcpy(&screen_mem[NUM_COLS],
               wind_menu_confirm_short_labels[state->show_preset_stage], 1);
        screen_mem[NUM_COLS + 1] = '=';
        wind_menu_render_value(&presets[state->current_preset].config,
                               &screen_mem[NUM_COLS + 2], NUM_COLS - 2,
                               state->show_preset_stage);
    } else {
        strcpy(screen_mem, "select preset:");
        memcpy(&screen_mem[NUM_COLS], name, strlen(name));
    }
}

static void wind_preset_menu_handle_rotation(struct MenuScreen* screen,
                                             int delta) {
    struct WindPresetMenuState* state = screen->state;
    if (state->preset_selected) {
        if ((delta < 0 && state->show_preset_stage == 0) ||
            (delta > 0 && state->show_preset_stage == WIND_MENU_CONFIRM - 1)) {
            return;
        }
        state->show_preset_stage += delta > 0 ? 1 : -1;
    } else {
        if ((delta < 0 && state->current_preset == 0) ||
            (delta > 0 && state->current_preset == NUM_PRESETS - 1)) {
            return;
        }
        if (delta > 0) {
            state->current_preset++;
        } else {
            state->current_preset--;
        }
    }
}

static void wind_preset_menu_handle_click(struct MenuScreen* screen) {
    struct WindPresetMenuState* state = screen->state;
    if (state->preset_selected) {
        state->preset_selected = false;
        state->current_preset = 0;
        menu_begin_winding(presets[state->current_preset].config);
    } else {
        state->preset_selected = true;
        state->show_preset_stage = 0;
    }
}

static void wind_preset_menu_handle_hold(struct MenuScreen* screen) {
    struct WindPresetMenuState* state = screen->state;
    current_menu_screen = screen->prev;
    state->current_preset = 0;
    state->preset_selected = false;
    state->show_preset_stage = 0;
}

struct WindPresetMenuState wind_preset_menu_state = {
    .current_preset = 0, .preset_selected = false, .show_preset_stage = 0};

struct MenuScreen wind_preset_menu_screen = {
    .prev = NULL,
    .state = &wind_preset_menu_state,
    .render = wind_preset_menu_render,
    .handle_click = wind_preset_menu_handle_click,
    .handle_hold = wind_preset_menu_handle_hold,
    .handle_rotation = wind_preset_menu_handle_rotation};

// -------------------------------------------------------------------------------------------------

// Measure Inductance Menu
// -------------------------------------------------------------------------------------------------

struct MeasureInductanceMenuState {
    bool selected_auto;
    bool is_on_main_subscreen;
    uint pulse_period_us;
};

static void measure_inductance_menu_render(struct MenuScreen* screen,
                                           char* screen_mem) {
    struct MeasureInductanceMenuState* state = screen->state;
    if (state->is_on_main_subscreen) {
        strcpy(screen_mem, "mode:");
        const char* options[] = {"auto", "manual"};
        render_menu_options(&screen_mem[NUM_COLS], options, 2,
                            state->selected_auto ? 0 : 1);
    } else {
        strcpy(screen_mem, "pulse duration:");
        snprintf(&screen_mem[NUM_COLS], NUM_COLS, "%u [us]",
                 state->pulse_period_us);
    }
}

static void measure_inductance_menu_handle_rotation(struct MenuScreen* screen,
                                                    int delta) {
    struct MeasureInductanceMenuState* state = screen->state;
    if (state->is_on_main_subscreen) {
        if (delta > 0 && state->selected_auto) {
            state->selected_auto = false;
        } else if (delta < 0 && !state->selected_auto) {
            state->selected_auto = true;
        }
    } else {
        if (delta > 0) {
            state->pulse_period_us += (uint)delta;
        } else if (state->pulse_period_us > (uint)-delta) {
            state->pulse_period_us -= (uint)-delta;
        } else {
            state->pulse_period_us = 1;
        }
    }
}

static void measure_inductance_menu_handle_click(struct MenuScreen* screen) {
    struct MeasureInductanceMenuState* state = screen->state;
    if (state->is_on_main_subscreen && state->selected_auto) {
        last_inductance_measurement_config.auto_mode = true;
        state_switch_to_measuring_inductance(
            last_inductance_measurement_config);
    } else if (state->is_on_main_subscreen) {
        state->is_on_main_subscreen = false;
    } else {
        last_inductance_measurement_config.auto_mode = false;
        last_inductance_measurement_config.num_desired_measurements =
            0;  // will do 16 or whatever imf has configured as max
        last_inductance_measurement_config.pulse_duration_us =
            state->pulse_period_us;
        state_switch_to_measuring_inductance(
            last_inductance_measurement_config);
    }
}

static void measure_inductance_menu_handle_hold(struct MenuScreen* screen) {
    state_switch_to_idle();
    switch_to_previous_menu_screen();
}

static struct MeasureInductanceMenuState measure_inductance_menu_state = {
    .is_on_main_subscreen = true, .pulse_period_us = 1, .selected_auto = true};

static struct MenuScreen measure_inductance_menu_screen = {
    .prev = NULL,
    .state = &measure_inductance_menu_state,
    .render = measure_inductance_menu_render,
    .handle_click = measure_inductance_menu_handle_click,
    .handle_hold = measure_inductance_menu_handle_hold,
    .handle_rotation = measure_inductance_menu_handle_rotation};

// -------------------------------------------------------------------------------------------------

// Inductance Measuring Finished Menu
// -------------------------------------------------------------------------------------------------

static void pretty_render_inductance_with_tolerance_and_unit(char* target,
                                                             size_t target_size,
                                                             uint L_nh,
                                                             uint L_nh_std) {
    uint L_vals[2], L_std_vals[2];
    const char* unit;

    if (L_nh >= 1000000) {
        L_vals[0] = MIN(L_nh / 1000000, 999);
        L_vals[1] = (L_nh / 1000) % 1000;

        L_std_vals[0] = L_nh_std / 1000000;
        L_std_vals[1] = (L_nh_std / 1000) % 1000;

        unit = "mH";
    } else {
        L_vals[0] = (L_nh / 1000) % 1000;
        L_vals[1] = L_nh % 1000;

        L_std_vals[0] = L_nh_std / 1000;
        L_std_vals[1] = L_nh_std % 1000;

        unit = "uH";
    }

    char buff[NUM_COLS + 1];

    snprintf(buff, sizeof(buff), "%u.%03u+%u.%03u%s", L_vals[0], L_vals[1],
             L_std_vals[0], L_std_vals[1], unit);

    if (strlen(buff) > NUM_COLS) {
        L_vals[1] /= 10;
        L_std_vals[1] /= 10;

        snprintf(buff, sizeof(buff), "%u.%02u+%u.%02u%s", L_vals[0], L_vals[1],
                 L_std_vals[0], L_std_vals[1], unit);
    }

    if (strlen(buff) > NUM_COLS) {
        L_vals[1] /= 10;
        L_std_vals[1] /= 10;

        snprintf(buff, sizeof(buff), "%u.%01u+%u.%01u%s", L_vals[0], L_vals[1],
                 L_std_vals[0], L_std_vals[1], unit);
    }

    if (strlen(buff) > NUM_COLS) {
        snprintf(buff, sizeof(buff), "%u+%u%s", L_vals[0], L_std_vals[0], unit);
    }

    memcpy(target, buff, MIN(NUM_COLS, target_size));
}

enum InductanceMeasurementFinishedMainFinishedMenuOption {
    IMF_MAIN_MENU_OPTION_Ok = 0,
    IMF_MAIN_MENU_OPTION_DETAILS,
    IMF_MAIN_MENU_OPTION_RETRY,
};

#define IMF_MAIN_SUBSCREEN_NUM_OPTIONS 3
static const char* imf_option_names[] = {"ok", "info", "retry"};

enum InductanceMeasurementFinishedSubScreen {
    IMF_SUB_SCREEN_MAIN,
    IMF_SUB_SCREEN_DETAILS
};

#define IMF_DETAILS_SUBSCREEN_NUM_OPTIONS 7

enum IMDetailOption {
    IM_DETAIL_OPTION_L = 0,
    IM_DETAIL_OPTION_PERIOD_DRIFT,
    IM_DETAIL_OPTION_AVG_PULSE_DURATION,
    IM_DETAIL_OPTION_N_MEASURES,
    IM_DETAIL_OPTION_N_CYCLES,
    IM_DETAIL_OPTION_CYCLE_COUNT,
    IM_DETAIL_OPTION_N_ATTEMPTED_CYCLES,
};

static const char* imf_detail_option_labels[] = {
    "L",        "period drift", "avg pulse dur",  "N measures",
    "N cycles", "cycle count",  "N attmpt cycles"};

struct InductanceMeasurementFinishedScreenState {
    struct InductanceMeasurementResults results;
    enum InductanceMeasurementFinishedSubScreen cur_subscreen;
    size_t main_screen_selected_option;
    size_t details_screen_cur_option;
    bool details_show_filtered_value;
};

static void inductance_measurement_finished_menu_render(
    struct MenuScreen* screen, char* screen_mem) {
    struct InductanceMeasurementFinishedScreenState* state = screen->state;
    if (state->cur_subscreen == IMF_SUB_SCREEN_MAIN) {
        if (state->results.success) {
            struct InductanceMeasurementResultStats* stats =
                state->results.show_filtered_by_default
                    ? &state->results.stats
                    : &state->results.stats_before_filtering;
            memcpy(screen_mem, "L=", 2);
            pretty_render_inductance_with_tolerance_and_unit(
                &screen_mem[2], NUM_COLS, stats->L_nH, stats->L_nh_std);
        } else {
            strcpy(screen_mem, "failed");
        }
        render_menu_options(&screen_mem[NUM_COLS], imf_option_names,
                            IMF_MAIN_SUBSCREEN_NUM_OPTIONS,
                            state->main_screen_selected_option);
    } else if (state->cur_subscreen == IMF_SUB_SCREEN_DETAILS) {
        if (state->results.success) {
            struct InductanceMeasurementResultStats* stats =
                state->details_show_filtered_value
                    ? &state->results.stats
                    : &state->results.stats_before_filtering;
            size_t option_len = strlen(
                imf_detail_option_labels[state->details_screen_cur_option]);
            strcpy(screen_mem,
                   imf_detail_option_labels[state->details_screen_cur_option]);
            if (state->details_show_filtered_value &&
                state->details_screen_cur_option !=
                    IMF_DETAILS_SUBSCREEN_NUM_OPTIONS - 1) {
                memcpy(&screen_mem[option_len], " (f)", 4);
            }
            char* row2 = &screen_mem[NUM_COLS];
            if (state->details_screen_cur_option == IM_DETAIL_OPTION_L) {
                pretty_render_inductance_with_tolerance_and_unit(
                    row2, NUM_COLS, stats->L_nH, stats->L_nh_std);

            } else if (state->details_screen_cur_option ==
                       IM_DETAIL_OPTION_PERIOD_DRIFT) {
                snprintf(row2, NUM_COLS, "%u%%", stats->period_drift_percent);
            } else if (state->details_screen_cur_option ==
                       IM_DETAIL_OPTION_CYCLE_COUNT) {
                snprintf(row2, NUM_COLS, "%u +- %u", stats->avg_cycle_count,
                         stats->cycle_count_std);
            } else {
                uint val = 0;
                if (state->details_screen_cur_option ==
                    IM_DETAIL_OPTION_AVG_PULSE_DURATION) {
                    val = stats->avg_pulse_duration;
                } else if (state->details_screen_cur_option ==
                           IM_DETAIL_OPTION_N_MEASURES) {
                    val = stats->num_total_measurements;
                } else if (state->details_screen_cur_option ==
                           IM_DETAIL_OPTION_N_ATTEMPTED_CYCLES) {
                    val = state->results.total_attempted_cycle_measurements;
                } else if (state->details_screen_cur_option ==
                           IM_DETAIL_OPTION_N_CYCLES) {
                    val = stats->num_total_cycles;
                }
                snprintf(row2, NUM_COLS, "%u", val);
            }
        } else {
            strcpy(screen_mem, "try manual or bigger pulse dur");
        }
    }
}

static void inductance_measurement_finished_menu_handle_rotation(
    struct MenuScreen* screen, int delta) {
    struct InductanceMeasurementFinishedScreenState* state = screen->state;
    if (state->cur_subscreen == IMF_SUB_SCREEN_MAIN) {
        if (delta < 0 && state->main_screen_selected_option > 0) {
            state->main_screen_selected_option--;
        } else if (delta > 0 && state->main_screen_selected_option <
                                    IMF_MAIN_SUBSCREEN_NUM_OPTIONS - 1) {
            state->main_screen_selected_option++;
        }
    } else if (state->cur_subscreen == IMF_SUB_SCREEN_DETAILS &&
               state->results.success) {
        if (delta < 0 && state->details_screen_cur_option > 0) {
            state->details_screen_cur_option--;
        } else if (delta > 0 && state->details_screen_cur_option <
                                    IMF_DETAILS_SUBSCREEN_NUM_OPTIONS - 1) {
            state->details_screen_cur_option++;
        }
    }
}

static void inductance_measurement_finished_menu_handle_click(
    struct MenuScreen* screen) {
    struct InductanceMeasurementFinishedScreenState* state = screen->state;
    if (state->cur_subscreen == IMF_SUB_SCREEN_MAIN) {
        if (state->main_screen_selected_option == IMF_MAIN_MENU_OPTION_Ok) {
            state_switch_to_idle();
            switch_to_previous_menu_screen();
        } else if (state->main_screen_selected_option ==
                   IMF_MAIN_MENU_OPTION_DETAILS) {
            state->cur_subscreen = IMF_SUB_SCREEN_DETAILS;
            state->details_screen_cur_option = 0;
        } else if (state->main_screen_selected_option ==
                   IMF_MAIN_MENU_OPTION_RETRY) {
            state_switch_to_measuring_inductance(
                last_inductance_measurement_config);
        }
    } else if (state->cur_subscreen == IMF_SUB_SCREEN_DETAILS) {
        if (state->results.success) {
            state->details_show_filtered_value =
                !state->details_show_filtered_value;
        }
    }
}

static void inductance_measurement_finished_menu_handle_hold(
    struct MenuScreen* screen) {
    struct InductanceMeasurementFinishedScreenState* state = screen->state;
    if (state->cur_subscreen == IMF_SUB_SCREEN_MAIN) {
        state_switch_to_idle();
        switch_to_previous_menu_screen();
    } else {
        state->cur_subscreen = IMF_SUB_SCREEN_MAIN;
    }
}

struct InductanceMeasurementFinishedScreenState
    inductance_measurement_finished_menu_state;

struct MenuScreen inductance_measurement_finished_menu_screen = {
    .prev = NULL,
    .state = &inductance_measurement_finished_menu_state,
    .render = inductance_measurement_finished_menu_render,
    .handle_click = inductance_measurement_finished_menu_handle_click,
    .handle_hold = inductance_measurement_finished_menu_handle_hold,
    .handle_rotation = inductance_measurement_finished_menu_handle_rotation};

// -------------------------------------------------------------------------------------------------

// Global Config Menu
// -------------------------------------------------------------------------------------------------

enum GlobalConfigMenuOptions {
    GLOBAL_CONFIG_STALL_THRESHOLD = 0,
    GLOBAL_CONFIG_DEFAULT_SPEED,
    GLOBAL_CONFIG_SAVE
};

enum GlobalConfigSaveMenuOptions {
    GLOBAL_CONFIG_SAVE_MENU_OPTION_SAVE = 0,
    GLOBAL_CONFIG_SAVE_MENU_OPTION_DEFAULT
};

struct GlobalConfigMenuState {
    enum GlobalConfigMenuOptions current_option;
    bool selected_for_modification;

    struct StoredSettings settings;
    uint selected_save_option;
};

static void global_config_menu_render(struct MenuScreen* screen,
                                      char* screen_mem) {
    struct GlobalConfigMenuState* state = screen->state;
    char *row1 = screen_mem, *row2 = &screen_mem[NUM_COLS];
    if (state->current_option == GLOBAL_CONFIG_STALL_THRESHOLD) {
        strcpy(row1, "stall threshold");
        snprintf(row2, NUM_COLS, "0=no stall;%lu%%",
                 state->settings.stall_threshold_percent);
    } else if (state->current_option == GLOBAL_CONFIG_DEFAULT_SPEED) {
        strcpy(row1, "default speed");
        snprintf(row2, NUM_COLS, "%lu%%",
                 state->settings.default_speed_percent);
    } else if (state->current_option == GLOBAL_CONFIG_SAVE) {
        strcpy(row1, "confirm");
        const char* save_menu_options[] = {"save", "default"};
        render_menu_options(row2, save_menu_options, 2,
                            state->selected_save_option);
    }
}

static void global_config_menu_handle_rotation(struct MenuScreen* screen,
                                               int delta) {
    struct GlobalConfigMenuState* state = screen->state;
    if (state->selected_for_modification) {
        if (state->current_option == GLOBAL_CONFIG_SAVE) {
            state->selected_save_option =
                rotation_clip_uint(state->selected_save_option, delta, 0, 1);
        } else {
            if (state->current_option == GLOBAL_CONFIG_STALL_THRESHOLD) {
                state->settings.stall_threshold_percent = rotation_clip_uint(
                    state->settings.stall_threshold_percent, delta,
                    MIN_STALL_THRESHOLD_PERCENT, MAX_STALL_THRESHOLD_PERCENT);
            } else if (state->current_option == GLOBAL_CONFIG_DEFAULT_SPEED) {
                state->settings.default_speed_percent = rotation_clip_uint(
                    state->settings.default_speed_percent, delta,
                    MIN_DEFAULT_SPEED_PERCENT, MAX_DEFAULT_SPEED_PERCENT);
            }
        }
    } else {
        if (delta > 0 && state->current_option < GLOBAL_CONFIG_SAVE) {
            state->current_option++;
        } else if (delta < 0 && state->current_option > 0) {
            state->current_option--;
        }
    }
}

static void global_config_menu_handle_click(struct MenuScreen* screen) {
    struct GlobalConfigMenuState* state = screen->state;
    if (!state->selected_for_modification ||
        state->current_option != GLOBAL_CONFIG_SAVE) {
        state->selected_for_modification = !state->selected_for_modification;
        return;
    }
    if (state->selected_save_option == GLOBAL_CONFIG_SAVE_MENU_OPTION_SAVE) {
        persistent_storage_save_stored_settings(state->settings);
    } else if (state->selected_save_option ==
               GLOBAL_CONFIG_SAVE_MENU_OPTION_DEFAULT) {
        persistent_storage_get_default_settings(&state->settings);
        persistent_storage_save_stored_settings(state->settings);
    }
    switch_to_previous_menu_screen();
}

static void global_config_menu_handle_hold(struct MenuScreen* screen) {
    switch_to_previous_menu_screen();
}

static struct GlobalConfigMenuState global_config_menu_state;

static void global_config_menu_state_init(void) {
    global_config_menu_state.current_option = 0;
    global_config_menu_state.selected_for_modification = false;
    global_config_menu_state.selected_save_option = 0;
    persistent_storage_get_stored_settings_or_defaults(
        &global_config_menu_state.settings);
}

static struct MenuScreen global_config_menu_screen = {
    .prev = NULL,
    .state = &global_config_menu_state,
    .render = global_config_menu_render,
    .handle_click = global_config_menu_handle_click,
    .handle_hold = global_config_menu_handle_hold,
    .handle_rotation = global_config_menu_handle_rotation};

// -------------------------------------------------------------------------------------------------

// Winding Error Menu
// -------------------------------------------------------------------------------------------------

// Main Menu
// -------------------------------------------------------------------------------------------------

struct MainMenuState {
    uint current_position;
    bool wind_menu_entered;
};

#define MAIN_MENU_ROW1_NUM_OPTIONS 3
static const char* main_menu_options_row1[] = {"Wind", "Jog", "Wind_P"};
#define MAIN_MENU_ROW2_NUM_OPTIONS 2
static const char* main_menu_options_row2[] = {"Hold", "Measure_L"};

enum main_menu_options {
    MAIN_MENU_WIND = 0,
    MAIN_MENU_JOG,
    MAIN_MENU_WIND_PRESET,
    MAIN_MENU_HOLD,
    MAIN_MENU_MEASURE_L,
};

static void main_menu_render(struct MenuScreen* screen, char* screen_mem) {
    struct MainMenuState* state = screen->state;
    size_t screen_mem_pos = 0;
    size_t options_counter = 0;

    for (size_t j = 0; j < 2; j++) {
        screen_mem_pos = j * NUM_COLS;
        size_t num_options =
            j == 0 ? MAIN_MENU_ROW1_NUM_OPTIONS : MAIN_MENU_ROW2_NUM_OPTIONS;
        const char** options =
            j == 0 ? main_menu_options_row1 : main_menu_options_row2;

        for (size_t i = 0; i < num_options; i++) {
            if (state->current_position == options_counter) {
                screen_mem[screen_mem_pos++] = SELECTED_OPTION_CHAR;
            }
            options_counter++;
            size_t option_size = strlen(options[i]);
            memcpy(&screen_mem[screen_mem_pos], options[i], option_size);
            screen_mem_pos += option_size;
            if (i < num_options - 1) {
                screen_mem[screen_mem_pos++] = ' ';
            }
        }
    }
}

static void main_menu_handle_rotation(struct MenuScreen* screen, int delta) {
    struct MainMenuState* state = screen->state;
    if (delta > 0 && state->current_position < MAIN_MENU_ROW1_NUM_OPTIONS +
                                                   MAIN_MENU_ROW2_NUM_OPTIONS -
                                                   1) {
        state->current_position++;
    } else if (delta < 0 && state->current_position > 0) {
        state->current_position--;
    }
}

static void main_menu_handle_click(struct MenuScreen* screen) {
    struct MainMenuState* state = screen->state;
    if (state->current_position == MAIN_MENU_WIND) {
        if (!state->wind_menu_entered) {
            state->wind_menu_entered = true;
            struct StoredSettings settings;
            persistent_storage_get_stored_settings_or_defaults(&settings);
            wind_menu_state.config.speed_percent =
                settings.default_speed_percent;
        }
        current_menu_screen = &wind_menu_screen;
    } else if (state->current_position == MAIN_MENU_WIND_PRESET) {
        current_menu_screen = &wind_preset_menu_screen;
    } else if (state->current_position == MAIN_MENU_JOG) {
        output_screen_set_row(0, "Jog in progress");
        output_screen_set_row(1, "press to pause");
        state_switch_to_jog();
    } else if (state->current_position == MAIN_MENU_HOLD) {
        state_toggle_hold_motors_while_idle();
        menu_disable_encoder_click(250);
    } else if (state->current_position == MAIN_MENU_MEASURE_L) {
        current_menu_screen = &measure_inductance_menu_screen;
        measure_inductance_menu_state.is_on_main_subscreen = true;
        measure_inductance_menu_state.selected_auto = true;
    }
    current_menu_screen->prev = screen;
}

static void main_menu_handle_hold(struct MenuScreen* screen) {
    current_menu_screen->prev = screen;
    global_config_menu_state_init();
    current_menu_screen = &global_config_menu_screen;
}

static struct MainMenuState main_menu_state = {.current_position = 0,
                                               .wind_menu_entered = false};

static struct MenuScreen main_menu_screen = {
    .prev = NULL,
    .state = &main_menu_state,
    .render = main_menu_render,
    .handle_click = main_menu_handle_click,
    .handle_rotation = main_menu_handle_rotation,
    .handle_hold = main_menu_handle_hold};

// -------------------------------------------------------------------------------------------------

// Winding Paused Menu
// -------------------------------------------------------------------------------------------------

static void winding_paused_menu_render(struct MenuScreen* screen,
                                       char* screen_mem) {
    strcpy(screen_mem, "click to resume");
    memset(&screen_mem[NUM_COLS], ' ', NUM_COLS);
    strcpy(&screen_mem[NUM_COLS], "hold to stop");
}

static void winding_paused_menu_handle_rotation(struct MenuScreen* screen,
                                                int delta) {
    // noop
}

static void winding_paused_menu_handle_click(struct MenuScreen* screen) {
    state_switch_to_winding_resumed();
    menu_disable_encoder_click(250);
}

static void winding_paused_menu_handle_hold(struct MenuScreen* screen) {
    state_switch_to_idle();
    current_menu_screen = &main_menu_screen;
}

struct MenuScreen winding_paused_menu_screen = {
    .prev = NULL,
    .state = NULL,
    .render = winding_paused_menu_render,
    .handle_click = winding_paused_menu_handle_click,
    .handle_hold = winding_paused_menu_handle_hold,
    .handle_rotation = winding_paused_menu_handle_rotation};

// -------------------------------------------------------------------------------------------------

// Winding Error Menu
// -------------------------------------------------------------------------------------------------

struct WindingErrorMenuState {
    enum WindingErrorCode error_code;
    uint selected_stall_option;
};

static const char* winding_error_names[] = {
    "ILLEGAL", [WINDING_ERROR_GUIDE_POSITION_MISMATCH] = "GUIDE_POS",
    [WINDING_ERROR_MOTOR_STALLED] = "STALLED"};

static void winding_error_menu_render(struct MenuScreen* screen,
                                      char* screen_mem) {
    struct WindingErrorMenuState* state = screen->state;
    if (state->error_code >=
        sizeof(winding_error_names) / sizeof(winding_error_names[0])) {
        state->error_code = 0;
    }
    snprintf(screen_mem, 2 * NUM_COLS, "ERR=%s",
             winding_error_names[state->error_code]);
    if (state->error_code == WINDING_ERROR_GUIDE_POSITION_MISMATCH) {
        strcpy(&screen_mem[NUM_COLS], "chck motor sync");
    } else if (state->error_code == WINDING_ERROR_MOTOR_STALLED) {
        const char* options[] = {"resume", "w SD off"};
        render_menu_options(&screen_mem[NUM_COLS], options, 2,
                            state->selected_stall_option);
    }
}

static void winding_error_menu_handle_rotation(struct MenuScreen* screen,
                                               int delta) {
    struct WindingErrorMenuState* state = screen->state;
    if (state->error_code == WINDING_ERROR_MOTOR_STALLED) {
        if (delta > 0 && state->selected_stall_option == 0) {
            state->selected_stall_option++;
        } else if (delta < 0 && state->selected_stall_option > 0) {
            state->selected_stall_option--;
        }
    }
}

static void winding_error_menu_handle_click(struct MenuScreen* screen) {
    struct WindingErrorMenuState* state = screen->state;
    if (state->error_code == WINDING_ERROR_GUIDE_POSITION_MISMATCH) {
        state_switch_to_idle();
        current_menu_screen = &main_menu_screen;
    } else {
        if (state->selected_stall_option == 1) {
            state_disable_stall_detection();
        }
        state_switch_to_winding_resumed();
        menu_disable_encoder_click(250);
    }
}

static void winding_error_menu_handle_hold(struct MenuScreen* screen) {
    state_switch_to_idle();
    current_menu_screen = &main_menu_screen;
}

static struct WindingErrorMenuState winding_error_menu_state = {
    .error_code = 0, .selected_stall_option = 0};

static struct MenuScreen winding_error_menu_screen = {
    .prev = NULL,
    .state = &winding_error_menu_state,
    .render = winding_error_menu_render,
    .handle_click = winding_error_menu_handle_click,
    .handle_hold = winding_error_menu_handle_hold,
    .handle_rotation = winding_error_menu_handle_rotation};

// -------------------------------------------------------------------------------------------------

// Winding Finished Menu
// -------------------------------------------------------------------------------------------------

enum WindingFinishedMenuSubscreen {
    WINDING_FINISHED_MENU_MAIN = 0,
    WINDING_FINISHED_MENU_STATS,
    WINDING_FINISHED_MENU_EXTRA_TURNS
};

enum WindingFinishedMenuOption {
    WINDING_FINISHED_OPTION_OK = 0,
    WINDING_FINISHED_OPTION_STATS,
    WINDING_FINISHED_OPTION_EXTRA_TURNS
};

#define NUM_WINDING_FINISHED_MAIN_SUBSCREEN_OPTIONS 3
static const char* winding_finished_option_names[] = {"ok", "stats", "more"};

struct WindingFinishedMenuState {
    enum WindingFinishedMenuSubscreen current_subscreen;
    enum WindingFinishedMenuOption main_screen_selected_option;
    uint num_extra_turns;

    uint current_step;
    uint value_snapshot;
    uint value_snapshot_timestamp_ms;
    int last_rotation_direction;
};

static void winding_finished_menu_render(struct MenuScreen* screen,
                                         char* screen_mem) {
    struct WindingFinishedMenuState* state = screen->state;
    if (state->current_subscreen == WINDING_FINISHED_MENU_MAIN) {
        strcpy(screen_mem, "winding done");
        render_menu_options(&screen_mem[NUM_COLS],
                            winding_finished_option_names,
                            NUM_WINDING_FINISHED_MAIN_SUBSCREEN_OPTIONS,
                            state->main_screen_selected_option);
    } else if (state->current_subscreen == WINDING_FINISHED_MENU_STATS) {
        char* row1 = screen_mem;
        int sz = snprintf(row1, NUM_COLS + 1, "N=%u",
                          last_winding_config.number_of_turns);
        assert(sz >= 0 && sz < NUM_COLS - 3);
        row1[sz] = ' ';
        row1[sz + 1] = 'L';
        row1[sz + 2] = '=';
        pretty_render_microns_with_unit(
            &row1[sz + 3], (size_t)(NUM_COLS - sz - 3),
            last_winding_config.length_microns, true);
    } else {
        // WINDING_FINISHED_OPTION_EXTRA_TURNS
        strcpy(screen_mem, "Extra turns:");
        snprintf(&screen_mem[NUM_COLS], NUM_COLS, "%u sum=%u",
                 state->num_extra_turns,
                 state->num_extra_turns + last_winding_config.number_of_turns);
    }
}

static void winding_finished_menu_handle_rotation(struct MenuScreen* screen,
                                                  int delta) {
    struct WindingFinishedMenuState* state = screen->state;
    if (state->current_subscreen == WINDING_FINISHED_MENU_MAIN) {
        if (delta < 0 && state->main_screen_selected_option > 0) {
            state->main_screen_selected_option--;
        } else if (delta > 0 &&
                   state->main_screen_selected_option <
                       NUM_WINDING_FINISHED_MAIN_SUBSCREEN_OPTIONS - 1) {
            state->main_screen_selected_option++;
        }
    } else if (state->current_subscreen == WINDING_FINISHED_MENU_EXTRA_TURNS) {
        int direction = (delta > 0) - (delta < 0);
        if (direction != 0 && state->last_rotation_direction != 0 &&
            direction != state->last_rotation_direction) {
            state->value_snapshot = state->num_extra_turns;
            state->current_step = 1;
        }
        if (direction != 0) {
            state->last_rotation_direction = direction;
        }
        uint now_ms = to_ms_since_boot(get_absolute_time());

        if (now_ms - state->value_snapshot_timestamp_ms > SNAPSHOT_PERIOD_MS) {
            int cur_snapshot_delta =
                (int)state->num_extra_turns - (int)state->value_snapshot;
            if (cur_snapshot_delta == 0 ||
                now_ms - state->value_snapshot_timestamp_ms >
                    2 * SNAPSHOT_PERIOD_MS) {
                state->current_step =
                    wind_menu_base_step_values[WIND_MENU_NUMBER_OF_TURNS];
            } else if (abs(cur_snapshot_delta) > 4 * state->current_step) {
                if (state->current_step >
                    wind_menu_base_step_values[WIND_MENU_NUMBER_OF_TURNS] *
                        10) {
                    state->current_step *= 2;
                } else {
                    state->current_step *= 10;
                }
                state->current_step =
                    MIN(state->current_step,
                        wind_menu_max_step[WIND_MENU_NUMBER_OF_TURNS]);
            }
        }
        int actual_delta = delta * (int)state->current_step;
        state->num_extra_turns =
            rotation_clip_uint(state->num_extra_turns, actual_delta, 1, 0);
        if (now_ms - state->value_snapshot_timestamp_ms > SNAPSHOT_PERIOD_MS) {
            state->value_snapshot = state->num_extra_turns;
            state->value_snapshot_timestamp_ms = now_ms;
        }
    }
}

static void winding_finished_menu_handle_click(struct MenuScreen* screen) {
    struct WindingFinishedMenuState* state = screen->state;
    if (state->current_subscreen == WINDING_FINISHED_MENU_MAIN) {
        if (state->main_screen_selected_option == WINDING_FINISHED_OPTION_OK) {
            current_menu_screen = &main_menu_screen;
            state_switch_to_idle();
        } else if (state->main_screen_selected_option ==
                   WINDING_FINISHED_OPTION_STATS) {
            state->current_subscreen = WINDING_FINISHED_MENU_STATS;
        } else {
            state->current_subscreen = WINDING_FINISHED_MENU_EXTRA_TURNS;
            state->current_step = 1;
            state->num_extra_turns = 1;
            state->value_snapshot_timestamp_ms =
                to_ms_since_boot(get_absolute_time());
            state->value_snapshot = state->num_extra_turns;
        }
    } else if (state->current_subscreen == WINDING_FINISHED_MENU_STATS) {
        state->current_subscreen = WINDING_FINISHED_MENU_MAIN;
    } else {
        last_winding_config.number_of_turns += state->num_extra_turns;
        state_switch_to_continue_winding_after_finished(state->num_extra_turns);
    }
}

static void winding_finished_menu_handle_hold(struct MenuScreen* screen) {
    struct WindingFinishedMenuState* state = screen->state;
    if (state->current_subscreen != WINDING_FINISHED_MENU_MAIN) {
        state->current_subscreen = WINDING_FINISHED_MENU_MAIN;
    }
}

static struct WindingFinishedMenuState winding_finished_menu_state = {
    .current_step = 1,
    .current_subscreen = WINDING_FINISHED_MENU_MAIN,
    .main_screen_selected_option = 0,
    .num_extra_turns = 1,
    .value_snapshot = 0,
    .value_snapshot_timestamp_ms = 0};

static struct MenuScreen winding_finished_menu_screen = {
    .prev = NULL,
    .state = &winding_finished_menu_state,
    .render = winding_finished_menu_render,
    .handle_click = winding_finished_menu_handle_click,
    .handle_hold = winding_finished_menu_handle_hold,
    .handle_rotation = winding_finished_menu_handle_rotation};

// -------------------------------------------------------------------------------------------------

void menu_init(struct LCD* lcd_screen, struct RotaryEncoder* encoder) {
    assert(lcd_screen->rows * lcd_screen->cols >= SCREEN_BUFF_SIZE);
    output_screen = lcd_screen;
    rotary_encoder = encoder;
    memset(prev_screen_buff, ' ', SCREEN_BUFF_SIZE);
    memset(screen_buff, ' ', SCREEN_BUFF_SIZE);
    wind_menu_reset_snapshot_state(wind_menu_screen.state);
    current_menu_screen = &main_menu_screen;
}

static void menu_begin_winding(struct WindingConfig config) {
    last_winding_config = config;
    state_switch_to_winding(config);
    current_menu_screen = &main_menu_screen;
}

void menu_next_frame(void) {
    static uint last_press_state_change_ms = 0;
    static bool holding_button = false;
    static bool waiting_for_release_after_hold = false;
    static uint last_render_ms = 0;

    uint now_ms = to_ms_since_boot(get_absolute_time());

    if (now_ms - last_render_ms < MENU_REFRESH_PERIOD_MS) {
        return;
    }
    last_render_ms = now_ms;

    memset(screen_buff, ' ', SCREEN_BUFF_SIZE);
    current_menu_screen->render(current_menu_screen, screen_buff);
    for (size_t i = 0; i < SCREEN_BUFF_SIZE; i++) {
        if (screen_buff[i] == '\0') {
            screen_buff[i] = ' ';
        }
    }
    for (size_t i = 0; i < NUM_ROWS; i++) {
        if (memcmp(&screen_buff[i * NUM_COLS], &prev_screen_buff[i * NUM_COLS],
                   NUM_COLS) != 0) {
            lcd_i2c_clear_row_fast(output_screen, ' ', (uint8_t)i);
            lcd_i2c_set_cursor(output_screen, 0, (uint8_t)i);
            memcpy(&prev_screen_buff[i * NUM_COLS], &screen_buff[i * NUM_COLS],
                   NUM_COLS);
            for (size_t j = 0; j < NUM_COLS; j++) {
                lcd_i2c_putc(output_screen, screen_buff[i * NUM_COLS + j]);
            }
        }
    }

    if (is_encoder_click_disabled) {
        if (now_ms < encoder_click_disabled_end_timestamp_ms) {
            return;
        }
        is_encoder_click_disabled = true;
    }

    if (!holding_button) {
        int delta = rotary_encoder_get_and_reset_delta(rotary_encoder);
        if (delta != 0) {
            current_menu_screen->handle_rotation(current_menu_screen, delta);
        } else if (now_ms - last_press_state_change_ms >
                       ENCODER_PRESS_DEBOUNCE_MS &&
                   rotary_encoder_is_button_pressed(rotary_encoder)) {
            last_press_state_change_ms = now_ms;
            holding_button = true;
        }
    } else {
        if (waiting_for_release_after_hold) {
            if (!rotary_encoder_is_button_pressed(rotary_encoder)) {
                holding_button = false;
                last_press_state_change_ms = now_ms;
                waiting_for_release_after_hold = false;
            }
        } else if (now_ms - last_press_state_change_ms >
                   ENCODER_PRESS_DEBOUNCE_MS) {
            if (now_ms - last_press_state_change_ms >
                ENCODER_PRESS_MIN_HOLD_DURATION_MS) {
                if (current_menu_screen->handle_hold != NULL) {
                    current_menu_screen->handle_hold(current_menu_screen);
                }
                waiting_for_release_after_hold = true;
            } else if (!rotary_encoder_is_button_pressed(rotary_encoder)) {
                current_menu_screen->handle_click(current_menu_screen);
                holding_button = false;
                last_press_state_change_ms = now_ms;
            }
        }
    }
}

void menu_show_jog_finished_and_wait_for_confirm(void) {
    output_screen_set_row(0, "jog finished");
    output_screen_set_row(1, "");
    while (true) {
        if (rotary_encoder_is_button_pressed(rotary_encoder)) {
            state_switch_to_idle();
            current_menu_screen = &main_menu_screen;
            menu_disable_encoder_click(250);
            return;
        }
        sleep_ms(5);
    }
}

void menu_jog_paused(void) { menu_disable_encoder_click(250); }

void menu_show_winding_finished_screen(void) {
    winding_finished_menu_state.current_subscreen = WINDING_FINISHED_MENU_MAIN;
    winding_finished_menu_state.main_screen_selected_option = 0;
    current_menu_screen = &winding_finished_menu_screen;
    menu_disable_encoder_click(500);
}

void menu_show_winding_paused_screen(void) {
    current_menu_screen = &winding_paused_menu_screen;
    menu_disable_encoder_click(500);
}

void menu_show_winding_error_screen(enum WindingErrorCode error_code) {
    winding_error_menu_state.error_code = error_code;
    winding_error_menu_state.selected_stall_option = 0;
    current_menu_screen = &winding_error_menu_screen;
}

void menu_show_winding_progress(int guide_position_microns,
                                uint num_current_turns, uint num_total_turns,
                                int speed_steps_delta) {
    char row[NUM_COLS + 1];
    memset(row, 0, sizeof(row));
    if (speed_steps_delta == 0) {
        snprintf(row, sizeof(row), "Winding N=%u", num_total_turns);
    } else {
        snprintf(row, sizeof(row), "Wind N=%ud%s%d", num_total_turns,
                 speed_steps_delta > 0 ? "+" : "-", abs(speed_steps_delta));
    }
    if (memcmp(row, prev_screen_buff, NUM_COLS) != 0) {
        output_screen_set_row(0, row);
    }
    memset(row, 0, sizeof(row));
    snprintf(row, sizeof(row), "x=%d.%umm N=%u", guide_position_microns / 1000,
             ((uint)guide_position_microns % 1000) / 100, num_current_turns);
    if (memcmp(row, &prev_screen_buff[NUM_COLS], NUM_COLS) != 0) {
        output_screen_set_row(1, row);
    }
}

void menu_show_inductance_measurement_finished_screen(
    struct InductanceMeasurementResults results) {
    current_menu_screen = &inductance_measurement_finished_menu_screen;
    inductance_measurement_finished_menu_state.results = results;
    inductance_measurement_finished_menu_state.cur_subscreen =
        IMF_SUB_SCREEN_MAIN;
    inductance_measurement_finished_menu_state.main_screen_selected_option = 0;
    inductance_measurement_finished_menu_state.details_screen_cur_option = 0;
    inductance_measurement_finished_menu_state.details_show_filtered_value =
        results.show_filtered_by_default;
    menu_disable_encoder_click(500);
}

static void switch_to_previous_menu_screen(void) {
    if (current_menu_screen != NULL) {
        if (current_menu_screen->prev != NULL) {
            current_menu_screen = current_menu_screen->prev;
        } else {
            current_menu_screen = &main_menu_screen;
        }
    }
}
