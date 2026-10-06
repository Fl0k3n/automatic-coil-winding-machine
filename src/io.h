#pragma once

#include "lcd_driver.h"
#include "pico/stdlib.h"
#include "rotary_encoder_driver.h"
#include "state.h"

/*

options:
1. async buttons, inactive during winding:
    - coil motor wind  X
    - coil motor unwind X
    - screw motor left X
    - screw motor right X
    - encoder press during winding -> stop, when not winding encoder controls
menu
2. menu settings:
    a) main level:
        - wind_p (winding using hardcoded presets for convenience) -> press
shows preset config then Y/N confirmation, on Y starts
        - wind -> passes to b)
        - jog -> moves screw motor to hit homing button
        - measure L (later)
        - unwind (maybe later)
    b) wind level, each config option after another, defaults most common.
encoder hold goes back to main menu
        - wire diameter -> 0.5, 0.25, manual, on press to manual, word manual is
changed to value and encoder controls delta
        - length -> 18, 30, manual, same as above
        - number of turns -> manual, 100, 500, same as above
        - speed -> default, ?
        after inputting all that menu will show selection and ask for
confirmation

- during winding menu updates progress periodically, X/total where X is current
number of turns
*/

void menu_init(struct LCD* lcd_screen, struct RotaryEncoder* encoder);
void menu_next_frame(void);
void menu_show_jog_finished_and_wait_for_confirm(void);
void menu_jog_paused(void);
void menu_show_winding_finished_screen(void);
void menu_show_winding_paused_screen(void);
void menu_show_winding_error_screen(enum WindingErrorCode error_code);
void menu_show_winding_progress(int guide_position_microns,
                                uint num_current_turns, uint num_total_turns,
                                int speed_steps_delta);
void menu_show_inductance_measurement_finished_screen(
    struct InductanceMeasurementResults results);