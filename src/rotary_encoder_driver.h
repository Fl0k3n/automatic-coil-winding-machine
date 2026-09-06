#pragma once

#include "pico/stdlib.h"

struct RotaryEncoder {
    uint s1_pin, s2_pin, key_pin;

    volatile int delta;
    volatile bool key_pressed;
    volatile uint32_t key_press_timestamp_ms;

    int8_t transition_accum;
    uint8_t last_state;
};

void rotary_encoder_init(struct RotaryEncoder* re, uint s1_pin, uint s2_pin,
                         uint key_pin);

void rotary_encoder_handle_gpio_irq(struct RotaryEncoder* re, uint gpio,
                                    uint32_t events);

static inline void rotary_encoder_reset(struct RotaryEncoder* re) {
    re->delta = 0;
}

int rotary_encoder_get_and_reset_delta(struct RotaryEncoder* re);

bool rotary_encoder_get_button_state(struct RotaryEncoder* re,
                                     uint32_t* press_timestamp_ms);

bool rotary_encoder_is_button_pressed(struct RotaryEncoder* re);
