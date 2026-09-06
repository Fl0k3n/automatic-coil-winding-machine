#include "rotary_encoder_driver.h"

#include "hardware/sync.h"

#define KEY_DEBOUNCE_MS 150

void rotary_encoder_init(struct RotaryEncoder* re, uint s1_pin, uint s2_pin,
                         uint key_pin) {
    re->key_pin = key_pin;
    re->s1_pin = s1_pin;
    re->s2_pin = s2_pin;

    gpio_init(re->s1_pin);
    gpio_set_dir(re->s1_pin, GPIO_IN);
    gpio_pull_up(re->s1_pin);
    gpio_init(re->s2_pin);
    gpio_set_dir(re->s2_pin, GPIO_IN);
    gpio_pull_up(re->s2_pin);

    gpio_init(re->key_pin);
    gpio_set_dir(re->key_pin, GPIO_IN);
    gpio_pull_up(re->key_pin);

    re->delta = 0;
    re->transition_accum = 0;
    re->key_press_timestamp_ms = 0;
    re->last_state = (gpio_get(re->s1_pin) << 1) | gpio_get(re->s2_pin);
    re->key_pressed = false;
    if (!gpio_get(re->key_pin)) {
        re->key_pressed = true;
        re->key_press_timestamp_ms = to_ms_since_boot(get_absolute_time());
    }
}

static const int8_t quad_table[16] = {0,  -1, 1, 0, 1, 0, 0,  -1,
                                      -1, 0,  0, 1, 0, 1, -1, 0};

void rotary_encoder_handle_gpio_irq(struct RotaryEncoder* re, uint gpio,
                                    uint32_t events) {
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    if (gpio == re->key_pin) {
        if (now_ms - re->key_press_timestamp_ms >= KEY_DEBOUNCE_MS) {
            bool pressed = !gpio_get(re->key_pin);
            re->key_pressed = pressed;
            if (pressed) {
                re->key_press_timestamp_ms = now_ms;
            }
        }
        return;
    }

    if (gpio != re->s1_pin && gpio != re->s2_pin) {
        return;
    }

    uint8_t s1 = gpio_get(re->s1_pin);
    uint8_t s2 = gpio_get(re->s2_pin);

    uint8_t state = (s1 << 1) | s2;
    uint8_t transition = (re->last_state << 2) | state;

    re->transition_accum += quad_table[transition];
    if (re->transition_accum >= 4) {
        re->delta++;
        re->transition_accum -= 4;
    } else if (re->transition_accum <= -4) {
        re->delta--;
        re->transition_accum += 4;
    }
    re->last_state = state;
}

int rotary_encoder_get_and_reset_delta(struct RotaryEncoder* re) {
    uint32_t irq_save = save_and_disable_interrupts();
    int delta = re->delta;
    re->delta = 0;
    restore_interrupts(irq_save);
    return delta;
}

bool rotary_encoder_get_button_state(struct RotaryEncoder* re,
                                     uint32_t* press_timestamp_ms) {
    bool pressed = !gpio_get(re->key_pin);
    *press_timestamp_ms = 0;
    uint32_t irq_save = save_and_disable_interrupts();
    if (pressed && re->key_pressed) {
        *press_timestamp_ms = re->key_press_timestamp_ms;
    }
    restore_interrupts(irq_save);
    return pressed;
}

bool rotary_encoder_is_button_pressed(struct RotaryEncoder* re) {
    return !gpio_get(re->key_pin);
}
