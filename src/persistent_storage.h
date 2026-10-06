#pragma once

#include "pico/stdlib.h"

#define MIN_STALL_THRESHOLD_PERCENT 0
#define MAX_STALL_THRESHOLD_PERCENT 90
#define MIN_DEFAULT_SPEED_PERCENT 10
#define MAX_DEFAULT_SPEED_PERCENT 150

struct StoredSettings {
    uint32_t stall_threshold_percent;
    uint32_t default_speed_percent;
};

void persistent_storage_get_stored_settings_or_defaults(
    struct StoredSettings* settings);
void persistent_storage_save_stored_settings(struct StoredSettings settings);
void persistent_storage_get_default_settings(struct StoredSettings* settings);
