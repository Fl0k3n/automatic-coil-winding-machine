#include "persistent_storage.h"

#include <string.h>

#include "hardware/flash.h"
#include "pico/flash.h"

#define PERSISTENT_DATA_MAGIC 0x1234ABCDu
#define PERSISTENT_DATA_VERSION 1

#define STORAGE_SIZE PERSISTENT_FLASH_SIZE
#define STORAGE_OFFSET (PICO_FLASH_SIZE_BYTES - STORAGE_SIZE)

static inline const uint8_t* storage_read_base(void) {
    return (const uint8_t*)(XIP_BASE + STORAGE_OFFSET);
}

struct PersistentData {
    uint32_t magic;
    uint32_t version;

    struct StoredSettings stored_settings;
};

static const struct StoredSettings default_settings = {
    .default_speed_percent = 80, .stall_threshold_percent = 75};

static bool tried_reading_settings = false;
static bool successfuly_read_settings = false;
static struct PersistentData persistent_data;

void persistent_storage_get_stored_settings_or_defaults(
    struct StoredSettings* settings) {
    if (!tried_reading_settings) {
        tried_reading_settings = true;

        memcpy(&persistent_data, storage_read_base(), sizeof(persistent_data));

        if (persistent_data.magic == PERSISTENT_DATA_MAGIC &&
            persistent_data.version == PERSISTENT_DATA_VERSION) {
            persistent_data.stored_settings.default_speed_percent =
                MIN(MAX(persistent_data.stored_settings.default_speed_percent,
                        MIN_DEFAULT_SPEED_PERCENT),
                    MAX_DEFAULT_SPEED_PERCENT);
            persistent_data.stored_settings.stall_threshold_percent =
                MIN(MAX(persistent_data.stored_settings.stall_threshold_percent,
                        MIN_STALL_THRESHOLD_PERCENT),
                    MAX_STALL_THRESHOLD_PERCENT);
            successfuly_read_settings = true;
        } else {
            successfuly_read_settings = false;
        }
    }
    *settings = successfuly_read_settings ? persistent_data.stored_settings
                                          : default_settings;
}

void persistent_storage_get_default_settings(struct StoredSettings* settings) {
    *settings = default_settings;
}

static void do_flash_write_multicore_safe(void* arg) {
    // safe as long as core1 calls flash_safe_execute_core_init
    const uint8_t* page = arg;
    flash_range_erase(STORAGE_OFFSET, STORAGE_SIZE);
    flash_range_program(STORAGE_OFFSET, page, FLASH_PAGE_SIZE);
}

void persistent_storage_save_stored_settings(struct StoredSettings settings) {
    if (tried_reading_settings && successfuly_read_settings &&
        memcmp(&settings, &persistent_data.stored_settings, sizeof(settings)) ==
            0) {
        return;
    }
    uint8_t flash_page[FLASH_PAGE_SIZE];
    memset(flash_page, 0xFF, sizeof(flash_page));

    // assumed to be valid
    persistent_data.stored_settings = settings;
    persistent_data.magic = PERSISTENT_DATA_MAGIC;
    persistent_data.version = PERSISTENT_DATA_VERSION;

    static_assert(sizeof(persistent_data) <= sizeof(flash_page));
    memcpy(flash_page, &persistent_data, sizeof(persistent_data));

    int res = flash_safe_execute(do_flash_write_multicore_safe,
                                 (void*)flash_page, UINT32_MAX);
    hard_assert(res == PICO_OK);

    tried_reading_settings = true;
    successfuly_read_settings = true;
}
