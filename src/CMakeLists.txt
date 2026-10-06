# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()

set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)

set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(coil_winder C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()


# -----------------------------------------------------------------------------
# Application sources
# -----------------------------------------------------------------------------

set(APP_SOURCES
    tmc2209_driver.c
    lcd_driver.c
    rotary_encoder_driver.c
    persistent_storage.c
    winder.c
    io.c
    inductance_meter.c
    controller.c
    main.c
)

add_executable(coil_winder
    ${APP_SOURCES}
)

# -----------------------------------------------------------------------------
# Persistent flash storage
# -----------------------------------------------------------------------------

set(PERSISTENT_FLASH_SIZE 4096)

# PICO_FLASH_SIZE_BYTES comes from the pico_w board definition and is the
# actual physical flash size (2 MiB).
math(EXPR APP_FLASH_SIZE
    "${PICO_FLASH_SIZE_BYTES} - ${PERSISTENT_FLASH_SIZE}"
)

set(LINKER_OVERRIDE_DIR
    "${CMAKE_CURRENT_BINARY_DIR}/linker_overrides"
)

file(MAKE_DIRECTORY "${LINKER_OVERRIDE_DIR}")

# the purpose of this is only a sanity assert that linker doesn't put anything in the memory we use for persistent storage
file(WRITE
    "${LINKER_OVERRIDE_DIR}/pico_flash_region.ld"
    "FLASH(rx) : ORIGIN = 0x10000000, LENGTH = ${APP_FLASH_SIZE}\n"
)

pico_add_linker_script_override_path(
    coil_winder
    "${LINKER_OVERRIDE_DIR}"
    FILES pico_flash_region.ld
)

target_compile_definitions(coil_winder PRIVATE
    PERSISTENT_FLASH_SIZE=${PERSISTENT_FLASH_SIZE}
)


# -----------------------------------------------------------------------------
# Strict warnings — ONLY for our source files
# -----------------------------------------------------------------------------

set(APP_WARNING_FLAGS
    -Wall
    -Wextra
    -Werror

    -Wconversion
    -Wsign-conversion
    -Wfloat-conversion
    -Wdouble-promotion

    -Wshadow
    -Wundef

    -Wformat=2
    -Wformat-overflow=2
    -Wformat-truncation=2

    -Wcast-qual
    -Wcast-align=strict
    -Wcast-function-type
    -Wwrite-strings

    -Wstrict-prototypes
    -Wold-style-definition

    -Wswitch-enum
    -Wswitch-default
    -Wimplicit-fallthrough=5

    -Wenum-conversion
    -Wenum-compare

    -Wduplicated-cond
    -Wduplicated-branches
    -Wlogical-op
    -Wnull-dereference

    -Wframe-larger-than=512
    -Wstack-usage=1024

    -Wvla
    -Walloca

    # Intentional exceptions
    -Wno-unused-parameter
    -Wno-pointer-arith
    -Wno-format-truncation
)

set_source_files_properties(
    ${APP_SOURCES}
    PROPERTIES
        COMPILE_OPTIONS "${APP_WARNING_FLAGS}"
)


# -----------------------------------------------------------------------------
# Pico executable configuration
# -----------------------------------------------------------------------------

pico_set_program_name(coil_winder "coil_winder")
pico_set_program_version(coil_winder "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(coil_winder 0)
pico_enable_stdio_usb(coil_winder 1)


# -----------------------------------------------------------------------------
# Includes
# -----------------------------------------------------------------------------

target_include_directories(coil_winder PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)


# -----------------------------------------------------------------------------
# Libraries
# -----------------------------------------------------------------------------

target_link_libraries(coil_winder
    pico_stdlib

    hardware_uart
    hardware_i2c
    hardware_pio
    hardware_flash

    pico_multicore
    pico_sync
    pico_flash
)


# -----------------------------------------------------------------------------
# PIO
# -----------------------------------------------------------------------------

pico_generate_pio_header(
    coil_winder
    ${CMAKE_CURRENT_LIST_DIR}/freq_meter.pio
)


# Generate .uf2, .bin, .hex, etc.
pico_add_extra_outputs(coil_winder)