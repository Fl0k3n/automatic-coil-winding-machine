#include "tmc2209_driver.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/uart.h"
#include "pico/stdlib.h"

#define TMC2209_REG_SG_RESULT 0x41

// TMC2209 UART CRC.
// Same algorithm used by the janelia-arduino/TMC2209 library.
static uint8_t tmc2209_crc(const uint8_t* data, size_t len) {
    uint8_t crc = 0;

    for (size_t i = 0; i < len; ++i) {
        uint8_t byte = data[i];

        for (int bit = 0; bit < 8; ++bit) {
            if (((crc >> 7) ^ (byte & 1)) != 0)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;

            byte >>= 1;
        }
    }

    return crc;
}

// Write one TMC2209 register.
static void tmc2209_write(uart_inst_t* uart, uint8_t slave_addr, uint8_t reg,
                          uint32_t value) {
    uint8_t frame[8];

    frame[0] = 0x05;  // sync
    frame[1] = slave_addr;
    frame[2] = reg | 0x80;  // bit 7 = write

    // TMC2209 sends register data MSB first
    frame[3] = (uint8_t)(value >> 24);
    frame[4] = (uint8_t)(value >> 16);
    frame[5] = (uint8_t)(value >> 8);
    frame[6] = (uint8_t)(value >> 0);

    frame[7] = tmc2209_crc(frame, 7);

    uart_write_blocking(uart, frame, sizeof(frame));
    uart_tx_wait_blocking(uart);
}

bool tmc2209_read(uart_inst_t* uart, uint8_t slave_addr, uint8_t reg,
                  uint32_t* value) {
    uint8_t request[4];

    request[0] = 0x05;
    request[1] = slave_addr;
    request[2] = reg & 0x7F;
    request[3] = tmc2209_crc(request, 3);

    // Remove anything left in RX FIFO.
    while (uart_is_readable(uart)) uart_getc(uart);

    // Send read request.
    uart_write_blocking(uart, request, sizeof(request));
    uart_tx_wait_blocking(uart);

    /*
     * Because TX and RX are connected to the same single-wire UART bus,
     * we'll receive our own 4 transmitted bytes first.
     */
    for (int i = 0; i < 4; i++) {
        uint64_t start = time_us_64();

        while (!uart_is_readable(uart)) {
            if (time_us_64() - start > 5000) {
                // printf("FAILED EARLY\n");
                return false;
            }
        }

        (void)uart_getc(uart);  // discard echo
    }

    // Now receive the TMC2209's 8-byte reply.
    uint8_t reply[8];

    for (int i = 0; i < 8; i++) {
        uint64_t start = time_us_64();

        while (!uart_is_readable(uart)) {
            if (time_us_64() - start > 5000) {
                // printf("FAILED LATE\n");
                return false;
            }
        }

        reply[i] = uart_getc(uart);
    }

    // Expected reply:
    //
    // 05 FF REG DATA3 DATA2 DATA1 DATA0 CRC

    bool ok = true;

    if (reply[0] != 0x05) ok = false;

    if (reply[1] != 0xFF) ok = false;

    if ((reply[2] & 0x7F) != (reg & 0x7F)) ok = false;

    if (tmc2209_crc(reply, 7) != reply[7]) ok = false;

    if (!ok) {
        // printf("read failed\n");
        // for (size_t i = 0; i < 8; i++) {
        //     printf("%X", reply[i]);
        // }
        // printf("\n");
        return false;
    }

    *value = ((uint32_t)reply[3] << 24) | ((uint32_t)reply[4] << 16) |
             ((uint32_t)reply[5] << 8) | ((uint32_t)reply[6] << 0);

    return true;
}

// static void txn_test(uart_inst_t* uart) {
//     uint8_t TMC2209_ADDR = 1;  // MS1 = 1, MS2 = 0
//     uint32_t before, after;

//     if (!tmc2209_read(uart, TMC2209_ADDR, 0x02, &before)) {
//         printf("Cannot read TMC2209\n");
//         return;
//     }

//     // Perform one write
//     tmc2209_write(uart, TMC2209_ADDR, 0x00, 0x000001C1);

//     if (!tmc2209_read(uart, TMC2209_ADDR, 0x02, &after)) {
//         printf("Cannot read TMC2209 after write\n");
//         return;
//     }

//     printf("IFCNT before: %u\n", (unsigned)(before & 0xff));
//     printf("IFCNT after:  %u\n", (unsigned)(after & 0xff));

//     uint32_t gconf;

//     if (tmc2209_read(uart, TMC2209_ADDR, 0x00, &gconf))
//         printf("GCONF = 0x%08lx\n", gconf);

//     printf("done");
// }

void tmc2209_uart_init(uart_inst_t* uart, uint tx_pin, uint rx_pin, uint baud) {
    gpio_set_function(tx_pin, UART_FUNCSEL_NUM(uart, tx_pin));
    gpio_set_function(rx_pin, UART_FUNCSEL_NUM(uart, rx_pin));

    uart_init(uart, baud);
    uart_set_format(uart, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(uart, false, false);

    sleep_ms(10);
    // txn_test(uart);
}

// Switch to UART-controlled microstep resolution
// and configure STEP input for full-step operation.
void tmc2209_set_fullstep(uart_inst_t* uart, uint8_t addr) {
    /*
     * GCONF, address 0x00
     * bit 0: scale analog     = 1
     * bit 6: pdn_disable      = 1
     * bit 7: mstep_reg_select = 1
     * bit 8: multistep_filt   = 1
     *
     * This is exactly what the Janelia library's
     * setOperationModeToSerial() constructs.
     */
    tmc2209_write(uart, addr, 0x00,
                  (1u << 0) | (1u << 6) | (1u << 7) | (1u << 8));

    /*
     * CHOPCONF, address 0x6C.
     *
     * Janelia default:
     *     0x10000053
     *
     * MRES is bits 27:24.
     *
     * MRES = 0b1000 => FULLSTEP
     *
     * Therefore:
     *     0x10000053
     *   | 0x08000000
     *   = 0x18000053
     */
    tmc2209_write(uart, addr, 0x6C, 0x18000053);
}

bool tmc2209_get_stall_guard_result(uart_inst_t* uart, uint8_t addr,
                                    uint16_t* result) {
    uint32_t v = 0;
    if (!tmc2209_read(uart, addr, TMC2209_REG_SG_RESULT, &v)) {
        return false;
    }
    *result = (uint16_t)(v & 0x3ff);
    return true;
}
