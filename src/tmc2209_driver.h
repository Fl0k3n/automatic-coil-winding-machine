#include <stddef.h>
#include <stdint.h>

#include "hardware/uart.h"
#include "pico/stdlib.h"

void tmc2209_uart_init(uart_inst_t* uart, uint tx_pin, uint rx_pin, uint baud);

void tmc2209_set_fullstep(uart_inst_t* uart, uint8_t addr);

// measuring motor, 10b value, result low => high motor load, result high => low
// motor load
bool tmc2209_get_stall_guard_result(uart_inst_t* uart, uint8_t addr,
                                    uint16_t* result);
