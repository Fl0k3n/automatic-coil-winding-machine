#include <stddef.h>
#include <stdint.h>

#include "hardware/uart.h"
#include "pico/stdlib.h"

void tmc2209_uart_init(uart_inst_t* uart, uint tx_pin, uint rx_pin, uint baud);

void tmc2209_set_fullstep(uart_inst_t* uart, uint8_t addr);