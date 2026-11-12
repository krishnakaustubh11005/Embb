/* Pico-only clock, GPIO, ADC and display boundary. */
#ifndef CONSOLE_BOARD_H
#define CONSOLE_BOARD_H
#include <stdint.h>
#include "platform/input.h"
/** Configure clocks, USB, switch controls, ADC, buttons and display; reset blocks. */
void board_init(void);
/** Sample individually switched X and direct Y axes; both switches finish off. */
void board_read_axes(uint16_t axes[2][2]);
/** Launch the 1 kHz button sampler and debouncer on core 1. */
void board_start_button_sampler(void);
/** Collect held buttons and all press/release events latched since the last call. */
void board_collect_buttons(ButtonEvents *events);
/** Measured average µs of one muxed X read and one direct Y read; resets them. */
void board_axis_timing(unsigned int *x_us, unsigned int *y_us);
/** Running total of µs core 1 spent working (excludes its idle waiting). */
uint32_t board_core1_busy_us(void);
/** Start sending a finished frame by DMA; waits only for the previous frame. */
void board_present(const uint16_t *pixels);
/** Read the divider-selected SPI rate, not an external electrical measurement. */
unsigned int board_spi_hz(void);
#endif
