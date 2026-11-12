/* Pico hardware boundary with isolated, sequential shared-X ADC sampling. */
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/sync.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/regs/clocks.h"
#include "hardware/spi.h"
#include "config.h"
#include "platform/board.h"
#include "platform/st7789.h"
static const unsigned int pins[2][5] = {P1_BUTTON_PINS, P2_BUTTON_PINS};

/** Break both shared-X paths before enabling P1 (0), P2 (1), or neither (-1).
 * @note IC controls 13/5 connect to GP14/15; both are never high together.
 */
static void select_x_axis(int player) {
    gpio_put(P1_X_ENABLE_PIN, 0);
    gpio_put(P2_X_ENABLE_PIN, 0);
    if (player < 0) return;
    busy_wait_us_32(MUX_BREAK_US);
    gpio_put(player == 0 ? P1_X_ENABLE_PIN : P2_X_ENABLE_PIN, 1);
}

/** Select ADC channel 0/1/2, wait settle_us microseconds, discard stale
 * conversions, then return an averaged raw 12-bit value (0..4095).
 * @note Settling happens after ADC selection, not just after mux selection.
 */
static uint16_t sample_adc(unsigned int channel, uint32_t settle_us) {
    adc_select_input(channel);
    busy_wait_us_32(settle_us);
    for (unsigned int i = 0; i < ADC_DISCARD_SAMPLES; i++) (void)adc_read();
    uint32_t sum = 0;
    for (unsigned int i = 0; i < ADC_AVERAGE_SAMPLES; i++) sum += adc_read();
    return (uint16_t)(sum / ADC_AVERAGE_SAMPLES);
}

/** Read player 0/1 X as raw 0..4095 after break-before-make and settling.
 * @note Only the selected joystick drives ADC2; both switches finish off.
 */
static uint32_t x_read_us, x_reads, y_read_us, y_reads;   // Measured axis timing (core 0).

static uint16_t sample_x_axis(unsigned int player) {
    uint32_t start = time_us_32();
    select_x_axis((int)player);
    uint16_t value = sample_adc(SHARED_X_ADC, MUX_SETTLE_US);
    select_x_axis(-1);
    x_read_us += time_us_32() - start; x_reads++;
    return value;
}

/** Read and time one direct (unswitched) Y axis. */
static uint16_t sample_y_axis(unsigned int channel) {
    uint32_t start = time_us_32();
    uint16_t value = sample_adc(channel, ADC_SETTLE_US);
    y_read_us += time_us_32() - start; y_reads++;
    return value;
}

/** Average measured time of one muxed X read and one direct Y read since the
 * last call, in microseconds; resets the averages. */
void board_axis_timing(unsigned int *x_us, unsigned int *y_us) {
    *x_us = x_reads ? x_read_us / x_reads : 0;
    *y_us = y_reads ? y_read_us / y_reads : 0;
    x_read_us = x_reads = y_read_us = y_reads = 0;
}

/** Initialize 125 MHz CPU/peripheral, USB, ADC, buttons and the ST7789.
 * @note Preloads switch outputs low. Display startup includes blocking waits.
 */
void board_init(void) {
    set_sys_clock_khz(CONSOLE_CPU_KHZ, true);
    clock_configure_undivided(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                             clock_get_hz(clk_sys));
    stdio_init_all();
    const unsigned int controls[] = {P1_X_ENABLE_PIN, P2_X_ENABLE_PIN};
    for (unsigned int i = 0; i < 2; i++) {
        gpio_init(controls[i]);
        gpio_put(controls[i], 0);
        gpio_set_dir(controls[i], GPIO_OUT);
    }
    for (unsigned int p = 0; p < 2; p++) {
        for (unsigned int bit = 0; bit < 5; bit++) {
            gpio_init(pins[p][bit]);
            gpio_set_dir(pins[p][bit], GPIO_IN);
            gpio_pull_up(pins[p][bit]);
        }
    }
    adc_init();
    adc_gpio_init(SHARED_X_ADC_PIN);
    adc_gpio_init(P1_Y_PIN);
    adc_gpio_init(P2_Y_PIN);
    st7789_init();
}

/** Sample the four joystick axes sequentially into a caller-owned array.
 * @note Includes bounded ADC/settling waits (about 1.1 ms). Samples are not
 *       simultaneous. Leaves both shared-X controls low. Buttons are sampled
 *       separately at 1 kHz on core 1.
 */
void board_read_axes(uint16_t axes[2][2]) {
    axes[0][0] = sample_x_axis(0);
    axes[0][1] = sample_y_axis(P1_Y_ADC);
    axes[1][1] = sample_y_axis(P2_Y_ADC);
    axes[1][0] = sample_x_axis(1);
}

/** Wait for the previous frame's DMA to finish, then start sending this one.
 * @note Returns as soon as the new transfer is running; core 0 then draws the
 *       next frame into the other buffer while DMA feeds this one to the SPI.
 */
void board_present(const uint16_t *pixels) {
#if DISPLAY_USE_DMA
    st7789_push_frame_dma(pixels);
#else
    st7789_push_frame(pixels);   // Blocking: core 0 waits for every bit.
#endif
}

/** Return the SPI divider-selected baud rate; does not verify display reception. */
unsigned int board_spi_hz(void) { return spi_get_baudrate(DISPLAY_SPI); }

/* ---- 1 kHz button sampler on core 1 ----
 * Core 1 reads all ten buttons every BUTTON_SAMPLE_US with one register read
 * and runs the debouncer. Core 0 collects the latched events once per frame.
 * The debouncer is the only data both cores touch; a hardware spinlock makes
 * each sample and each collect atomic, so a frame never sees half an update.
 */
static ButtonDebouncer debouncer;
static spin_lock_t *button_lock;
static volatile uint32_t core1_busy_us;   // Time core 1 spent working, not waiting.

/** Read both controllers' five buttons in one GPIO register read; 1 = pressed. */
static void read_buttons(uint8_t raw[2]) {
    uint32_t levels = gpio_get_all();   // All GPIO input levels at the same instant.
    for (unsigned int p = 0; p < 2; p++) {
        raw[p] = 0;
        for (unsigned int bit = 0; bit < 5; bit++) {
            if (!(levels & (1u << pins[p][bit]))) raw[p] |= (uint8_t)(1u << bit);  // Active low.
        }
    }
}

/** Core 1 entry point: sample and debounce every BUTTON_SAMPLE_US, forever.
 * @note Uses an absolute deadline so timing errors do not accumulate. If the
 *       core ever falls more than ten periods behind, it resynchronises
 *       instead of bursting samples. Busy-waits: core 1 has no other work.
 */
static void button_sampler_core1(void) {
    uint64_t next = time_us_64();
    while (true) {
        uint32_t start = time_us_32();
        uint8_t raw[2];
        read_buttons(raw);
        uint32_t saved = spin_lock_blocking(button_lock);
        buttons_sample(&debouncer, raw, time_us_64());
        spin_unlock(button_lock, saved);
        core1_busy_us += time_us_32() - start;   // Busy-waiting below counts as idle.
        next += BUTTON_SAMPLE_US;
        uint64_t now = time_us_64();
        if (now > next + 10u * BUTTON_SAMPLE_US) next = now;
        else busy_wait_until(from_us_since_boot(next));
    }
}

/** Start the 1 kHz button sampler on core 1; call once after board_init(). */
void board_start_button_sampler(void) {
    buttons_init(&debouncer);
    button_lock = spin_lock_instance((unsigned int)spin_lock_claim_unused(true));
    multicore_launch_core1(button_sampler_core1);
}

/** Copy the held buttons and every press/release latched since the last call. */
void board_collect_buttons(ButtonEvents *events) {
    uint32_t saved = spin_lock_blocking(button_lock);
    buttons_collect(&debouncer, events);
    spin_unlock(button_lock, saved);
}

/** Total microseconds core 1 has spent sampling (wraps after ~71 minutes;
 * differences between two reads stay correct). */
uint32_t board_core1_busy_us(void) { return core1_busy_us; }
