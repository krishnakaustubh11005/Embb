/* Portable two-controller snapshots; axes are right/up positive.
 *
 * Buttons and sticks are handled at different rates:
 *  - ButtonDebouncer runs every BUTTON_SAMPLE_US (1 kHz, on core 1 on the Pico).
 *    It debounces each button and LATCHES press/release events until the main
 *    loop collects them, so a press shorter than one frame is never lost.
 *  - input_process runs once per frame on core 0: it takes the collected button
 *    events and the freshly sampled axes and builds the InputSnapshot games read.
 * No hardware access here, so both run in the PC unit tests.
 */
#ifndef CONSOLE_INPUT_H
#define CONSOLE_INPUT_H
#include <stdbool.h>
#include <stdint.h>
enum { KEY_A=1u, KEY_B=2u, KEY_C=4u, KEY_D=8u, KEY_MENU=16u };
typedef struct {
    uint16_t raw[2], centre[2];
    float x, y;
    uint8_t raw_buttons, held, pressed, released;
} Controller;
typedef struct {
    Controller player[2];
    uint64_t timestamp_us;
    unsigned int calibration_samples;
    bool calibrated;
    uint8_t centre_valid;
} InputSnapshot;

/* Fast-rate button state. Masks use KEY_* bits; 1 means pressed. */
typedef struct {
    uint8_t raw[2], candidate[2], held[2];
    uint8_t pressed[2], released[2];   // Latched since the last collect.
    uint64_t changed[2][5];            // When each button's raw level last changed.
    uint32_t samples;                  // Total samples taken (for the polling-rate report).
    bool initialized;
} ButtonDebouncer;

/* What one frame receives from the debouncer. */
typedef struct {
    uint8_t raw[2], held[2], pressed[2], released[2];
    uint32_t samples;
} ButtonEvents;

typedef struct {
    InputSnapshot snapshot;
    uint32_t sums[2][2];
} InputProcessor;

/** Zero debouncer state; the first sample sets the held buttons without events. */
void buttons_init(ButtonDebouncer *debouncer);
/** Debounce one raw sample of both controllers taken at monotonic time now (µs).
 * @note A level is accepted once it has been stable for INPUT_DEBOUNCE_US.
 *       Accepted changes are OR-ed into the latched pressed/released masks.
 */
void buttons_sample(ButtonDebouncer *debouncer, const uint8_t raw[2], uint64_t now);
/** Copy the current held state and all latched events into out, then clear the latches. */
void buttons_collect(ButtonDebouncer *debouncer, ButtonEvents *out);

/** Zero processor/calibration RAM. */
void input_init(InputProcessor *processor);
/** Build the frame's snapshot from raw [player][axis] samples and collected button events. */
void input_process(InputProcessor *processor, const uint16_t axes[2][2],
                   const ButtonEvents *buttons, uint64_t now);
/** Restart centre collection; button state is unaffected. */
void input_recalibrate(InputProcessor *processor);
#endif
