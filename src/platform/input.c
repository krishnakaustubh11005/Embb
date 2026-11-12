/* Pure controller processing, separate from ADC switching and GPIO reads. */
#include <math.h>
#include <string.h>
#include "config.h"
#include "platform/input.h"

/** Zero debouncer RAM; no hardware or allocation is used. */
void buttons_init(ButtonDebouncer *debouncer) { memset(debouncer, 0, sizeof(*debouncer)); }

/** Debounce one sample; see input.h.
 * @note Per button: a raw change restarts that button's stability clock. When
 *       the raw level differs from the debounced level and has not changed for
 *       INPUT_DEBOUNCE_US, the change is accepted exactly once. Sampled every
 *       1 ms, a 20 ms threshold really means 20 ms of stable contact, so
 *       bounce (1-10 ms) and single-sample glitches never produce events.
 */
void buttons_sample(ButtonDebouncer *debouncer, const uint8_t raw[2], uint64_t now) {
    for (unsigned int p = 0; p < 2; p++) {
        if (!debouncer->initialized) {   // Buttons held at power-up are not "pressed".
            debouncer->candidate[p] = debouncer->held[p] = raw[p];
            for (unsigned int bit = 0; bit < 5; bit++) debouncer->changed[p][bit] = now;
        }
        for (unsigned int bit = 0; bit < 5; bit++) {
            uint8_t mask = (uint8_t)(1u << bit);
            if ((raw[p] ^ debouncer->candidate[p]) & mask) {
                debouncer->candidate[p] ^= mask;
                debouncer->changed[p][bit] = now;
            }
            if (((raw[p] ^ debouncer->held[p]) & mask) &&
                now - debouncer->changed[p][bit] >= INPUT_DEBOUNCE_US) {
                debouncer->held[p] ^= mask;
                if (raw[p] & mask) debouncer->pressed[p] |= mask;
                else debouncer->released[p] |= mask;
            }
        }
        debouncer->raw[p] = raw[p];
    }
    debouncer->initialized = true;
    debouncer->samples++;
}

/** Hand the latched events to the caller and clear them; see input.h. */
void buttons_collect(ButtonDebouncer *debouncer, ButtonEvents *out) {
    for (unsigned int p = 0; p < 2; p++) {
        out->raw[p] = debouncer->raw[p];
        out->held[p] = debouncer->held[p];
        out->pressed[p] = debouncer->pressed[p];
        out->released[p] = debouncer->released[p];
        debouncer->pressed[p] = debouncer->released[p] = 0;
    }
    out->samples = debouncer->samples;
}

/** Zero processor/calibration RAM; no hardware or allocation is used. */
void input_init(InputProcessor *processor) { memset(processor, 0, sizeof(*processor)); }

/** Reset calibration sums/counts; buttons are owned by the debouncer. */
void input_recalibrate(InputProcessor *processor) {
    memset(processor->sums, 0, sizeof(processor->sums));
    processor->snapshot.calibration_samples = 0;
    processor->snapshot.calibrated = false;
    processor->snapshot.centre_valid = 0;
}

/** Convert a 12-bit reading to [-1,1], removing INPUT_DEAD_ZONE around centre. */
static float normalize_axis(uint16_t raw, uint16_t centre) {
    int delta = (int)raw - centre;
    int distance = delta < 0 ? -delta : delta;
    if (distance <= INPUT_DEAD_ZONE) return 0;
    int span = (delta < 0 ? centre : 4095 - centre) - INPUT_DEAD_ZONE;
    if (span <= 0) return 0;
    return copysignf(fminf((float)(distance - INPUT_DEAD_ZONE) / span, 1.0f), (float)delta);
}

/** Copy collected button events, then process axes and 32-sample centre calibration.
 * @note Release sticks during calibration. Rail-like centres are flagged and
 *       replaced by 2048; this detects suspicious readings, not module presence.
 */
void input_process(InputProcessor *processor, const uint16_t axes[2][2],
                   const ButtonEvents *buttons, uint64_t now) {
    InputSnapshot *out = &processor->snapshot;
    bool collecting = !out->calibrated;
    for (unsigned int p = 0; p < 2; p++) {
        Controller *c = &out->player[p];
        c->raw_buttons = buttons->raw[p];
        c->held = buttons->held[p];
        c->pressed = buttons->pressed[p];
        c->released = buttons->released[p];
        for (unsigned int axis = 0; axis < 2; axis++) {
            c->raw[axis] = axes[p][axis];
            if (collecting) processor->sums[p][axis] += axes[p][axis];
        }
    }
    if (collecting && ++out->calibration_samples == INPUT_CALIBRATION_SAMPLES) {
        for (unsigned int p = 0; p < 2; p++) {
            for (unsigned int axis = 0; axis < 2; axis++) {
                unsigned int centre = processor->sums[p][axis] / INPUT_CALIBRATION_SAMPLES;
                bool valid = centre >= 512 && centre <= 3583;
                out->player[p].centre[axis] = valid ? centre : 2048;
                if (valid) out->centre_valid |= (uint8_t)(1u << (p * 2 + axis));
            }
        }
        out->calibrated = true;
    }
    for (unsigned int p = 0; p < 2; p++) {
        out->player[p].x = out->calibrated ? normalize_axis(axes[p][0], out->player[p].centre[0]) : 0;
        out->player[p].y = out->calibrated ? normalize_axis(axes[p][1], out->player[p].centre[1]) : 0;
    }
    out->timestamp_us = now;
}
