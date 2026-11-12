/* Pico-only audio output: PWM "DAC" on AUDIO_PIN fed by two chained DMA channels. */
#ifndef CONSOLE_AUDIO_H
#define CONSOLE_AUDIO_H
#include <stdint.h>
/** Start PWM, the DMA pacing timer and the double-buffered DMA ring.
 * @note Call after board_init() and sound_init(). After this, sound plays
 *       from the DMA interrupt with no further main-loop work.
 */
void audio_init(void);
/** Return how many 11.6 ms audio blocks have been rendered (proves the IRQ runs). */
unsigned int audio_blocks(void);
/** Running total of µs spent in the audio refill interrupt (core 0). */
uint32_t audio_irq_us(void);
#endif
