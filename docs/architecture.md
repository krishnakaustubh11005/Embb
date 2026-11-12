# Firmware architecture

Core 0 runs the main loop: sample the joystick axes, collect button events,
update the selected game's state at a fixed 120 Hz step, render into the back
RGB565 framebuffer, then hand that finished buffer to a DMA channel that feeds
the ST7789's SPI while core 0 starts the next frame in the other buffer. Core 1 samples and debounces all ten buttons every 1 ms
(`BUTTON_SAMPLE_US`), independent of the frame rate, and latches press and
release events until core 0 collects them once per frame. The debouncer is the
only data the two cores share; a hardware spinlock makes each sample and each
collect atomic.

Display DMA with double buffering: two 153,600-byte framebuffers. `fb_swap()`
returns the finished frame and redirects drawing to the other buffer.
`st7789_push_frame_dma()` first waits for any earlier transfer (so the buffer
about to be drawn into is never still being sent), sets the window, switches
SPI to 16-bit frames and starts a DMA channel paced by the SPI TX DREQ. Core 0
returns immediately; frame time becomes roughly the larger of (input + update
+ draw) and the 19.7 ms wire time instead of their sum. The `[PERF]` field
`SPIwait` is how long core 0 still had to wait for the previous transfer.

Why core 1 samples buttons: with one poll per frame, a press shorter than one
frame (about 25-33 ms) could fall between two polls and be lost, and the 20 ms
debounce rule reduced to "two consecutive frames agree". At 1 kHz the 20 ms
rule is real, any press held for 20 ms is delivered, and latency no longer
depends on FPS. The `[PERF]` line reports the measured `buttons=` rate.

```text
core 1: buttons every 1 ms ─> debouncer ─(spinlock)─┐
core 0, each frame:                                  v
USB serial ─> command parser ──────────> app state + game rules ─> framebuffer
ADC axes ─> calibrated axes ──────────────┘                         │
ST7789 <──── SPI <── DMA (sends frame N) <── fb_swap() ─────────────┘
```

`src/platform/board.c` owns Pico GPIO, ADC, clock, and display setup.
`src/platform/input.c` holds the portable button debouncer (run by core 1)
and turns axes plus collected button events into calibrated snapshots.
`src/app/` selects screens and games. Each game has rule/update and renderer
files under `src/games/`. `src/graphics/` owns the RGB565 framebuffer and
primitive drawing functions. `src/diagnostics/metrics.c` reports loop timing.

## Screen and game flow

The application provides a launcher (a 2x2 tile grid plus a diagnostics
entry), static game instructions, four game modes, pause/resume, result
screens, and diagnostics. Snake moves on a grid
with button controls. Pong has two independently controlled paddles, each
moving vertically and horizontally within its half; a hit reflects the ball
with an angle influenced by impact position and paddle movement. Soccer Pool
has five disks per side and one ball; players select a disk and take turns
shooting until pieces settle, with a goal limit of three.

Shade Duel is a two-player silhouette fighting game (best of three 60-second
rounds). Fighters are not bitmaps: `src/games/fighter/fighter_pose.c` stores
each move as a few keyframes of joint angles, blends them with a smoothstep
ease, and uses forward kinematics to place the joints. The lowest body point is
snapped to the floor, so crouches, sweeps and knockdowns stay grounded. Limbs
are drawn as filled capsules (`gfx_capsule`). The stick walks, jumps (up) and
crouches (down). Buttons are A punch / forward+A uppercut, B kick / down+B
sweep / airborne B flying kick, C hold to block (down+C low block, back+C dodge
roll), and D Brazilian kick, which costs one of two energy cells. Energy fills
when you land, take or block hits. Attack heights matter: high attacks miss
crouching fighters, standing block stops high/mid, crouch block stops low/mid.
Hits are detected from the striking fist or foot of the animated skeleton
against the opponent's body box. Presses are buffered for 150 ms.

All screens share one palette and rounded-card style from `include/graphics/gfx.h`
(P1 blue, P2 red). Each `GameSpec` supplies a tile colour, a launcher icon
and a winner callback that colours the results card.

Each game consumes an `InputSnapshot` and does not access hardware directly.
This keeps hardware sampling separate from the game rules and makes the rules
host-testable. Simulation time is accumulated in fixed steps, with a bounded
catch-up count so a slow display frame cannot cause an unbounded update loop.

## USB diagnostics and developer mode

USB stdio is non-blocking in the main loop. The parser consumes a bounded
number of available characters per frame. `HELP` lists commands; `STATUS`
prints screen/game and timing information; `MENU` returns to the launcher;
`CAL` recalibrates controller centers; `MUTE`/`UNMUTE` silence the speaker;
`THEME` switches the Shade Duel sound theme; `PLAY 0`–`PLAY 3` starts a game
directly (used to time each game without pressing buttons); and `DEV ON`/`DEV OFF` toggles the
separate developer assist in Snake. Developer mode does not bypass the other
games' rules. Commands are processed as text lines over USB CDC, not by
`picotool` after firmware has booted.

## Audio

`src/audio/sound.c` is portable: games call `sound_play(SND_...)` and
`sound_music(...)`, which only push one byte into a 16-entry ring queue. The
Pico layer `src/platform/audio_pwm.c` drives GP22 with 10-bit PWM (122 kHz
carrier). A DMA pacing timer copies one level per 1/22050 s from RAM into the
PWM compare register. Two chained DMA channels ping-pong between two 256-sample
buffers; when one finishes, its interrupt drains the queue and synthesizes the
next 11.6 ms. Because DMA, not the CPU, delivers samples, a stall in the main
loop does not interrupt sound.

Effects are tables of tones (waveform, pitch slide, volume slide, duration),
so each costs bytes rather than kilobytes. Five effect voices and three music
voices (koto-style lead, bass, taiko drums) are mixed. Shade Duel's loop is
an original 8-bar tune in the Japanese In scale (D Eb G A Bb) at 132 BPM.
The CLIPS theme swaps punch, kick and heavy hits for 8-bit PCM made from
`kick.mp4`, `punch1.mp4` and `punch2.mp4` by `tools/audio_to_c.py` (about
29 KB of flash). Re-run that script after replacing the recordings.

## Timing and memory

The current clock configuration is 125 MHz CPU/peripheral and 62.5 MHz SPI
(the SDK divider is checked and the actual rate is reported). A 320x240
RGB565 framebuffer occupies 153,600 bytes of SRAM; double buffering uses two
(307,200 bytes, about 59% of the 520 KB). The display update is a full-frame
DMA transfer that overlaps the next frame's work, so FPS is limited by the
larger of the 19.7 ms wire time and the CPU's per-frame work. The earlier
Test_Proj benchmark's 42 FPS observation (blocking SPI) is not a measurement
of this firmware.

The display and framebuffer dimensions are landscape (320x240); the physical
panel glass is commonly described as 240x320 in portrait orientation. Verify
orientation and colors on hardware after flashing. DMA feeds both the display
(one channel) and audio (two channels). There is no score persistence or
enclosure-dependent power management.

## Measuring CPU use

Every second the firmware prints a `[CPU]` line after `[PERF]`: the measured
CPU clock, core 0 busy % (100% minus the time blocked in `st7789_wait_frame()`),
the audio interrupt's share of core 0, core 1 busy % (only the sampling work,
not its waiting), and the measured average time of one muxed X read and one
direct Y read. Configuring with `-DCONSOLE_DISPLAY_DMA=OFF` builds a
blocking-SPI variant that exists only to measure what display DMA gains.
