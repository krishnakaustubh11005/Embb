/* Pico Arcade: bounded input/commands, fixed updates, SRAM rendering and SPI. */
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "config.h"
#include "platform/board.h"
#include "platform/audio.h"
#include "platform/st7789.h"
#include "audio/sound.h"
#include "platform/input.h"
#include "platform/line_parser.h"
#include "graphics/framebuffer.h"
#include "app/app.h"
#include "diagnostics/metrics.h"

/* Large persistent state belongs in SRAM, not the small function-call stack. */
static App app;
static InputProcessor input;
static LineParser commands;
static uint32_t button_samples;   // Core 1 sample counter, copied each frame.

/** Process at most 32 available USB characters; never wait for a full command. */
static void poll_commands(void) {
    for (unsigned int i=0;i<SERIAL_CHARS_PER_FRAME;i++) {
        int character=getchar_timeout_us(0);
        if (character==PICO_ERROR_TIMEOUT) break;
        LineStatus status=line_parser_feed(&commands,(char)character);
        if (status==LINE_READY) printf("[APP] %s\n",app_command(&app,commands.text));
        if (status==LINE_TOO_LONG) printf("[INPUT] Command discarded: line too long\n");
    }
}

/** Print measured CPU utilization of both cores and measured axis read times.
 * @param now Current time in µs.
 * @note Core 0 busy = 100% minus the time it sat blocked waiting for the display
 *       DMA. The audio interrupt runs on core 0 and is shown separately (it is
 *       inside "busy" unless it fired during a DMA wait). Core 1 busy = time
 *       spent sampling; its waiting between 1 ms samples counts as idle. Mux
 *       settling is a busy-wait, so it counts as core 0 busy time.
 */
static void print_cpu(uint64_t now) {
    static uint64_t last; static uint32_t last_wait, last_core1, last_irq;
    uint32_t wait=st7789_wait_us(), core1=board_core1_busy_us(), irq=audio_irq_us();
    if (last) {
        float elapsed=(float)(now-last);
        unsigned int x_us, y_us;
        board_axis_timing(&x_us,&y_us);
        printf("[CPU] cpu=%u MHz %s | core0=%.1f%% (audio irq %.1f%%, dma wait %.1f%%) core1=%.2f%% | "
               "X read=%u us Y read=%u us\n",
               (unsigned int)(clock_get_hz(clk_sys)/1000000u),DISPLAY_USE_DMA?"dma":"blocking-spi",
               (double)(100.0f*(1.0f-(float)(wait-last_wait)/elapsed)),
               (double)(100.0f*(float)(irq-last_irq)/elapsed),
               (double)(100.0f*(float)(wait-last_wait)/elapsed),
               (double)(100.0f*(float)(core1-last_core1)/elapsed),x_us,y_us);
    }
    last=now; last_wait=wait; last_core1=core1; last_irq=irq;
}

/** Print one real timing report; included in the following interval's FPS and gap. */
static void print_report(const PerfReport *report) {
    /* Button polling rate measured from core 1's sample counter, not assumed. */
    static uint32_t last_samples; static uint64_t last_time;
    uint64_t now=time_us_64();
    unsigned int button_hz=last_time?(unsigned int)((uint64_t)(button_samples-last_samples)*1000000u/(now-last_time)):0;
    last_samples=button_samples; last_time=now;
    const char *name=app.selected<GAME_COUNT?game_get(app.selected)->name:"UI";
    printf("[PERF] %s screen=%u %.2f FPS | input=%u update=%u draw=%u SPIwait=%u us | "
           "frame_max=%u gap_max=%u over_budget=%u dropped=%u | SPI clock=%u Hz | buttons=%u Hz | audio blocks=%u%s\n",
           name,(unsigned int)app.screen,(double)report->fps,
           report->average_us[0],report->average_us[1],report->average_us[2],report->average_us[3],
           report->max_frame_us,report->max_gap_us,report->over_budget,report->dropped_ticks,report->spi_hz,
           button_hz,audio_blocks(),sound_muted()?" (muted)":"");
    print_cpu(now);
}

/**
 * Run the production four-game console with the approved wiring and clock setup.
 * @return Never returns during normal operation.
 * @note Core 0 (this loop): reads the sticks, collects the button events core 1
 *       latched since the last frame, routes them once, runs bounded 120 Hz
 *       updates, draws into the back framebuffer, then hands it to the display
 *       DMA and swaps buffers. It only waits if the previous frame is still
 *       being sent, so drawing frame N+1 overlaps sending frame N.
 *       Core 1: samples and debounces all buttons every 1 ms (board.c); the
 *       only shared data is the spinlock-protected debouncer. Audio runs from
 *       DMA interrupts (audio_pwm.c). There is no heap or flash persistence.
 */
int main(void) {
    board_init();
    sound_init();
    audio_init();
    input_init(&input);
    board_start_button_sampler();   // Core 1: buttons at 1 kHz from here on.
    uint64_t previous=time_us_64();
    app_init(&app,(uint32_t)previous);
    Metrics metrics;
    metrics_init(&metrics,previous);
    PerfReport report={.spi_hz=board_spi_hz()};
    printf("[BOOT] Pico Arcade CPU=%u peri=%u SPI=%u Hz; type HELP\n",
        (unsigned int)clock_get_hz(clk_sys),(unsigned int)clock_get_hz(clk_peri),report.spi_hz);
    while (true) {
        uint64_t begin=time_us_64();
        uint16_t axes[2][2]; ButtonEvents buttons;
        board_read_axes(axes);
        board_collect_buttons(&buttons);   // Everything core 1 latched since last frame.
        button_samples=buttons.samples;
        input_process(&input,axes,&buttons,time_us_64());
        uint64_t sampled=time_us_64();
        poll_commands();
        app_input(&app,&input.snapshot);
        if (app.recalibrate_requested) {
            input_recalibrate(&input); app.recalibrate_requested=false;
        }
        unsigned int dropped=app_update(&app,&input.snapshot,sampled-previous);
        previous=sampled;
        uint64_t updated=time_us_64();
        app_draw(&app,&input.snapshot,&report);
        uint64_t drawn=time_us_64();
        board_present(fb_swap());   // DMA sends this frame; drawing moves to the other buffer.
        uint64_t end=time_us_64();
        uint64_t stages[]={sampled-begin,updated-sampled,drawn-updated,end-drawn};
        metrics_record(&metrics,begin,sampled,end,stages,dropped);
        if (metrics_report(&metrics,end,&report)) print_report(&report);
    }
}
