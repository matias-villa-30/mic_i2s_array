/**
 * main.c - 3-microphone TDOA (time difference of arrival) test
 * Matias Villa
 *
 * PURPOSE
 *   Reads three I2S MEMS microphones at the same instant and finds by how
 *   many samples each mic hears a sound later than another. These delays are
 *   the input for a direction estimate (azimuth) for a UGV. The servo is
 *   only a test rig.
 *
 * HARDWARE (Pico 2, 150 MHz)
 *   BCLK  GPIO20   shared by all mics (PIO output)
 *   WS    GPIO21   shared by all mics (PIO output, BCLK pin + 1)
 *   DATA  one GPIO per mic, see DATA_BASE and mic_pin[] (parse.h / parse.c)
 *   SEL   all mics to GND -> each mic talks in the LEFT slot only
 *   Sample rate ~48 kHz (150 MHz / 24.4 / 2 / 64), 24-bit samples.
 *
 * DATA PATH
 *   mics -> PIO -> RX FIFO -> DMA -> ring buffer (RAM) -> main loop
 *   The DMA copies every word into a 32 KB ring without stopping, so the
 *   mics' clock never stalls while the CPU is busy.
 *
 * MAIN LOOP (polls every ~1 ms while quiet)
 *   1. snapshot()      copy and decode the newest SAMPLES frames (parse.c)
 *   2. window_is_loud() skip the window if any mic is below TDOA_GATE (parse.c)
 *   3. compute_tdoa()  center, correlate, normalize; false if any pair is
 *                      below MIN_CONF (mic_tdoa.c)
 *   4. report_tdoa()   print the delays and move the servo test rig (report.c)
 *
 * TUNING
 *   TDOA_GATE      loudness threshold (parse.h)
 *   MIN_CONF       minimum match quality per pair (parse.h)
 *   MIC_DISTANCE   largest mic spacing in cm -> sets MAX_SHIFT (mic_tdoa.h)
 */

#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "mic_i2s.pio.h"
#include "mic_tdoa.h"
#include "servo.h"
#include "parse.h"
#include "report.h"

// File-scope, not inside main(): far too big for the stack.
static mic_ctx ctx;

int main() {
    stdio_init_all();
    sleep_ms(200);
    printf("|BOOT|\n");

    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &mic_i2s_program);
    mic_i2s_program_init(pio, sm, offset, DATA_BASE, CLOCK_PIN);
    for (uint pin = DATA_BASE; pin < DATA_BASE + PIO_WINDOW; pin++) {
        gpio_pull_down(pin);   // every pin in the PIO window, including unused ones
    }
    config_servo();            // PWM setup, centres the servo, computes mic geometry

    // DMA: set up BEFORE enabling the state machine.
    // Count 0xFFFFFFFF words = ~3.1 h at 384000 words/s; then the ring stops updating.
    uint dreq = pio_get_dreq(pio, sm, false);
    ctx.dma_channel = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(ctx.dma_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);    // FIFO: fixed address
    channel_config_set_write_increment(&c, true);    // ring: advance each word
    channel_config_set_ring(&c, true, RING_BITS);    // wrap the WRITE address
    channel_config_set_dreq(&c, dreq);
    dma_channel_configure(ctx.dma_channel, &c, ctx.ring, &pio->rxf[sm], 0xFFFFFFFF, true);

    pio_sm_set_enabled(pio, sm, true);
    sleep_ms(100);   // let the mics' startup transient pass

    while (true) {
        snapshot(&ctx);

        if (!window_is_loud(&ctx)) {
            sleep_ms(1);       // quiet: poll fast so short sounds are not missed
            continue;
        }

        tdoa_result r;
        if (!compute_tdoa(ctx.samples, ctx.center, MIN_CONF, &r)) {
            sleep_ms(50);      // loud but no clear match: ignore this window
            continue;
        }

        report_tdoa(&r);
        sleep_ms(200);         // rate limit: don't report the same sound twice
    }
}
