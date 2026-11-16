/**
 * main.c - 3-microphone TDOA (time difference of arrival) test
 * Matias Villa
 *
 * PURPOSE
 *   Reads three I2S MEMS microphones at the same instant and finds by how
 *   many samples each mic hears a sound later than another. These delays are
 *   the input for a direction estimate.
 *
 * HARDWARE (Pico 2, 150 MHz)
 *   BCLK  GPIO20   shared by all mics (PIO output)
 *   WS    GPIO21   shared by all mics (PIO output, BCLK pin + 1)
 *   DATA  GPIO9 = mic1, GPIO8 = mic2, GPIO7 = mic3 (one wire per mic)
 *   SEL   all mics to GND -> each mic talks in the LEFT slot only
 *   Sample rate ~48 kHz (150 MHz / 24.4 / 2 / 64), 24-bit samples.
 *
 * DATA PATH
 *   mics -> PIO -> RX FIFO -> DMA -> ring buffer (RAM) -> main loop
 *   - The PIO (`in pins, 4`) samples GPIO7..GPIO10 on every BCLK tick, so each
 *     4-bit group holds one bit per pin (GPIO10 is sampled but unused).
 *   - 8 groups fill a 32-bit word; a slot (32 clocks) = 4 words; a frame
 *     (left + right slot) = 8 words. Only the left slot carries data.
 *   - The DMA copies every word into a 32 KB ring (8192 words) without
 *     stopping, so the mics' clock never stalls while the CPU is busy.
 *
 * SAMPLE FORMAT
 *   Each mic's raw 32-bit word has the I2S delay bit on top, so it is decoded
 *   as ((int32_t)(w << 1)) >> 8: a signed 24-bit value with a DC offset of
 *   about -360000 counts (full scale +-8388608).
 *
 * MAIN LOOP (every ~200 ms)
 *   1. Snapshot  copy and decode the newest SAMPLES (512) frames = 10.7 ms.
 *                Snapshot size must be <= half the ring (checked at compile time).
 *   2. Gate      if any mic's peak-to-peak < TDOA_GATE, print "quiet" and skip.
 *   3. Center    subtract each mic's mean (removes the DC offset).
 *   4. Correlate find_best_k_xx() tries shifts -MAX_SHIFT..+MAX_SHIFT and returns
 *                the shift with the highest cross-correlation score.
 *   5. Confidence score / sqrt(energy_a * energy_b) is (-1 [..] +1) per pair.
 *                Below MIN_CONF the two mics did not hear the same sound.
 *   6. Print     k12, k13, k23 in samples and microseconds, plus check.
 *
 * OUTPUT
 *   k12=  5 (  104.2 us) | k13=-12 ( -250.0 us) | k23=-16 ( -333.3 us) | check=-1
 *   k > 0 : the second mic of the pair hears the sound k samples LATER.
 *   check = k13 - (k12 + k23) must be 0 (+-1): delay(1->3) = delay(1->2) + delay(2->3).
 *           A larger value means at least one pair picked a wrong peak.
 *   quiet  p-p: a | b | c    window too quiet; the three peak-to-peak levels.
 *
 * TUNING (mic_tdoa.h / this file)
 *   TDOA_GATE      loudness threshold for running the correlation
 *   MIN_CONF       minimum match quality (0.5 to start; tune from printed c12/c13/c23)
 *   SAMPLES        window length (max 512 with the 32 KB ring)
 *   MIC_DISTANCE   largest mic spacing in cm -> sets MAX_SHIFT (22 at 15 cm)
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "mic_i2s.pio.h"
#include "mic_tdoa.h"
#include "servo.h"
#include "parse.h"

// Mic order in the arrays and in the printout. Nibble bit position = pin - DATA_BASE,
// so changing DATA_BASE or moving a wire only requires editing this table.

// (static, not local: 512 * 12 bytes each would overflow the small default stack)

// File-scope, not inside main(): 512 * 12 bytes each would overflow the small default stack.
// ring must keep its alignment: the DMA write wrap depends on it.
uint32_t          ring[RING_WORDS] __attribute__((aligned(32768)));

int main() {
    stdio_init_all();
    sleep_ms(200);
    printf("|BOOT|\n");

    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &mic_i2s_program);
    mic_i2s_program_init(pio, sm, offset, DATA_BASE, CLOCK_PIN);   // ONE call
    for (uint pin = DATA_BASE; pin < DATA_BASE + PIO_WINDOW; pin++) {
        gpio_pull_down(pin);   // every pin in the PIO window, including unused ones
    }
    config_servo();
    // DMA: set up BEFORE enabling the state machine
    // Count 0xFFFFFFFF words = ~3.1 h at 384000 words/s; then the ring stops updating.
    uint dreq  = pio_get_dreq(pio, sm, false);
    int dma_ch = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_ch);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);    // FIFO: fixed address
    channel_config_set_write_increment(&c, true);    // ring: advance each word
    channel_config_set_ring(&c, true, RING_BITS);    // wrap the WRITE address
    channel_config_set_dreq(&c, dreq);
    dma_channel_configure(dma_ch, &c, ring, &pio->rxf[sm], 0xFFFFFFFF, true);

    pio_sm_set_enabled(pio, sm, true);
    sleep_ms(100);   // let the mics' startup transient pass

    while (true) {
        // 1. Snapshot: copy and decode the newest SAMPLES frames out of the ring
        uint32_t wr  = ((uint32_t)dma_hw->ch[dma_ch].write_addr - (uint32_t)ring) / 4;
        uint32_t end = wr & ~(uint32_t)(WORDS_PER_FRAME - 1);   // frame boundary: index%8==0 is the start of a left slot

        for (int i = 0; i < SAMPLES; i++) {
            uint32_t p = (end + RING_WORDS - WORDS_PER_FRAME * SAMPLES + WORDS_PER_FRAME * i) % RING_WORDS;

            uint32_t left[WORDS_PER_SLOT] = { ring[p], ring[p + 1], ring[p + 2], ring[p + 3] };
            uint32_t m[NUM_MICS];
            deinterleave3(left, m); // left slot only
            samples[i].mic1 = ((int32_t)(m[0] << 1)) >> 8;
            samples[i].mic2 = ((int32_t)(m[1] << 1)) >> 8;
            samples[i].mic3 = ((int32_t)(m[2] << 1)) >> 8;
            // right slot = ring[p+4..p+7]: all zeros with SEL on GND
        }

        // 2. Level check: skip the correlation in a quiet room (it would just lock onto noise)
        int32_t pp1 = peak_to_peak(samples, 1), pp2 = peak_to_peak(samples, 2), pp3 = peak_to_peak(samples, 3);
        if (pp1 < TDOA_GATE || pp2 < TDOA_GATE || pp3 < TDOA_GATE) {
            printf("quiet  p-p: %ld | %ld | %ld\n", (long)pp1, (long)pp2, (long)pp3);
            sleep_ms(1);
            continue;
        }

        // 3. Remove each mic's DC offset
        float mean1 = calc_mean(samples, 1);
        float mean2 = calc_mean(samples, 2);
        float mean3 = calc_mean(samples, 3);
        for (int i = 0; i < SAMPLES; i++) {
            center[i].mic1 = samples[i].mic1 - mean1;
            center[i].mic2 = samples[i].mic2 - mean2;
            center[i].mic3 = samples[i].mic3 - mean3;
        }
        
        // Energy of each mic = mean of squares of its centered samples (how loud it is).
        // Used below to normalize the scores. Never 0 here, because the gate already passed. 
        float e1 = calc_energy(center, 1);
        float e2 = calc_energy(center, 2);
        float e3 = calc_energy(center, 3);
        
        // Raw correlation score at the best shift for each pair (12, 13, 23).
        // Filled in by find_best_k_xx(). It grows with loudness, so it can't be compared
        // between windows until it is divided by the energies (see c12 etc. bel
        
        float s12;
        float s13;
        float s23;
        
        // 4. Best shift for each mic pair (k > 0: the second mic hears it k samples later)
        int k12 = find_best_k_12(center, MAX_SHIFT, &s12);
        int k13 = find_best_k_13(center, MAX_SHIFT, &s13);
        int k23 = find_best_k_23(center, MAX_SHIFT, &s23);
        
        // Confidence: raw score / sqrt(energy_a * energy_b) = -1..+1, independent of loudness.
        // Near 1: both mics heard the same sound. Near 0: the "best" shift is just noise.
        float c12 = s12 / sqrtf(e1 * e2);
        float c13 = s13 / sqrtf(e1 * e3);
        float c23 = s23 / sqrtf(e2 * e3);

        if (c12 < MIN_CONF || c13 < MIN_CONF || c23 < MIN_CONF){
            sleep_ms(50); // sound loud but not clear enough
            continue;
        }

        float t12_us = samples_to_seconds(k12, SAMPLE_RATE_HZ) * 1e6f;
        float t13_us = samples_to_seconds(k13, SAMPLE_RATE_HZ) * 1e6f;
        float t23_us = samples_to_seconds(k23, SAMPLE_RATE_HZ) * 1e6f;

        // Consistency check: delay(1->3) must equal delay(1->2) + delay(2->3), so this should be 0 (+-1)
        int check = k13 - (k12 + k23);
        
        printf("k12=%3d (%7.1f us) | k13=%3d (%7.1f us) | k23=%3d (%7.1f us) | check=%d\n", k12, t12_us, k13, t13_us, k23, t23_us, check);

        float az;
        if (servo_track_delays(k12, k13, k23, check, MAX_SHIFT, &az)) printf("  -> azimuth %.0f deg\n", az);

        sleep_ms(200); // rate limit: avoid reporting the same sound
    }
}
