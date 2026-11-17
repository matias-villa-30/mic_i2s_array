/*
 * parse.h 
 * Matias Villa
*/
#ifndef PARSE_H
#define PARSE_H

#include <stdint.h>
#include "mic_tdoa.h"

#define NUM_MICS      3
#define DATA_BASE     7      // lowest data pin the PIO samples (window = GPIO7..GPIO10, 4 bits per clock)
#define PIO_WINDOW    4      // must match "in pins, 4" in mic_i2s.pio
#define CLOCK_PIN     20     // BCLK = GPIO20, WS = GPIO21

// Frame layout in the ring: 8 words = left slot (4 words) + right slot (4 words)
#define WORDS_PER_SLOT   4
#define WORDS_PER_FRAME  8

// The analysis window is SAMPLES frames (defined in mic_tdoa.h)
#define RING_BITS     15
#define RING_WORDS    8192   // 2^15 / 4
_Static_assert(WORDS_PER_FRAME * SAMPLES * 2 <= RING_WORDS,
               "snapshot must be at most half the ring");

// Only run the correlation when every mic is clearly above the quiet-room floor
// (quiet room ~10000-20000 counts p-p, 1 kHz tone ~35000+). Tune to your setup.
#define TDOA_GATE     25000

#define MIN_CONF      0.5f

typedef struct __attribute__((aligned(32768))){
    uint32_t            ring[RING_WORDS];   // DMA target
    i2s_sample_t        samples[SAMPLES];   // Decoded 24-bit i2s samples 
    centered_sample_t   center[SAMPLES];    // DC offset removed 
    int                 dma_channel;        // DIRECT MEMORY ACCESS channel 
} mic_ctx;

// Split one slot (4 words, 8 nibbles each) into one raw 32-bit word per mic.
// Each nibble holds one bit per pin: bit (pin - DATA_BASE) belongs to that pin.
void deinterleave3(const uint32_t w[WORDS_PER_SLOT], uint32_t m[NUM_MICS]);

// Peak to peak amplitudes (maximum wave amplitude minus minimum wave amplitude) of one mic over measure window.
// Loudness measure: compared against TDOA_GATE to skip quiet windows.
int32_t peak_to_peak(const mic_ctx *ctx, int mic);

// Copy and decode sample frames 
void snapshot(mic_ctx *ctx);

// True if every mic peak to peak level is at the same level or above TDOA_GATE
bool window_is_loud(const mic_ctx *ctx);

#endif
