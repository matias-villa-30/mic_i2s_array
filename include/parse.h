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

i2s_sample_t      samples[SAMPLES];   // decoded 24-bit samples, one struct per frame
centered_sample_t center[SAMPLES];    // same samples with the DC offset removed
// Split one slot (4 words, 8 nibbles each) into one raw 32-bit word per mic.
// Each nibble holds one bit per pin: bit (pin - DATA_BASE) belongs to that pin.
void deinterleave3(const uint32_t w[WORDS_PER_SLOT], uint32_t m[NUM_MICS]);
int32_t peak_to_peak(const i2s_sample_t *s, int mic);

#endif
