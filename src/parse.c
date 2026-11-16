/*
 * parse.c 
 * Matias Villa
*/

#include "parse.h"

static const int mic_pin[NUM_MICS] = { 9, 8, 7 };   // mic1 = GPIO9, mic2 = GPIO8, mic3 = GPIO7

void deinterleave3(const uint32_t w[WORDS_PER_SLOT], uint32_t m[NUM_MICS]) {
    for (int n = 0; n < NUM_MICS; n++) m[n] = 0;
    for (int k = 0; k < WORDS_PER_SLOT; k++) {
        for (int j = 0; j < 8; j++) {
            uint32_t nib = (w[k] >> (28 - 4 * j)) & 0xFu;
            for (int n = 0; n < NUM_MICS; n++) {
                m[n] = (m[n] << 1) | ((nib >> (mic_pin[n] - DATA_BASE)) & 1u);
            }
        }
    }
}

int32_t peak_to_peak(const i2s_sample_t *s, int mic) {
    int32_t lo = 0x7FFFFFFF, hi = -0x7FFFFFFF;
    for (int i = 0; i < SAMPLES; i++) {
        int32_t v = (mic == 1) ? samples[i].mic1 : (mic == 2) ? samples[i].mic2 : samples[i].mic3;
        if (v < lo) lo = v;
        if (v > hi) hi = v;
    }
    return hi - lo;
}
