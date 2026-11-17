/*
 * parse.c 
 * Matias Villa
*/
#include "hardware/dma.h"
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

void snapshot(mic_ctx *ctx) {
    uint32_t wr  = ((uint32_t)dma_hw->ch[ctx->dma_channel].write_addr - (uint32_t)ctx->ring) / 4;
    uint32_t end = wr & ~(uint32_t)(WORDS_PER_FRAME - 1);   // frame boundary

    for (int i = 0; i < SAMPLES; i++) {
        uint32_t p = (end + RING_WORDS - WORDS_PER_FRAME * SAMPLES + WORDS_PER_FRAME * i) % RING_WORDS;
        uint32_t left[WORDS_PER_SLOT] = { ctx->ring[p], ctx->ring[p+1], ctx->ring[p+2], ctx->ring[p+3] };
        uint32_t m[NUM_MICS];
        deinterleave3(left, m);
        ctx->samples[i].mic1 = ((int32_t)(m[0] << 1)) >> 8;
        ctx->samples[i].mic2 = ((int32_t)(m[1] << 1)) >> 8;
        ctx->samples[i].mic3 = ((int32_t)(m[2] << 1)) >> 8;
    }
}

int32_t peak_to_peak(const mic_ctx *ctx, int mic) {
    int32_t lo = 0x7FFFFFFF, hi = -0x7FFFFFFF;
    for (int i = 0; i < SAMPLES; i++) {
        const i2s_sample_t *s = &ctx->samples[i];
        int32_t v = (mic == 1) ? s->mic1 : (mic == 2) ? s->mic2 : s->mic3;
        if (v < lo) lo = v;
        if (v > hi) hi = v;
    }
    return hi - lo;
}


bool window_is_loud(const mic_ctx *ctx){
    return  peak_to_peak(ctx, 1)    >= TDOA_GATE &&
            peak_to_peak(ctx, 2)    >= TDOA_GATE &&
            peak_to_peak(ctx, 3)    >= TDOA_GATE;
}

