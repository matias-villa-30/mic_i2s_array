/*
 * mic_tdoa.h 
 * Matias Villa
*/ 

#include "mic_tdoa.h"

float score_for_shift_12(centered_sample_t center[SAMPLES], int k){
    float score = 0;
    int count = 0;
    for (int i = 0; i < SAMPLES; i++) {
        int j = i + k;
        if (j >= 0 && j < SAMPLES) {
            score += center[i].mic1 * center[j].mic2;
            count++;
        }
    }
    return count > 0 ? score / count : 0.0f;
}

float score_for_shift_13(centered_sample_t center[SAMPLES], int k){
    float score = 0;
    int count = 0;
    for (int i = 0; i < SAMPLES; i++) {
        int j = i + k;
        if (j >= 0 && j < SAMPLES) {
            score += center[i].mic1 * center[j].mic3;
            count++;
        }
    }
    return count > 0 ? score / count : 0.0f;
}
float score_for_shift_23(centered_sample_t center[SAMPLES], int k){
    float score = 0;
    int count = 0;
    for (int i = 0; i < SAMPLES; i++) {
        int j = i + k;
        if (j >= 0 && j < SAMPLES) {
            score += center[i].mic2 * center[j].mic3;
            count++;
        }
    }
    return count > 0 ? score / count : 0.0f;
}

int find_best_k_12(centered_sample_t center[SAMPLES], int max_shift, float *score_out) {
    float best_score = score_for_shift_12(center, -max_shift);
    int best_k = -max_shift;
    
    for (int k = -max_shift + 1; k <= max_shift; k++) {
        float score = score_for_shift_12(center, k);
        if (score > best_score) {
            best_score = score;
            best_k = k;
        }
    }
    // Sub-sample refinement: fit a parabola through the best score and its two neighbours
    float frac = 0.0f;

    if (best_k > -max_shift && best_k < max_shift){
        float y0 = score_for_shift_12(center, best_k - 1);
        float y2 = score_for_shift_12(center, best_k + 1);
        float d = y0 - 2.0f * best_score + y2;
        if (d != 0.0f) frac = 0.5f * (y0 - y2) / d;
    }

    *score_out = best_score;
    return best_k + frac;
}

int find_best_k_13(centered_sample_t center[SAMPLES], int max_shift, float *score_out) {
    float best_score = score_for_shift_13(center, -max_shift);
    int best_k = -max_shift;
    
    for (int k = -max_shift + 1; k <= max_shift; k++) {
        float score = score_for_shift_13(center, k);
        if (score > best_score) {
            best_score = score;
            best_k = k;
        }
    }
    // Sub-sample refinement: fit a parabola through the best score and its two neighbours
    float frac = 0.0f;

    if (best_k > -max_shift && best_k < max_shift){
        float y0 = score_for_shift_13(center, best_k - 1);
        float y2 = score_for_shift_13(center, best_k + 1);
        float d = y0 - 2.0f * best_score + y2;
        if (d != 0.0f) frac = 0.5f * (y0 - y2) / d;
    }

    *score_out = best_score;
    return best_k + frac;
}

int find_best_k_23(centered_sample_t center[SAMPLES], int max_shift, float *score_out) {
    float best_score = score_for_shift_23(center, -max_shift);
    int best_k = -max_shift;
    
    for (int k = -max_shift + 1; k <= max_shift; k++) {
        float score = score_for_shift_23(center, k);
        if (score > best_score) {
            best_score = score;
            best_k = k;
        }
    }
    // Sub-sample refinement: fit a parabola through the best score and its two neighbours
    float frac = 0.0f;

    if (best_k > -max_shift && best_k < max_shift){
        float y0 = score_for_shift_23(center, best_k - 1);
        float y2 = score_for_shift_23(center, best_k + 1);
        float d = y0 - 2.0f * best_score + y2;
        if (d != 0.0f) frac = 0.5f * (y0 - y2) / d;
    }

    *score_out = best_score;
    return best_k + frac;
}

float samples_to_seconds(int best_k, float sample_rate_hz) {
    float sample_period = 1.0f / sample_rate_hz;
    return best_k * sample_period;
}

float calc_mean(i2s_sample_t samples[SAMPLES], int mic){
   int64_t sum = 0;   // signed 64-bit: samples are negative (DC offset ~ -360000)

   for(int i = 0; i < SAMPLES; i++) {
      if(mic == 1)
          sum += samples[i].mic1;
      else if(mic == 2)
          sum += samples[i].mic2;
      else if(mic == 3)
          sum += samples[i].mic3;
   }

   return (float)((double)sum / SAMPLES);
}

float calc_energy(centered_sample_t center[SAMPLES], int mic) {
    float sum = 0;
    for (int i = 0; i < SAMPLES; i++) {
        float v = (mic == 1) ? center[i].mic1 : (mic == 2) ? center[i].mic2 : center[i].mic3;
        sum += v * v;
    }
    return sum / SAMPLES;
}
