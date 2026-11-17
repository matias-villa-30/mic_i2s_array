/*
 * mic_tdoa.h 
 * Matias Villa
*/

#ifndef MIC_TDOA_H
#define MIC_TDOA_H

#include <stdint.h>
#include <stdbool.h>

#define SAMPLES         512          // frames per analysis window (must be <= 512 with the 32 KB ring)
#define MIC_DISTANCE    17           // largest mic spacing, cm
#define SPEED_OF_SOUND  343          // m/s
#define SAMPLE_RATE_HZ  48000.0f     // 150 MHz / 24.4 / 2 / 64

// Largest delay (in samples) a source can cause between any two mics, plus a 2-sample margin.
// 0.17 m / 343 m/s * 48000 Hz = 20.99 samples -> 22
#define MAX_SHIFT       ((int)(((MIC_DISTANCE / 100.0f) / SPEED_OF_SOUND) * SAMPLE_RATE_HZ) + 2)

typedef struct {
  int32_t mic1;
  int32_t mic2;
  int32_t mic3;
}   i2s_sample_t;

typedef struct {
  float mic1;
  float mic2;
  float mic3;
} centered_sample_t;

typedef struct {
   float k_scores;
} correlation_score;

typedef struct {
    float k12, k13, k23;
    float c12, c13, c23;
} tdoa_result; 

// Convention: score_for_shift_ab(k) = mean of mic_a[i] * mic_b[i+k].
// The best k is positive when mic_b hears the sound k samples LATER than mic_a.
float score_for_shift_12(centered_sample_t center[SAMPLES], int k);
float score_for_shift_13(centered_sample_t center[SAMPLES], int k);
float score_for_shift_23(centered_sample_t center[SAMPLES], int k);
float calc_mean(const i2s_sample_t samples[SAMPLES], int mic);
float samples_to_seconds(int best_k, float sample_rate_hz);

int find_best_k_12(centered_sample_t center[SAMPLES], int max_shift, float *score_out);
int find_best_k_13(centered_sample_t center[SAMPLES], int max_shift, float *score_out);
int find_best_k_23(centered_sample_t center[SAMPLES], int max_shift, float *score_out);

// Mean of squares of one mic's centered samples.
float calc_energy(centered_sample_t center[SAMPLES], int mic);

// Returns false if any pair is below min_conf: mics did not hear the same sound 
bool compute_tdoa(const i2s_sample_t *samples, centered_sample_t *center, float min_conf, tdoa_result *out);

#endif
