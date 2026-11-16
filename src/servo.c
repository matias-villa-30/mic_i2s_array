/*
 * servo.c 
 * Matias Villa
 * 
*/
#include <math.h>
#include <stdlib.h>
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "servo.h"
#include "mic_tdoa.h"      // SPEED_OF_SOUND, SAMPLE_RATE_HZ

servo_cfg srv_motor;

#define RAD2DEG        57.29578f
#define CM_PER_SAMPLE  (SPEED_OF_SOUND * 100.0f / SAMPLE_RATE_HZ)   // 343 m/s at 48 kHz = 0.715 cm per sample

// servo driver

void servo_set_angle(float deg) {
    if (deg < 0.0f)   deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    srv_motor.angle_deg = deg;

    float us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * (deg / 180.0f);
    pwm_set_chan_level(srv_motor.slice_servo, srv_motor.channel, (uint16_t)(us + 0.5f));
}

/* Move toward target, but never more than SERVO_MAX_STEP_DEG per call. */
static void servo_step_toward(float target) {
    float d = target - srv_motor.angle_deg;
    if (d >  SERVO_MAX_STEP_DEG) d =  SERVO_MAX_STEP_DEG;
    if (d < -SERVO_MAX_STEP_DEG) d = -SERVO_MAX_STEP_DEG;
    servo_set_angle(srv_motor.angle_deg + d);
}
/*
   microphone geometry
   Frame (centimetres, looking from above):
       mic1 at the origin, mic2 on the +x axis, mic3 on the +y side.
   Positions follow from the three measured side lengths (law of cosines).
 */

static float bx[3], by[3];      // baseline vectors: [0] = mic1->mic2, [1] = mic1->mic3, [2] = mic2->mic3
static float s_xx, s_xy, s_yy, s_det;   // sum of b*b^T over the three baselines

static void geometry_init(void) {
    float d12 = MIC_D12_CM, d13 = MIC_D13_CM, d23 = MIC_D23_CM;

    float x2 = d12;
    float x3 = (d12 * d12 + d13 * d13 - d23 * d23) / (2.0f * d12);
    float h2 = d13 * d13 - x3 * x3;
    float y3 = MIC3_SIDE * sqrtf(h2 > 0.0f ? h2 : 0.0f);

    bx[0] = x2;      by[0] = 0.0f;   // mic1 -> mic2
    bx[1] = x3;      by[1] = y3;     // mic1 -> mic3
    bx[2] = x3 - x2; by[2] = y3;     // mic2 -> mic3

    s_xx = s_xy = s_yy = 0.0f;
    for (int i = 0; i < 3; i++) {
        s_xx += bx[i] * bx[i];
        s_xy += bx[i] * by[i];
        s_yy += by[i] * by[i];
    }
    s_det = s_xx * s_yy - s_xy * s_xy;
}

void config_servo(void) {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    srv_motor.slice_servo = pwm_gpio_to_slice_num(SERVO_PIN);
    srv_motor.channel     = pwm_gpio_to_channel(SERVO_PIN);

    pwm_config cfg = pwm_get_default_config();
    // The counter runs at 1 MHz whatever the system clock is (125 MHz Pico, 150 MHz Pico 2),
    // so one count = 1 us and the compare level is the pulse width in microseconds.
    pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) / 1000000.0f);
    pwm_config_set_wrap(&cfg, 20000 - 1);                       // 20 ms period (50 Hz)
    pwm_init(srv_motor.slice_servo, &cfg, true);

    servo_set_angle(90.0f);                                     // centre
    geometry_init();
}

void calib_servo(void) {
    // Slow sweep so you can see (and hear) where the mechanical stops are.
    // If the servo buzzes at an end, narrow SERVO_MIN_US / SERVO_MAX_US in servo.h.
    for (float a = 90.0f; a >= 0.0f;   a -= 1.0f) { servo_set_angle(a); sleep_ms(15); }
    for (float a = 0.0f;  a <= 180.0f; a += 1.0f) { servo_set_angle(a); sleep_ms(15); }
    for (float a = 180.0f; a >= 90.0f; a -= 1.0f) { servo_set_angle(a); sleep_ms(15); }
}

/* 
   delays -> direction
 
   Far-field model: sound arrives as a flat wave travelling from direction
   u = (ux, uy) (unit vector pointing from the array toward the source).
   A mic further along u hears the sound earlier, so for mics i and j:
 
        u . (r_j - r_i) = -k_ij * CM_PER_SAMPLE

   There are 3 pairs and 2 unknowns (ux, uy), so solve by least squares:
         (sum b b^T) u = sum s b       with b = baseline, s = -k * CM_PER_SAMPLE
*/

static float median_of(const float *v, int n) {
    float t[DOA_MEDIAN_N];
    for (int i = 0; i < n; i++) t[i] = v[i];
    for (int i = 1; i < n; i++) {                 // insertion sort
        float x = t[i]; int j = i - 1;
        while (j >= 0 && t[j] > x) { t[j + 1] = t[j]; j--; }
        t[j + 1] = x;
    }
    return t[n / 2];
}

bool servo_track_delays(int k12, int k13, int k23, int check, int max_shift, float *azimuth_deg) {
    static float hist[DOA_MEDIAN_N];
    static int   hist_count = 0, hist_pos = 0;

    // 1. Reject unreliable readings
    if (abs(check) > 1) return false;                                   // the three delays don't agree
    if (abs(k12) >= max_shift || abs(k13) >= max_shift || abs(k23) >= max_shift)
        return false;                                                   // stuck at the search limit

    // 2. Path-length differences in cm
    float s[3] = { -k12 * CM_PER_SAMPLE, -k13 * CM_PER_SAMPLE, -k23 * CM_PER_SAMPLE };

    // 3. Least-squares direction vector
    float rx = 0.0f, ry = 0.0f;
    for (int i = 0; i < 3; i++) { rx += s[i] * bx[i]; ry += s[i] * by[i]; }
    float ux = (s_yy * rx - s_xy * ry) / s_det;
    float uy = (s_xx * ry - s_xy * rx) / s_det;

    // 4. Quality: for a real far sound the vector has length ~1
    float norm = hypotf(ux, uy);
    if (norm < DOA_NORM_MIN || norm > DOA_NORM_MAX) return false;

    // 5. Angle from the front (+y) axis, positive toward +x, then shift by the front offset
    float az = atan2f(ux, uy) * RAD2DEG - SERVO_FRONT_OFFSET_DEG;
    if (az >  180.0f) az -= 360.0f;
    if (az <= -180.0f) az += 360.0f;
    if (fabsf(az) > SERVO_MAX_AZIMUTH_DEG + SERVO_AZIMUTH_MARGIN_DEG)
        return false;                                                   // behind the array: servo can't show it
    if (az >  SERVO_MAX_AZIMUTH_DEG) az =  SERVO_MAX_AZIMUTH_DEG;       // slightly past the end: clamp
    if (az < -SERVO_MAX_AZIMUTH_DEG) az = -SERVO_MAX_AZIMUTH_DEG;

    // 6. Median of the last few accepted angles (removes single wrong readings)
    hist[hist_pos] = az;
    hist_pos = (hist_pos + 1) % DOA_MEDIAN_N;
    if (hist_count < DOA_MEDIAN_N) hist_count++;
    float smooth = median_of(hist, hist_count);

    // 7. Servo: 90 deg = straight ahead
    float target = SERVO_REVERSE ? (90.0f - smooth) : (90.0f + smooth);
    servo_step_toward(target);

    if (azimuth_deg) *azimuth_deg = smooth;
    return true;
}
