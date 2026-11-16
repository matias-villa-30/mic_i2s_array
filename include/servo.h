/*
 * servo.h 
 * Matias Villa
*/

#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include <stdbool.h>

// Servo hardware 
#define SERVO_PIN              16       // GPIO16 = physical pin 21 (PWM slice 0, channel A)
#define SERVO_MIN_US           700      // pulse for 0 deg.   Typical micro servo: 500-2400 us
#define SERVO_MAX_US           2300     // pulse for 180 deg. If the servo buzzes at an end, narrow these
#define SERVO_MAX_STEP_DEG     15.0f    // max movement per update (limits twitching and current spikes)
#define SERVO_REVERSE          0        // 1 = mirror left/right if the wing moves the wrong way

// Microphone geometry (measure centre of sound hole to centre of sound hole) 
#define MIC_D12_CM             17.0f    // mic1 - mic2
#define MIC_D13_CM             15.5f    // mic1 - mic3
#define MIC_D23_CM             16.5f    // mic2 - mic3
#define MIC3_SIDE              1.0f     // +1: mic3 is on the +y side of the mic1->mic2 line, -1: other side

// Direction estimate settings 
#define SERVO_FRONT_OFFSET_DEG 0.0f     // rotate the "front" (0 deg = straight toward mic3's side)
#define SERVO_MAX_AZIMUTH_DEG  90.0f    // servo covers +-90 deg around the front
#define SERVO_AZIMUTH_MARGIN_DEG 10.0f  // up to 10 deg beyond +-90 is clamped to the end; further is ignored
#define DOA_NORM_MIN           0.5f     // quality: length of the solved direction (1 = sound in the array plane; small = sound above/in front of it)
#define DOA_NORM_MAX           1.4f
#define DOA_MEDIAN_N           5        // median of the last N accepted angles

typedef struct {
    uint32_t slice_servo;
    uint32_t channel;
    float    angle_deg;                 // current servo angle, 0..180
} servo_cfg;

extern servo_cfg srv_motor;

void config_servo(void);                // PWM setup + mic geometry; moves servo to 90 deg
void servo_set_angle(float deg);        // 0..180, clamped
void calib_servo(void);                 // slow sweep 90 -> 0 -> 180 -> 90 to find the mechanical limits

/*
  Turn the three mic-pair delays (in samples, convention k > 0: second mic hears it later)
  into a sound direction and move the servo toward it.
    check      : k13 - (k12 + k23); must be 0 or +-1
    max_shift  : the correlation search limit; a result at the limit is clipped and rejected
    azimuth_deg: out, direction of the sound in degrees from the front axis, positive toward +x
  Returns true if the reading was accepted (and the servo was moved), false if it was rejected.
 */
bool servo_track_delays(int k12, int k13, int k23, int check, int max_shift, float *azimuth_deg);

#endif
