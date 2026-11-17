/*
 * report.c
 * Matias Villa
 */

#include <stdio.h>
#include "report.h"
#include "servo.h"

void report_tdoa(const tdoa_result *r) {
    float check = r->k13 - (r->k12 + r->k23);   // should be 0 (+-1)
    printf("k12=%6.2f | k13=%6.2f | k23=%6.2f | check=%.2f | c=%.2f %.2f %.2f\n",
           r->k12, r->k13, r->k23, check, r->c12, r->c13, r->c23);

    float az;
    if (servo_track_delays(r->k12, r->k13, r->k23, check, MAX_SHIFT, &az))
        printf("  -> azimuth %.0f deg\n", az);
}
