#ifndef REPORT_H
#define REPORT_H

#include "mic_tdoa.h"

// Prints one result line (delays, consistency check, confidences) and moves the
// servo test rig toward the estimated direction.
void report_tdoa(const tdoa_result *r);

#endif
