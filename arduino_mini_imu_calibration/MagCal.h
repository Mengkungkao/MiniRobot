#pragma once

#include <Arduino.h>

#include "Config.h"

// Figure-of-eight hard/soft iron calibration. Blocking for durationMs plus a
// shorter check pass. Returns false if it was aborted or the rotation did not
// cover enough of the sphere to mean anything.
bool calibrateMagnetometer(uint32_t durationMs);

// Apply the offsets and scales currently in the results, in exactly the form
// the firmware applies them. Used by the check pass, LIVE and MOTORS, so all
// three see the field the robot would actually fuse.
void magApplyCorrection(const float raw[AXIS_COUNT], float out[AXIS_COUNT]);
