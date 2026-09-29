#pragma once
/* ---------------------------------------------------------------------------
 * SensorKalman  --  3-sample scalar Kalman on one SensorDict.
 *
 * Heading and distance beams are advanced by this tick's measured motion
 * before the correction, so a steady turn or a steady approach is tracked
 * with no lag and the update only rejects noise. Rates, accelerations, and
 * wheel speeds stay a plain random walk. Wheel-travel increments and encoder
 * positions are left raw: they are summed into pose, and smoothing them would
 * lag that sum. An invalid distance beam is skipped; three misses in a row
 * drop that beam's memory so the next return starts clean.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"

void applySensorKalman(SensorDict& sensors);
void resetSensorKalman();
