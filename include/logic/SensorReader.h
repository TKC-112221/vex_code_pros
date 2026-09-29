#pragma once
/* ---------------------------------------------------------------------------
 * SensorReader  --  single entry point that gathers the unified RobotReading
 * package ({sensors, joystick, buttons}) for one control tick. sensors is the
 * dict localization reads: chassis plus distance D1..D4.
 *
 * Noisy channels are denoised by applySensorKalman (math/SensorKalman.h).
 * Wheel-travel increments stay raw.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"
#include "interfaces/IDriveBase.h"

// Reads all sensors and human inputs once. Downstream stages must consume
// this package rather than calling IDriveBase sense methods directly.
RobotReading readRobotInput(IDriveBase& drive, double prev_heading_rad);

// Drop filter memory. Call after a calibration or a heading realignment so
// the next samples are not blended with the pre-reset measurement.
void resetSensorFilters();
