#pragma once
/* ---------------------------------------------------------------------------
 * SensorReader  --  single entry point that gathers the unified RobotReading
 * package ({sensors, joystick, buttons}) for one control tick. sensors is the
 * dict localization reads: chassis plus distance D1..D4.
 *
 * Noisy channels are denoised by the filter selected in
 * Settings::ControlToggles::SAVITZKY_GOLAY_FILTER: a causal Savitzky-Golay
 * (math/SensorSavitzkyGolay.h) or the 3-sample Kalman (math/SensorKalman.h).
 * Wheel-travel increments stay raw. The pre-filter dict is kept on
 * RobotReading::raw for the recorder.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"
#include "interfaces/IDriveBase.h"

// Reads all sensors and human inputs once. Downstream stages must consume
// this package rather than calling IDriveBase sense methods directly.
RobotReading readRobotInput(IDriveBase& drive, double prev_heading_rad);

// Drop filter memory. Call after a calibration or a heading realignment so
// the next samples are not blended with the pre-reset measurement.
void resetSensorFilters();
