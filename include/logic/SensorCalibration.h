#pragma once
/* ---------------------------------------------------------------------------
 * SensorCalibration  --  PreAuton / mid-match sensor calibration entry point.
 * Hardcodes the calibration sequence; initialize() and the Up-button path
 * both call calibrateRobotSensors().
 * ------------------------------------------------------------------------- */

#include "interfaces/IDriveBase.h"
#include "interfaces/IEstimator.h"
#include "interfaces/ICommandProcessor.h"
#include "interfaces/IUIManager.h"
#include "api.h"

// Stop actuators, calibrate onboard sensors (IMU, etc.), and reset pose /
// command-processor state so the next tick starts from a known origin.
// When loc_mutex is provided, estimator reset and heading bookkeeping are
// done under that lock so they stay coherent with the async localization task.
void calibrateRobotSensors(IDriveBase& drive,
                           IEstimator& estimator,
                           ICommandProcessor& processor,
                           IUIManager* ui = nullptr,
                           pros::Mutex* loc_mutex = nullptr,
                           double* prev_heading = nullptr,
                           bool* obs_pending = nullptr);
