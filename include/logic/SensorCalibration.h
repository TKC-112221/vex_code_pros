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
// Blocks until calibration finishes. Status lines are written when ui is set.
void calibrateRobotSensors(IDriveBase& drive,
                           IEstimator& estimator,
                           ICommandProcessor& processor,
                           IUIManager* ui = nullptr,
                           pros::Mutex* loc_mutex = nullptr,
                           double* prev_heading = nullptr,
                           bool* obs_pending = nullptr);

// Block until a calibration started with IDriveBase::beginSensorCalibration()
// finishes, then reset pose and command-processor state. Does not write to
// the brain, so a startup clip can own the screen until this returns.
void finishRobotSensorCalibration(IDriveBase& drive,
                                  IEstimator& estimator,
                                  ICommandProcessor& processor,
                                  pros::Mutex* loc_mutex = nullptr,
                                  double* prev_heading = nullptr,
                                  bool* obs_pending = nullptr);

// Pose and command-processor reset only. Startup runs the IMU wait on
// another task during the clip, then calls this so the clip's thread never
// touches the sensor again.
void commitRobotSensorCalibration(IEstimator& estimator,
                                  ICommandProcessor& processor,
                                  pros::Mutex* loc_mutex = nullptr,
                                  double* prev_heading = nullptr,
                                  bool* obs_pending = nullptr);
