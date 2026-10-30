#include "logic/SensorCalibration.h"
#include "logic/SensorReader.h"

void commitRobotSensorCalibration(IEstimator& estimator,
                                  ICommandProcessor& processor,
                                  pros::Mutex* loc_mutex,
                                  double* prev_heading,
                                  bool* obs_pending) {
    resetSensorFilters();

    if (loc_mutex) loc_mutex->take();
    estimator.reset();
    if (prev_heading) *prev_heading = 0.0;
    if (obs_pending)  *obs_pending = false;
    if (loc_mutex) loc_mutex->give();

    processor.reset();
}

void finishRobotSensorCalibration(IDriveBase& drive,
                                  IEstimator& estimator,
                                  ICommandProcessor& processor,
                                  pros::Mutex* loc_mutex,
                                  double* prev_heading,
                                  bool* obs_pending) {
    drive.waitSensorCalibration();
    commitRobotSensorCalibration(estimator, processor, loc_mutex, prev_heading, obs_pending);
}

void calibrateRobotSensors(IDriveBase& drive,
                           IEstimator& estimator,
                           ICommandProcessor& processor,
                           IUIManager* ui,
                           pros::Mutex* loc_mutex,
                           double* prev_heading,
                           bool* obs_pending) {
    if (ui) ui->displayMessage("CALIBRATING...");
    drive.stop();

    // IMU reset via the drive. begin returns immediately; the wait lives
    // in finish. Distance sensors are probed in initialize and do not
    // block this sequence.
    drive.beginSensorCalibration();
    finishRobotSensorCalibration(drive, estimator, processor,
                                 loc_mutex, prev_heading, obs_pending);

    if (ui) ui->displayMessage("RESET COMPLETE");
}
