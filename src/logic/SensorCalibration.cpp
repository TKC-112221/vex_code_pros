#include "logic/SensorCalibration.h"
#include "logic/SensorReader.h"

void calibrateRobotSensors(IDriveBase& drive,
                           IEstimator& estimator,
                           ICommandProcessor& processor,
                           IUIManager* ui,
                           pros::Mutex* loc_mutex,
                           double* prev_heading,
                           bool* obs_pending) {
    if (ui) ui->displayMessage("CALIBRATING...");
    drive.stop();

    // Hardcoded sensor calibration sequence. Today this is IMU reset via the
    // drivebase; extend here when additional sensors need an explicit init.
    drive.calibrateSensors();
    resetSensorFilters();

    if (loc_mutex) loc_mutex->take();
    estimator.reset();
    if (prev_heading) *prev_heading = 0.0;
    if (obs_pending)  *obs_pending = false;
    if (loc_mutex) loc_mutex->give();

    processor.reset();

    if (ui) ui->displayMessage("RESET COMPLETE");
}
