#include "logic/SensorReader.h"
#include "config/Settings.h"
#include "math/SensorKalman.h"
#include "math/SensorSavitzkyGolay.h"

void resetSensorFilters() {
    resetSensorKalman();
    resetSensorSavitzkyGolay();
}

RobotReading readRobotInput(IDriveBase& drive, double prev_heading_rad) {
    RobotReading reading;
    reading.sensors.chassis = drive.readObservation(prev_heading_rad);
    drive.readDistances(reading.sensors.distance);
    reading.raw = reading.sensors;
    if (Settings::ControlToggles::SAVITZKY_GOLAY_FILTER) {
        applySensorSavitzkyGolay(reading.sensors);
    } else {
        applySensorKalman(reading.sensors);
    }
    reading.joystick = drive.readJoystick();
    reading.buttons  = drive.readButtons();
    return reading;
}
