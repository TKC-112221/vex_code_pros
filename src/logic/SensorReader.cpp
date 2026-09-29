#include "logic/SensorReader.h"
#include "math/SensorKalman.h"

void resetSensorFilters() {
    resetSensorKalman();
}

RobotReading readRobotInput(IDriveBase& drive, double prev_heading_rad) {
    RobotReading reading;
    reading.sensors.chassis = drive.readObservation(prev_heading_rad);
    drive.readDistances(reading.sensors.distance);
    applySensorKalman(reading.sensors);
    reading.joystick = drive.readJoystick();
    reading.buttons  = drive.readButtons();
    return reading;
}
