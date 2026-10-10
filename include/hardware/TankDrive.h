#pragma once
/* ---------------------------------------------------------------------------
 * TankDrive  --  six-motor differential (tank) chassis + intake/roller/
 * pneumatics subsystems. Concrete IDriveBase, built on the PROS V5 API.
 *
 * Forward kinematics averages the two wheel banks into forward travel and fuses
 * IMU heading. Inverse kinematics mixes a chassis effort into left/right banks
 * (left = linear - angular, right = linear + angular) and applies the
 * per-actuator safety v1 performed inline (stick-zero brake, back-EMF current
 * limit, ratio-preserving voltage clamp, battery scaling).
 * ------------------------------------------------------------------------- */

#include "interfaces/IDriveBase.h"
#include "interfaces/IFeedForward.h"
#include "hardware/DistanceBank.h"
#include "api.h"

class TankDrive : public IDriveBase {
public:
    explicit TankDrive(const IFeedForward& feedforward,
                       double max_volts = 12.0, bool scale_battery = false);
    ~TankDrive() override;

    void initialize() override;
    void beginSensorCalibration() override;
    void waitSensorCalibration() override;
    void calibrateSensors() override;

    JoystickData readJoystick() override;
    ButtonState  readButtons() override;

    ChassisObservation readObservation(double prev_heading_rad) override;
    void readDistances(DistanceReading (&out)[kDistanceChannels]) override;

    ChassisCommand applyDrive(const ChassisCommand& cmd, const ChassisObservation& obs) override;
    void applySubsystems(const SubsystemCommand& sub) override;
    void stop() override;

    void alignHeadingTo(double target_rad) override;

private:
    // Encapsulated PROS hardware
    pros::MotorGroup* left_drive;
    pros::MotorGroup* right_drive;
    pros::Imu*        imu;
    pros::Controller* pilot;
    pros::Motor*      intake;
    pros::Motor*      roller_top;
    pros::Motor*      roller_bottom;
    pros::adi::DigitalOut* pneumatics_g;
    pros::adi::DigitalOut* pneumatics_b;

    const IFeedForward* feedforward_;
    double max_voltage;
    bool   use_battery_scaling;

    // Forward-kinematics memory (meters)
    double prev_l_ = 0.0, prev_r_ = 0.0;
    double imu_offset_ = 0.0;
    DistanceBank ranges_;

    void applyCurrentLimit(double& left_v, double& right_v, double v_lin, double v_ang) const;
};
