#pragma once
/* ---------------------------------------------------------------------------
 * TankDrive  --  six-motor differential (tank) chassis plus the MAIN1/MAIN2
 * cascade-intake differential and two pneumatics. Concrete IDriveBase.
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
#include <cstdint>

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
    CascadeReading readCascade() const override;
    void stop() override;

    void alignHeadingTo(double target_rad) override;

private:
    // Encapsulated PROS hardware
    pros::MotorGroup* left_drive;
    pros::MotorGroup* right_drive;
    pros::Imu*        imu;
    pros::Controller* pilot;
    pros::Motor*      main1;
    pros::Motor*      main2;
    pros::Motor*      sub1;
    pros::Motor*      sub2;
    pros::adi::DigitalOut* pneumatics_g;
    pros::adi::DigitalOut* pneumatics_b;

    const IFeedForward* feedforward_;
    double max_voltage;
    bool   use_battery_scaling;

    // Forward-kinematics memory (meters). have_* is false until the first
    // finite encoder sample, so a boot position is a baseline, not travel.
    double prev_l_ = 0.0, prev_r_ = 0.0;
    bool have_l_ = false, have_r_ = false;
    double imu_offset_ = 0.0;
    bool imu_wheels_ = false;
    bool have_heading_ = false;
    double held_heading_ = 0.0;
    std::uint32_t last_sense_ms_ = 0;
    bool have_sense_ms_ = false;
    DistanceBank ranges_;

    void applyCurrentLimit(double& left_v, double& right_v, double v_lin, double v_ang) const;

    // Previous velocity command, so a caller that omitted accel still gets kA.
    double volt_surge_ = 0.0;
    double volt_yaw_ = 0.0;
    bool volt_primed_ = false;
};
