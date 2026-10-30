#pragma once
/* ---------------------------------------------------------------------------
 * XDrive  --  four-omni holonomic (X) chassis. Concrete IDriveBase.
 *
 * Wheels sit at the four corners at 45 deg. Because it implements the same
 * IDriveBase contract as TankDrive, the estimator, controller, trajectory and
 * the whole pipeline are reused unchanged -- swapping this in is a one-line
 * change in main.cpp's wiring block.
 *
 * Convention (chassis frame: x forward, y left, theta CCW):
 *   linear  = forward effort,  lateral = left effort,  angular = CCW effort
 * Inverse kinematics (positive spin = forward when all equal):
 *   FL = linear - lateral - angular      FR = linear + lateral + angular
 *   BL = linear + lateral - angular      BR = linear - lateral + angular
 * Note: with lateral = 0 the left pair (FL,BL) and right pair (FR,BR) collapse
 * to the exact TankDrive mix (left = linear-angular, right = linear+angular),
 * so the diff-drive CascadedController drives an X-drive correctly in arcade.
 *
 * Members are protected so ParenthesisDrive can extend the wheel set.
 * ------------------------------------------------------------------------- */

#include "interfaces/IDriveBase.h"
#include "hardware/DistanceBank.h"
#include "api.h"
#include <cstdint>

class XDrive : public IDriveBase {
public:
    explicit XDrive(double max_volts = 12.0, bool scale_battery = false);
    ~XDrive() override;

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

protected:
    // Corner drive motors
    pros::Motor* fl_;  pros::Motor* fr_;
    pros::Motor* bl_;  pros::Motor* br_;
    // Shared peripherals
    pros::Imu*        imu_;
    pros::Controller* pilot_;
    pros::Motor*      main1_;
    pros::Motor*      main2_;
    pros::Motor*      sub1_;
    pros::Motor*      sub2_;
    pros::adi::DigitalOut* pneumatics_g_;
    pros::adi::DigitalOut* pneumatics_b_;

    double max_voltage_;
    bool   use_battery_scaling_;
    double prev_fl_ = 0, prev_fr_ = 0, prev_bl_ = 0, prev_br_ = 0;
    bool have_fl_ = false, have_fr_ = false, have_bl_ = false, have_br_ = false;
    double imu_offset_ = 0.0;
    bool imu_wheels_ = false;
    bool have_heading_ = false;
    double held_heading_ = 0.0;
    std::uint32_t last_sense_ms_ = 0;
    bool have_sense_ms_ = false;
    DistanceBank ranges_;

    // Shared helpers reused by ParenthesisDrive.
    double batteryScale() const;                       // MAX_VOLTS/battery or 1
    static double clampScale(const double* v, int n, double max_v); // ratio-preserving
    void fillImuChannels(double prev_heading_rad, ChassisObservation& o);
    // Measured tick length. A gap rebases encoder baselines via `gap`.
    double takeSampleDt(bool& gap);

    // Previous velocity command. ParenthesisDrive shares it.
    double volt_surge_ = 0.0;
    double volt_yaw_ = 0.0;
    bool volt_primed_ = false;
};
