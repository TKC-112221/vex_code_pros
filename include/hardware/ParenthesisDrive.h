#pragma once
/* ---------------------------------------------------------------------------
 * ParenthesisDrive  --  a six-motor "( )" chassis: an XDrive plus two extra
 * forward-facing traction wheels on the mid-left and mid-right (the two side
 * wheels look like parentheses hugging the X). Concrete IDriveBase, implemented
 * as a descendant of XDrive so all shared hardware handling is reused.
 *
 * The four corner omnis provide full holonomic motion. The two side wheels face
 * straight forward, so they can only push along the forward axis; they add
 * forward + turning torque and, being traction wheels, physically resist lateral
 * slip -- an "enforced linear directional torque" that makes straight-line drive
 * strong and stable while strafe is handled solely by the corners.
 *
 * Inverse kinematics augments XDrive with a differential side pair:
 *   FL = linear - lateral - angular    FR = linear + lateral + angular
 *   BL = linear + lateral - angular    BR = linear - lateral + angular
 *   SideL = linear - angular           SideR = linear + angular   (no lateral)
 *
 * The forward-facing side wheels double as clean tracking wheels: odometry takes
 * forward distance and speed from them (no 45 deg factor), strafe from the omnis.
 * ------------------------------------------------------------------------- */

#include "hardware/XDrive.h"

class ParenthesisDrive : public XDrive {
public:
    explicit ParenthesisDrive(double max_volts = 12.0, bool scale_battery = false);
    ~ParenthesisDrive() override;

    void initialize() override;
    void stop() override;

    ChassisObservation readObservation(double prev_heading_rad) override;
    ChassisCommand applyDrive(const ChassisCommand& cmd, const ChassisObservation& obs) override;

private:
    pros::Motor* side_l_;
    pros::Motor* side_r_;
    double prev_sl_ = 0, prev_sr_ = 0;
};
