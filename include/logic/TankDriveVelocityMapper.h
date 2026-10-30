#pragma once
/* ---------------------------------------------------------------------------
 * TankDriveVelocityMapper  --  relative arcade tankdrive stick -> CommandVector.
 * left_y  -> surge,  -right_x -> yaw,  sway = 0.
 * Outputs a velocity. Full deflection is the chassis free speed. The drive
 * base turns that into volts with its VAS model.
 * ------------------------------------------------------------------------- */

#include "interfaces/IVelocityMapper.h"
#include "core/Specs.h"

class TankDriveVelocityMapper : public IVelocityMapper {
public:
    explicit TankDriveVelocityMapper(double max_volts = System::MAX_VOLTS)
        : max_volts_(max_volts) {}

    CommandVector map(double /*imu_heading_rad*/, const JoystickData& joy) override {
        (void)max_volts_;
        CommandVector cmd;
        cmd.request = DriveRequest::Motion;
        cmd.surge = joy.left_y * Robot::MAX_VEL_LIN;
        cmd.sway  = 0.0;                 // differential: no lateral
        cmd.yaw   = -joy.right_x * Robot::MAX_VEL_ANG;
        return cmd;
    }

private:
    double max_volts_;
};
