#pragma once
/* ---------------------------------------------------------------------------
 * TankDriveVelocityMapper  --  relative arcade tankdrive stick -> CommandVector.
 * left_y  -> surge,  -right_x -> yaw,  sway = 0.
 * Outputs open-loop chassis effort (stick * max_volts) for the passthrough
 * controller path; a richer ICommandProcessor can reinterpret units later.
 * ------------------------------------------------------------------------- */

#include "interfaces/IVelocityMapper.h"
#include "core/Specs.h"

class TankDriveVelocityMapper : public IVelocityMapper {
public:
    explicit TankDriveVelocityMapper(double max_volts = System::MAX_VOLTS)
        : max_volts_(max_volts) {}

    CommandVector map(double /*imu_heading_rad*/, const JoystickData& joy) override {
        CommandVector cmd;
        cmd.surge = joy.left_y * max_volts_;
        cmd.sway  = 0.0;                         // differential: no lateral
        cmd.yaw   = (-joy.right_x) * max_volts_; // match prior turn sign
        return cmd;
    }

private:
    double max_volts_;
};
