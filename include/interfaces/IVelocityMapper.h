#pragma once
/* ---------------------------------------------------------------------------
 * IVelocityMapper  --  joystick (+ optional IMU) -> chassis command vector.
 * Default: relative tankdrive (forward is robot-forward, not field-forward).
 * ------------------------------------------------------------------------- */

#include "core/Types.h"

class IVelocityMapper {
public:
    virtual ~IVelocityMapper() = default;

    // imu_heading_rad is available for field-oriented / global schemes; the
    // default relative tankdrive mapper ignores it.
    virtual CommandVector map(double imu_heading_rad, const JoystickData& joy) = 0;
};
