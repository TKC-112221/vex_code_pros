#pragma once
/* ---------------------------------------------------------------------------
 * Specs.h  --  The Physics: wheel geometry, gearing, track width, loop timing.
 * Only changes when the robot is physically rebuilt. Ported verbatim from v1
 * (constants/specs.h) but exposed exclusively through scoped names.
 * ------------------------------------------------------------------------- */

namespace Robot {

    constexpr double PI = 3.14159265358979323846;

    namespace Chassis {
        constexpr double WHEEL_DIAMETER_IN = 3.25;
        constexpr double WHEEL_DIAMETER_M  = WHEEL_DIAMETER_IN * 0.0254;
        constexpr double GEAR_RATIO        = 48.0 / 36.0;
        constexpr double TRACK_WIDTH_M     = 0.27051;
    }

    namespace Drive {
        constexpr double MOTOR_RPM_CART     = 600.0;
        constexpr double VELOCITY_EFFICIENCY = 0.90;
    }

    namespace Align {
        constexpr double POINT_X_M = -11.5 * 0.0254;
        constexpr double POINT_Y_M = -2.0 * 0.0254;
    }
    constexpr double ODOM_IMU_WEIGHT = 1.0;

    // Derived values used throughout the codebase.
    constexpr double METERS_PER_TURN     = Chassis::WHEEL_DIAMETER_M * PI * 0.75;
    constexpr double RPM_TO_MPS          = METERS_PER_TURN / 60.0;
    constexpr double THEORETICAL_MAX_VEL = (Drive::MOTOR_RPM_CART / 60.0) * PI
                                           * Chassis::WHEEL_DIAMETER_M * Chassis::GEAR_RATIO;
    constexpr double MAX_VEL_LIN         = THEORETICAL_MAX_VEL * Drive::VELOCITY_EFFICIENCY;
    constexpr double MAX_VEL_ANG         = MAX_VEL_LIN / (Chassis::TRACK_WIDTH_M / 2.0);
}

namespace System {
    constexpr double LOOP_TIME_MS   = 10.0;
    constexpr int    MAX_RECORD_FRAMES = 32000;
    constexpr double MAX_VOLTS      = 12.0;
    constexpr double MAX_VOLTAGE_DROP = 12.0;
    constexpr double DT_SEC         = LOOP_TIME_MS / 1000.0;
}
