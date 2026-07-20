#pragma once
/* ---------------------------------------------------------------------------
 * Gains.h  --  The Software: PID gains, Ramsete constants, curve/slew tuning,
 * teleop face/pivot behavior, replay corrections. Ported verbatim from v1
 * (constants/gains.h). These are the numbers you retune during practice.
 * ------------------------------------------------------------------------- */

namespace PidGains {
    struct PID { double kP, kI, kD, iDecay; };

    namespace Velocity {   // inner-loop velocity PIDs
        constexpr PID Linear  = {0.83056, 0.06080, 1.35075, 0.91308};
        constexpr PID Angular = {0.76591, 0.00935, 0.32684, 0.91853};
        constexpr double DERIV_FILTER_ALPHA       = 0.26941;
        constexpr double ACCEL_DERIV_FILTER_ALPHA = 0.38747;
    }
    namespace Position {   // outer-loop position PIDs (used when Ramsete off)
        constexpr PID Linear  = {1.09030, 0.02273, 0.03127, 0.91604};
        constexpr PID Angular = {0.94256, 0.03513, 0.53024, 0.75898};
        constexpr double DERIV_FILTER_ALPHA = 0.10271;
    }
}

namespace Gains {
    namespace Ramsete {
        constexpr double B    = 1.78734;
        constexpr double ZETA = 0.88048;
    }
    constexpr double FF_BOOST_LIN = 0.75424;
    constexpr double FF_BOOST_ANG = 0.97979;
}

namespace Tuning {
    constexpr double SLEW_STEP_VOLTAGE = 0.3;
    constexpr double CUBE_RATIO        = 0.8;
    constexpr double TURN_INTENSITY    = 0.4;
}

namespace Face {   // teleop "face 0/pi" assist
    constexpr double TOLERANCE_RAD = 0.03;
    constexpr double ANG_KP = 9.0;
    constexpr double ANG_KI = 0.0;
    constexpr double ANG_KD = 0.7;
}

namespace Pivot {  // teleop asymmetric pivot
    constexpr double RATIO_LEFT   = 3.0;
    constexpr double RATIO_RIGHT  = 10.0;
    constexpr double MAX_VOLTAGE  = 10.0;
}

namespace Replay {
    constexpr double WHEEL_LIN_KP = 2.0;
    constexpr double WHEEL_ANG_KP = 0.5;
}

namespace Controls {
    constexpr double TINY_BRAKE_VOLTAGE  = 0.3;
    constexpr double STICK_ZERO_DEADBAND = 0.02;
}
