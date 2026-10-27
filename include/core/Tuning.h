#pragma once
/* ---------------------------------------------------------------------------
 * Tuning.h  --  every hand-tuned number for the robot.
 *
 * Wheel geometry, sensor mounts, and published noise live in Specs.h.
 * Wiring lives in Ports.h. Feature flags live in Settings.h. The Python
 * path follower and the MockDrive plant have their own parameter files.
 *
 * USE_TUNING_PASTE swaps the searched knobs for python/tuning_paste.txt.
 * Generated accel ceilings come from data/generated_limits.h. The fallback
 * below is used only when that file is absent. With the paste flag on,
 * TURN_RADIUS_MAX_M is the literal from the paste.
 * ------------------------------------------------------------------------- */

#include "config/Settings.h"
#include "core/Specs.h"
#include "core/Types.h"

#if __has_include("data/generated_limits.h")
    #include "data/generated_limits.h"
#endif

namespace PidGains {
    struct PID { double kP, kI, kD, iDecay; };

    namespace Velocity {
        constexpr PID Linear  = {0.83056, 0.06080, 1.35075, 0.91308};
        constexpr PID Angular = {0.76591, 0.00935, 0.32684, 0.91853};
        constexpr double DERIV_FILTER_ALPHA       = 0.26941;
        constexpr double ACCEL_DERIV_FILTER_ALPHA = 0.38747;
    }
    namespace Position {
        constexpr PID Linear  = {1.09030, 0.02273, 0.03127, 0.91604};
        constexpr PID Angular = {0.94256, 0.03513, 0.53024, 0.75898};
        constexpr double DERIV_FILTER_ALPHA = 0.10271;
        constexpr double INTEGRAL_LIMIT = 2.0;
    }
}

namespace Gains {
    namespace Ramsete {
        constexpr double B    = 1.78734;
        constexpr double ZETA = 0.88048;
        // Floor on the speed used inside the Ramsete gain, m/s and rad/s.
        constexpr double SPEED_FLOOR = 0.2;
    }
    constexpr double FF_BOOST_LIN = 0.75424;
    constexpr double FF_BOOST_ANG = 0.97979;
    // Slow the command when the pose error is at least this far, meters.
    constexpr double SPATIAL_WARP_M = 0.1;
}

namespace Tuning {
    constexpr double SLEW_STEP_VOLTAGE = 0.3;
    constexpr double CUBE_RATIO        = 0.8;
    constexpr double TURN_INTENSITY    = 0.4;
    // Extra scale on the turn stick after the drive curve.
    constexpr double TURN_STICK_SCALE  = 0.6;
}

namespace Face {
    constexpr double TOLERANCE_RAD = 0.03;
    constexpr double ANG_KP = 9.0;
    constexpr double ANG_KI = 0.0;
    constexpr double ANG_KD = 0.7;
}

namespace Pivot {
    constexpr double RATIO_LEFT   = 3.0;
    constexpr double RATIO_RIGHT  = 10.0;
    constexpr double MAX_VOLTAGE  = 10.0;
}

// Goal lock. The drive attaches when the front edge comes within
// FRONT_TOUCH_M of a goal point. Forward is dropped. The yaw stick is
// the orbit rate about that point, capped at MAX_YAW_RAD_S. Reverse
// lets go.
namespace GoalAlign {
    constexpr double FRONT_TOUCH_M = 0.04;
    constexpr double MAX_YAW_RAD_S = 3.0;
    constexpr double VEL_DEADBAND = 0.04;
    // Square the nose onto the point after a touch. The time constant is
    // short so the bumper centers as it arrives, then the yaw stick takes over.
    constexpr double CENTER_TAU_SEC = 0.08;
    constexpr double CENTER_POS_M = 0.003;
    constexpr double CENTER_HEAD_RAD = 0.02;
    // Latched swing. Surge presses the bumper into the point. Yaw sweeps the
    // heading. Their sum is the rail, and surge stays the larger one: the
    // inside wheel is surge - yaw, and a negative inside wheel backs the
    // bumper off the goal. The yaw volts follow the brake curve up to
    // TURN_YAW_RAD_S, so a short remainder does not keep the full turn.
    constexpr double TURN_SURGE_V = 7.0;
    constexpr double TURN_YAW_V = 5.0;
    constexpr double TURN_YAW_RAD_S = 6.0;
    constexpr double TURN_YAW_ACCEL = 8.0;
    constexpr double TURN_SETTLE_RAD_S = 0.25;
}

// Bumper impact used as a pose fix. A linear specific-force spike past
// DECEL_G, while the front edge was still moving, seats the front center
// on the nearest goal along that edge. Heading is kept. The spike is not
// required to lie on the forward axis: a yawing hit stops the bumper
// sideways. A forward accel above LAUNCH_G is a launch, not a stop.
// Drivetrain braking stays below DECEL_G; a clipped hit reaches the
// inertial sensor's ±4 g rail.
namespace GoalBump {
    constexpr double DECEL_G = 3.0;
    constexpr double LAUNCH_G = 1.0;
    constexpr double APPROACH_MPS = 0.15;
    constexpr double LOOKAHEAD_M = 0.35;
    // Estimate overshoot while yawing. The goal can sit this far behind
    // the front edge and still be the bumper that just stopped.
    constexpr double BEHIND_M = 0.15;
    constexpr double SEATED_M = 0.02;
    constexpr std::uint32_t REFRACTORY_MS = 400;
}

namespace Replay {
    constexpr double WHEEL_LIN_KP = 2.0;
    constexpr double WHEEL_ANG_KP = 0.5;
}

namespace Controls {
    constexpr double TINY_BRAKE_VOLTAGE  = 0.3;
    constexpr double STICK_ZERO_DEADBAND = 0.02;
    // Wheel speed below this is already stopped for the stick-zero brake, m/s.
    constexpr double BRAKE_SPEED_MPS = 0.02;
    // Below this the battery reading is ignored for scaling and replay warp.
    constexpr double BATTERY_VALID_V = 5.0;
}

// V5 controller analog full scale, and LemLib's motor-power full scale.
// 127 power corresponds to System::MAX_VOLTS.
namespace Controller {
    constexpr double FULL_SCALE = 127.0;
}

// LemLib's sample ControllerSettings for moveToPose (boomerang).
// Lateral error is inches, angular error is degrees, and the PID output is
// motor power in [-127, 127]. The derivative is per 10 ms tick.
namespace Boomerang {
    struct Axis {
        double kP, kI, kD, windup;
        double small_error, small_ms;
        double large_error, large_ms;
        double slew;
    };

#if USE_TUNING_PASTE
#define TUNING_PASTE_PART 7
#include "data/tuning_paste.inc"
#else
    constexpr Axis Lateral = {10, 0, 3, 3, 1, 100, 3, 500, 5};
    // Heading trim, LemLib power units (127 is 12 V). Two power per degree,
    // light damping, no integral. DriftFollower reads these four.
    constexpr Axis Angular = {2, 0, 0.5, 3, 1, 100, 3, 500, 0};
    // Square up inside this many inches of a stop goal.
    constexpr double CLOSE_IN = 6.0;
#endif
    constexpr double CLOSE_MIN_POWER = 60.0;
    constexpr double CROSS_TRACK_IN = 4.0;
}

namespace Physics {
#if __has_include("data/generated_limits.h")
    constexpr double MAX_ACCEL_LIN     = Limits::MAX_ACCEL_LIN;
    constexpr double MAX_ACCEL_ANG     = Limits::MAX_ACCEL_ANG;
    constexpr double SLEW_STEP_LIN     = Limits::SLEW_STEP_LIN;
    constexpr double SLEW_STEP_ANG     = Limits::SLEW_STEP_ANG;
#else
    constexpr double MAX_ACCEL_LIN     = 2.0;
    constexpr double MAX_ACCEL_ANG     = 8.0;
    constexpr double SLEW_STEP_LIN     = 0.02;
    constexpr double SLEW_STEP_ANG     = 0.08;
#endif
#if USE_TUNING_PASTE
#define TUNING_PASTE_PART 6
#include "data/tuning_paste.inc"
#elif __has_include("data/generated_limits.h")
    constexpr double TURN_RADIUS_MAX_M = Limits::TURN_RADIUS_MAX_M;
#else
    constexpr double TURN_RADIUS_MAX_M = 10.0;
#endif
    // Yaw rate below this is treated as straight for the turn-radius clamp.
    constexpr double OMEGA_EPS = 0.01;
}

namespace Motion {
#if USE_TUNING_PASTE
#define TUNING_PASTE_PART 4
#include "data/tuning_paste.inc"
#else
    // Smallest path acceleration the speed planner will use, m/s^2.
    constexpr double MIN_ACCEL_MPS2 = 0.15;
    // |kappa| below this is a straight line when picking a turn radius.
    constexpr double KAPPA_EPS = 1e-3;
#endif
    // |speed - exit speed| inside this counts as settled, m/s.
    constexpr double SETTLE_SPEED_MPS = 0.2;
}

// Armature volts = kS sgn(v) + kV v + kA a.
// The defaults follow this chassis. Free speed is Robot::MAX_VEL_LIN, and
// 6 m/s^2 is a 6-motor 600 rpm stall with a dressed robot. Yaw volts are
// V_R − V_L, so kV and kA are the linear model taken across the track.
// ANG_KA_COEFF uses motor_count / 2, and ANG_KA_MOTOR_COUNT below is 6.
namespace Feedforward {
#if USE_TUNING_PASTE
#define TUNING_PASTE_PART 2
#include "data/tuning_paste.inc"
#else
    constexpr double KS_LIN = 0.6;
    constexpr double KS_ANG = 1.2;
    constexpr double LIN_SPEED_AT_SAT = Robot::MAX_VEL_LIN;
    constexpr double LIN_ACCEL_AT_FULL_V = 6.0;
    constexpr double ANG_LEVER_M = Robot::Chassis::TRACK_WIDTH_M;
    constexpr double ANG_RATE_PER_VOLT = 2.0 * (System::MAX_VOLTS - KS_LIN) / LIN_SPEED_AT_SAT;
    constexpr double ANG_KA_LENGTH_M = ANG_LEVER_M;
    constexpr double ANG_KA_COEFF = (System::MAX_VOLTS / LIN_ACCEL_AT_FULL_V) * ANG_LEVER_M * 3.0;
#endif
    constexpr double ANG_KA_MOTOR_COUNT = 6.0;

    constexpr double KV_LIN = (System::MAX_VOLTS - KS_LIN) / LIN_SPEED_AT_SAT;
    constexpr double KA_LIN = System::MAX_VOLTS / LIN_ACCEL_AT_FULL_V;
    constexpr double KV_ANG = ANG_RATE_PER_VOLT * ANG_LEVER_M / 2.0;
    constexpr double KA_ANG = ANG_KA_COEFF * (2.0 * ANG_KA_LENGTH_M)
                            / (ANG_KA_MOTOR_COUNT * ANG_LEVER_M);
}

namespace Drift {
    constexpr double EXP_RATE = -2.0;
    constexpr double DRIFT_ACCEL = 5.5;
    // Lateral accel the timed profile may spend, m/s^2. v²|κ| stays under this.
    constexpr double CENTRIPETAL_ACCEL = 6.0;
    // Scales the soft speed caps (centripetal, yaw accel, path accel). 1 keeps
    // the tracking budget. Higher spends more of it and holds more speed.
    constexpr double SPEED_REWARD = 2.0;
#if USE_TUNING_PASTE
#define TUNING_PASTE_PART 3
#include "data/tuning_paste.inc"
#else
    // Brisk enough for a skills leg, slow enough for the 10 ms loop to track.
    constexpr double PATH_ACCEL = 3.0;
    // Sideslip in the heading model decays in about a quarter second.
    constexpr double K_SLIP = 4.0;
    constexpr double A_MAX = 8.0;
    // Nose lead stays inside about 35 degrees. Past that the lateral loop steers.
    constexpr double MAX_SLIP_RAD = 0.6;
    // Volts per meter of cross-track, and the derivative in V per (m/s).
    constexpr double LAT_KP = 4.0;
    constexpr double LAT_KI = 0.2;
    constexpr double LAT_KD = 0.3;
    constexpr double LAT_KD2 = 0.0;
    constexpr double LAT_I_LIMIT = 1.0;
#endif
    constexpr double BOOMERANG_LEAD = 1.0;
    constexpr double BOOMERANG_MAX_SLIP = 0.7;
    constexpr double PROBE_M = 0.01;
    constexpr double CARROT_FLOOR_M = 0.05;
    constexpr int CURVE_STEPS = 40;
}

namespace Turn {
    constexpr double FALLBACK_KV = 0.45;
    constexpr double FALLBACK_KA = 0.12;
    constexpr double FALLBACK_HOLD = 1.5;
    constexpr double HOLD_ACCEL = 2.0;
    constexpr double HOLD_DEADBAND = 0.02;
    constexpr double MIN_TOLERANCE_RAD = 1e-4;
    constexpr double MIN_SETTLE_RATE = 0.02;
    constexpr double MIN_ACCEL = 0.05;
    constexpr double MIN_VEL = 0.05;
}

// Button A practice drive, and the accel the plant binding uses for a guide.
namespace Practice {
    constexpr double GOAL_X = 0.0;
    constexpr double GOAL_Y = 36.0;
    constexpr double GOAL_HEADING_RAD = Robot::PI / 2.0;
    constexpr double EXIT_SPEED_MPS = 2.0;
#if USE_TUNING_PASTE
#define TUNING_PASTE_PART 5
#include "data/tuning_paste.inc"
#else
    constexpr double MAX_ACCEL_MPS2 = 4.0;
#endif
    constexpr double LINE_LENGTH = 0.5;
}

namespace driftpath {
    inline constexpr double kOnAxisM = 1e-3;
    inline constexpr double kMomentumSpeed = 0.15;
    inline constexpr double kHermiteLookSec = 0.25;
    inline constexpr double kHermiteLookMin = 0.30;
    inline constexpr double kHermiteLookFrac = 0.35;
    inline constexpr double kGraphAlongMin = 0.1736;
    inline constexpr double kGraphSlopeMax = 5.671;
    inline constexpr int kLengthSteps = 32;
    // Timed elastic band. Poses include the pinned ends. The hard curvature
    // cap is the tightest bend the band may keep while the speed is still low.
    inline constexpr int kTebPoses = 13;
    inline constexpr int kTebIters = 16;          // one tick, seeded again from the base polyline
    inline constexpr int kTebWarmIters = 8;       // a tick that continues a band already in hand
    inline constexpr int kTebObstacles = 4;
    inline constexpr double kTebMinSegM = 0.04;
    // Steering look for the committed band. The Hermite floor is 0.30 m,
    // which sits past the first free knot, so the command would track a
    // part of the curve the robot has already left.
    inline constexpr double kTebLookMin = 0.10;
    inline constexpr double kTebKappaHard = 8.0;
    // Time-optimality step on a segment's Δt, seconds per pass. The kinematic
    // floor stops the step. Obstacle weight is the fraction of the penetration
    // pushed out in one pass. Biharmonic gain fair-curves the free poses.
    inline constexpr double kTebTimeStep = 0.015;
    inline constexpr double kTebObstacleWeight = 0.55;
    inline constexpr double kTebBiharmonic = 0.16;
    // Third derivative of the slipped heading, rad/s³. The band walks a free
    // pose down that jerk. 40 rad/s³ is an 8 rad/s² yaw accel changing over
    // 0.2 s, about as fast as the drift feedforward can be tracked. The step
    // is one pass, in meters, and it scales down when the jerk is already small.
    inline constexpr double kTebYawJerk = 40.0;
    inline constexpr double kTebJerkStep = 0.012;
    inline constexpr int kSpeedKnots = 64;
}

// Feature scales and the settle window. Layer sizes stay with the actor.
namespace DrivePolicy {
    constexpr double kKappaUnitPerIn = 0.08;
    constexpr double kKappaRateUnit = 0.008;
    constexpr double kCrossUnitIn = 12.0;
    constexpr double kPositionToleranceIn = Boomerang::CLOSE_IN;
    constexpr double kSpeedToleranceMps = Motion::SETTLE_SPEED_MPS;
    constexpr double kHeadingToleranceRad = 5.0 * Robot::PI / 180.0;
    constexpr double kDepartMarginIn = 18.0;
}

namespace WallNudge {
    // Fraction of the wall-range fix kept each distance-sensor period.
    constexpr double ODOM_GAIN = 0.05;
    constexpr double STEP_M = 0.04;
    constexpr double GATE_M = 0.20;
}

namespace Particles {
    constexpr int COUNT = 400;
    constexpr int MAX = 3000;
    constexpr int MIN = 8;
    constexpr double RESAMPLE_RATIO = 0.2;
}

namespace Slip {
    constexpr int COUNT = 100;
    constexpr int MAX = 128;
    constexpr int MIN = Particles::MIN;
    constexpr int RANGE_STRIDE = 5;
    constexpr double FORWARD_SCALE_SPAN = 0.04;
    constexpr double STRAFE_SCALE_SPAN = 0.08;
    constexpr double SKID_PER_M_SPAN = 0.10;
    constexpr double SKID_PER_RAD_SPAN = 0.08;
    constexpr double HEADING_SCALE_SPAN = 0.03;
    constexpr double NOISE_SCALE_MIN = 0.25;
    constexpr double NOISE_SCALE_MAX = 1.25;
    constexpr double ALPHA_TRANS = 0.015;
    constexpr double ALPHA_TRANS_ROT = 0.01;
    constexpr double ALPHA_LATERAL = 0.01;
    constexpr double ALPHA_ROT = 0.02;
    constexpr double ALPHA_ROT_TRANS = 0.01;
    constexpr double BETA_TRANS = 0.0;
    constexpr double BETA_ROT = 0.0;
    constexpr double SIGMA_HEADING = 0.01;
    constexpr double NUDGE_GAIN = 0.35;
    constexpr double NUDGE_STEP_M = WallNudge::STEP_M;
    constexpr double NUDGE_STEP_RAD = 0.05;
    constexpr double RANGE_GATE_M = WallNudge::GATE_M;
    constexpr double MAX_GRAD_SQ = 16.0;
    constexpr double MAX_H_THETA = 3.0;
    constexpr double FIX_GAIN_NUM = 0.08;
    constexpr double FIX_GAIN_MIN = 0.2;
}

// -0.5 * sigma^2 floors shared by the particle filter and the slip ensemble.
namespace RangeLik {
    constexpr double REJECT_SIGMAS = 5.0;
    constexpr double MAX_PENALTY = -0.5 * REJECT_SIGMAS * REJECT_SIGMAS;
    constexpr double KIDNAP_SIGMAS = 4.0;
    constexpr double KIDNAP_LOG_LIK = -0.5 * KIDNAP_SIGMAS * KIDNAP_SIGMAS;
}

namespace Ekf {
    constexpr double Q_POS = 1e-5;
    constexpr double Q_VEL = 1e-4;
    constexpr double Q_ACC = 0.1;
    constexpr double R_POS = 0.002 * 0.002;
    constexpr double R_VEL = 0.04 * 0.04;
    constexpr double R_ACC_CMD = 2.0 * 2.0;
    constexpr double ACT_BLEND = 0.2;
    constexpr double CMD_TAU_SEC = 0.04;
    constexpr double WALL_FIX_MAE_M = 0.08;
    constexpr double WALL_FIX_STEP_M = 0.05;
    constexpr double WALL_GATE_SIGMAS = 3.0;
    constexpr double P0_POSE = 0.01;
    constexpr double P0_VEL = 0.05;
    constexpr double P0_ACC = 1.0;
    constexpr double P0_DIAG = 0.1;
}

namespace ImuBias {
    constexpr double SATURATION_G = 8.0;
    constexpr double STILL_LIN_MPS = 0.05;
    constexpr double STILL_ANG_RAD_S = 0.15;
    constexpr double STILL_ACCEL_G = 0.04;
    constexpr double BIAS_TAU_SEC = 0.4;
    constexpr double COAST_TAU_SEC = 0.05;
}

namespace SensorFilter {
    // 21 ticks is 210 ms. Degree 1 holds a steady rate and averages a
    // change in acceleration across that window.
    constexpr int SAVITZKY_WINDOW = 21;
    constexpr int SAVITZKY_DEGREE = 1;
    constexpr int KALMAN_WINDOW = 3;
}

namespace WallSolve {
    constexpr double FIELD_MARGIN_M = 0.05;
    constexpr double INNOV_GATE_M = 0.40;
    constexpr int ITERATIONS = 8;
}
