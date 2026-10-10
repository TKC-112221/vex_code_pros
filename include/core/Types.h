#pragma once
/* ---------------------------------------------------------------------------
 * Types.h  --  The universal vocabulary of the v3 pipeline.
 *
 * Everything that flows between pipeline stages is expressed in these
 * drivebase-agnostic types. In particular NOTHING here mentions "left" or
 * "right": the chassis speaks in terms of a Twist (velocity in each direction
 * of the chassis frame) and a ChassisCommand (generalized effort in each
 * direction). Converting those to/from individual wheels/motors is the private
 * business of a concrete DriveBase (TankDrive, XDrive, ...).
 * ------------------------------------------------------------------------- */

#include <cstdint>

// ---- Operating modes -------------------------------------------------------
// COMMAND runs the declarative routine in auton/ on this same 10 ms tick.
enum class RobotMode { IDLE, RECORDING, REPLAYING, COMMAND };

// Published field position is inches. Wheel travel, distance ranges, and the
// estimator's internal geometry stay in meters and are converted at the
// IEstimator boundary. Speeds stay m/s and accelerations stay m/s^2.
constexpr double kMetersPerInch = 0.0254;
constexpr double kInchesPerMeter = 1.0 / kMetersPerInch;

// ---- Geometry --------------------------------------------------------------
// Position only. Path samples, particles, and logs use this.
struct Pose2d {
    double x = 0;      // inches, field frame
    double y = 0;      // inches, field frame
    double theta = 0;  // radians, field frame. 0 = +X, CCW positive
};

inline double metersToInches(double meters) { return meters * kInchesPerMeter; }
inline double inchesToMeters(double inches) { return inches * kMetersPerInch; }

inline Pose2d poseMetersToInches(const Pose2d& meters) {
    return Pose2d{metersToInches(meters.x), metersToInches(meters.y), meters.theta};
}
inline Pose2d poseInchesToMeters(const Pose2d& inches) {
    return Pose2d{inchesToMeters(inches.x), inchesToMeters(inches.y), inches.theta};
}

// Pose plus the velocity at that pose. This is the setpoint autonomous
// routines and the driver-control curve override both hand to the follower.
// x and y are inches. heading is the direction the nose faces. v is m/s
// along the nose; negative v drives in reverse. omega is yaw rate, rad/s,
// CCW. A measured StateEstimate becomes one of these through motionPoseOf().
struct MotionPose {
    double x = 0;
    double y = 0;
    double heading = 0;
    double v = 0;
    double omega = 0;

    Pose2d geometry() const { return Pose2d{x, y, heading}; }
};

// Extra limits for a MotionPose drive. max_accel_mps2 < 0 keeps the
// follower's own acceleration. handoff_in > 0 ends the drive that far from
// the pose while v is still non-zero. Lengths are inches, same as Pose2d.
// The line-length default is the old 0.5 m finish.
struct PathConstraints {
    double max_accel_mps2 = -1.0;
    double line_length_in = 0.5 * kInchesPerMeter;
    bool ignore_heading = false;
    double handoff_in = 0.0;
};

// A chassis-frame velocity/effort vector.
//   vx    -> along the robot's forward axis
//   vy    -> along the robot's left axis (holonomic only; 0 for differential)
//   omega -> about the robot's vertical axis
// Used both as a velocity (m/s, rad/s) reference and, reinterpreted, as a
// generalized command. Differential drivebases simply ignore vy.
struct Twist {
    double vx = 0;
    double vy = 0;
    double omega = 0;
};

// ---- Estimated state -------------------------------------------------------
// The rich output of an IEstimator. OdometryEstimator fills pose + velocity;
// the Kalman estimator additionally fills the acceleration / derivative
// channels the controller uses for feedforward and the PID D-term.
struct StateEstimate {
    Pose2d pose;
    double v_lin = 0;      // m/s   smoothed forward velocity
    double v_ang = 0;      // rad/s smoothed angular velocity
    double a_lin = 0;      // m/s^2 command-tracking linear accel estimate
    double a_ang = 0;      // rad/s^2 command-tracking angular accel estimate
    double deriv_lin = 0;  // (a_cmd_est - a_act_est) for the velocity PID D-term
    double deriv_ang = 0;
};

// v is body-forward speed (v_lin). omega is yaw rate (v_ang).
inline MotionPose motionPoseOf(const StateEstimate& state) {
    return MotionPose{state.pose.x, state.pose.y, state.pose.theta, state.v_lin, state.v_ang};
}

// ---- Motion setpoint fed to a controller -----------------------------------
// The universal target: where we want to be (pose), how fast (velocity twist),
// and how hard we are accelerating (feedforward accel). For replay we may also
// carry a recorded chassis-frame effort to use directly as feedforward.
struct MotionTarget {
    Pose2d pose;
    Twist  velocity;        // reference velocities (vx, vy, omega)
    double accel_lin = 0;   // a_ref
    double accel_ang = 0;   // alpha_ref

    // Chassis-frame recorded feedforward. In teleop these stay 0 (so a
    // "use recorded volts" controller naturally contributes zero feedforward,
    // matching v1). In replay the TrajectoryManager fills them from the log.
    double ref_linear_effort  = 0;
    double ref_lateral_effort = 0;
    double ref_angular_effort = 0;
    // If true, the controller outputs the recorded effort directly, skipping
    // feedback (models v1's REPLAY_VOLTAGE_FALLBACK path).
    bool   passthrough_effort = false;
};

// ---- Controller output -----------------------------------------------------
// Generalized chassis-frame effort. For a voltage-driven plant these carry
// volts, but the units are the DriveBase's concern. main.cpp never unpacks
// this into wheels; only the DriveBase does.
struct ChassisCommand {
    double linear = 0;    // effort along forward axis
    double lateral = 0;   // effort along left axis (holonomic only)
    double angular = 0;   // effort about vertical axis
    bool   brake = false; // request a gentle per-actuator brake (stick-zero)
};

// Pipeline command vector (plan vocabulary: surge / sway / yaw).
// Same chassis-frame axes as ChassisCommand::{linear, lateral, angular}.
struct CommandVector {
    double surge = 0;   // forward
    double sway  = 0;   // left (holonomic)
    double yaw   = 0;   // CCW about vertical

    ChassisCommand toChassis(bool brake = false) const {
        return ChassisCommand{surge, sway, yaw, brake};
    }
    static CommandVector fromChassis(const ChassisCommand& c) {
        return CommandVector{c.linear, c.lateral, c.angular};
    }
};

// ---- Subsystem (non-drive) output ------------------------------------------
// Volts the drivebase applies. Produced from a SubsystemIntent, or, on the
// replay path, from a recorded button bitmask.
struct SubsystemCommand {
    int  intake_pct = 0;
    int  roller_top_pct = 0;
    int  roller_bottom_pct = 0;
    bool pneumatics_g_state = false;
    bool pneumatics_b_state = false;
    uint8_t raw_bitmask = 0;   // exact requested button state (for logging/replay)
};

// Shared mechanism mode. Autonomous commands and the driver macros both
// store this. toCommand() is the one percent table.
enum class IntakeState { Idle, In, Out, Jammed };
enum class ShooterState { Idle, Spin, Reverse };

struct SubsystemIntent {
    IntakeState intake = IntakeState::Idle;
    ShooterState shooter = ShooterState::Idle;
    bool pneumatic_g = false;
    bool pneumatic_b = false;

    // Solo intake matches a driver holding only R1 or R2, including the
    // small roller nudge from that button mix. Shooter matches L1 / L2.
    // Jammed and Idle contribute no motor voltage.
    SubsystemCommand toCommand() const {
        SubsystemCommand out;
        switch (intake) {
            case IntakeState::In:  out.intake_pct = 100;  break;
            case IntakeState::Out: out.intake_pct = -100; break;
            case IntakeState::Idle:
            case IntakeState::Jammed: break;
        }
        switch (shooter) {
            case ShooterState::Spin:
                out.roller_top_pct = 106;
                out.roller_bottom_pct = 97;
                break;
            case ShooterState::Reverse:
                out.roller_top_pct = -24;
                out.roller_bottom_pct = -103;
                break;
            case ShooterState::Idle: break;
        }
        if (shooter == ShooterState::Idle
            && (intake == IntakeState::In || intake == IntakeState::Out)) {
            out.roller_top_pct = -4;
            out.roller_bottom_pct = 2;
        }
        out.pneumatics_g_state = pneumatic_g;
        out.pneumatics_b_state = pneumatic_b;
        return out;
    }
};

// ---- Raw human input -------------------------------------------------------
// Four analog axes from the V5 controller, each normalized to -1.0 .. 1.0.
struct JoystickData {
    double left_y  = 0;  // Axis 1 — typically forward/back
    double left_x  = 0;  // Axis 2 — typically strafe (holonomic)
    double right_y = 0;  // Axis 3
    double right_x = 0;  // Axis 4 — typically turn
};

struct ButtonState {
    bool l1=false, l2=false, r1=false, r2=false;
    bool up=false, down=false, left=false, right=false;
    bool a=false, b=false, x=false, y=false;
};

// ---- Kinematic observation from the DriveBase ------------------------------
// A DriveBase turns its own wheel encoders + IMU into this chassis-frame
// observation via forward kinematics. Estimators consume ONLY this, so they
// never need to know how many wheels there are or how they are arranged.
struct ChassisObservation {
    double forward_delta = 0;   // meters travelled along heading since last read
    double strafe_delta = 0;    // meters travelled sideways (holonomic; 0 for tank)
    double heading_rad = 0;     // absolute heading estimate to fuse
    double v_lin = 0;           // measured forward velocity (m/s)
    double v_ang = 0;           // measured angular velocity (rad/s)
    double battery_voltage = 0;

    // Wheel-bank channels (drivebase maps its geometry into left/right).
    double left_pos_deg = 0,  right_pos_deg = 0;
    double left_vel_rpm = 0,  right_vel_rpm = 0;   // logged raw on LogFrame
    double left_current_amps = 0, right_current_amps = 0;
    double left_temp_c = 0, right_temp_c = 0;

    // Raw IMU (G, deg/s). Unfiltered so recordings keep the sensor bandwidth.
    double imu_accel_x = 0, imu_accel_y = 0;
    double imu_rate_z_dps = 0;

    // Filled by ImuLinearDelta after each accel integration step.
    double v_inertial_fwd = 0;  // m/s, body forward
    double v_inertial_lat = 0;  // m/s, body left
};

// One VEX distance-sensor sample. Invalid readings (no return, out of range,
// PROS_ERR) have valid == false and are ignored by estimators.
constexpr int kDistanceChannels = 4;

struct DistanceReading {
    double range_m = 0;    // meters along the beam
    int    confidence = 0; // 0..63; PROS returns 10 at or below 200 mm
    bool   valid = false;
};

// Named sensor bag for one control tick. Localization reads only the channels
// it uses: chassis for dead reckoning, distance[0..3] for Ports::D1..D4.
struct SensorDict {
    ChassisObservation chassis{};
    DistanceReading distance[kDistanceChannels]{};

    const DistanceReading& d1() const { return distance[0]; }
    const DistanceReading& d2() const { return distance[1]; }
    const DistanceReading& d3() const { return distance[2]; }
    const DistanceReading& d4() const { return distance[3]; }
    DistanceReading& d1() { return distance[0]; }
    DistanceReading& d2() { return distance[1]; }
    DistanceReading& d3() { return distance[2]; }
    DistanceReading& d4() { return distance[3]; }
};

// Unified sense package produced once per control tick. Every downstream stage
// consumes this structure (or a slice of it) rather than re-reading hardware.
// `sensors` is denoised. `raw` is the same tick before the denoiser, kept so
// recording can store the sensor bandwidth without reading hardware twice
// (a second read would consume the encoder deltas).
struct RobotReading {
    SensorDict   sensors{};
    SensorDict   raw{};
    JoystickData joystick{};
    ButtonState  buttons{};
};

// Output of the user-input-override stage.
struct OverrideResult {
    CommandVector commands{};
    ButtonState   buttons{};
};
