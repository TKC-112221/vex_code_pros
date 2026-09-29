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
enum class RobotMode { IDLE, RECORDING, REPLAYING, SYSID_DRIVE };

// ---- Geometry --------------------------------------------------------------
struct Pose2d {
    double x = 0;      // meters, field frame
    double y = 0;      // meters, field frame
    double theta = 0;  // radians, field frame
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
struct SubsystemCommand {
    int  intake_pct = 0;
    int  roller_top_pct = 0;
    int  roller_bottom_pct = 0;
    bool pneumatics_g_state = false;
    bool pneumatics_b_state = false;
    uint8_t raw_bitmask = 0;   // exact requested button state (for logging/replay)
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
    // Extra channels retained purely so recordings match v1's log schema.
    double left_pos_deg = 0,  right_pos_deg = 0;
    double left_vel_rpm = 0,  right_vel_rpm = 0;
    double left_current_amps = 0, right_current_amps = 0;
    double imu_accel_x = 0, imu_accel_y = 0;
    double imu_heading_deg = 0, imu_rate_z_dps = 0;
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
struct RobotReading {
    SensorDict   sensors{};
    JoystickData joystick{};
    ButtonState  buttons{};
};

// Output of the user-input-override stage.
struct OverrideResult {
    CommandVector commands{};
    ButtonState   buttons{};
};
