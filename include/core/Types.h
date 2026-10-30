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
// Origin is the center of the bottom edge of the 12 ft field (Robot::Range).
// +x toward the right wall, +y toward the far wall.
// A chassis pose locates the middle of the front bumper. The geometric
// center, where wheel travel is measured, sits half a body length behind it.
struct Pose2d {
    double x = 0;      // inches, field frame
    double y = 0;      // inches, field frame
    double theta = 0;  // radians, field frame. 0 = +Y, CCW positive
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

// What DriftFollower reads off the stored polyline each tick.
// Curvature passes κ, dκ/ds, and d²κ/ds² at the lookahead.
// Heading passes the travel tangent at regular stations ahead and rebuilds
// κ and dκ/ds from the change in that tangent. d²κ/ds² stays zero.
enum class FollowerInput { Curvature, Heading };

// Which chassis point a command's x, y, and heading name.
// Tracking is the front bumper localization reports. Rotation is the
// centre the chassis yaws about. Followers track Rotation. A Tracking
// command is shifted by tracking::kRotationToTrackingForwardM before the plan.
enum class CommandPoint { Tracking, Rotation };

// Keep-out for a line-segment plan. x, y, and min_dist are inches, the same
// frame as Pose2d. min_dist is the obstacle's own radius. The segment
// planner grows it by the bumper corner and a clearance margin. fixed
// means min_dist is already the circle the path stays outside.
struct PathObstacle {
    double x = 0;
    double y = 0;
    double min_dist = 0;
    bool fixed = false;
};

// Nine align goals, plus room for keep-outs a path adds itself.
constexpr int kPathObstacles = 16;

// Extra limits for a MotionPose drive. max_accel_mps2 < 0 keeps the
// follower's own acceleration. handoff_in > 0 ends the drive that far from
// the pose while v is still non-zero. Lengths are inches, same as Pose2d.
// The line-length default is the old 0.5 m finish.
// turn_on_goal assumes the front edge is already on a goal. The heading in
// the pose is the direction the nose should finish facing. The center rides
// the orbit around that point, and the command is a hard forward press plus
// yaw. The press stays larger than the yaw so the inside wheel never reverses
// and walks the bumper off the point.
// follower_input selects the geometry the slip and yaw feedforward read.
// line_segments plans a polyline instead of a curve. obstacles and
// field_bound apply to that plan. The orbit stays on the curve follower.
struct PathConstraints {
    double max_accel_mps2 = -1.0;
    double line_length_in = 0.5 * kInchesPerMeter;
    bool ignore_heading = false;
    double handoff_in = 0.0;
    bool turn_on_goal = false;
    FollowerInput follower_input = FollowerInput::Curvature;
    // Tracking names the front bumper. Rotation names the yaw centre.
    CommandPoint point = CommandPoint::Tracking;
    bool line_segments = false;
    bool field_bound = false;
    int obstacle_n = 0;
    PathObstacle obstacles[kPathObstacles] {};
};

inline PathConstraints turnOnGoal(PathConstraints path = {}) {
    path.turn_on_goal = true;
    return path;
}

// Travel tangents at regular stations, instead of κ and its derivatives.
inline PathConstraints headingInput(PathConstraints path = {}) {
    path.follower_input = FollowerInput::Heading;
    return path;
}

// Straight segments instead of a curve. Keep-outs on the constraints are
// circles the polyline stays outside. An orbit (turn_on_goal) still uses
// the curve follower.
inline PathConstraints segmentPath(PathConstraints path = {}) {
    path.line_segments = true;
    return path;
}

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

// What a ChassisCommand / CommandVector is carrying.
// Motion is a velocity: linear and lateral are m/s, angular is rad/s,
// accel is m/s^2, alpha is rad/s^2. The drive base turns that into volts
// with its VAS model. Volts is finished motor voltage, used when a log
// replays the voltage that was applied. Stick and Press remain so older
// call sites still compile; new commands use Motion.
enum class DriveRequest { Volts, Stick, Motion, Press };

// ---- Controller output -----------------------------------------------------
// Chassis-frame command. main.cpp never unpacks this into wheels.
// Motion is the control input. Only a drive-base class turns it into volts.
struct ChassisCommand {
    double linear = 0;    // forward, m/s. Volts when request is Volts.
    double lateral = 0;   // left, m/s. Volts when request is Volts.
    double angular = 0;   // CCW, rad/s. Volts when request is Volts.
    bool   brake = false; // request a gentle per-actuator brake (stick-zero)
    DriveRequest request = DriveRequest::Motion;
    double accel = 0;     // planned forward accel, m/s^2. Zero lets the drive difference the velocity.
    double alpha = 0;     // planned yaw accel, rad/s^2
    bool pivot = false;   // in-place yaw. The drive does not add a separate scrub gain.
};

// Pipeline command vector (plan vocabulary: surge / sway / yaw).
// Same chassis-frame axes and the same DriveRequest as ChassisCommand.
struct CommandVector {
    double surge = 0;   // forward, m/s unless request is Volts
    double sway  = 0;   // left (holonomic), m/s unless request is Volts
    double yaw   = 0;   // CCW, rad/s unless request is Volts
    DriveRequest request = DriveRequest::Motion;
    double accel = 0;
    double alpha = 0;
    bool pivot = false;

    ChassisCommand toChassis(bool brake = false) const {
        ChassisCommand cmd;
        cmd.linear = surge;
        cmd.lateral = sway;
        cmd.angular = yaw;
        cmd.brake = brake;
        cmd.request = request;
        cmd.accel = accel;
        cmd.alpha = alpha;
        cmd.pivot = pivot;
        return cmd;
    }
    static CommandVector fromChassis(const ChassisCommand& c) {
        CommandVector cmd;
        cmd.surge = c.linear;
        cmd.sway = c.lateral;
        cmd.yaw = c.angular;
        cmd.request = c.request;
        cmd.accel = c.accel;
        cmd.alpha = c.alpha;
        cmd.pivot = c.pivot;
        return cmd;
    }
};

// ---- Subsystem (non-drive) output ------------------------------------------
// Percents the drivebase mixes onto MAIN1/MAIN2. Same-direction voltage is
// the cascade. Opposite-direction voltage is the intake. Pneumatic B is the
// claw (true = released). Produced from a SubsystemIntent, or, on the replay
// path, from a recorded button bitmask.
struct SubsystemCommand {
    int  cascade_pct = 0;   // + up, - down. Both motors, same direction.
    int  intake_pct = 0;    // + in, - out. The two motors, opposite directions.
    bool pneumatics_g_state = false;
    bool pneumatics_b_state = false;
    bool score = false;        // rising edge starts the scoring macro
    // Bits in RobotMacro.h. A routine holds a bit while it wants that macro.
    // The runner starts the matching macro. Buttons still start the same ones.
    uint8_t macro_request = 0;
    uint8_t raw_bitmask = 0;   // exact requested button state (for logging/replay)
    // When set, the cascade wrapper replaces cascade_pct with the voltage
    // that holds cascade_target_in. Inches, zeroed at program start.
    bool cascade_height = false;
    double cascade_target_in = 0;
    // Driver or replayed shoulder command. Open-loop percent is not cut
    // off at CascadeLift::kMaxIn.
    bool cascade_manual = false;
};

// Common-mode reading of the MAIN1/MAIN2 differential. Positive is up.
struct CascadeReading {
    double position_deg = 0;
    double velocity_rpm = 0;
    double current_amps = 0;   // average motor current
    double torque_nm = 0;      // common-mode torque, positive up
};

// Shoulder buttons, shared by live decoding and recorded replay.
// R1 intake in, R2 intake out, L1 cascade up, L2 cascade down.
// Both buttons of a pair cancel that pair.
inline SubsystemCommand mixShoulderButtons(bool r1, bool r2, bool l1, bool l2,
                                           bool pneumatic_g, bool pneumatic_b,
                                           uint8_t bitmask) {
    SubsystemCommand sub;
    if (r1 && !r2) sub.intake_pct = 100;
    else if (r2 && !r1) sub.intake_pct = -100;
    if (l1 && !l2) sub.cascade_pct = 100;
    else if (l2 && !l1) sub.cascade_pct = -100;
    sub.pneumatics_g_state = pneumatic_g;
    sub.pneumatics_b_state = pneumatic_b;
    sub.raw_bitmask = bitmask;
    sub.cascade_manual = true;
    return sub;
}

// Shared mechanism mode. Autonomous commands and the driver macros both
// store this. toCommand() is the one percent table.
enum class IntakeState { Idle, In, Out, Jammed };
enum class CascadeState { Idle, Up, Down };

struct SubsystemIntent {
    IntakeState intake = IntakeState::Idle;
    CascadeState cascade = CascadeState::Idle;
    bool pneumatic_g = false;
    bool pneumatic_b = false;
    bool score = false;
    uint8_t macro_request = 0;

    // Solo intake matches a driver holding only R1 or R2. Cascade matches
    // L1 / L2. Jammed and Idle contribute no motor voltage.
    SubsystemCommand toCommand() const {
        SubsystemCommand out;
        switch (intake) {
            case IntakeState::In:  out.intake_pct = 100;  break;
            case IntakeState::Out: out.intake_pct = -100; break;
            case IntakeState::Idle:
            case IntakeState::Jammed: break;
        }
        switch (cascade) {
            case CascadeState::Up:   out.cascade_pct = 100;  break;
            case CascadeState::Down: out.cascade_pct = -100; break;
            case CascadeState::Idle: break;
        }
        out.pneumatics_g_state = pneumatic_g;
        out.pneumatics_b_state = pneumatic_b;
        out.score = score;
        out.macro_request = macro_request;
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
    double forward_delta = 0;   // meters the geometric center travelled along heading
    double strafe_delta = 0;    // meters the geometric center travelled sideways (left +)
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

    // Seconds this travel sample actually covers. 0 means the caller did not
    // measure it, and the localization step uses System::DT_SEC.
    double sample_dt = 0;
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
