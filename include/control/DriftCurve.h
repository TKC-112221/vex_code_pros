#pragma once
/* ---------------------------------------------------------------------------
 * DriftCurve  --  one control tick of a curve drive with DriftBoomerang's
 * sideslip heading.
 *
 * The path meets the target along the exit direction. A straight finish of
 * target.line_length inches runs into the target on that heading. The
 * curve is fitted to the far end of that line, so it arrives tangent to
 * the same heading and the line carries the robot the rest of the way.
 * Zero length fits the curve to the target itself. With
 * target.ignore_direction set, the exit heading is not used: the path is
 * the straight line to the target, and only exit_speed is kept. shape
 * picks the family:
 *
 *   Arc — the circle through the robot. Its center lies on the normal
 *   through the join, and the signed radius is
 *
 *     r = |robot - target|² / (2 * lateral)
 *
 *   Positive r bends left (CCW), so κ = 1/r. On the heading axis the
 *   circle opens into that line.
 *
 *   Exponential — a graph over the heading axis,
 *
 *     lateral(ξ) ∝ e^{k ξ} − 1 − k ξ
 *
 *   with k = exp_rate (1/m). It passes through the robot, is flat at the
 *   join, and ξ is distance along the heading (negative behind the
 *   join). Positive k holds the offset and hooks in near the join.
 *   Negative k sheds the offset early. Zero is the parabola ξ².
 *
 *   Quartic — a graph over the same axis,
 *
 *     lateral(ξ) ∝ ξ⁴
 *
 *   It passes through the robot. At the join the tangent, the curvature,
 *   and the curvature slope are all zero, the same values the straight
 *   finish has. Yaw rate and yaw acceleration are
 *
 *     ω = v κ
 *     α = a κ + v² dκ/ds
 *
 *   so both match the line there, including while braking. Curvature
 *   builds on the way in and falls away, instead of holding a constant
 *   κ up to a cut.
 *
 *   QuarticHermite — the same graph, with the quartic's contact at the
 *   join and the measured motion at the robot. The lateral offset is
 *
 *     η = η0 (c6 u⁶ + c5 u⁵ + c4 u⁴),   u = ξ / ξ0
 *
 *   Every term is degree 4 or higher, so η, η', η'', and η''' are zero
 *   at the join. Tangent, curvature, and curvature slope match the
 *   straight finish, and so do yaw rate and yaw acceleration. The three
 *   coefficients take the robot's offset, the slope of the measured
 *   momentum, and the measured curvature κ = ω / |v_lin|. Below 0.15 m/s
 *   the direction is the travel heading and the curvature is zero. The
 *   pure quartic is the one case of this polynomial: slope factor 4 and
 *   curvature 12 η / ξ². |κ| is held to 10/L. The command sample is the
 *   one Hermite uses: 0.25 s of travel speed, at least 0.30 m, and at
 *   most 35% of the remaining length.
 *
 *   Quintic — a quintic Bezier graphed on the same axis. Six control
 *   points run from the robot to the join, evenly spaced in ξ, so the
 *   path cannot fold back on itself. The last three lie on the axis:
 *   arrival tangent and arrival curvature match the straight finish.
 *   The first two handles are scaled from the offset by the
 *   misalignment μ = |lateral| / |along|:
 *
 *     k = 2.5 + 1.5 / (1 + μ)
 *     η1 = η (1 − k/5)
 *     η2 = η1 (0.25 + 0.35 μ / (1 + μ))
 *
 *   k is the slope at the robot in units of η/ξ (the quartic's slope
 *   factor is 4). It is 4 on the axis and falls toward 2.5 as μ grows,
 *   so a wide miss pulls the first handle inward less. The second
 *   handle's fraction rises with μ, which moves the bend into the
 *   middle of the curve.
 *
 *   Hermite — the same quintic Bezier, with Hermite data at both ends:
 *   position, tangent, and curvature. The opening tangent is the
 *   measured momentum and the opening curvature is the measured yaw,
 *   so the path leaves along the motion the chassis already has. The
 *   closing tangent is the exit direction and the closing curvature is
 *   zero, so the slip angle vanishes on arrival and the heading is the
 *   exit direction.
 *
 *   Wheel speed lies along the chassis heading. At heading θ the field
 *   velocity is v_lin (cos θ, sin θ). Below 0.15 m/s that direction is
 *   the travel heading (opposite the nose when backing up) and the
 *   curvature is zero. With chord length L, unit momentum T0, its left
 *   normal N0, exit direction T1, and κ = ω / |v_lin|:
 *
 *     P1 = P0 + T0 L / 5
 *     P2 = P0 + 2 T0 L / 5 + N0 κ L² / 20
 *     P4 = P5 − T1 L / 5
 *     P3 = P5 − 2 T1 L / 5
 *
 *   which is B'(0) = 5 (P1 − P0) = L T0 and B''(0) = L² κ N0, and the
 *   same pair at the join with κ = 0. P1 = P0 + v̂ Δt with lookahead
 *   Δt = L / (5 |v|). P3, P4, and P5 lie on the exit axis. κ is held
 *   to 10/L so the P2 offset stays within L/2.
 *
 *   The tick commands the tangent and curvature a short distance along
 *   that curve. The sample at the robot repeats the measured motion, so
 *   the join would never steer the chassis. The distance is 0.25 s of
 *   travel speed, at least 0.30 m, and at most 35% of the remaining
 *   length. As that length falls, the sample draws in on the join,
 *   where the tangent is the exit direction and the curvature is zero.
 *
 * The travel direction φ is the path tangent at the robot, pointed along
 * the bend that arrives with the target heading. The nose leads that
 * tangent by the same centripetal slip as DriftBoomerang:
 *
 *   tan(σ) = v² κ / drift_accel
 *   chassis heading = φ + σ
 *
 * so the outward slide brings the velocity back onto the path. The slip
 * uses the planned speed, not the measured one.
 *
 * Remaining length, curvature, and the curvature slope pick the speed.
 * A backward pass from the exit speed takes the tightest of
 *
 *   v ≤ cruise_speed
 *   v ≤ sqrt(exit_speed² + 2 * accel * Δs)  at every sample ahead
 *   v² |κ| ≤ centripetal_accel
 *   v |κ| ≤ Robot::MAX_VEL_ANG
 *   v² |dκ/ds| ≤ Physics::MAX_ACCEL_ANG
 *
 * so a straight runs up to cruise and a corner only gives up the speed
 * the tires and the yaw motor cannot hold. accel is target.max_accel
 * when that is set, and config.path_accel otherwise. Each tick the
 * command steps from the measured speed toward that ceiling by at most
 * accel * dt. target.exit_speed is m/s along
 * target.exit_direction. Zero stops. Linear volts are the feedforward of
 * that speed and of the acceleration the step used. With exit_speed at 0
 * and the robot inside close_m, the heading target becomes the finish
 * heading (exit_direction, or the opposite of it when forwards is false).
 *
 * The endpoint is `target`, the same kind of public configuration as
 * `config`:
 *
 *   static DriftCurve nav;
 *   nav.target = {0.0, 0.0, 0.0, 0.0};   // x, y, exit direction, exit speed
 *   nav.target.max_accel = 1.5;          // optional, m/s^2; else config.path_accel
 *   nav.target.line_length = 12;         // inches of straight travel into the target
 *   nav.target.ignore_direction = true;  // straight at the point; keep exit_speed
 *   nav.target.backwards = true;         // nose points opposite the direction of travel
 *   nav.reset();
 *   out.commands = CommandVector::fromChassis(nav.compute(estimate));
 *
 * Keep one instance and call reset() when the target changes. compute()
 * expects a 10 ms tick; the Boomerang derivative gains are per tick.
 * ------------------------------------------------------------------------- */

#include "control/DriftPathFit.h"
#include "core/Types.h"
#include "core/Specs.h"
#include "core/Tuning.h"
#include "interfaces/IFeedForward.h"
#include "math/MathUtils.h"
#include <cmath>
#include <math.h>
#include <optional>

enum class DriftCurveShape {
    Arc, Exponential, Quartic, Quintic, Hermite, QuarticHermite
};

struct DriftCurveConfig {
    // DriftCurveShape shape = DriftCurveShape::Arc;
    // DriftCurveShape shape = DriftCurveShape::Quartic;
    // DriftCurveShape shape = DriftCurveShape::Exponential;
    // DriftCurveShape shape = DriftCurveShape::Quintic;
    DriftCurveShape shape = DriftCurveShape::Hermite;
    // DriftCurveShape shape = DriftCurveShape::QuarticHermite;
    double exp_rate = Drift::EXP_RATE;       // 1/m, Exponential only. See the header note.
    double close_m = Boomerang::CLOSE_IN * kMetersPerInch;
    double drift_accel = Drift::DRIFT_ACCEL; // m/s^2; tan(slip) = v^2 * kappa / this
    double max_slip_rad = Drift::MAX_SLIP_RAD;
    double centripetal_accel = Drift::CENTRIPETAL_ACCEL; // m/s^2; v^2 |κ| cap on the profile
    double speed_reward = Drift::SPEED_REWARD;           // >1 holds more speed, looser tracking
    bool   forwards = true;            // false backs up; the nose finishes opposite exit_direction
    double max_volts = System::MAX_VOLTS;

    double path_accel = Drift::PATH_ACCEL; // m/s^2, used when target.max_accel is unset
    double cruise_speed = Robot::MAX_VEL_LIN;
};

// One drive. x and y are inches, the same frame localization publishes.
// exit_direction is the field heading of the velocity at the target (0 = +X,
// CCW positive). exit_speed is m/s along that direction; 0 stops. max_accel
// is the most the speed may change, in m/s^2. Leave it unset to use
// config.path_accel. line_length is the straight finish into the target, in
// inches. 0 fits the curve to the target. compute() converts these to meters
// before the curvature and braking math, which stays in m/s.
// ignore_direction drops exit_direction: the robot drives straight at the
// point and only the exit speed is used. backwards drives in reverse: the
// nose points opposite the direction of travel. config.forwards still
// applies when backwards is false.
struct DriftCurveTarget {
    double x = 0;
    double y = 0;
    double exit_direction = 0;
    double exit_speed = 100;
    std::optional<double> max_accel;
    double line_length = 0.5 * kInchesPerMeter;
    bool ignore_direction = false;
    bool backwards = false;
    // Latched swing. The plan is the orbit about the goal, and the command
    // presses into the point while yawing onto exit heading. See PathConstraints.
    bool turn_on_goal = false;

    // heading is the nose. v along the nose becomes exit_speed; negative v
    // reverses, so the travel direction is opposite the nose.
    static DriftCurveTarget fromMotion(const MotionPose& pose, const PathConstraints& path = {}) {
        DriftCurveTarget target;
        const bool reverse = pose.v < 0.0;
        target.x = pose.x;
        target.y = pose.y;
        target.exit_direction = reverse
            ? mathutil::normalize_angle(pose.heading + Robot::PI)
            : pose.heading;
        target.exit_speed = std::fabs(pose.v);
        target.line_length = path.line_length_in;
        target.ignore_direction = path.ignore_heading;
        target.backwards = reverse;
        target.turn_on_goal = path.turn_on_goal;
        if (path.max_accel_mps2 >= 0.0) target.max_accel = path.max_accel_mps2;
        return target;
    }
};

// Public target is inches. Path fitting and braking run in meters.
inline DriftCurveTarget targetInMeters(const DriftCurveTarget& field) {
    DriftCurveTarget out = field;
    out.x = inchesToMeters(field.x);
    out.y = inchesToMeters(field.y);
    out.line_length = inchesToMeters(field.line_length);
    return out;
}

class DriftCurve {
public:
    DriftCurveConfig config;
    DriftCurveTarget target;

    void reset();

    // Chassis model for the speed rule. Null leaves the linear volts at
    // zero. The pointer is not owned.
    void bindFeedforward(const IFeedForward* model) { model_ = model; }

    // state : field pose, forward speed (v_lin, m/s), yaw rate (v_ang, rad/s).
    // Hermite and QuarticHermite read the speed and the yaw rate.
    // The other shapes use the pose.
    // Uses `target`. Call reset() after changing it.
    ChassisCommand compute(const StateEstimate& state);

    // Heading the chassis was told to face on the last compute(), radians.
    double goalHeading() const { return goal_heading_; }

    // True inside the Boomerang pose window and within 0.2 m/s of
    // target.exit_speed. The caller decides when to drop the override.
    bool settled() const { return settled_; }

    // Polyline of the path compute() is tracking, from `from` to `target`.
    // Returns the number of points written, at most cap. Zero when the
    // robot is already on the target. v_lin (m/s along the nose) and
    // v_ang (rad/s) are the measured momentum. Hermite and QuarticHermite
    // open along them.
    int samplePath(const Pose2d& from, Pose2d* out, int cap,
                   double v_lin = 0, double v_ang = 0) const;

private:
    double i_lin_ = 0, prev_lin_ = 0;
    double i_ang_ = 0, prev_ang_ = 0;
    double goal_heading_ = 0;
    bool   primed_ = false;
    bool   settled_ = false;
    const IFeedForward* model_ = nullptr;
};
