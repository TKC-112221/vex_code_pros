#pragma once
/* ---------------------------------------------------------------------------
 * DriftCurve  --  one control tick of a curve drive with DriftBoomerang's
 * sideslip heading.
 *
 * The path meets the target tangent to the target heading. shape picks
 * the family:
 *
 *   Arc — the circle through the robot. Its center lies on the normal
 *   through the target, and the signed radius is
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
 *   target, and ξ is distance along the heading (negative behind the
 *   target). Positive k holds the offset and hooks in near the target.
 *   Negative k sheds the offset early. Zero is the parabola ξ².
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
 * Remaining length s and the measured travel speed pick the speed. The
 * fastest speed that still arrives at target.exit_speed under the
 * acceleration limit is
 *
 *   v_ideal = min(cruise_speed, sqrt(exit_speed² + 2 * accel * s))
 *
 * accel is target.max_accel when that is set, and config.path_accel
 * otherwise. Each tick the command steps from the measured speed toward
 * v_ideal by at most accel * dt. target.exit_speed is m/s along
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
 *   nav.reset();
 *   out.commands = CommandVector::fromChassis(nav.compute(estimate));
 *
 * Keep one instance and call reset() when the target changes. compute()
 * expects a 10 ms tick; the Boomerang derivative gains are per tick.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"
#include "core/Specs.h"
#include "interfaces/IFeedForward.h"
#include <math.h>
#include <optional>

enum class DriftCurveShape { Arc, Exponential };

struct DriftCurveConfig {
    DriftCurveShape shape = DriftCurveShape::Arc;
    // DriftCurveShape shape = DriftCurveShape::Exponential;
    double exp_rate = -2.0;             // 1/m, Exponential only. See the header note.
    double close_m = 7.5 * 0.0254;     // settle distance, same window as DriftBoomerang
    double drift_accel = 5.0;//9.81;         // m/s^2; tan(slip) = v^2 * kappa / this
    double max_slip_rad = M_PI/2;         // clamp on the nose lead angle
    bool   forwards = true;            // false backs up; the nose finishes opposite exit_direction
    double max_volts = System::MAX_VOLTS;

    double path_accel = 2.0;           // m/s^2, used when target.max_accel is unset
    double cruise_speed = Robot::MAX_VEL_LIN;
};

// One drive. Coordinates are meters. exit_direction is the field heading
// of the velocity at the target (0 = +X, CCW positive). exit_speed is m/s
// along that direction; 0 stops. max_accel is the most the speed may change,
// in m/s^2. Leave it unset to use config.path_accel.
struct DriftCurveTarget {
    double x = 0;
    double y = 0;
    double exit_direction = 0;
    double exit_speed = 100;
    std::optional<double> max_accel;
};

class DriftCurve {
public:
    DriftCurveConfig config;
    DriftCurveTarget target;

    void reset();

    // Chassis model for the speed rule. Null leaves the linear volts at
    // zero. The pointer is not owned.
    void bindFeedforward(const IFeedForward* model) { model_ = model; }

    // state : field pose plus measured forward speed (v_lin, m/s)
    // Uses `target`. Call reset() after changing it.
    ChassisCommand compute(const StateEstimate& state);

    // Heading the chassis was told to face on the last compute(), radians.
    double goalHeading() const { return goal_heading_; }

    // True inside the Boomerang pose window and within 0.2 m/s of
    // target.exit_speed. The caller decides when to drop the override.
    bool settled() const { return settled_; }

    // Polyline of the path compute() is tracking, from `from` to `target`.
    // Returns the number of points written, at most cap. Zero when the
    // robot is already on the target.
    int samplePath(const Pose2d& from, Pose2d* out, int cap) const;

private:
    double i_lin_ = 0, prev_lin_ = 0;
    double i_ang_ = 0, prev_ang_ = 0;
    double goal_heading_ = 0;
    bool   primed_ = false;
    bool   settled_ = false;
    const IFeedForward* model_ = nullptr;
};
