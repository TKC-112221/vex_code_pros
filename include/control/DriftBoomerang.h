#pragma once
/* ---------------------------------------------------------------------------
 * DriftBoomerang  --  one control tick of a boomerang drive whose nose is
 * aimed off the carrot by the expected sideslip.
 *
 * The chase point is LemLib's moveToPose carrot: a point `lead * distance`
 * behind the target, opposite the approach heading. LemLib steers the nose
 * at that point. This tick steers the nose so the velocity after lateral
 * drift points there instead.
 *
 * The carrot field has a curvature κ (how fast the bearing to the carrot
 * turns as the robot advances along it). At forward speed v the path needs
 * a centripetal acceleration a = v² κ. A linear slip model turns that into
 * a sideslip angle
 *
 *   tan(σ) = v² κ / drift_accel
 *
 * with σ signed the same way as κ (positive κ is a left / CCW bend). The
 * chassis heading is the carrot bearing plus σ, so the nose leads into the
 * bend and the outward slide brings the velocity back onto the carrot.
 * drift_accel is the lateral acceleration that would make σ = 45°; it is
 * the same centripetal relation LemLib writes as v = sqrt(μ r g), solved
 * for heading instead of a speed cap. Raise it for a grippier robot.
 *
 * Inside close_m of the target the carrot collapses onto the target and the
 * heading target becomes the requested heading, matching LemLib's settle.
 *
 * Output is a chassis-frame effort in volts, the same units
 * TankDriveVelocityMapper writes, so the caller can replace an override
 * command with it:
 *
 *   static DriftBoomerang nav;
 *   out.commands = CommandVector::fromChassis(nav.compute(estimate, goal));
 *
 * Keep one instance: the PID and the linear slew remember the previous tick.
 * Call reset() when the goal changes. compute() expects a 10 ms tick; the
 * Boomerang derivative gains are per tick, matching LemLib.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"
#include "core/Specs.h"

struct DriftBoomerangConfig {
    double lead = 1.0;                 // LemLib moveToPose lead, 0..1
    double close_m = 7.5 * 0.0254;     // LemLib's hardcoded 7.5 inch settle
    double drift_accel = 5.5;//9.81;         // m/s^2; tan(slip) = v^2 * kappa / this
    double max_slip_rad = 0.7;         // clamp on the nose lead angle
    bool   forwards = true;            // false reverses along the approach
    double max_volts = System::MAX_VOLTS;
};

class DriftBoomerang {
public:
    DriftBoomerangConfig config;

    void reset();

    // state  : field pose plus measured forward speed (v_lin, m/s)
    // target : field pose to finish on (x, y, heading)
    ChassisCommand compute(const StateEstimate& state, const Pose2d& target);

    // Heading the chassis was told to face on the last compute(), radians.
    double goalHeading() const { return goal_heading_; }

    // True when the robot is inside the Boomerang small-error window of the
    // target pose. The caller decides when to drop the override.
    bool settled() const { return settled_; }

private:
    double i_lin_ = 0, prev_lin_ = 0;
    double i_ang_ = 0, prev_ang_ = 0;
    double prev_surge_ = 0;
    double goal_heading_ = 0;
    bool   primed_ = false;
    bool   settled_ = false;
};
