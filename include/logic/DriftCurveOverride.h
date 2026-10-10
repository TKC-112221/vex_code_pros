#pragma once
/* ---------------------------------------------------------------------------
 * DriftCurveOverride  --  one user-input override: follow DriftCurve::target.
 *
 * Set the endpoint with DriftCurveTarget::fromMotion on a MotionPose
 * (x, y, heading, speed), then set g_drift_curve_on.
 * g_drift_curve_abort stops it. The default target
 * is a stop at the origin, leaving along +X. A alone while idle arms whatever
 * target is currently stored.
 * ------------------------------------------------------------------------- */

#include "control/DriftCurve.h"
#include "core/Types.h"

class IFeedForward;

// The curve this override drives. Write `target` before turning it on.
//DriftCurve& driftCurve();

// Running. The dispatcher in UserInputOverride reads this.
extern bool g_drift_curve_on;

// Abort commanded (sticks while A is released, leaving driver control,
// calibrate, auton, disable). The dispatcher clears the run flag when this is set.
extern bool g_drift_curve_abort;

// A edge, stick deflection while A is released, and leaving idle. Updates the two flags.
void driftCurveOverridePoll(const ButtonState& buttons,
                            const JoystickData& raw_sticks,
                            RobotMode mode);

// One tick of the curve. Caller runs this only while g_drift_curve_on.
CommandVector driftCurveOverride(const StateEstimate& estimate,
                                 const IFeedForward* model);

// Drop the run flag, clear the abort, and reset the curve.
void driftCurveOverrideStop();

// Polyline of driftCurve().target. Returns 0 when the curve is too short.
// v_lin and v_ang are the measured momentum Hermite opens along.
int driftCurveOverridePath(const Pose2d& from, Pose2d* out, int cap,
                           double v_lin = 0, double v_ang = 0);
