#pragma once
/* ---------------------------------------------------------------------------
 * GoalAligner  --  lock the front edge onto a goal point.
 *
 * A goal is a Field::Point the aligner is allowed to use. When any part of
 * the front edge comes within GoalAlign::FRONT_TOUCH_M of that point, the
 * chassis attaches and squares up: the heading turns onto the point and the
 * middle of the front edge settles on it.
 *
 * The stick override is the driver layer. Forward is dropped, the yaw stick
 * is the orbit rate, and reverse lets go for the rest of the tick. The plant
 * grab is the physics layer. It runs after commanded motion and wall contact,
 * in driver control, autonomous, and replay, and it overwrites that slice so
 * the point wins. Backing the wheels off releases the grab. Changing mode
 * leaves the latch alone.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"

class IFeedForward;

// One tick. `commands` is the mapped stick effort. `body` is the chassis
// the touch test uses, in field inches. Forward is read from the raw stick
// so the deadband matches the rest of driver control.
// Returns `commands` when no goal owns the drive.
CommandVector goalAlignOverride(const CommandVector& commands,
                                const Pose2d& body,
                                const JoystickData& raw_sticks,
                                const IFeedForward* model);

// Let go for the rest of this tick. Reverse uses this. The plant will not
// grab again until goalAlignPhysicsDone, so a release is not undone by the
// same step that is still overlapping the point.
void goalAlignRelease();

// The grab and the stick override. The firmware leaves this on. A rollout
// turns it off so a bumper touch does not replace the commanded motion.
void goalAlignSetEnabled(bool enabled);

// Close the one-tick release window. stepContacts calls this after the grab.
void goalAlignPhysicsDone();

// Latched Field::Point index, or -1. The field window draws that goal hot.
int goalAlignIndex();

// Plant grab, after this slice has already moved. x_m, y_m, and theta are
// that pose in meters and radians. forward_mps is the body-forward speed at
// the start of the physics step. On true, gx and gy are the latched goal in
// meters and the caller replaces the slice. Backing up, or a release earlier
// this tick, returns false.
bool goalAlignGrab(double x_m, double y_m, double theta, double forward_mps,
                   double& gx, double& gy);

// Arc of the center rotating about the latched goal, nose centered on it.
// Returns 0 when nothing is latched.
int goalAlignPath(const Pose2d& from, Pose2d* out, int cap);

// Point a latched turn pivots on, in meters. The latched goal when one is
// held. Otherwise the nearest align goal to the front center, when that
// point is close enough to be the bumper the robot is on. Otherwise the
// front center itself, which is where a seated nose sits.
void goalAlignPivot(const Pose2d& pose_in, double& gx, double& gy);
