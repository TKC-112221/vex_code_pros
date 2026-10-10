#pragma once
/* ---------------------------------------------------------------------------
 * UserInputOverride  --  optional command rewrite after stick mapping.
 *
 * Each override lives in its own file. This function only checks that
 * override's run flag and its abort flag, then calls it. With neither set,
 * the mapped command passes through.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"

class IFeedForward;

// sensor    : latest ChassisObservation from RobotReading
// commands  : surge/sway/yaw from the velocity mapper
// buttons   : live button state
// estimate  : localization snapshot (pose + velocity)
// raw_sticks: joystick before the drive curve, for the stick-abort deadband
// mode      : current robot mode
// model     : chassis feedforward used by an override that needs one
// Returns possibly-overridden commands and buttons.
OverrideResult userInputOverride(const ChassisObservation& sensor,
                                 const CommandVector& commands,
                                 const ButtonState& buttons,
                                 const StateEstimate& estimate,
                                 const JoystickData& raw_sticks,
                                 RobotMode mode,
                                 const IFeedForward* model);

// Polyline of whichever override is running, for the field window.
// Returns 0 when none is. v_lin and v_ang are the measured momentum
// Hermite opens along.
int userInputOverridePath(const Pose2d& from, Pose2d* out, int cap,
                          double v_lin = 0, double v_ang = 0);
