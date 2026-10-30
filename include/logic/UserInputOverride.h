#pragma once
/* ---------------------------------------------------------------------------
 * UserInputOverride  --  optional command rewrite after stick mapping.
 *
 * Each override lives in its own file. The drift curve runs while its
 * flag is set. Otherwise the goal lock may replace the stick command
 * when the real chassis front edge is on a goal point. The plant snap
 * is separate and stays armed while a curve or a routine is driving.
 * With neither stick override active, the mapped command passes through.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"

class IFeedForward;

// sensor    : latest ChassisObservation from RobotReading
// commands  : surge/sway/yaw from the velocity mapper
// buttons   : live button state
// estimate  : localization snapshot (pose + velocity)
// body      : chassis pose the goal touch test uses. The host passes the
//             plant. The brain passes the estimate.
// raw_sticks: joystick before the drive curve, for the stick-abort deadband
// mode      : current robot mode
// model     : chassis feedforward used by an override that needs one
// motion    : when set, this control vector replaces the stick command.
//             A routine passes the vector it just calculated. Null leaves
//             the stick path, including the drift curve and the goal lock.
// Returns possibly-overridden commands and buttons.
OverrideResult userInputOverride(const ChassisObservation& sensor,
                                 const CommandVector& commands,
                                 const ButtonState& buttons,
                                 const StateEstimate& estimate,
                                 const Pose2d& body,
                                 const JoystickData& raw_sticks,
                                 RobotMode mode,
                                 const IFeedForward* model,
                                 const CommandVector* motion = nullptr);

// Polyline of whichever override is running, for the field window.
// Returns 0 when none is. v_lin and v_ang are the measured momentum
// Hermite opens along.
// `body` is the chassis the goal arc is drawn from. Drift preview still
// opens at `from`.
int userInputOverridePath(const Pose2d& from, const Pose2d& body,
                          Pose2d* out, int cap,
                          double v_lin = 0, double v_ang = 0);
