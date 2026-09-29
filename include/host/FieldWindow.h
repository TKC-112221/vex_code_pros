#pragma once
/* ---------------------------------------------------------------------------
 * FieldWindow  --  desktop stand-in for the V5 controller and brain screen.
 * The host link provides the Win32 implementation. The V5 image does not
 * compile this directory.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"

namespace fieldwindow {

// Sample the keyboard into the joystick and button state MockDrive returns.
void pollInput(JoystickData& joy, ButtonState& buttons);

// Open the field window if needed. truth is the plant; estimate is the
// pose the running localizer currently reports. particles is a sample of
// the filter cloud (ignored when count is 0).
void render(const Pose2d& truth, const Pose2d& estimate,
            const Pose2d* particles = nullptr, int particle_count = 0);

// Curve the controller is tracking, in field meters. While active, the
// window shows this polyline. When active goes false the last curve stays
// for 5 seconds and then clears. count 0 is not a curve.
void setFollowPath(const Pose2d* points, int count, bool active);

}
