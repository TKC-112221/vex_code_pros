#pragma once
/* ---------------------------------------------------------------------------
 * SkillsRoutine  --  the competition command routine.
 * Edit src/auton/SkillsRoutine.cpp. The scheduler runs whatever build()
 * returns. An empty routine finishes on the first tick and leaves the robot
 * stopped.
 * ------------------------------------------------------------------------- */

#include "auton/Routine.h"

class IFeedForward;

namespace auton {

Routine buildSkillsRoutine(const IFeedForward* model);

} // namespace auton
