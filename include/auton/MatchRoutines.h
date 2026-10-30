#pragma once
/* ---------------------------------------------------------------------------
 * MatchRoutines  --  match routes listed by the autonomous selector.
 * Edit the builders here. Register a name in AutonCatalog. startAt() on the
 * routine is the pose that route begins at. Poses are the red side. The
 * selector's color toggle mirrors the route onto blue.
 * ------------------------------------------------------------------------- */

#include "auton/Routine.h"

class IFeedForward;

namespace auton {

Routine buildLeftSide2(const IFeedForward* model);
Routine buildRightSide2(const IFeedForward* model);

} // namespace auton
