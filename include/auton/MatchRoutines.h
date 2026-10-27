#pragma once
/* ---------------------------------------------------------------------------
 * MatchRoutines  --  red and blue routes listed by the autonomous selector.
 * Edit the builders here. Register a name and a start pose in AutonCatalog.
 * ------------------------------------------------------------------------- */

#include "auton/Routine.h"

class IFeedForward;

namespace auton {

Routine buildRedLow(const IFeedForward* model);
Routine buildRedHigh(const IFeedForward* model);
Routine buildRedCenter(const IFeedForward* model);
Routine buildBlueLow(const IFeedForward* model);
Routine buildBlueHigh(const IFeedForward* model);
Routine buildBlueCenter(const IFeedForward* model);

} // namespace auton
