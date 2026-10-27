#pragma once
/* ---------------------------------------------------------------------------
 * AutonCatalog  --  the routines the selector can run.
 *
 * Red, blue, and skills each have their own list. The host field window
 * draws the list for the side that is open and the path of the highlighted
 * routine. Competition autonomous() and controller Y run that same choice.
 * On the brain the choice stays at skills until something selects another.
 *
 * A new routine is a builder plus one row in src/auton/AutonCatalog.cpp.
 * The row's start pose is where the field drawing begins. The follower
 * still plans from wherever the robot actually is when the routine runs.
 * ------------------------------------------------------------------------- */

#include "auton/Command.h"
#include <memory>

class IFeedForward;

namespace auton {

enum class AutonSide { Red = 0, Blue = 1, Skills = 2 };

struct AutonSelection {
    AutonSide side = AutonSide::Skills;
    int index = 0;
};

const char* autonSideName(AutonSide side);
int autonCount(AutonSide side);
const char* autonName(AutonSide side, int index);

AutonSelection currentSelection();
void selectSide(AutonSide side);
void selectIndex(int index);

std::unique_ptr<Command> buildSelected(const IFeedForward* model);

// Polyline of the selected routine from its start pose, field inches.
// 0 when that side has no routine.
int previewSelected(Pose2d* out, int cap);

} // namespace auton
