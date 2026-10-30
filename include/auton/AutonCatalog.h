#pragma once
/* ---------------------------------------------------------------------------
 * AutonCatalog  --  the routines the selector can run.
 *
 * Match and skills each have their own list. A match routine is written for
 * the red side; the color toggle mirrors it onto blue. The host field window
 * draws the list for the side that is open and the path of the highlighted
 * routine. Competition autonomous() and controller Y run that same choice.
 * On the brain the choice stays at skills until something selects another.
 *
 * A new routine is a builder plus one row in src/auton/AutonCatalog.cpp.
 * Routine::startAt() is where the field drawing begins and where the robot
 * is placed when that routine runs. A routine that omits it is drawn from
 * the field center and runs from the robot's current pose.
 * ------------------------------------------------------------------------- */

#include "auton/Command.h"
#include <memory>

class IFeedForward;

namespace auton {

enum class Alliance { Red = 0, Blue = 1 };

enum class AutonSide { Match = 0, Skills = 1 };

struct AutonSelection {
    AutonSide side = AutonSide::Skills;
    int index = 0;
    Alliance color = Alliance::Red;
};

const char* autonSideName(AutonSide side);
const char* allianceName(Alliance color);
int autonCount(AutonSide side);
const char* autonName(AutonSide side, int index);

AutonSelection currentSelection();
void selectSide(AutonSide side);
void selectIndex(int index);
void selectAlliance(Alliance color);

// The selected routine, ready to schedule. has_start is true when the
// routine named a start pose. command is null when that side has no routine.
struct BuiltAuton {
    std::unique_ptr<Command> command;
    bool has_start = false;
    Pose2d start{};
};

BuiltAuton buildSelected(const IFeedForward* model);

// Polyline of the selected routine from its start pose, field inches.
// 0 when that side has no routine.
int previewSelected(Pose2d* out, int cap);

} // namespace auton
