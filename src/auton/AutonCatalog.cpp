/* ---------------------------------------------------------------------------
 * AutonCatalog.cpp  --  names and builders for each side.
 *
 * The field drawing samples a built routine from that routine's startAt()
 * pose. A match routine is mirrored when blue is selected. Changing the
 * highlighted row does not move the robot. Controller Y and competition
 * autonomous() build the highlighted row with the live model, and a routine
 * that named a start is placed there before it runs. The plant module
 * defines PLANT_MODULE and has no PROS kernel, so its copy of this lock is
 * a standard mutex.
 * ------------------------------------------------------------------------- */

#include "auton/AutonCatalog.h"
#include "auton/MatchRoutines.h"
#include "auton/Routine.h"
#include "auton/SkillsRoutine.h"
#include "core/Specs.h"
#if defined(PLANT_MODULE)
#include <mutex>
#else
#include "pros/rtos.hpp"
#endif

namespace {

using Builder = auton::Routine (*)(const IFeedForward*);

struct Entry {
    const char* name;
    Builder build;
};

const Entry kMatch[] = {
    {"Left side",  auton::buildLeftSide2},
    {"Right side", auton::buildRightSide2},
};

const Entry kSkills[] = {
    {"Skills", auton::buildSkillsRoutine},
};

// A routine that omits startAt() is drawn from the field center, the pose
// calibration leaves the simulated robot in.
Pose2d drawingOrigin(const auton::Routine& route) {
    if (route.hasStart()) return route.startPose();
    return Pose2d{0.0, Robot::Range::CENTER_Y_IN, 0.0};
}

const Entry* entries(auton::AutonSide side, int& count) {
    switch (side) {
        case auton::AutonSide::Match:
            count = (int)(sizeof(kMatch) / sizeof(kMatch[0]));
            return kMatch;
        case auton::AutonSide::Skills:
            count = (int)(sizeof(kSkills) / sizeof(kSkills[0]));
            return kSkills;
    }
    count = 0;
    return nullptr;
}

int sideSlot(auton::AutonSide side) {
    const int slot = static_cast<int>(side);
    if (slot < 0 || slot > 1) return 1;
    return slot;
}

// Match routes are written on the red side. Blue is that route reflected
// across the field centerline. Skills stays where it was written.
bool mirrorRoute(auton::AutonSide side, auton::Alliance color) {
    return side == auton::AutonSide::Match && color == auton::Alliance::Blue;
}

#if defined(PLANT_MODULE)
std::mutex g_mu;
#else
pros::Mutex g_mu;
#endif
auton::AutonSide g_side = auton::AutonSide::Skills;
auton::Alliance g_alliance = auton::Alliance::Red;
int g_index[2] = {0, 0};

class Lock {
public:
#if defined(PLANT_MODULE)
    explicit Lock(std::mutex& mutex) : mutex_(mutex) { mutex_.lock(); }
    ~Lock() { mutex_.unlock(); }
#else
    explicit Lock(pros::Mutex& mutex) : mutex_(mutex) { mutex_.take(); }
    ~Lock() { mutex_.give(); }
#endif
    Lock(const Lock&) = delete;
    Lock& operator=(const Lock&) = delete;
private:
#if defined(PLANT_MODULE)
    std::mutex& mutex_;
#else
    pros::Mutex& mutex_;
#endif
};

void clampIndex(auton::AutonSide side) {
    int n = 0;
    entries(side, n);
    int& index = g_index[sideSlot(side)];
    if (n <= 0) index = 0;
    else if (index >= n) index = n - 1;
    else if (index < 0) index = 0;
}

} // namespace

namespace auton {

const char* autonSideName(AutonSide side) {
    switch (side) {
        case AutonSide::Match: return "Match";
        case AutonSide::Skills: return "Skills";
    }
    return "";
}

const char* allianceName(Alliance color) {
    switch (color) {
        case Alliance::Red: return "Red";
        case Alliance::Blue: return "Blue";
    }
    return "";
}

int autonCount(AutonSide side) {
    int n = 0;
    entries(side, n);
    return n;
}

const char* autonName(AutonSide side, int index) {
    int n = 0;
    const Entry* table = entries(side, n);
    if (table == nullptr || index < 0 || index >= n) return "";
    return table[index].name;
}

AutonSelection currentSelection() {
    const Lock lock(g_mu);
    return AutonSelection{g_side, g_index[sideSlot(g_side)], g_alliance};
}

void selectSide(AutonSide side) {
    const Lock lock(g_mu);
    g_side = side;
    clampIndex(side);
}

void selectIndex(int index) {
    const Lock lock(g_mu);
    g_index[sideSlot(g_side)] = index;
    clampIndex(g_side);
}

void selectAlliance(Alliance color) {
    const Lock lock(g_mu);
    g_alliance = color;
}

BuiltAuton buildSelected(const IFeedForward* model) {
    Builder build = nullptr;
    AutonSide side = AutonSide::Skills;
    Alliance color = Alliance::Red;
    {
        const Lock lock(g_mu);
        int n = 0;
        const Entry* table = entries(g_side, n);
        const int index = g_index[sideSlot(g_side)];
        if (table != nullptr && index >= 0 && index < n) build = table[index].build;
        side = g_side;
        color = g_alliance;
    }
    BuiltAuton out;
    if (build == nullptr) return out;
    Routine route = build(model);
    if (mirrorRoute(side, color)) route.mirror();
    out.has_start = route.hasStart();
    if (out.has_start) out.start = route.startPose();
    out.command = route.build();
    return out;
}

int previewSelected(Pose2d* out, int cap) {
    if (out == nullptr || cap < 2) return 0;
    Builder build = nullptr;
    AutonSide side = AutonSide::Skills;
    Alliance color = Alliance::Red;
    {
        const Lock lock(g_mu);
        int n = 0;
        const Entry* table = entries(g_side, n);
        const int index = g_index[sideSlot(g_side)];
        if (table != nullptr && index >= 0 && index < n) build = table[index].build;
        side = g_side;
        color = g_alliance;
    }
    if (build == nullptr) return 0;
    Routine route = build(nullptr);
    if (mirrorRoute(side, color)) route.mirror();
    std::unique_ptr<Command> command = route.build();
    if (!command) return 0;
    Pose2d from = drawingOrigin(route);
    return command->appendRoute(from, out, 0, cap);
}

} // namespace auton
