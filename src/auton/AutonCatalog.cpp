/* ---------------------------------------------------------------------------
 * AutonCatalog.cpp  --  names, start poses, and builders for each side.
 *
 * The field drawing samples a built routine from the row's start pose.
 * Changing the highlighted row does not move the robot. Controller Y and
 * competition autonomous() build the highlighted row with the live model.
 * The plant module defines PLANT_MODULE and has no PROS kernel, so its
 * copy of this lock is a standard mutex.
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
    Pose2d start;
    Builder build;
};

Pose2d startAt(double x_in, double y_in, double heading_deg) {
    return Pose2d{x_in, y_in, heading_deg * (Robot::PI / 180.0)};
}

const Entry kRed[] = {
    {"Low side",  startAt(56, 20, 90),   auton::buildRedLow},
    {"High side", startAt(56, 124, 180), auton::buildRedHigh},
    {"Center",    startAt(40, 18, 90),   auton::buildRedCenter},
};

const Entry kBlue[] = {
    {"Low side",  startAt(-56, 20, 90),  auton::buildBlueLow},
    {"High side", startAt(-56, 124, 0),  auton::buildBlueHigh},
    {"Center",    startAt(-40, 18, 90),  auton::buildBlueCenter},
};

// Skills begins at the field center, facing +X, which is where calibration
// leaves the simulated robot.
const Entry kSkills[] = {
    {"Skills", startAt(0, Robot::Range::CENTER_Y_IN, 0), auton::buildSkillsRoutine},
};

const Entry* entries(auton::AutonSide side, int& count) {
    switch (side) {
        case auton::AutonSide::Red:
            count = (int)(sizeof(kRed) / sizeof(kRed[0]));
            return kRed;
        case auton::AutonSide::Blue:
            count = (int)(sizeof(kBlue) / sizeof(kBlue[0]));
            return kBlue;
        case auton::AutonSide::Skills:
            count = (int)(sizeof(kSkills) / sizeof(kSkills[0]));
            return kSkills;
    }
    count = 0;
    return nullptr;
}

int sideSlot(auton::AutonSide side) {
    const int slot = static_cast<int>(side);
    if (slot < 0 || slot > 2) return 2;
    return slot;
}

#if defined(PLANT_MODULE)
std::mutex g_mu;
#else
pros::Mutex g_mu;
#endif
auton::AutonSide g_side = auton::AutonSide::Skills;
int g_index[3] = {0, 0, 0};

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
        case AutonSide::Red: return "Red auto";
        case AutonSide::Blue: return "Blue auto";
        case AutonSide::Skills: return "Skills";
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
    return AutonSelection{g_side, g_index[sideSlot(g_side)]};
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

std::unique_ptr<Command> buildSelected(const IFeedForward* model) {
    Builder build = nullptr;
    {
        const Lock lock(g_mu);
        int n = 0;
        const Entry* table = entries(g_side, n);
        const int index = g_index[sideSlot(g_side)];
        if (table != nullptr && index >= 0 && index < n) build = table[index].build;
    }
    if (build == nullptr) return nullptr;
    return build(model).build();
}

int previewSelected(Pose2d* out, int cap) {
    if (out == nullptr || cap < 2) return 0;
    Entry chosen{};
    bool ok = false;
    {
        const Lock lock(g_mu);
        int n = 0;
        const Entry* table = entries(g_side, n);
        const int index = g_index[sideSlot(g_side)];
        if (table != nullptr && index >= 0 && index < n && table[index].build != nullptr) {
            chosen = table[index];
            ok = true;
        }
    }
    if (!ok) return 0;
    std::unique_ptr<Command> route = chosen.build(nullptr).build();
    if (!route) return 0;
    Pose2d from = chosen.start;
    return route->appendRoute(from, out, 0, cap);
}

} // namespace auton
