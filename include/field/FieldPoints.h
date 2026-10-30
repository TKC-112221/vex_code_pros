#pragma once
/* ---------------------------------------------------------------------------
 * FieldPoints  --  named positions on the 12 ft field.
 *
 * x and y are inches in the Pose2d frame: origin at the center of the
 * bottom edge, +x toward the right wall, +y toward the far wall. The
 * geometric center of the field is (0, 72). A spot has no heading.
 * Pass the approach heading into BoomerangProcessor::start when a drive begins.
 *
 * kind says what the spot is. A Goal whose hitbox_in is greater than
 * standoff_in is a lock target: the aligner may seat the middle of the
 * front edge on it. A Loader is drawn with the goals and the aligner
 * ignores it. Set hitbox_in to 0 on a Goal to keep that spot as a named
 * position the lock will not use.
 *
 * Add a spot by inserting an enumerator above Count and a row in kSpots in
 * the same order. A loader row sets kind to Kind::Loader. A goal row may
 * leave kind off; it defaults to Kind::Goal.
 *
 * base_height_in is the cascade height, in inches, of an empty goal. The
 * score sequence remembers a height per goal, starting at this value, and
 * adds to it when a piece is scored. Skills and match autos reset every
 * goal to that base when they start. Loaders leave it at 0. goalHeightIn()
 * is the live height.
 * ------------------------------------------------------------------------- */

#include "core/Specs.h"
#include "core/Types.h"
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace Field {

enum class Point : uint8_t {
    X2Y2,
    XM2Y2,
    X4Y4,
    XM4Y4,
    X0Y12,
    X4Y8,
    XM4Y8,
    X2Y10,
    XM2Y10,
    Loader,
    LoaderM,
    Count
};

// Goal is a lock target. Loader is the same named point with no aligner.
enum class Kind : uint8_t {
    Goal,
    Loader,
};

// Shared goal size, inches. A row in kSpots may use its own radii instead.
constexpr double kHitboxIn = 4.0;
constexpr double kStandoffIn = 2.0;

struct Spot {
    double x;
    double y;
    const char* name;
    double hitbox_in;
    double standoff_in;
    // Empty-goal cascade height, inches. Sits before kind so a goal row
    // can omit kind. The live height starts here.
    double base_height_in = 0.0;
    Kind kind = Kind::Goal;
};

// Positions are feet from the field origin, stored as inches.
constexpr double kInPerFt = 12.0;

constexpr Spot kSpots[] = {
    { 2.0 * kInPerFt,  2.0 * kInPerFt,  "2, 2",   kHitboxIn, kStandoffIn, 3.5},
    {-2.0 * kInPerFt,  2.0 * kInPerFt, "-2, 2",   kHitboxIn, kStandoffIn, 6.5},
    { 4.0 * kInPerFt,  4.0 * kInPerFt,  "4, 4",   kHitboxIn, kStandoffIn, 3.5},
    {-4.0 * kInPerFt,  4.0 * kInPerFt, "-4, 4",   kHitboxIn, kStandoffIn, 6.5},
    { 0.0 * kInPerFt,  6.0 * kInPerFt,  "0, 6",   kHitboxIn, kStandoffIn, 9.5},
    { 4.0 * kInPerFt,  8.0 * kInPerFt,  "4, 8",   kHitboxIn, kStandoffIn, 6.5},
    {-4.0 * kInPerFt,  8.0 * kInPerFt, "-4, 8",   kHitboxIn, kStandoffIn, 3.5},
    { 2.0 * kInPerFt, 10.0 * kInPerFt,  "2, 10",  kHitboxIn, kStandoffIn, 6.5},
    {-2.0 * kInPerFt, 10.0 * kInPerFt, "-2, 10",  kHitboxIn, kStandoffIn, 3.5},
    // Inches, on the near wall. Five feet either side of the origin.
    { 60.0, 0.0,  "loader",  0.0, 0.0, 0.0, Kind::Loader},
    {-60.0, 0.0, "-loader", 0.0, 0.0, 0.0, Kind::Loader},
};

static_assert(sizeof(kSpots) / sizeof(kSpots[0]) == static_cast<std::size_t>(Point::Count),
              "kSpots must have one row per Field::Point, in enum order");

inline const Spot& spot(Point id) {
    auto i = static_cast<std::size_t>(id);
    if (i >= static_cast<std::size_t>(Point::Count)) i = 0;
    return kSpots[i];
}

// The goal lock can attach to this spot and center the front edge on it.
// Loaders are never lock targets.
inline bool isAlignGoal(const Spot& s) {
    return s.kind == Kind::Goal
        && s.hitbox_in > s.standoff_in
        && s.standoff_in >= 0.0;
}

inline bool isLoader(const Spot& s) {
    return s.kind == Kind::Loader;
}

// Remembered cascade height of each spot, inches. Goals start at
// base_height_in. The score sequence writes a new value when it scores.
inline double* goalHeights() {
    static double h[static_cast<std::size_t>(Point::Count)]{};
    static bool ready = false;
    if (!ready) {
        for (std::size_t i = 0; i < static_cast<std::size_t>(Point::Count); ++i)
            h[i] = kSpots[i].base_height_in;
        ready = true;
    }
    return h;
}

inline double goalHeightIn(int index) {
    if (index < 0 || index >= static_cast<int>(Point::Count)) return 0.0;
    return goalHeights()[static_cast<std::size_t>(index)];
}

inline void setGoalHeightIn(int index, double inches) {
    if (index < 0 || index >= static_cast<int>(Point::Count)) return;
    if (!std::isfinite(inches) || inches < 0.0) inches = 0.0;
    goalHeights()[static_cast<std::size_t>(index)] = inches;
}

// Puts every spot back to base_height_in. Called when a skills or match
// auto starts, so a previous run does not carry scored height forward.
inline void resetGoalHeights() {
    double* h = goalHeights();
    for (std::size_t i = 0; i < static_cast<std::size_t>(Point::Count); ++i)
        h[i] = kSpots[i].base_height_in;
}

// Radius of the keep-out around an align goal, inches. The path stays
// at least this far from the goal point. It is not grown by the chassis.
// A drive whose target sits inside the circle may enter it on the last
// segment; every other segment stays outside.
inline double goalAvoidIn() {
    return 12.0;
}

// One circle per align goal. Loaders are left out. Returns how many were
// written, at most cap.
inline int goalAvoidObstacles(PathObstacle* out, int cap) {
    if (out == nullptr || cap <= 0) return 0;
    const double radius = goalAvoidIn();
    int n = 0;
    const int spots = static_cast<int>(Point::Count);
    for (int i = 0; i < spots && n < cap; ++i) {
        const Spot& s = kSpots[i];
        if (!isAlignGoal(s)) continue;
        out[n] = PathObstacle{s.x, s.y, radius, true};
        ++n;
    }
    return n;
}

} // namespace Field
