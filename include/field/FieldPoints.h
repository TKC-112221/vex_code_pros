#pragma once
/* ---------------------------------------------------------------------------
 * FieldPoints  --  named positions on the 12 ft field.
 *
 * x and y are inches in the Pose2d frame: origin at the center of the
 * bottom edge, +x toward the right wall, +y toward the far wall. The
 * geometric center of the field is (0, 72). A spot has no heading.
 * Pass the approach heading into BoomerangProcessor::start when a drive begins.
 *
 * hitbox_in and standoff_in only decide whether the goal lock may use the
 * spot. A spot is a goal when hitbox_in is greater than standoff_in. Set
 * hitbox_in to 0 to keep the spot as a named position. The lock itself
 * seats the middle of the front edge on the point.
 *
 * Add a spot by inserting an enumerator above Count and a row in kSpots in
 * the same order.
 * ------------------------------------------------------------------------- */

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
    Count
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
};

// Positions are feet from the field origin, stored as inches.
constexpr double kInPerFt = 12.0;

constexpr Spot kSpots[] = {
    { 2.0 * kInPerFt,  2.0 * kInPerFt,  "2, 2",   kHitboxIn, kStandoffIn},
    {-2.0 * kInPerFt,  2.0 * kInPerFt, "-2, 2",   kHitboxIn, kStandoffIn},
    { 4.0 * kInPerFt,  4.0 * kInPerFt,  "4, 4",   kHitboxIn, kStandoffIn},
    {-4.0 * kInPerFt,  4.0 * kInPerFt, "-4, 4",   kHitboxIn, kStandoffIn},
    { 0.0 * kInPerFt, 6.0 * kInPerFt,  "0, 6",  kHitboxIn, kStandoffIn},
    { 4.0 * kInPerFt,  8.0 * kInPerFt,  "4, 8",   kHitboxIn, kStandoffIn},
    {-4.0 * kInPerFt,  8.0 * kInPerFt, "-4, 8",   kHitboxIn, kStandoffIn},
    { 2.0 * kInPerFt, 10.0 * kInPerFt,  "2, 10",  kHitboxIn, kStandoffIn},
    {-2.0 * kInPerFt, 10.0 * kInPerFt, "-2, 10",  kHitboxIn, kStandoffIn},
};

static_assert(sizeof(kSpots) / sizeof(kSpots[0]) == static_cast<std::size_t>(Point::Count),
              "kSpots must have one row per Field::Point, in enum order");

inline const Spot& spot(Point id) {
    auto i = static_cast<std::size_t>(id);
    if (i >= static_cast<std::size_t>(Point::Count)) i = 0;
    return kSpots[i];
}

// The goal lock can attach to this spot and center the front edge on it.
inline bool isAlignGoal(const Spot& s) {
    return s.hitbox_in > s.standoff_in && s.standoff_in >= 0.0;
}

} // namespace Field
