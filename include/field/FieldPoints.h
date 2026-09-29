#pragma once
/* ---------------------------------------------------------------------------
 * FieldPoints  --  named positions on the 12 ft field.
 *
 * x and y are meters in the Pose2d frame: origin at the field center, +x
 * toward the field's +X wall, +y toward the +Y wall. A spot has no heading.
 * Pass the approach heading into BoomerangProcessor::start when a drive begins.
 *
 * Add a spot by inserting an enumerator above Count and a row in kSpots in
 * the same order.
 * ------------------------------------------------------------------------- */

#include "core/Specs.h"

#include <cstddef>
#include <cstdint>

namespace Field {

enum class Point : uint8_t {
    Center,
    Align,
    Count
};

struct Spot {
    double x;
    double y;
    const char* name;
};

constexpr Spot kSpots[] = {
    {0.0, 0.0, "center"},
    {Robot::Align::POINT_X_M, Robot::Align::POINT_Y_M, "align"},
};

static_assert(sizeof(kSpots) / sizeof(kSpots[0]) == static_cast<std::size_t>(Point::Count),
              "kSpots must have one row per Field::Point, in enum order");

inline const Spot& spot(Point id) {
    auto i = static_cast<std::size_t>(id);
    if (i >= static_cast<std::size_t>(Point::Count)) i = 0;
    return kSpots[i];
}

} // namespace Field
