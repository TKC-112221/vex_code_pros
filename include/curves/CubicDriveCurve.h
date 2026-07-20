#pragma once
/* ---------------------------------------------------------------------------
 * CubicDriveCurve  --  blend of cubic and linear response:
 *   y = x^3 * ratio + x * (1 - ratio)
 * ratio = 1 -> pure cubic (fine low-speed control); ratio = 0 -> linear.
 * Ported from v1's a3_cmd/a1_cmd shaping. A descendant of IDriveCurve.
 * ------------------------------------------------------------------------- */

#include "interfaces/IDriveCurve.h"

class CubicDriveCurve : public IDriveCurve {
public:
    explicit CubicDriveCurve(double ratio) : ratio_(ratio) {}
    double apply(double x) const override {
        return x * x * x * ratio_ + x * (1.0 - ratio_);
    }
private:
    double ratio_;
};
