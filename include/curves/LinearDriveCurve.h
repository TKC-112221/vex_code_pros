#pragma once
/* ---------------------------------------------------------------------------
 * LinearDriveCurve  --  identity response (no shaping). A descendant of
 * IDriveCurve, provided to demonstrate the swap and as a debugging baseline.
 * ------------------------------------------------------------------------- */

#include "interfaces/IDriveCurve.h"

class LinearDriveCurve : public IDriveCurve {
public:
    double apply(double x) const override { return x; }
};
