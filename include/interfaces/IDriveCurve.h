#pragma once
/* ---------------------------------------------------------------------------
 * IDriveCurve  --  a joystick response shaping curve.
 *
 * Maps a normalized input in [-1, 1] to a shaped output in [-1, 1]. Descendants:
 * CubicDriveCurve (v1's cube blend), LinearDriveCurve (identity/passthrough).
 * ------------------------------------------------------------------------- */

class IDriveCurve {
public:
    virtual ~IDriveCurve() = default;
    virtual double apply(double input) const = 0;
};
