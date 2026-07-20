#pragma once
/* ---------------------------------------------------------------------------
 * IInputFilter  --  a scalar signal filter.
 *
 * The superclass for every "smooth a single stream of numbers" strategy:
 * LowPassFilter (EMA), SlewLimiter, and (future) a 1-D Kalman filter all share
 * this contract, so any stage that filters a signal can hold an IInputFilter*
 * and swap the strategy without changing its own code.
 * ------------------------------------------------------------------------- */

class IInputFilter {
public:
    virtual ~IInputFilter() = default;
    // Push one raw sample, get the filtered value back.
    virtual double apply(double sample) = 0;
    // Reset internal state (e.g. on calibration or mode change).
    virtual void reset(double value = 0.0) = 0;
};
