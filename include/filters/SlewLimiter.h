#pragma once
/* ---------------------------------------------------------------------------
 * SlewLimiter  --  rate limiter; caps the per-tick change of a signal.
 * Ported from v1's InternalSlewLimiter. A descendant of IInputFilter.
 * ------------------------------------------------------------------------- */

#include "interfaces/IInputFilter.h"

class SlewLimiter : public IInputFilter {
public:
    explicit SlewLimiter(double max_step) : max_step_(max_step) {}

    double apply(double target) override {
        double change = target - current_;
        if (change >  max_step_) change =  max_step_;
        if (change < -max_step_) change = -max_step_;
        current_ += change;
        return current_;
    }
    void reset(double value = 0.0) override { current_ = value; }

    // Retune the per-tick cap (used when a dynamic accel ceiling drives the rate).
    void setMaxStep(double max_step) { max_step_ = max_step; }

private:
    double max_step_;
    double current_ = 0.0;
};
