#pragma once
/* ---------------------------------------------------------------------------
 * LowPassFilter  --  first-order exponential moving average (EMA).
 *   y[k] = alpha * x[k] + (1 - alpha) * y[k-1]
 * Ported from v1's LowPassFilter. A descendant of IInputFilter.
 * ------------------------------------------------------------------------- */

#include "interfaces/IInputFilter.h"

class LowPassFilter : public IInputFilter {
public:
    explicit LowPassFilter(double alpha = 1.0) : alpha_(alpha) {}

    double apply(double sample) override {
        current_ = alpha_ * sample + (1.0 - alpha_) * current_;
        return current_;
    }
    void reset(double value = 0.0) override { current_ = value; }

    void setAlpha(double alpha) { alpha_ = alpha; }

private:
    double alpha_;
    double current_ = 0.0;
};
