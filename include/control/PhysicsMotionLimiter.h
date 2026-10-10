#pragma once
/* ---------------------------------------------------------------------------
 * PhysicsMotionLimiter  --  a command clamp that predicts the feasible max
 * velocity / acceleration at the current operating point instead of using fixed
 * constants. It inverts an IDynamicsModel ("voltage required for this motion")
 * against the available bus voltage:
 *
 *   max velocity : largest v with  |volts(v, accel=0)| <= V_available
 *   max accel    : largest a with  |volts(v_now, a)|   <= V_available
 *
 * So as the robot speeds up (back-EMF eats headroom) or the battery sags, the
 * predicted accel/vel ceilings drop -- a physically grounded, time-varying clamp.
 * Inversion is a bounded bisection (the model is ~monotonic in v and a).
 * ------------------------------------------------------------------------- */

#include "interfaces/IMotionLimiter.h"
#include "interfaces/IDynamicsModel.h"
#include "core/Specs.h"
#include "core/Tuning.h"
#include <algorithm>
#include <cmath>

class PhysicsMotionLimiter : public IMotionLimiter {
public:
    // headroom_frac in (0,1]: fraction of available volts to spend (margin).
    explicit PhysicsMotionLimiter(const IDynamicsModel& model, double headroom_frac = 1.0)
        : model_(model), headroom_(headroom_frac) {}

    double maxLinearVelocity(const LimiterContext& c) const override {
        double V = available(c);
        return bisect([&](double v){ return std::fabs(model_.linearVolts(v, 0.0, 0.0)); },
                      V, Robot::MAX_VEL_LIN * 2.0);
    }
    double maxLinearAccel(const LimiterContext& c) const override {
        double V = available(c);
        double v = std::fabs(c.v_lin);
        return bisect([&](double a){ return std::fabs(model_.linearVolts(v, a, 0.0)); },
                      V, Physics::MAX_ACCEL_LIN * 3.0);
    }
    double maxAngularVelocity(const LimiterContext& c) const override {
        double V = available(c);
        return bisect([&](double w){ return std::fabs(model_.angularVolts(w, 0.0, 0.0)); },
                      V, Robot::MAX_VEL_ANG * 2.0);
    }
    double maxAngularAccel(const LimiterContext& c) const override {
        double V = available(c);
        double w = std::fabs(c.v_ang), vl = std::fabs(c.v_lin);
        return bisect([&](double al){ return std::fabs(model_.angularVolts(w, al, vl)); },
                      V, Physics::MAX_ACCEL_ANG * 3.0);
    }

private:
    const IDynamicsModel& model_;
    double headroom_;

    double available(const LimiterContext& c) const {
        double b = (c.battery_voltage > 1.0) ? c.battery_voltage : System::MAX_VOLTS;
        return headroom_ * std::min(b, System::MAX_VOLTS);
    }

    // Largest x in [0, hi] with volts_at(x) <= Vmax, assuming volts_at is
    // nondecreasing in x. Returns 0 if even x=0 already exceeds Vmax.
    template <class F>
    static double bisect(F volts_at, double Vmax, double hi) {
        if (volts_at(0.0) >= Vmax) return 0.0;
        double lo = 0.0;
        for (int i = 0; i < 24; ++i) {
            double mid = 0.5 * (lo + hi);
            if (volts_at(mid) > Vmax) hi = mid; else lo = mid;
        }
        return lo;
    }
};
