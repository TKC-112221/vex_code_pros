#pragma once
/* ---------------------------------------------------------------------------
 * IMotionLimiter  --  supplies the max velocity / acceleration used to clamp
 * commanded motion. Mother interface for the "command clamp": a StaticMotion-
 * Limiter returns fixed tuned ceilings, while a PhysicsMotionLimiter predicts
 * the feasible ceiling at the current operating point (velocity + battery).
 *
 * The context carries the operating point the limit is evaluated "at the time".
 * ------------------------------------------------------------------------- */

struct LimiterContext {
    double v_lin = 0.0;             // current linear velocity  (m/s)
    double v_ang = 0.0;             // current angular velocity (rad/s)
    double battery_voltage = 12.0;  // available bus voltage    (V)
};

class IMotionLimiter {
public:
    virtual ~IMotionLimiter() = default;

    virtual double maxLinearVelocity(const LimiterContext& ctx) const = 0;
    virtual double maxLinearAccel(const LimiterContext& ctx) const = 0;
    virtual double maxAngularVelocity(const LimiterContext& ctx) const = 0;
    virtual double maxAngularAccel(const LimiterContext& ctx) const = 0;
};
