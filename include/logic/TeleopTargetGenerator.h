#pragma once
/* ---------------------------------------------------------------------------
 * TeleopTargetGenerator  --  turns driver input into a motion intent.
 * Concrete ITeleopSource.
 *
 * The teleop "motion source": conditions the sticks (curve -> scale -> low-pass
 * -> slew), integrates a virtual "ghost robot" pose, and folds in the two teleop
 * assists tightly coupled to that integration (face on A, pivot on B). The stick
 * curve is injected (IDriveCurve); the velocity/accel ceilings used to scale and
 * rate-limit come from an injected IMotionLimiter, so the command clamp can be
 * static or physics-predicted without touching this class.
 * ------------------------------------------------------------------------- */

#include "interfaces/ITeleopSource.h"
#include "interfaces/IDriveCurve.h"
#include "interfaces/IMotionLimiter.h"
#include "control/StaticMotionLimiter.h"   // defaultMotionLimiter()
#include "filters/LowPassFilter.h"
#include "filters/SlewLimiter.h"

class TeleopTargetGenerator : public ITeleopSource {
public:
    explicit TeleopTargetGenerator(const IDriveCurve& curve,
                                   const IMotionLimiter& limiter = defaultMotionLimiter());

    void resetToPose(const Pose2d& pose) override;

    TeleopDrive generate(RobotMode mode, const JoystickData& joy, const ButtonState& btn,
                         const Pose2d& pose, const ChassisObservation& obs) override;

private:
    const IDriveCurve&    curve_;
    const IMotionLimiter& limiter_;
    LowPassFilter lpf_linear_{0.3};
    LowPassFilter lpf_angular_{0.3};
    SlewLimiter   slew_linear_;
    SlewLimiter   slew_angular_;

    // Ghost robot pose
    double vt_x_ = 0, vt_y_ = 0, vt_theta_ = 0;
    double prev_target_v_ = 0, prev_target_omega_ = 0;

    // Teleop assist state
    bool prev_a_ = false, prev_b_ = false, pivot_active_ = false;
};
