#pragma once
/* ---------------------------------------------------------------------------
 * VASController  --  IFeedForward using the traditional armature model
 *
 *   volts = kS * sgn(velocity) + kV * velocity + kA * acceleration
 *
 * Linear and angular channels carry their own gains (m/s vs rad/s). Extra
 * kinematic context used by the neural model is ignored.
 * ------------------------------------------------------------------------- */

#include "interfaces/IFeedForward.h"
#include "math/MathUtils.h"

class VASController : public IFeedForward {
public:
    VASController(double kS_lin, double kV_lin, double kA_lin,
                  double kS_ang, double kV_ang, double kA_ang)
        : kS_lin_(kS_lin), kV_lin_(kV_lin), kA_lin_(kA_lin),
          kS_ang_(kS_ang), kV_ang_(kV_ang), kA_ang_(kA_ang) {}

    void setGains(double kS_lin, double kV_lin, double kA_lin,
                  double kS_ang, double kV_ang, double kA_ang) {
        kS_lin_ = kS_lin; kV_lin_ = kV_lin; kA_lin_ = kA_lin;
        kS_ang_ = kS_ang; kV_ang_ = kV_ang; kA_ang_ = kA_ang;
    }

    double linearVolts(double velocity, double accel, double /*abs_lin_vel*/,
                       double /*abs_lin_accel*/, double /*omega*/,
                       double /*turn_radius*/) const override {
        return kS_lin_ * mathutil::sign(velocity) + kV_lin_ * velocity + kA_lin_ * accel;
    }

    double angularVolts(double omega, double alpha, double /*abs_lin_vel*/,
                        double /*abs_lin_accel*/, double /*turn_radius*/) const override {
        return kS_ang_ * mathutil::sign(omega) + kV_ang_ * omega + kA_ang_ * alpha;
    }

private:
    double kS_lin_, kV_lin_, kA_lin_;
    double kS_ang_, kV_ang_, kA_ang_;
};
