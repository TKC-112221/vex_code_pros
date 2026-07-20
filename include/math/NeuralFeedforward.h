#pragma once
/* ---------------------------------------------------------------------------
 * NeuralFeedforward  --  thin, typed wrapper over the auto-generated MLP
 * dynamics model (data/mlp_feedforward.h). It predicts the steady-state chassis
 * voltage required to achieve a requested (velocity, acceleration) -- the
 * feedforward term of the CascadedController. Keeping it behind this facade
 * means the controller depends on an intent ("predict linear volts"), not on
 * the generated function names.
 * ------------------------------------------------------------------------- */

#include "data/mlp_feedforward.h"
#include "math/MathUtils.h"
#include <cmath>

namespace NeuralFeedforward {

// Linear channel: volts_avg to achieve (velocity, accel) given kinematic context.
inline double linearVolts(double velocity, double accel, double abs_lin_vel,
                          double abs_lin_accel, double omega, double turn_radius) {
    float sign_vel = (std::fabs(velocity) < 0.02f) ? 0.0f : (float)mathutil::sign(velocity);
    return (double)mlp_predict_lin((float)velocity, (float)accel, sign_vel,
                                   (float)std::fabs(abs_lin_vel), (float)std::fabs(abs_lin_accel),
                                   (float)std::fabs(omega), (float)turn_radius);
}

// Angular channel: volts_diff (V_R - V_L) to achieve (omega, alpha).
inline double angularVolts(double omega, double alpha, double abs_lin_vel,
                           double abs_lin_accel, double turn_radius) {
    float sign_omega = (std::fabs(omega) < 0.02f) ? 0.0f : (float)mathutil::sign(omega);
    return (double)mlp_predict_ang((float)omega, (float)alpha, sign_omega,
                                   (float)std::fabs(abs_lin_vel), (float)std::fabs(abs_lin_accel),
                                   (float)turn_radius);
}

} // namespace NeuralFeedforward
