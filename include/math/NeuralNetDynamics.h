#pragma once
/* ---------------------------------------------------------------------------
 * NeuralNetDynamics  --  IDynamicsModel backed by the learned MLP feedforward
 * (data/mlp_feedforward.h). This is the same identified dynamics the controller
 * uses for feedforward, exposed behind the IDynamicsModel contract so the
 * motion limiter can invert it.
 * ------------------------------------------------------------------------- */

#include "interfaces/IDynamicsModel.h"
#include "math/NeuralFeedforward.h"
#include "core/Limits.h"
#include <algorithm>
#include <cmath>

class NeuralNetDynamics : public IDynamicsModel {
public:
    double linearVolts(double velocity, double accel, double omega) const override {
        double turn_r = (std::fabs(omega) < 0.01) ? Physics::TURN_RADIUS_MAX_M
                      : std::min(std::fabs(velocity) / (std::fabs(omega) + 0.01), Physics::TURN_RADIUS_MAX_M);
        return NeuralFeedforward::linearVolts(velocity, accel, std::fabs(velocity),
                                              std::fabs(accel), omega, turn_r);
    }
    double angularVolts(double omega, double alpha, double abs_lin_vel) const override {
        double turn_r = (std::fabs(omega) < 0.01) ? Physics::TURN_RADIUS_MAX_M
                      : std::min(std::fabs(abs_lin_vel) / (std::fabs(omega) + 0.01), Physics::TURN_RADIUS_MAX_M);
        return NeuralFeedforward::angularVolts(omega, alpha, std::fabs(abs_lin_vel), 0.0, turn_r);
    }
};
