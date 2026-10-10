#pragma once
/* ---------------------------------------------------------------------------
 * NeuralNetDynamics  --  IDynamicsModel backed by an injected IFeedForward
 * (the same model the controller uses). Exposed behind the IDynamicsModel
 * contract so the motion limiter can invert it.
 * ------------------------------------------------------------------------- */

#include "interfaces/IDynamicsModel.h"
#include "interfaces/IFeedForward.h"
#include "core/Tuning.h"
#include <algorithm>
#include <cmath>

class NeuralNetDynamics : public IDynamicsModel {
public:
    explicit NeuralNetDynamics(const IFeedForward& feedforward) : feedforward_(&feedforward) {}

    double linearVolts(double velocity, double accel, double omega) const override {
        double turn_r = (std::fabs(omega) < Physics::OMEGA_EPS) ? Physics::TURN_RADIUS_MAX_M
                      : std::min(std::fabs(velocity) / (std::fabs(omega) + Physics::OMEGA_EPS), Physics::TURN_RADIUS_MAX_M);
        return feedforward_->linearVolts(velocity, accel, std::fabs(velocity),
                                         std::fabs(accel), omega, turn_r);
    }
    double angularVolts(double omega, double alpha, double abs_lin_vel) const override {
        double turn_r = (std::fabs(omega) < Physics::OMEGA_EPS) ? Physics::TURN_RADIUS_MAX_M
                      : std::min(std::fabs(abs_lin_vel) / (std::fabs(omega) + Physics::OMEGA_EPS), Physics::TURN_RADIUS_MAX_M);
        return feedforward_->angularVolts(omega, alpha, std::fabs(abs_lin_vel), 0.0, turn_r);
    }

private:
    const IFeedForward* feedforward_;
};
