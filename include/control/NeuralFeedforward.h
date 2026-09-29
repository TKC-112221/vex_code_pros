#pragma once
/* ---------------------------------------------------------------------------
 * NeuralFeedforward  --  IFeedForward backed by learned MLP dynamics.
 * Two NeuralNetwork heads (linear and angular) predict steady-state chassis
 * voltage. init() reads identified limits and weights from the SD card. Until
 * those weights load, predictions stay zero so the rest of the pipeline links.
 * ------------------------------------------------------------------------- */

#include "interfaces/IFeedForward.h"
#include "math/NeuralNetwork.h"

class NeuralFeedforward : public IFeedForward {
public:
    NeuralFeedforward();

    void init() override;

    double linearVolts(double velocity, double accel, double abs_lin_vel,
                       double abs_lin_accel, double omega, double turn_radius) const override;

    double angularVolts(double omega, double alpha, double abs_lin_vel,
                        double abs_lin_accel, double turn_radius) const override;

private:
    NeuralNetwork linear_;
    NeuralNetwork angular_;
};
