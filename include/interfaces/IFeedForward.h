#pragma once
/* ---------------------------------------------------------------------------
 * IFeedForward  --  predicts the chassis voltage required to achieve a
 * requested motion. The controller and the drivebase current limiter depend on
 * this contract, never on a particular model.
 *
 * Descendants: NeuralFeedforward (learned MLP), VASController (kV / kA / kS).
 * ------------------------------------------------------------------------- */

class IFeedForward {
public:
    virtual ~IFeedForward() = default;

    // Load model data (weights, identified limits). No-op for analytic models.
    virtual void init() {}

    // Linear channel: volts_avg to achieve (velocity, accel) given kinematic context.
    virtual double linearVolts(double velocity, double accel, double abs_lin_vel,
                               double abs_lin_accel, double omega, double turn_radius) const = 0;

    // Angular channel: volts_diff (V_R - V_L) to achieve (omega, alpha).
    virtual double angularVolts(double omega, double alpha, double abs_lin_vel,
                                double abs_lin_accel, double turn_radius) const = 0;
};
