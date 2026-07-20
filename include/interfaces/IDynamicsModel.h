#pragma once
/* ---------------------------------------------------------------------------
 * IDynamicsModel  --  the "physics simulator": a model of the drivetrain that
 * predicts the voltage required to hold/achieve a given motion. Mother interface
 * so the model is swappable (learned MLP today; an analytical motor model or a
 * higher-fidelity sim could implement the same contract). Used by
 * PhysicsMotionLimiter to invert "voltage(motion)" into "max motion(voltage)".
 * ------------------------------------------------------------------------- */

class IDynamicsModel {
public:
    virtual ~IDynamicsModel() = default;

    // Volts to achieve linear (velocity, accel) at the given angular rate.
    virtual double linearVolts(double velocity, double accel, double omega) const = 0;
    // Volts (differential) to achieve angular (omega, alpha) at the given speed.
    virtual double angularVolts(double omega, double alpha, double abs_lin_vel) const = 0;
};
