#pragma once
/* ---------------------------------------------------------------------------
 * IEstimator  --  fuses chassis observations into a pose/velocity estimate.
 *
 * Consumes a SensorDict. Every estimator reads chassis; distance[0..3] is
 * Ports::D1..D4 and is used only by estimators that can fuse a wall range
 * (KalmanEstimator, ParticleFilterEstimator, SlipEnsembleEstimator,
 * OdometryEstimator).
 * ------------------------------------------------------------------------- */

#include "core/Types.h"

class IEstimator {
public:
    virtual ~IEstimator() = default;

    virtual void initialize() {}
    virtual void reset() = 0;
    virtual void overridePose(const Pose2d& pose) = 0;

    // Advance the estimate with one sensor dict. Encoder deltas in
    // sensors.chassis must be applied exactly once.
    virtual void update(const SensorDict& sensors) = 0;

    // Optional absolute field fix (GPS, vision). Estimators that only
    // dead-reckon ignore these. sigma is the 1-sigma accuracy of the fix.
    virtual void observePosition(double /*x_m*/, double /*y_m*/, double /*sigma_m*/) {}
    virtual void observePose(const Pose2d& /*pose*/, double /*sigma_xy_m*/,
                             double /*sigma_theta_rad*/) {}

    // Current best estimate (pose is authoritative; velocity is convenience).
    virtual StateEstimate getState() const = 0;

    // Unwrapped heading accumulator, for display only.
    virtual double getTotalHeading() const { return 0.0; }

    // Copy up to cap particle poses for display. Odometry returns 0.
    virtual int sampleParticles(Pose2d* /*out*/, int /*cap*/) const { return 0; }
};
