#pragma once
/* ---------------------------------------------------------------------------
 * IEstimator  --  fuses chassis observations into a pose/velocity estimate.
 *
 * Consumes a SensorDict. Every estimator reads chassis; distance[0..3] is
 * Ports::D1..D4 and is used only by estimators that can fuse a wall range
 * (KalmanEstimator, ParticleFilterEstimator, SlipEnsembleEstimator,
 * OdometryEstimator).
 *
 * getState(), overridePose(), sampleParticles(), and the optional absolute
 * fixes use field inches. The pose is the middle of the front bumper.
 * update() still takes the geometric center's travel and ranges in meters;
 * each filter keeps that geometry internally and converts here.
 * ------------------------------------------------------------------------- */

#include "core/Types.h"

class IEstimator {
public:
    virtual ~IEstimator() = default;

    virtual void initialize() {}
    virtual void reset() = 0;
    virtual void overridePose(const Pose2d& pose) = 0;

    // Published body speed. A routine that places the robot at rest calls
    // this after overridePose so the next snapshot is stopped.
    virtual void clearVelocity() {}

    // Advance the estimate with one sensor dict. Encoder deltas in
    // sensors.chassis must be applied exactly once.
    virtual void update(const SensorDict& sensors) = 0;

    // Optional absolute field fix (GPS, vision). Estimators that only
    // dead-reckon ignore these. x/y and sigma are inches, the published frame.
    virtual void observePosition(double /*x_in*/, double /*y_in*/, double /*sigma_in*/) {}
    virtual void observePose(const Pose2d& /*pose*/, double /*sigma_xy_in*/,
                             double /*sigma_theta_rad*/) {}

    // Current best estimate. Pose x/y are inches; velocity stays m/s and rad/s.
    virtual StateEstimate getState() const = 0;

    // Unwrapped heading accumulator, for display only.
    virtual double getTotalHeading() const { return 0.0; }

    // Copy up to cap particle poses for display. Odometry returns 0.
    virtual int sampleParticles(Pose2d* /*out*/, int /*cap*/) const { return 0; }
};
