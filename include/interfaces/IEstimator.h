#pragma once
/* ---------------------------------------------------------------------------
 * IEstimator  --  fuses chassis observations into a pose/velocity estimate.
 *
 * Consumes ONLY the drivebase-agnostic ChassisObservation, so it never needs
 * to know the chassis geometry. Descendants: OdometryEstimator (trapezoidal
 * dead-reckoning, the v1 default) and KalmanEstimator (EKF-smoothed pose).
 * ------------------------------------------------------------------------- */

#include "core/Types.h"

class IEstimator {
public:
    virtual ~IEstimator() = default;

    virtual void initialize() {}
    virtual void reset() = 0;
    virtual void overridePose(const Pose2d& pose) = 0;

    // Advance the estimate with one observation.
    virtual void update(const ChassisObservation& obs) = 0;

    // Current best estimate (pose is authoritative; velocity is convenience).
    virtual StateEstimate getState() const = 0;

    // Unwrapped heading accumulator, for display only.
    virtual double getTotalHeading() const { return 0.0; }
};
