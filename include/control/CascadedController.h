#pragma once
/* ---------------------------------------------------------------------------
 * CascadedController  --  v1's adaptive follower, re-expressed as an IController.
 *
 * Pipeline: reference warp -> (Ramsete OR position-PID) pose correction ->
 * injected feedforward (or recorded-volt feedforward) -> velocity PID feedback.
 * The controller carries an internal EKF purely to smooth measured velocity and
 * the command-derivative used for feedforward and the PID D-term, exactly as in
 * v1. Its output is a chassis-frame effort; the DriveBase mixes to actuators.
 * ------------------------------------------------------------------------- */

#include "interfaces/IController.h"
#include "interfaces/IFeedForward.h"
#include "interfaces/IMotionLimiter.h"
#include "control/StaticMotionLimiter.h"   // defaultMotionLimiter()
#include "math/ExtendedKalmanFilter.h"

class CascadedController : public IController {
public:
    explicit CascadedController(const IFeedForward& feedforward,
                                const IMotionLimiter& limiter = defaultMotionLimiter())
        : feedforward_(&feedforward), limiter_(&limiter) {}

    void init() override;
    void reset() override;
    ChassisCommand computeCommand(const Pose2d& pose,
                                  const MotionTarget& target,
                                  const ChassisObservation& obs) override;

    double lastSpatialError() const override { return spatial_error_; }
    double lastAngularError() const override { return angular_error_; }
    double lastTimeWarp() const override     { return time_warp_; }

private:
    // Velocity-loop PID state
    double integral_error_lin = 0, prev_error_lin = 0;
    double integral_error_ang = 0, prev_error_ang = 0;
    // Position-loop PID state
    double integral_error_lin_pos = 0, prev_error_lin_pos = 0;
    double integral_error_ang_pos = 0, prev_error_ang_pos = 0;
    // Command-derivative memory
    double prev_v_cmd = 0, prev_omega_cmd = 0;

    ExtendedKalmanFilter ekf_;
    const IFeedForward* feedforward_;
    const IMotionLimiter* limiter_;

    // Diagnostics from the last call
    double spatial_error_ = 0, angular_error_ = 0, time_warp_ = 1.0;
};
