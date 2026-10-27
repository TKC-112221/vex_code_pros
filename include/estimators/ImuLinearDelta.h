#pragma once
/* ---------------------------------------------------------------------------
 * ImuLinearDelta  --  chassis travel for one localization sample.
 *
 * Forward travel is the wheel-encoder delta already on the observation.
 * That distance has already happened, so the step does not wait on the
 * stick. Sideways travel is the omni skid, the same discrete solution the
 * plant integrates:
 *
 *   d v_lat / dt = -ω v_fwd - v_lat / τ
 *
 * ω is the gyro rate and v_fwd is this tick's encoder speed. τ is the
 * plant's skid time constant. Every estimator reads the two deltas.
 *
 * Walls, a bumper snap, and the goal orbit are not part of this step.
 * LocalizationOverride rewrites the sample after apply().
 * ------------------------------------------------------------------------- */

#include "core/Types.h"
#include "core/Specs.h"
#include "hardware/MockDriveParams.h"
#include <cmath>

class ImuLinearDelta {
public:
    void reset() {
        v_fwd_ = 0.0;
        v_lat_ = 0.0;
    }

    double forwardVelocity() const { return v_fwd_; }
    double lateralVelocity() const { return v_lat_; }

    // obs.forward_delta stays the encoder distance. strafe_delta becomes
    // the skid over this same interval.
    void apply(ChassisObservation& obs, double dt_sec) {
        if (!(dt_sec > 0.0) || !std::isfinite(dt_sec)) {
            obs.v_inertial_fwd = v_fwd_;
            obs.v_inertial_lat = v_lat_;
            return;
        }
        double forward = obs.forward_delta;
        if (!std::isfinite(forward)) forward = 0.0;
        obs.forward_delta = forward;

        const double v_wheel = forward / dt_sec;
        const double omega = std::isfinite(obs.v_ang) ? obs.v_ang : 0.0;
        const double tau = MockPlant::SKID_TAU_SEC;
        const double limit = Robot::MAX_VEL_LIN * MockPlant::SPEED_SCALE;
        double v_lat = 0.0;
        if (tau > 1.0e-4) {
            const double decay = std::exp(-dt_sec / tau);
            v_lat = v_lat_ * decay + (-omega * v_wheel) * tau * (1.0 - decay);
            if (v_lat > limit) v_lat = limit;
            if (v_lat < -limit) v_lat = -limit;
        }
        obs.strafe_delta = 0.5 * (v_lat_ + v_lat) * dt_sec;
        v_fwd_ = v_wheel;
        v_lat_ = v_lat;
        obs.v_inertial_fwd = v_fwd_;
        obs.v_inertial_lat = v_lat_;
    }

    // Replace the integrated linear velocity. LocalizationOverride uses
    // this when a wall, a bumper, or a goal orbit keeps the sample.
    void setLinear(double v_fwd, double v_lat, ChassisObservation& obs) {
        v_fwd_ = v_fwd;
        v_lat_ = v_lat;
        obs.v_inertial_fwd = v_fwd_;
        obs.v_inertial_lat = v_lat_;
    }

private:
    double v_fwd_ = 0.0;
    double v_lat_ = 0.0;
};
