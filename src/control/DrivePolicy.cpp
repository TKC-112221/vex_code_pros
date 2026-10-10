/* ---------------------------------------------------------------------------
 * DrivePolicy.cpp  --  feature packing and the actor forward pass.
 * The training env calls read() through the pybind module so the 10 ms
 * firmware sees the same vector the policy was trained on.
 * ------------------------------------------------------------------------- */
#include "control/DrivePolicy.h"
#include "control/DrivePolicyWeights.h"
#include "core/Specs.h"
#include "math/MathUtils.h"
#include "math/NeuralNetwork.h"

#include <cmath>

namespace {

NeuralNetwork& network() {
    static NeuralNetwork model(DrivePolicy::kLayerSizes, DrivePolicy::kLayerCount,
                               NeuralNetwork::Activation::RELU);
    return model;
}

double noseTarget(const DriftCurveTarget& target) {
    if (target.backwards) {
        return mathutil::normalize_angle(target.exit_direction + Robot::PI);
    }
    return target.exit_direction;
}

float unit(double value) {
    return static_cast<float>(value);
}

} // namespace

namespace DrivePolicy {

void init() {
    if (!DrivePolicyWeights::kReady) return;
    assignWeights(DrivePolicyWeights::kValues, DrivePolicyWeights::kCount);
}

bool ready() {
    return network().ready();
}

bool assignWeights(const float* weights, int count) {
    return network().assignWeights(weights, count);
}

Reading read(const FollowInput& in) {
    Reading out;
    const double kappa_unit = (kKappaUnitPerIn > 1e-9) ? kKappaUnitPerIn : 1.0;
    const double rate_unit = (kKappaRateUnit > 1e-9) ? kKappaRateUnit : 1.0;
    const double cross_unit = (kCrossUnitIn > 1e-9) ? kCrossUnitIn : 1.0;
    const double heading_error = mathutil::normalize_angle(in.path_phi - in.vel_heading);

    out.features[kPrevSurge] = unit(in.prev_surge);
    out.features[kPrevSway] = unit(in.prev_sway);
    out.features[kPrevYaw] = unit(in.prev_yaw);
    out.features[kDirSin] = unit(std::sin(heading_error));
    out.features[kDirCos] = unit(std::cos(heading_error));
    out.features[kDPhiDs] = unit(in.dphi_ds / kappa_unit);
    out.features[kD2PhiDs2] = unit(in.d2phi_ds2 / rate_unit);
    out.features[kTargetThrust] = unit(in.target_thrust);
    out.features[kCrossTrack] = unit(in.cross_track_in / cross_unit);

    out.distance_in = std::fabs(in.cross_track_in);
    out.heading_error_rad = heading_error;
    out.at_target = out.distance_in <= kPositionToleranceIn;
    return out;
}

CommandVector command(const FollowInput& in) {
    CommandVector cmd;
    // A table assigned from Python (the export check) is left in place.
    // The compiled header loads only while the actor is still empty.
    if (!ready()) init();
    if (!ready()) return cmd;

    const Reading reading = read(in);
    float y[kOutputs] {};
    if (!network().predict(reading.features, y)) return cmd;

    const double volts = System::MAX_VOLTS;
    auto axis = [volts](float raw) {
        return static_cast<double>(std::tanh(raw)) * volts;
    };
    cmd.surge = axis(y[0]);
    cmd.sway = axis(y[1]);
    cmd.yaw = axis(y[2]);
    return cmd;
}

CommandVector command(const StateEstimate& state, const DriftCurveTarget& target) {
    FollowInput in;
    in.path_phi = noseTarget(target);
    const double vx = state.v_lin * std::cos(state.pose.theta);
    const double vy = state.v_lin * std::sin(state.pose.theta);
    if (std::hypot(vx, vy) < kStoppedSpeedMps) {
        in.vel_heading = state.pose.theta;
    } else {
        in.vel_heading = std::atan2(vy, vx);
    }
    const double dx = state.pose.x - target.x;
    const double dy = state.pose.y - target.y;
    const double phi = in.path_phi;
    in.cross_track_in = -std::sin(phi) * dx + std::cos(phi) * dy;
    in.target_thrust = 1.0;
    return command(in);
}

} // namespace DrivePolicy
