/* ---------------------------------------------------------------------------
 * DrivePolicy.cpp  --  feature packing and the actor forward pass.
 * The training env calls read() through the pybind module so the 10 ms
 * firmware sees the same vector the policy was trained on.
 * ------------------------------------------------------------------------- */
#include "control/DrivePolicy.h"
#include "control/DrivePolicyWeights.h"
#include "control/DriftPathFit.h"
#include "core/Specs.h"
#include "math/MathUtils.h"
#include "math/NeuralNetwork.h"

#include <cmath>

namespace {

NeuralNetwork& network() {
    static NeuralNetwork model(DrivePolicy::kLayerSizes, DrivePolicy::kLayerCount,
                               NeuralNetwork::Activation::SILU);
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

float clamped(double value) {
    return unit(mathutil::clamp_val(value, -1.0, 1.0));
}

struct Memory {
    double v_surge[DrivePolicy::kHistoryDepth] {};
    double v_sway[DrivePolicy::kHistoryDepth] {};
    double v_yaw[DrivePolicy::kHistoryDepth] {};
    double a_surge[DrivePolicy::kHistoryDepth] {};
    double a_yaw[DrivePolicy::kHistoryDepth] {};
    double last_surge = 0;
    double last_yaw = 0;
};

Memory& memory() {
    static Memory mem;
    return mem;
}

void bodyOffset(const Pose2d& pose, double px, double py, double& bx, double& by) {
    const double dx = px - pose.x;
    const double dy = py - pose.y;
    const double c = std::cos(pose.theta);
    const double s = std::sin(pose.theta);
    bx = c * dx + s * dy;
    by = -s * dx + c * dy;
}

void pointAlong(const driftpath::Quintic& path, double distance, double& x, double& y) {
    driftpath::quinticAt(path, 0.0, x, y);
    if (distance <= 1e-4) return;
    double px = x;
    double py = y;
    double length = 0.0;
    for (int i = 1; i <= driftpath::kLengthSteps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(driftpath::kLengthSteps);
        double qx = 0.0;
        double qy = 0.0;
        driftpath::quinticAt(path, t, qx, qy);
        const double step = std::hypot(qx - px, qy - py);
        if (length + step >= distance) {
            const double frac = (step > 1e-9) ? (distance - length) / step : 1.0;
            x = px + frac * (qx - px);
            y = py + frac * (qy - py);
            return;
        }
        length += step;
        px = qx;
        py = qy;
    }
    driftpath::quinticAt(path, 1.0, x, y);
}

// Body-frame samples of the Hermite from the robot to the target, at the
// fixed stations. Past the curve the point rides the straight finish and
// stops on the target. path_phi becomes the travel tangent at 0.2 m.
void fillLook(const StateEstimate& state, const DriftCurveTarget& target,
              double* look_x, double* look_y, double& path_phi) {
    const DriftCurveTarget meters = targetInMeters(target);
    const Pose2d pose = poseInchesToMeters(state.pose);
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.theta)) return;

    const double dist = std::hypot(meters.x - pose.x, meters.y - pose.y);
    double approach = meters.exit_direction;
    if (!std::isfinite(approach)) return;
    if (meters.ignore_direction && dist > 1e-4) {
        approach = std::atan2(meters.y - pose.y, meters.x - pose.x);
    }

    const driftpath::Finish fin = driftpath::finishOf(
        pose.x, pose.y, meters.x, meters.y, approach, meters.line_length);
    const bool to_join = fin.length > 1e-4 && !fin.past && !fin.on_line;
    const double fit_x = to_join ? fin.jx : meters.x;
    const double fit_y = to_join ? fin.jy : meters.y;

    driftpath::Quintic path;
    bool curved = false;
    if (!fin.on_line && dist > 1e-4) {
        const double v_lin = std::isfinite(state.v_lin) ? state.v_lin : 0.0;
        const double v_ang = std::isfinite(state.v_ang) ? state.v_ang : 0.0;
        const driftpath::Momentum momentum = driftpath::momentumOf(
            pose.theta, !meters.backwards, v_lin, v_ang);
        path = driftpath::hermiteThrough(
            pose.x, pose.y, fit_x, fit_y, approach,
            momentum.x, momentum.y, momentum.kappa);
        curved = path.curved && path.length_m > 1e-4;
    }

    if (curved) {
        double phi = approach;
        double kappa = 0.0;
        driftpath::quinticLook(path, DrivePolicy::kLookaheadM[0], phi, kappa);
        if (std::isfinite(phi)) path_phi = phi;
    } else if (dist > 1e-4) {
        path_phi = std::atan2(meters.y - pose.y, meters.x - pose.x);
    }

    for (int i = 0; i < DrivePolicy::kLookahead; ++i) {
        const double distance = DrivePolicy::kLookaheadM[i];
        double px = meters.x;
        double py = meters.y;
        if (curved && distance <= path.length_m) {
            pointAlong(path, distance, px, py);
        } else if (curved) {
            double end_x = 0.0;
            double end_y = 0.0;
            pointAlong(path, path.length_m, end_x, end_y);
            const double extra = distance - path.length_m;
            const double remain = std::hypot(meters.x - end_x, meters.y - end_y);
            if (remain < 1e-6 || extra >= remain) {
                px = meters.x;
                py = meters.y;
            } else {
                const double scale = extra / remain;
                px = end_x + (meters.x - end_x) * scale;
                py = end_y + (meters.y - end_y) * scale;
            }
        } else {
            const double along = (meters.x - pose.x) * std::cos(approach)
                               + (meters.y - pose.y) * std::sin(approach);
            if (along <= 1e-4 || distance >= along) {
                px = meters.x;
                py = meters.y;
            } else {
                const double scale = distance / along;
                px = pose.x + (meters.x - pose.x) * scale;
                py = pose.y + (meters.y - pose.y) * scale;
            }
        }
        bodyOffset(pose, px, py, look_x[i], look_y[i]);
    }
}

void pushMemory(double v_lin, double v_sway, double v_yaw, DrivePolicy::FollowInput& in) {
    Memory& mem = memory();
    for (int i = DrivePolicy::kHistoryDepth - 1; i > 0; --i) {
        mem.v_surge[i] = mem.v_surge[i - 1];
        mem.v_sway[i] = mem.v_sway[i - 1];
        mem.v_yaw[i] = mem.v_yaw[i - 1];
        mem.a_surge[i] = mem.a_surge[i - 1];
        mem.a_yaw[i] = mem.a_yaw[i - 1];
    }
    mem.v_surge[0] = v_lin;
    mem.v_sway[0] = v_sway;
    mem.v_yaw[0] = v_yaw;
    mem.a_surge[0] = mem.last_surge;
    mem.a_yaw[0] = mem.last_yaw;
    for (int frame = 0; frame < DrivePolicy::kHistory; ++frame) {
        const int lag = DrivePolicy::kHistoryLag[frame];
        in.v_surge[frame] = mem.v_surge[lag];
        in.v_sway[frame] = mem.v_sway[lag];
        in.v_yaw[frame] = mem.v_yaw[lag];
        in.a_surge[frame] = mem.a_surge[lag];
        in.a_yaw[frame] = mem.a_yaw[lag];
    }
}

void storeAction(const CommandVector& cmd) {
    const double volts = System::MAX_VOLTS;
    Memory& mem = memory();
    mem.last_surge = (volts > 1e-6) ? cmd.surge / volts : 0.0;
    mem.last_yaw = (volts > 1e-6) ? cmd.yaw / volts : 0.0;
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

void reset() {
    memory() = Memory{};
}

Reading read(const FollowInput& in) {
    Reading out;
    const double cross_unit = (kCrossUnitIn > 1e-9) ? kCrossUnitIn : 1.0;
    const double heading_error = mathutil::normalize_angle(in.path_phi - in.body_heading);
    const double lin_scale = Robot::MAX_VEL_LIN;
    const double ang_scale = Robot::MAX_VEL_ANG;

    for (int frame = 0; frame < kHistory; ++frame) {
        const int base = frame * kHistoryChannels;
        const double surge = in.v_surge[frame] / lin_scale;
        const double sway = in.v_sway[frame] / lin_scale;
        const double yaw = in.v_yaw[frame] / ang_scale;
        out.features[base + kHistSurge] = clamped(surge);
        out.features[base + kHistSway] = clamped(sway);
        out.features[base + kHistYaw] = clamped(yaw);
        out.features[base + kHistSurgeCmd] = clamped(in.a_surge[frame]);
        out.features[base + kHistYawCmd] = clamped(in.a_yaw[frame]);
    }

    out.features[kDirSin] = unit(std::sin(heading_error));
    out.features[kDirCos] = unit(std::cos(heading_error));
    for (int i = 0; i < kLookahead; ++i) {
        const double span = (kLookaheadM[i] > 1e-9) ? kLookaheadM[i] : 1.0;
        out.features[kLookStart + 2 * i] = clamped(in.look_x_m[i] / span);
        out.features[kLookStart + 2 * i + 1] = clamped(in.look_y_m[i] / span);
    }
    out.features[kTargetThrust] = unit(in.target_thrust);
    out.features[kCrossTrack] = unit(std::tanh(in.cross_track_in / cross_unit));

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
    cmd.sway = 0.0;
    cmd.yaw = axis(y[1]);
    return cmd;
}

CommandVector command(const StateEstimate& state, const DriftCurveTarget& target) {
    FollowInput in;
    const double nose = noseTarget(target);
    in.path_phi = nose;
    const double v_lin = std::isfinite(state.v_lin) ? state.v_lin : 0.0;
    const double v_ang = std::isfinite(state.v_ang) ? state.v_ang : 0.0;
    in.body_heading = std::isfinite(state.pose.theta) ? state.pose.theta : 0.0;
    const double dx = state.pose.x - target.x;
    const double dy = state.pose.y - target.y;
    in.cross_track_in = -std::sin(nose) * dx + std::cos(nose) * dy;
    in.target_thrust = 1.0;

    pushMemory(v_lin, 0.0, v_ang, in);
    fillLook(state, target, in.look_x_m, in.look_y_m, in.path_phi);

    const CommandVector cmd = command(in);
    storeAction(cmd);
    return cmd;
}

} // namespace DrivePolicy
