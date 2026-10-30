#include "logic/MatchLoader.h"
#include "logic/RobotMacro.h"
#include "field/FieldPoints.h"
#include "interfaces/IMacroController.h"
#include "logic/DriftCurveOverride.h"
#include "logic/GoalScore.h"
#include "core/Specs.h"
#include "math/MathUtils.h"
#include <cmath>

namespace {

GoalScoreOutput passthrough(const SubsystemCommand& driver) {
    GoalScoreOutput out;
    out.command = driver;
    return out;
}

// `bias_heading` halves the distance of a loader within the nose cone.
int nearestLoader(const Pose2d& from, bool bias_heading) {
    int best = -1;
    double best_d = 0.0;
    const double cone = MatchLoaderTune::kHeadingConeDeg * Robot::PI / 180.0;
    const int n = static_cast<int>(Field::Point::Count);
    for (int i = 0; i < n; ++i) {
        const Field::Spot& spot = Field::kSpots[i];
        if (!Field::isLoader(spot)) continue;
        double d = std::hypot(from.x - spot.x, from.y - spot.y);
        if (bias_heading && d > 1e-3) {
            const double bearing = mathutil::direction_heading(spot.x - from.x, spot.y - from.y);
            const double err = std::fabs(mathutil::normalize_angle(bearing - from.theta));
            if (err <= cone) d *= MatchLoaderTune::kHeadingDistScale;
        }
        if (best < 0 || d < best_d) {
            best = i;
            best_d = d;
        }
    }
    return best;
}

} // namespace

MatchLoader& liveMatchLoader() {
    static MatchLoader loader;
    return loader;
}

void MatchLoader::cancel() {
    running_ = false;
    backing_ = false;
    aim_set_ = false;
    nav_.reset();
}

void MatchLoader::aimAt(double x_in, double y_in) {
    aim_set_ = true;
    aim_x_ = x_in;
    aim_y_ = y_in;
}

void MatchLoader::clearAim() {
    aim_set_ = false;
}

int MatchLoader::samplePath(const Pose2d& from, Pose2d* out, int cap) const {
    if (!running_) return 0;
    return nav_.samplePath(from, out, cap);
}

int matchLoaderPath(const Pose2d& from, Pose2d* out, int cap) {
    return liveMatchLoader().samplePath(from, out, cap);
}

static bool nearPoint(const Pose2d& pose, double x, double y, double inches) {
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y)) return false;
    return std::hypot(pose.x - x, pose.y - y) <= inches;
}

void MatchLoader::begin(const Pose2d& pose, uint32_t now_ms) {
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.theta)) return;
    int index;
    if (aim_set_) {
        index = nearestLoader(Pose2d{aim_x_, aim_y_, 0.0}, false);
        aim_set_ = false;
    } else {
        index = nearestLoader(pose, true);
    }
    if (index < 0) return;
    const Field::Spot& spot = Field::kSpots[index];

    g_drift_curve_abort = true;
    nav_.reset();
    nav_.bindFeedforward(model_);
    MotionPose goal;
    goal.x = spot.x;
    goal.y = spot.y;
    // Always 180 degrees, facing the near wall. The straight finish is that
    // approach; the aim point does not change it.
    goal.heading = MatchLoaderTune::kFaceRad;
    goal.v = 0.0;
    PathConstraints path;
    path.point = CommandPoint::Tracking;
    path.line_length_in = MatchLoaderTune::kApproachIn;
    path.max_accel_mps2 = 1.5;
    nav_.config.forwards = true;
    nav_.config.cruise_speed = 1.0;
    nav_.config.path_accel = 1.5;
    nav_.config.max_volts = System::MAX_VOLTS;
    nav_.config.obstacle_n = Field::goalAvoidObstacles(nav_.config.obstacles, kPathObstacles);
    nav_.target = DriftCurveTarget::fromMotion(goal, path);
    running_ = true;
    backing_ = false;
    start_ms_ = now_ms;
}

void MatchLoader::startBackup(const Pose2d& pose) {
    const double c = mathutil::heading_cos(pose.theta);
    const double s = mathutil::heading_sin(pose.theta);
    nav_.reset();
    nav_.bindFeedforward(model_);
    MotionPose goal;
    goal.x = pose.x - MatchLoaderTune::kBackupIn * c;
    goal.y = pose.y - MatchLoaderTune::kBackupIn * s;
    goal.heading = pose.theta;
    goal.v = -1.0e-4;
    PathConstraints path;
    path.point = CommandPoint::Tracking;
    path.line_length_in = 0.0;
    path.max_accel_mps2 = 1.5;
    nav_.config.forwards = false;
    nav_.config.cruise_speed = 0.6;
    nav_.config.path_accel = 1.5;
    nav_.config.max_volts = System::MAX_VOLTS;
    nav_.config.obstacle_n = Field::goalAvoidObstacles(nav_.config.obstacles, kPathObstacles);
    nav_.target = DriftCurveTarget::fromMotion(goal, path);
    backing_ = true;
}

GoalScoreOutput MatchLoader::apply(const SubsystemCommand& driver,
                                   const ButtonState& buttons,
                                   const StateEstimate& state,
                                   uint32_t now_ms,
                                   const IFeedForward* model,
                                   IMacroController& macros) {
    model_ = model;
    const bool b_edge = buttons.b && !prev_b_;
    prev_b_ = buttons.b;
    const bool load_held = (driver.macro_request & kMacroMatchLoad) != 0;

    if (goalScoreRunning()) {
        cancel();
        return passthrough(driver);
    }

    // A+B reloads a recording. A alone is the practice drive, and it takes
    // the chassis back from this one. A held routine request starts the
    // drive once, and does not start it again while that drive is running.
    if (b_edge && !buttons.a) {
        if (running_) cancel();
        else begin(state.pose, now_ms);
    } else if (!running_ && load_held) {
        begin(state.pose, now_ms);
    }
    if (running_ && buttons.a && !buttons.b) cancel();
    if (!running_) return passthrough(driver);

    if (now_ms - start_ms_ >= MatchLoaderTune::kTimeoutMs) {
        GoalScoreOutput out = passthrough(driver);
        if (!backing_) {
            out.command.pneumatics_b_state = MatchLoaderTune::kClawClosed;
            macros.setPneumaticB(MatchLoaderTune::kClawClosed);
        }
        cancel();
        return out;
    }

    GoalScoreOutput out = passthrough(driver);
    out.drive_active = true;
    if (!backing_) {
        out.command.pneumatics_b_state = MatchLoaderTune::kClawReleased;
        macros.setPneumaticB(MatchLoaderTune::kClawReleased);
        out.drive = nav_.compute(state);
        const bool contact = nav_.stuck()
            && nearPoint(state.pose, nav_.target.x, nav_.target.y, MatchLoaderTune::kContactIn);
        if (nav_.settled() || contact) {
            out.command.pneumatics_b_state = MatchLoaderTune::kClawClosed;
            macros.setPneumaticB(MatchLoaderTune::kClawClosed);
            startBackup(state.pose);
            out.drive = nav_.compute(state);
        } else if (nav_.stuck()) {
            out.command.pneumatics_b_state = MatchLoaderTune::kClawClosed;
            macros.setPneumaticB(MatchLoaderTune::kClawClosed);
            cancel();
            out.drive_active = false;
            out.drive = ChassisCommand{};
        }
    } else {
        out.command.pneumatics_b_state = MatchLoaderTune::kClawClosed;
        out.drive = nav_.compute(state);
        if (nav_.settled() || nav_.stuck()) {
            cancel();
            out.drive_active = false;
            out.drive = ChassisCommand{};
        }
    }
    return out;
}
