#pragma once
/* ---------------------------------------------------------------------------
 * TurnToHeading  --  one pivot in place, up to a field heading.
 *
 * The wheels are not asked to follow a curve. Forward voltage only brakes
 * leftover v_lin. The yaw setpoint is the brake curve on heading error,
 *
 *   sign(error) * min(max_vel, sqrt(2 * accel * |error|))
 *
 * and the kA term uses how far the measured rate sits from that setpoint,
 * capped at the same accel. Once the heading is inside tolerance and the
 * measured yaw rate is already under the settle rate, the setpoint is zero
 * and the command waits there. The command finishes only after both have
 * stayed inside that window for settle_ticks. Shortest is the default
 * direction. A forced direction is ignored once the shortest error is
 * already inside the tolerance, so the robot does not take a full extra
 * revolution to satisfy a heading it has already reached.
 *
 * A pivot that asks for yaw and gets none for Motion::STUCK_SEC gives the
 * leg up. So does a pose that is not a number, and a pivot that is still
 * turning after the time its heading error allows. The routine starts the
 * next command.
 *
 * The command is a Motion control vector: zero forward speed, the brake
 * accel that kills leftover v_lin, and the yaw rate and yaw accel of the
 * brake curve. The drive base turns that into voltage. The model pointer
 * is unused; it stays so a caller can still pass one.
 * ------------------------------------------------------------------------- */

#include "auton/Command.h"
#include "auton/MotionLegGuard.h"
#include "auton/Rotation2d.h"
#include "control/MotionStuck.h"
#include "control/YawArmature.h"
#include "core/Specs.h"
#include "core/Tuning.h"
#include "interfaces/IFeedForward.h"
#include "math/MathUtils.h"
#include <cmath>

namespace auton {

enum class TurnDirection { Shortest, Clockwise, Counterclockwise };

struct TurnConstraints {
    // Turn:: caps. The bound feedforward turns (w_plan, alpha) into volts.
    // These two are how fast that plan may move.
    double max_vel_rad_s = Turn::MAX_VEL_RAD_S;
    double max_accel_rad_s2 = Turn::MAX_ACCEL_RAD_S2;
    double tolerance_rad = Robot::PI / 180.0; // 1 degree
    double settle_rate_rad_s = 0.2;      // rad/s. Must be stopped, not only pointed.
    int settle_ticks = 5;                // 50 ms inside the window
    TurnDirection direction = TurnDirection::Shortest;
};

class TurnToHeading : public Command {
public:
    TurnToHeading(double heading_rad, TurnConstraints turn, const IFeedForward* model)
        : heading_(mathutil::normalize_angle(heading_rad)), constraints_(turn), model_(model) {}

    void initialize(const CommandTick& tick) override {
        (void)tick;
        held_ = 0;
        stuck_.reset();
        guard_.reset();
        paused_ = false;
    }

    bool execute(CommandTick& tick) override {
        if (tick.pause_motion) {
            paused_ = true;
            stuck_.reset();
            return false;
        }
        if (paused_) {
            held_ = 0;
            guard_.reset();
            paused_ = false;
        }
        if (!MotionLegGuard::usable(tick.state)) {
            yieldMotion(tick);
            return true;
        }
        const double theta = tick.state.pose.theta;
        const double w_now = std::isfinite(tick.state.v_ang) ? tick.state.v_ang : 0.0;
        const double tol = tolerance();
        const double error = headingError(heading_, theta, constraints_.direction, tol);
        if (!std::isfinite(error)) {
            yieldMotion(tick);
            return true;
        }

        const double accel = maxAccel();
        const double rate_tol = settleRate();
        const bool in_window = std::fabs(error) <= tol && std::fabs(w_now) <= rate_tol;
        // Already slow and pointed at the heading: brake to a stop and stay.
        // Outside that window the brake curve is what slows the pivot down.
        const YawArmature plan = planYaw(in_window ? 0.0 : error, w_now, maxVel(), accel, true);

        if (in_window) {
            ++held_;
            if (held_ >= settleTicks()) {
                tick.setDrive(ChassisCommand{});
                return true;
            }
        } else {
            held_ = 0;
        }

        const double v_lin = std::isfinite(tick.state.v_lin) ? tick.state.v_lin : 0.0;
        const double a_lin = holdAccel(v_lin);
        (void)model_;

        ChassisCommand cmd;
        cmd.request = DriveRequest::Motion;
        cmd.linear = 0.0;
        cmd.accel = a_lin;
        cmd.angular = std::isfinite(plan.omega) ? plan.omega : 0.0;
        cmd.alpha = std::isfinite(plan.alpha) ? plan.alpha : 0.0;
        cmd.pivot = true;
        cmd.lateral = 0.0;
        cmd.brake = false;
        tick.setDrive(cmd);
        const bool no_answer = stuck_.update(false, !in_window, tick.state.v_lin, tick.state.v_ang);
        // Wall time, including ticks inside the window. The settle counter
        // finishes a pivot that is already lined up. This ends one that keeps
        // leaving that window.
        const double eta = std::fabs(error) / maxVel() + 0.5;
        const bool timed_out = guard_.overran(eta, 1.5, 4.0);
        if (no_answer || (!in_window && timed_out)) {
            yieldMotion(tick);
            return true;
        }
        return false;
    }

    // The pivot point. Both samples share the measured position; the second
    // carries the heading the nose is turning toward.
    int samplePath(const Pose2d& from, Pose2d* out, int cap,
                   double v_lin, double v_ang) const override {
        (void)v_lin;
        (void)v_ang;
        if (out == nullptr || cap < 2) return 0;
        out[0] = from;
        out[1] = from;
        out[1].theta = heading_;
        return 2;
    }

private:
    static double headingError(double target, double measured, TurnDirection direction,
                               double tolerance) {
        double error = mathutil::normalize_angle(target - measured);
        if (direction == TurnDirection::Shortest || std::fabs(error) <= tolerance) return error;
        if (direction == TurnDirection::Counterclockwise) {
            if (error < 0.0) error += 2.0 * Robot::PI;
        } else if (error > 0.0) {
            error -= 2.0 * Robot::PI;
        }
        return error;
    }

    double tolerance() const {
        return (constraints_.tolerance_rad > Turn::MIN_TOLERANCE_RAD)
            ? constraints_.tolerance_rad : Turn::MIN_TOLERANCE_RAD;
    }
    double settleRate() const {
        return (constraints_.settle_rate_rad_s > Turn::MIN_SETTLE_RATE)
            ? constraints_.settle_rate_rad_s : Turn::MIN_SETTLE_RATE;
    }
    double maxAccel() const {
        return (constraints_.max_accel_rad_s2 > Turn::MIN_ACCEL)
            ? constraints_.max_accel_rad_s2 : Turn::MIN_ACCEL;
    }
    double maxVel() const {
        return (constraints_.max_vel_rad_s > Turn::MIN_VEL)
            ? constraints_.max_vel_rad_s : Turn::MIN_VEL;
    }
    int settleTicks() const {
        return (constraints_.settle_ticks > 1) ? constraints_.settle_ticks : 1;
    }

    double holdAccel(double v_lin) const {
        if (std::fabs(v_lin) < Turn::HOLD_DEADBAND) return 0.0;
        const double raw = -v_lin / System::DT_SEC;
        return mathutil::clamp_val(raw, -Turn::HOLD_ACCEL, Turn::HOLD_ACCEL);
    }

    double heading_ = 0.0;
    TurnConstraints constraints_{};
    const IFeedForward* model_ = nullptr;
    int held_ = 0;
    bool paused_ = false;
    MotionStuck stuck_;
    MotionLegGuard guard_;
};

} // namespace auton
