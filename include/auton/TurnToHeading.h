#pragma once
/* ---------------------------------------------------------------------------
 * TurnToHeading  --  one pivot in place, up to a field heading.
 *
 * The wheels are not asked to follow a curve. Forward voltage only brakes
 * leftover v_lin, and the yaw command is a trapezoid on heading error:
 *
 *   w_stop = sqrt(2 * accel * |error|)
 *   w_plan slews from the measured yaw rate toward
 *          sign(error) * min(w_stop, max_vel)
 *   by at most accel * dt each tick.
 *
 * Once the heading is inside tolerance and the measured yaw rate is already
 * under the settle rate, the plan commands zero speed and waits there.
 * Until then the brake curve stays in charge, so a fast entry is not asked
 * to stop dead and overshoot. The command finishes only after both have
 * stayed inside that window for settle_ticks. Shortest is the default
 * direction. A forced direction is ignored once the shortest error is
 * already inside the tolerance, so the robot does not take a full extra
 * revolution to satisfy a heading it has already reached.
 *
 * The bound feedforward turns (w_plan, alpha) into differential volts.
 * That voltage is V_R - V_L, so it is halved into the chassis angular
 * command, the same scale TankDrive applies per side. With no model, a
 * fixed kV / kA on w_plan stands in.
 * ------------------------------------------------------------------------- */

#include "auton/Command.h"
#include "auton/Rotation2d.h"
#include "core/Specs.h"
#include "core/Tuning.h"
#include "interfaces/IFeedForward.h"
#include "math/MathUtils.h"
#include <cmath>

namespace auton {

enum class TurnDirection { Shortest, Clockwise, Counterclockwise };

struct TurnConstraints {
    double max_vel_rad_s = 4.0;          // below the kinematic ceiling, so the pivot ramps
    double max_accel_rad_s2 = 6.0;       // rad/s^2 either way
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
    }

    bool execute(CommandTick& tick) override {
        const double theta = tick.state.pose.theta;
        const double w_now = std::isfinite(tick.state.v_ang) ? tick.state.v_ang : 0.0;
        const double tol = tolerance();
        const double error = headingError(heading_, theta, constraints_.direction, tol);
        if (!std::isfinite(error)) {
            tick.setDrive(ChassisCommand{});
            held_ = 0;
            return false;
        }

        const double accel = maxAccel();
        const double rate_tol = settleRate();
        const bool in_window = std::fabs(error) <= tol && std::fabs(w_now) <= rate_tol;
        double w_cap = maxVel();
        const double w_stop = std::sqrt(2.0 * accel * std::fabs(error));
        if (w_stop < w_cap) w_cap = w_stop;
        // Already slow and pointed at the heading: brake to a stop and stay.
        // Outside that window the brake curve is what slows the pivot down.
        const double w_ideal = in_window ? 0.0 : mathutil::sign(error) * w_cap;

        const double step = accel * System::DT_SEC;
        double w_plan = w_ideal;
        if (w_plan > w_now + step) w_plan = w_now + step;
        if (w_plan < w_now - step) w_plan = w_now - step;
        const double alpha = (System::DT_SEC > 1e-6) ? (w_plan - w_now) / System::DT_SEC : 0.0;

        if (in_window) {
            ++held_;
            if (held_ >= settleTicks()) {
                tick.setDrive(ChassisCommand{});
                return true;
            }
        } else {
            held_ = 0;
        }

        const double radius = Robot::Chassis::TRACK_WIDTH_M * 0.5;
        const double v_lin = std::isfinite(tick.state.v_lin) ? tick.state.v_lin : 0.0;
        const double a_lin = holdAccel(v_lin);
        double surge = 0.0;
        double yaw = 0.0;
        if (model_ != nullptr) {
            surge = model_->linearVolts(0.0, a_lin, 0.0, std::fabs(a_lin), w_plan, radius);
            // angularVolts is V_R - V_L. The chassis command is half of that.
            yaw = 0.5 * model_->angularVolts(w_plan, alpha, 0.0, std::fabs(a_lin), radius);
        } else {
            surge = -Turn::FALLBACK_HOLD * v_lin;
            yaw = Turn::FALLBACK_KV * w_plan + Turn::FALLBACK_KA * alpha;
        }
        if (!std::isfinite(surge)) surge = 0.0;
        if (!std::isfinite(yaw)) yaw = 0.0;

        const double max_v = System::MAX_VOLTS;
        surge = mathutil::clamp_val(surge, -max_v, max_v);
        yaw = mathutil::clamp_val(yaw, -max_v, max_v);
        const double overturn = std::fabs(yaw) + std::fabs(surge) - max_v;
        if (overturn > 0.0) surge -= std::copysign(overturn, surge);

        ChassisCommand cmd;
        cmd.linear = surge;
        cmd.lateral = 0.0;
        cmd.angular = yaw;
        cmd.brake = false;
        tick.setDrive(cmd);
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
};

} // namespace auton
