/* ---------------------------------------------------------------------------
 * LogFrameReplayer.cpp  --  ported from v1 robot_logic.cpp (save/load/replay)
 * and the usercontrol logging block. The per-tick replay body becomes
 * getNextTarget(): it emits an interpolated MotionTarget carrying the recorded
 * chassis effort and advances the playback cursor with a battery time-warp.
 * ------------------------------------------------------------------------- */
#include "trajectory/LogFrameReplayer.h"
#include "config/Settings.h"
#include "core/Gains.h"
#include "math/MathUtils.h"
#include "platform/RobotFile.h"
#include "api.h"
#include <cstdio>
#include <cmath>
#include <algorithm>

using mathutil::normalize_angle;

// Decode a recorded subsystem bitmask into motor/pneumatics commands.
static SubsystemCommand decodeRecordedBitmask(uint8_t bitmask) {
    bool r1 = bitmask & (1 << 0), r2 = bitmask & (1 << 1);
    bool l1 = bitmask & (1 << 2), l2 = bitmask & (1 << 3);
    bool pg = bitmask & (1 << 4), pb = bitmask & (1 << 5);
    int intake = 0, top = 0, bottom = 0, intake_m = 2, score_m = 2;
    if (r1) { intake += 100;  top += -2;  bottom += 1;    intake_m -= 1; }
    if (r2) { intake += -100; top += -2;  bottom += 1;    intake_m -= 1; }
    if (l1) {                 top += 106; bottom += 97;   score_m -= 1;  }
    if (l2) {                 top += -24; bottom += -103; score_m -= 1;  }
    SubsystemCommand sub;
    sub.intake_pct        = intake * intake_m;
    sub.roller_top_pct    = top * score_m;
    sub.roller_bottom_pct = bottom * score_m;
    sub.pneumatics_g_state = pg;
    sub.pneumatics_b_state = pb;
    sub.raw_bitmask = bitmask;
    return sub;
}

void LogFrameReplayer::loadFromHeader() {
    length_ = 0;
    const char* filepath = Settings::SKILLS_MODE ? "/usd/skills_auto.bin" : "/usd/prematch.bin";
    FILE* file = openRobotFile(filepath, "rb");
    if (file) {
        length_ = fread(buffer_.data(), sizeof(LogFrame), System::MAX_RECORD_FRAMES, file);
        fclose(file);
    } else {
        printf("Error: Could not open %s from microSD card.\n", filepath);
    }
}

void LogFrameReplayer::beginRecording(RobotMode mode, const Pose2d& /*startPose*/) {
    recording_mode_ = mode;
    length_ = 0;
}

void LogFrameReplayer::recordFrame(const MotionTarget& target, const Pose2d& actual,
                                   const CommandVector& command, const ChassisCommand& applied,
                                   const SubsystemCommand& sub, const ChassisObservation& obs,
                                   double lin_error, double ang_error) {
    if (length_ >= System::MAX_RECORD_FRAMES) return;
    LogFrame& f = buffer_[length_];
    f.time = pros::millis() / 1000.0;

    bool sysid = (recording_mode_ == RobotMode::SYSID_DRIVE);
    f.x = sysid ? 0.0 : target.pose.x;
    f.y = sysid ? 0.0 : target.pose.y;
    f.theta = sysid ? 0.0 : target.pose.theta;

    f.v_ref = target.velocity.vx;
    f.omega_ref = target.velocity.omega;
    f.a_ref = target.accel_lin;
    f.alpha_ref = target.accel_ang;
    f.lin_error = lin_error;
    f.ang_error = ang_error;

    f.actual_x = actual.x;
    f.actual_y = actual.y;
    f.actual_theta = actual.theta;

    f.cmd_surge = command.surge;
    f.cmd_sway  = command.sway;
    f.cmd_yaw   = command.yaw;

    f.applied_linear  = applied.linear;
    f.applied_lateral = applied.lateral;
    f.applied_angular = applied.angular;

    f.l_pos_deg = obs.left_pos_deg;   f.r_pos_deg = obs.right_pos_deg;
    f.l_vel_rpm = obs.left_vel_rpm;   f.r_vel_rpm = obs.right_vel_rpm;
    f.l_current_amps = obs.left_current_amps;
    f.r_current_amps = obs.right_current_amps;
    f.battery_voltage = obs.battery_voltage;
    f.imu_accel_x = obs.imu_accel_x;  f.imu_accel_y = obs.imu_accel_y;
    f.imu_heading = obs.imu_heading_deg;
    f.imu_rate_z = obs.imu_rate_z_dps;
    f.subsystems = sub.raw_bitmask;

    length_++;
}

void LogFrameReplayer::saveToConsole() {
    const char* filepath = (recording_mode_ == RobotMode::SYSID_DRIVE) ? "/usd/sysid.bin" :
                           (Settings::SKILLS_MODE) ? "/usd/skills_auto.bin" : "/usd/prematch.bin";

    FILE* file = openRobotFile(filepath, "wb");
    if (file) {
        fwrite(buffer_.data(), sizeof(LogFrame), length_, file);
        fclose(file);
        printf("Successfully saved %d frames to %s\n", length_, filepath);
    } else {
        printf("Error: Could not save to %s. Is the microSD card inserted?\n", filepath);
    }
}

Pose2d LogFrameReplayer::getStartPose() const {
    if (length_ == 0) return Pose2d{};
    return Pose2d{buffer_[0].x, buffer_[0].y, buffer_[0].theta};
}

double LogFrameReplayer::getStartHeading() const {
    return (length_ == 0) ? 0.0 : buffer_[0].theta;
}

void LogFrameReplayer::resetPlayback() {
    replay_index_ = 0.0f;
    finished_ = false;
    ref_l_0_ = (length_ > 0) ? buffer_[0].l_pos_deg : 0.0;
    ref_r_0_ = (length_ > 0) ? buffer_[0].r_pos_deg : 0.0;
    has_valid_wheel_ref_ = (length_ > 1) &&
        (std::fabs(buffer_[length_ - 1].l_pos_deg - ref_l_0_) > 1e-6 ||
         std::fabs(buffer_[length_ - 1].r_pos_deg - ref_r_0_) > 1e-6);
    l_pos_start_ = 0.0; r_pos_start_ = 0.0;
    current_sub_ = SubsystemCommand{};
    baseline_captured_ = false;
}

MotionTarget LogFrameReplayer::getNextTarget(const Pose2d& pose, const ChassisObservation& obs) {
    namespace CT = Settings::ControlToggles;

    // Seed wheel-position baselines from the first replay tick.
    if (!baseline_captured_) {
        l_pos_start_ = obs.left_pos_deg;
        r_pos_start_ = obs.right_pos_deg;
        baseline_captured_ = true;
    }

    int idx = (int)replay_index_;
    if (length_ < 2 || idx >= length_ - 1) {
        finished_ = true;
        current_sub_ = SubsystemCommand{};
        return MotionTarget{};
    }

    float t = replay_index_ - idx;
    const LogFrame& f0 = buffer_[idx];
    const LogFrame& f1 = buffer_[idx + 1];
    auto lerp = [t](double a, double b) { return a + t * (b - a); };

    MotionTarget target;
    target.pose.x = lerp(f0.x, f1.x);
    target.pose.y = lerp(f0.y, f1.y);
    target.pose.theta = f0.theta + t * normalize_angle(f1.theta - f0.theta);
    target.velocity.vx = lerp(f0.v_ref, f1.v_ref);
    target.velocity.omega = lerp(f0.omega_ref, f1.omega_ref);
    target.accel_lin = lerp(f0.a_ref, f1.a_ref);
    target.accel_ang = lerp(f0.alpha_ref, f1.alpha_ref);

    double interp_lin = lerp(f0.applied_linear,  f1.applied_linear);
    double interp_lat = lerp(f0.applied_lateral, f1.applied_lateral);
    double interp_ang = lerp(f0.applied_angular, f1.applied_angular);

    // Optional wheel-position corrections (tank-oriented encoder channels).
    double ref_l = lerp(f0.l_pos_deg, f1.l_pos_deg);
    double ref_r = lerp(f0.r_pos_deg, f1.r_pos_deg);
    double delta_ref_l = ref_l - ref_l_0_;
    double delta_ref_r = ref_r - ref_r_0_;
    double delta_act_l = obs.left_pos_deg - l_pos_start_;
    double delta_act_r = obs.right_pos_deg - r_pos_start_;
    double lin_err_m = ((delta_ref_l - delta_act_l) + (delta_ref_r - delta_act_r)) / 2.0
                     * (Robot::METERS_PER_TURN / 360.0);
    double wheel_lin_corr = (CT::PID_ADJUSTMENT_REPLAY && has_valid_wheel_ref_)
                          ? (Replay::WHEEL_LIN_KP * lin_err_m) : 0.0;
    double ang_err = normalize_angle(target.pose.theta - pose.theta);
    double wheel_ang_corr = CT::PID_ADJUSTMENT_REPLAY ? (-Replay::WHEEL_ANG_KP * ang_err) : 0.0;

    target.ref_linear_effort  = interp_lin + wheel_lin_corr;
    target.ref_lateral_effort = interp_lat;
    target.ref_angular_effort = interp_ang - wheel_ang_corr;
    target.passthrough_effort = CT::REPLAY_VOLTAGE_FALLBACK;

    current_sub_ = decodeRecordedBitmask(f0.subsystems);

    // Battery-aware time warp: slow playback when volts would saturate.
    double batt_scale = CT::BATTERY_SCALING ? (System::MAX_VOLTS / obs.battery_voltage) : 1.0;
    double scaled_lin = (interp_lin + wheel_lin_corr) * batt_scale;
    double scaled_lat = interp_lat * batt_scale;
    double scaled_ang = (interp_ang - wheel_ang_corr) * batt_scale;
    // Conservative saturation check in chassis effort space (max |axis|).
    double peak = std::max({std::fabs(scaled_lin), std::fabs(scaled_lat), std::fabs(scaled_ang)});
    double warp = 1.0;
    double recorded_batt = f0.battery_voltage;
    if (recorded_batt > 5.0 && obs.battery_voltage > 5.0) {
        bool voltage_limited = peak > obs.battery_voltage;
        if (voltage_limited) {
            double ratio = obs.battery_voltage / recorded_batt;
            if (ratio < 1.0) warp *= ratio;
        }
    }
    replay_index_ += warp;
    return target;
}
