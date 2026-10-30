/* ---------------------------------------------------------------------------
 * MockDrive.cpp  --  differential-drive forward/inverse kinematics with the
 * same per-actuator safety as TankDrive, backed by a simulated wheel plant
 * instead of the PROS V5 API.
 *
 * Each side is a first-order lag toward (voltage / max_volts) * MAX_VEL_LIN,
 * so a held voltage settles at a finite speed and zero voltage coasts to a
 * stop. The field window can split that lag: linear and angular friction eat
 * voltage before the axis moves, and each axis has its own settled speed and
 * rise time. With those knobs on the compiled tick the sides stay one lag.
 * service() is the desktop stand-in for the 10 ms of real time that
 * elapses between pipeline ticks: it integrates the plant and redraws the
 * estimator pose. The keyboard is the V5 controller.
 * ------------------------------------------------------------------------- */
#include "hardware/MockDrive.h"
#include "hardware/ChassisVoltage.h"
#include "math/TrackingCenter.h"
#include "config/Settings.h"
#include "core/Specs.h"
#include "core/Tuning.h"
#include "math/ContactPhysics.h"
#include "math/MathUtils.h"
#include "math/WallRange.h"
#include "host/FieldWindow.h"
#include "logic/CascadeLift.h"
#if !defined(PLANT_MODULE)
#include "logic/GoalScore.h"
#include "logic/MatchLoader.h"
#endif
#include <algorithm>
#include <cmath>
#include <iostream>

// Simulated cascade travel. 0 is the bottom stop. The field window draws
// this span as a full bar.
constexpr double kCascadeTravelDeg = 900.0;

using mathutil::clamp_val;
using mathutil::sign;

MockDrive::MockDrive(const IFeedForward& feedforward, double max_volts, bool scale_battery)
    : feedforward_(&feedforward), max_voltage_(max_volts), use_battery_scaling_(scale_battery) {}

void MockDrive::initialize() {
    ensureSensorScales();
    cascadeLift().zero(readCascade());
}

void MockDrive::ensureSensorScales() {
    if (imu_scaled_) return;
    // Drawn once per sensor. Recalibration does not change the scale factor.
    imu_scale_ = 1.0 + normal_() * Robot::Imu::SCALE_SIGMA;
    imu_accel_scale_ = 1.0 + normal_() * Robot::Imu::ACCEL_SCALE_SIGMA;
    imu_scaled_ = true;
}

void MockDrive::resetImuIntegration() {
    ensureSensorScales();
    // Calibration measures the resting rate and subtracts it. What remains is
    // a small residual, not the chip's ±1 deg/s uncalibrated zero-rate.
    imu_bias_dps_ = normal_() * Robot::Imu::BIAS_SIGMA_DPS;
    imu_rotation_deg_ = 0.0;
    imu_plant_heading_ = heading_rad_;
    dist_tick_ = 0;
}

double MockDrive::reportedRotationDeg() const {
    const double lsb = Robot::Imu::HEADING_LSB_DEG;
    return std::round(imu_rotation_deg_ / lsb) * lsb;
}

int MockDrive::distanceConfidence(int mm) {
    if (mm <= static_cast<int>(Robot::Range::NEAR_M * 1000.0 + 0.5))
        return Robot::Range::CONF_NEAR;
    // A field wall fills the beam, so confidence stays high and eases off
    // toward the 2 m limit. The 0–63 scale is VEX's; this slope is not a
    // published curve, only a stand-in for return strength.
    const double z = static_cast<double>(mm) / 1000.0;
    const double t = (z - Robot::Range::NEAR_M) / (Robot::Range::MAX_M - Robot::Range::NEAR_M);
    const double mean = MockPlant::CONF_FULL - MockPlant::CONF_DROP * t;
    int conf = static_cast<int>(std::lround(mean + normal_() * MockPlant::CONF_SIGMA));
    if (conf < MockPlant::CONF_MIN) conf = MockPlant::CONF_MIN;
    if (conf > MockPlant::CONF_MAX) conf = MockPlant::CONF_MAX;
    return conf;
}

void MockDrive::beginSensorCalibration() {
    imu_offset_ = 0.0;
    lat_mps_ = 0.0;
    prev_v_fwd_ = prev_v_lat_ = 0.0;
    if (!Settings::ControlToggles::IMU_FALLBACK) heading_rad_ = 0.0;
    true_x_ = Robot::Range::CENTER_X_M;
    true_y_ = Robot::Range::CENTER_Y_M;
    resetImuIntegration();
}

void MockDrive::waitSensorCalibration() {}

void MockDrive::calibrateSensors() {
    beginSensorCalibration();
    waitSensorCalibration();
}

void MockDrive::stop() {
    plant_left_v_ = 0.0;
    plant_right_v_ = 0.0;
    volt_surge_ = 0.0;
    volt_yaw_ = 0.0;
    volt_primed_ = false;
}

void MockDrive::setJoystick(const JoystickData& joy) { joystick_ = joy; }

void MockDrive::setButtons(const ButtonState& buttons) { buttons_ = buttons; }

void MockDrive::setBatteryVoltage(double volts) {
    battery_voltage_ = (volts > 0.0) ? volts : 0.0;
}

MockDrive::PlantSnapshot MockDrive::plantSnapshot() const {
    const double v_fwd = (left_mps_ + right_mps_) * 0.5;
    const double omega = (right_mps_ - left_mps_) / Robot::Chassis::TRACK_WIDTH_M;
    PlantSnapshot s;
    s.x_in = metersToInches(true_x_);
    s.y_in = metersToInches(true_y_);
    s.theta = heading_rad_ + imu_offset_;
    s.v_lin = v_fwd;
    s.v_lat = lat_mps_;
    s.v_ang = omega;
    s.left_amps = left_amps_;
    s.right_amps = right_amps_;
    return s;
}

void MockDrive::setPlant(double x_in, double y_in, double theta,
                         double v_lin, double v_lat, double v_ang) {
    true_x_ = inchesToMeters(x_in);
    true_y_ = inchesToMeters(y_in);
    heading_rad_ = theta;
    imu_offset_ = 0.0;
    imu_plant_heading_ = theta;
    lat_mps_ = v_lat;
    const double half = Robot::Chassis::TRACK_WIDTH_M * 0.5;
    left_mps_ = clamp_val(v_lin - v_ang * half, -Robot::MAX_VEL_LIN, Robot::MAX_VEL_LIN);
    right_mps_ = clamp_val(v_lin + v_ang * half, -Robot::MAX_VEL_LIN, Robot::MAX_VEL_LIN);
    plant_left_v_ = plant_right_v_ = 0.0;
    left_amps_ = right_amps_ = 0.0;
    volt_surge_ = volt_yaw_ = 0.0;
    volt_primed_ = false;
    prev_v_fwd_ = (left_mps_ + right_mps_) * 0.5;
    prev_v_lat_ = lat_mps_;
    settleContacts(0.0);
    // Clockwise rotation. With the offset at zero, the next heading read
    // equals the placed heading, including a nudge settleContacts applied.
    imu_rotation_deg_ = -(heading_rad_ + imu_offset_) * (180.0 / M_PI);
}

JoystickData MockDrive::readJoystick() {
    pollKeyboard();
    return joystick_;
}

ButtonState MockDrive::readButtons() {
    pollKeyboard();
    return buttons_;
}

void MockDrive::alignHeadingTo(double target_rad) {
    if (Settings::ControlToggles::IMU_FALLBACK) { imu_offset_ = 0.0; return; }
    const double raw = -reportedRotationDeg() * (M_PI / 180.0);
    imu_offset_ = target_rad - raw;
}

void MockDrive::armLocalization(std::uint64_t seed) {
    rng_ = seed == 0 ? 0xC0FFEE1234ull : seed;
    imu_scaled_ = false;
    resetImuIntegration();
    // Reported rotation is clockwise. Truth heading stays heading_rad_ because
    // imu_offset_ is not touched.
    imu_rotation_deg_ = -heading_rad_ * (180.0 / M_PI);
    const double meters_per_tick = Robot::METERS_PER_TURN / Robot::Encoder::TICKS_PER_REV;
    prev_l_ = std::round(left_m_ / meters_per_tick) * meters_per_tick;
    prev_r_ = std::round(right_m_ / meters_per_tick) * meters_per_tick;
    for (int i = 0; i < kDistanceChannels; ++i) {
        dist_mm_[i] = 0;
        dist_conf_[i] = 0;
        dist_valid_[i] = false;
    }
    dist_tick_ = 0;
}

void MockDrive::resetSensors() {
    left_m_ = right_m_ = 0.0;
    left_mps_ = right_mps_ = 0.0;
    plant_left_v_ = plant_right_v_ = 0.0;
    left_amps_ = right_amps_ = 0.0;
    volt_surge_ = volt_yaw_ = 0.0;
    volt_primed_ = false;
    prev_l_ = prev_r_ = 0.0;
    heading_rad_ = 0.0;
    imu_offset_ = 0.0;
    lat_mps_ = 0.0;
    prev_v_fwd_ = prev_v_lat_ = 0.0;
    true_x_ = Robot::Range::CENTER_X_M;
    true_y_ = Robot::Range::CENTER_Y_M;
    resetImuIntegration();
}

double MockDrive::uniform_() {
    rng_ ^= rng_ << 13;
    rng_ ^= rng_ >> 7;
    rng_ ^= rng_ << 17;
    return static_cast<double>(rng_ >> 11) * (1.0 / 9007199254740992.0);
}

double MockDrive::normal_() {
    double u1 = uniform_();
    if (u1 < 1e-12) u1 = 1e-12;
    const double u2 = uniform_();
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(6.283185307179586 * u2);
}

ChassisObservation MockDrive::readObservation(double prev_heading_rad) {
    namespace CT = Settings::ControlToggles;
    ChassisObservation o;

    // Blue-cartridge tick on the absolute motor position. The error stays
    // inside half a tick instead of walking with distance.
    const double meters_per_tick = Robot::METERS_PER_TURN / Robot::Encoder::TICKS_PER_REV;
    const double qL = std::round(left_m_ / meters_per_tick) * meters_per_tick;
    const double qR = std::round(right_m_ / meters_per_tick) * meters_per_tick;
    const double held_l = prev_l_;
    const double held_r = prev_r_;
    double delta_l = qL - prev_l_;
    double delta_r = qR - prev_r_;
    // The baseline keeps moving while a bank is unplugged, so plugging it
    // back does not dump the travel that happened in between.
    prev_l_ = qL;
    prev_r_ = qR;

    const fieldwindow::PlantOverride sens = fieldwindow::plantOverride();
    const double vel_sigma = Robot::Encoder::VEL_SIGMA_RPM * Robot::RPM_TO_MPS;
    double l_mps = left_mps_ + normal_() * vel_sigma;
    double r_mps = right_mps_ + normal_() * vel_sigma;
    if (sens.enc_left_off) {
        delta_l = 0.0;
        l_mps = 0.0;
    }
    if (sens.enc_right_off) {
        delta_r = 0.0;
        r_mps = 0.0;
    }
    double l_vel = l_mps / Robot::RPM_TO_MPS;
    double r_vel = r_mps / Robot::RPM_TO_MPS;
    o.forward_delta = (delta_l + delta_r) / 2.0;
    // Encoders do not measure the omni skid, so the localizer sees no strafe.
    o.strafe_delta = 0.0;
    o.v_lin = (l_mps + r_mps) / 2.0;

    if (CT::IMU_FALLBACK) {
        const double v_ang = (r_mps - l_mps) / Robot::Chassis::TRACK_WIDTH_M;
        o.heading_rad = prev_heading_rad + (delta_r - delta_l) / Robot::Chassis::TRACK_WIDTH_M;
        o.v_ang = v_ang;
        o.imu_rate_z_dps  = -v_ang * (180.0 / M_PI);
        o.imu_accel_x = 0.0;
        o.imu_accel_y = 0.0;
        imu_plant_heading_ = heading_rad_;
    } else {
        // Integrate the gyro the way the V5 sensor does: scale, residual bias,
        // datasheet rate noise, then clip at ±1000 deg/s. Rotation past the
        // full-scale range is lost. The plant heading is CCW; get_rotation()
        // and get_gyro_rate().z are clockwise positive.
        const double dt = System::DT_SEC;
        const double dtheta = mathutil::normalize_angle(heading_rad_ - imu_plant_heading_);
        imu_plant_heading_ = heading_rad_;
        const double plant_dps = (dt > 1e-6) ? (dtheta / dt) * (180.0 / M_PI) : 0.0;
        double rate = -plant_dps * imu_scale_ + imu_bias_dps_ + normal_() * Robot::Imu::RATE_SIGMA_DPS;
        rate = clamp_val(rate, -Robot::Imu::GYRO_FS_DPS, Robot::Imu::GYRO_FS_DPS);
        imu_rotation_deg_ += rate * dt;
        const double reported = reportedRotationDeg();
        o.heading_rad = -reported * (M_PI / 180.0) + imu_offset_;
        o.imu_rate_z_dps = rate;
        o.v_ang = -rate * (M_PI / 180.0);
        // Specific force of the plant over the step that just finished, in G.
        // Localization integrates this, so the sim's skid is part of the delta.
        const double v_fwd = (left_mps_ + right_mps_) * 0.5;
        const double v_lat = lat_mps_;
        const double omega = (right_mps_ - left_mps_) / Robot::Chassis::TRACK_WIDTH_M;
        double ax = 0.0, ay = 0.0;
        if (dt > 1e-6) {
            ax = (v_fwd - prev_v_fwd_) / dt - omega * prev_v_lat_;
            ay = (v_lat - prev_v_lat_) / dt + omega * prev_v_fwd_;
        }
        prev_v_fwd_ = v_fwd;
        prev_v_lat_ = v_lat;
        auto accelG = [&](double mps2) {
            double g = (mps2 / Robot::G_MPS2) * imu_accel_scale_ + normal_() * Robot::Imu::ACCEL_SIGMA_G;
            return clamp_val(g, -Robot::Imu::ACCEL_FS_G, Robot::Imu::ACCEL_FS_G);
        };
        o.imu_accel_x = accelG(ax);
        o.imu_accel_y = accelG(ay);
    }

    // An unplugged IMU keeps the last heading and reports no rate. The
    // gyro integration above still runs, so connecting it again publishes
    // the rotation the sensor would have now.
    if (sens.imu_off) {
        if (!imu_held_) {
            imu_held_ = true;
            imu_held_heading_ = o.heading_rad;
        }
        o.heading_rad = imu_held_heading_;
        o.v_ang = 0.0;
        o.imu_rate_z_dps = 0.0;
        o.imu_accel_x = 0.0;
        o.imu_accel_y = 0.0;
    } else {
        imu_held_ = false;
    }

    o.left_pos_deg  = ((sens.enc_left_off ? held_l : qL) / Robot::METERS_PER_TURN) * 360.0;
    o.right_pos_deg = ((sens.enc_right_off ? held_r : qR) / Robot::METERS_PER_TURN) * 360.0;
    o.left_vel_rpm = l_vel;
    o.right_vel_rpm = r_vel;
    o.left_current_amps  = left_amps_;
    o.right_current_amps = right_amps_;
    o.left_temp_c  = MockPlant::REPORTED_TEMP_C;
    o.right_temp_c = MockPlant::REPORTED_TEMP_C;
    o.battery_voltage = battery_voltage_;
    return o;
}

void MockDrive::readDistances(DistanceReading (&out)[kDistanceChannels]) {
    // The beam updates at ~30 Hz. Between samples the port keeps returning
    // the last millimeter, which is what a 10 ms poll of the real sensor sees.
    const bool fresh = (dist_tick_ == 0);
    if (++dist_tick_ >= MockPlant::RANGE_PERIOD_TICKS) dist_tick_ = 0;
    const fieldwindow::PlantOverride sens = fieldwindow::plantOverride();

    // The plant stores the geometric center. Beams are offset from the
    // tracking centre, so the ray starts at the front middle.
    const Pose2d pose = tracking::trackingFromCenter(
        Pose2d{true_x_, true_y_, heading_rad_ + imu_offset_});
    const int min_mm = static_cast<int>(Robot::Range::MIN_M * 1000.0 + 0.5);
    const int max_mm = static_cast<int>(Robot::Range::MAX_M * 1000.0 + 0.5);
    for (int i = 0; i < kDistanceChannels; ++i) {
        if (fresh) {
            const WallHit hit = wallrange::predictWallRange(pose, i);
            dist_valid_[i] = false;
            dist_mm_[i] = 0;
            dist_conf_[i] = 0;
            if (hit.hit) {
                const double noisy = hit.range_m + normal_() * Robot::rangeAccuracyM(hit.range_m);
                const int mm = static_cast<int>(std::lround(noisy * 1000.0));
                if (mm >= min_mm && mm <= max_mm) {
                    dist_valid_[i] = true;
                    dist_mm_[i] = mm;
                    dist_conf_[i] = distanceConfidence(mm);
                }
            }
        }
        if (sens.distance_off[i]) {
            out[i] = {};
            continue;
        }
        out[i].valid = dist_valid_[i];
        out[i].range_m = dist_valid_[i] ? (static_cast<double>(dist_mm_[i]) / 1000.0) : 0.0;
        out[i].confidence = dist_valid_[i] ? dist_conf_[i] : 0;
    }
}

// Back-EMF current limit: clamp each side to its predicted EMF +/- max drop.
void MockDrive::applyCurrentLimit(double& left_v, double& right_v, double v_lin, double v_ang) const {
    double turn_r = (std::fabs(v_ang) < Physics::OMEGA_EPS) ? Physics::TURN_RADIUS_MAX_M
                  : std::min(std::fabs(v_lin) / (std::fabs(v_ang) + Physics::OMEGA_EPS), Physics::TURN_RADIUS_MAX_M);
    double v_avg_emf = feedforward_->linearVolts(v_lin, 0.0, std::fabs(v_lin), 0.0, v_ang, turn_r);
    double v_diff_emf = feedforward_->angularVolts(v_ang, 0.0, std::fabs(v_lin), 0.0, turn_r);
    double left_emf  = v_avg_emf - v_diff_emf / 2.0;
    double right_emf = v_avg_emf + v_diff_emf / 2.0;
    left_v  = clamp_val(left_v,  left_emf  - System::MAX_VOLTAGE_DROP, left_emf  + System::MAX_VOLTAGE_DROP);
    right_v = clamp_val(right_v, right_emf - System::MAX_VOLTAGE_DROP, right_emf + System::MAX_VOLTAGE_DROP);
}

ChassisCommand MockDrive::applyDrive(const ChassisCommand& cmd, const ChassisObservation& obs) {
    namespace CT = Settings::ControlToggles;

    ChassisVoltageState held;
    held.v = volt_surge_;
    held.w = volt_yaw_;
    held.primed = volt_primed_;
    const ChassisCommand volts = chassisVoltage(cmd, obs, feedforward_, held);
    volt_surge_ = held.v;
    volt_yaw_ = held.w;
    volt_primed_ = held.primed;

    // Inverse kinematics: chassis volts -> per-bank voltage.
    double left_v  = volts.linear - volts.angular;
    double right_v = volts.linear + volts.angular;

    // Stick-zero brake: gently oppose residual motion when the sticks are idle.
    if (CT::STICK_ZERO_BRAKE) {
        double fwd  = joystick_.left_y;
        double turn = -joystick_.right_x;
        if (std::fabs(fwd) < Controls::STICK_ZERO_DEADBAND &&
            std::fabs(turn) < Controls::STICK_ZERO_DEADBAND) {
            double left_mps  = obs.left_vel_rpm  * Robot::RPM_TO_MPS;
            double right_mps = obs.right_vel_rpm * Robot::RPM_TO_MPS;
            left_v  = (std::fabs(left_mps)  > Controls::BRAKE_SPEED_MPS) ? (-sign(left_mps)  * Controls::TINY_BRAKE_VOLTAGE) : 0.0;
            right_v = (std::fabs(right_mps) > Controls::BRAKE_SPEED_MPS) ? (-sign(right_mps) * Controls::TINY_BRAKE_VOLTAGE) : 0.0;
        }
    }

    if (CT::CURRENT_LIMITING) applyCurrentLimit(left_v, right_v, obs.v_lin, obs.v_ang);

    if (CT::VOLTAGE_CLAMP) {
        double max_abs = std::max(std::fabs(left_v), std::fabs(right_v));
        if (max_abs > System::MAX_VOLTS) {
            double scale = System::MAX_VOLTS / max_abs;
            left_v *= scale;
            right_v *= scale;
        }
    }

    double batt_scale = 1.0;
    if (use_battery_scaling_) {
        batt_scale = (battery_voltage_ > Controls::BATTERY_VALID_V) ? (System::MAX_VOLTS / battery_voltage_) : 1.0;
    }

    plant_left_v_  = left_v  * batt_scale;
    plant_right_v_ = right_v * batt_scale;

    // Report the applied effort (post-safety, pre battery scaling) as chassis.
    ChassisCommand applied;
    applied.linear  = (left_v + right_v) / 2.0;
    applied.angular = (right_v - left_v) / 2.0;
    return applied;
}

void MockDrive::applySubsystems(const SubsystemCommand& sub) {
    subsystems_ = sub;
    cascadeLift().writePercent(subsystems_, readCascade());
}

CascadeReading MockDrive::readCascade() const {
    CascadeReading reading;
    reading.position_deg = cascade_deg_;
    reading.velocity_rpm = cascade_rpm_;
    reading.current_amps = cascade_amps_;
    reading.torque_nm = cascade_torque_;
    return reading;
}

void MockDrive::stepCascade(double dt_seconds) {
    constexpr double kTopDeg = kCascadeTravelDeg;
    constexpr double kSeatDeg = 640.0;
    constexpr double kFreeRpm = 200.0;
    const double cmd = clamp_val(static_cast<double>(subsystems_.cascade_pct), -100.0, 100.0) / 100.0;
    double target_rpm = kFreeRpm * cmd;
    if (cascade_deg_ >= kTopDeg && target_rpm > 0.0) target_rpm = 0.0;
    if (cascade_deg_ <= 0.0 && target_rpm < 0.0) target_rpm = 0.0;
    const double blend = 1.0 - std::exp(-dt_seconds / 0.08);
    cascade_rpm_ += (target_rpm - cascade_rpm_) * blend;
    cascade_deg_ += cascade_rpm_ * 6.0 * dt_seconds;
    cascade_deg_ = clamp_val(cascade_deg_, 0.0, kTopDeg);
    if (cascade_deg_ >= kTopDeg && cascade_rpm_ > 0.0) cascade_rpm_ = 0.0;
    if (cascade_deg_ <= 0.0 && cascade_rpm_ < 0.0) cascade_rpm_ = 0.0;

    const bool loaded = cascade_deg_ > kSeatDeg;
    const bool against_stop = (cascade_deg_ >= kTopDeg - 1.0 && cmd > 0.2)
                           || (cascade_deg_ <= 1.0 && cmd < -0.2);
    double amps = 0.35;
    if (std::fabs(cmd) > 0.05) amps = 1.05;
    if (loaded && cmd < -0.05) amps = 1.70;
    if (!loaded && cmd < -0.05) amps = 0.40;
    if (std::fabs(cmd) < 0.05 && loaded) amps = 1.90;
    if (against_stop) amps = 2.80;
    cascade_amps_ = amps;
    cascade_torque_ = amps * 0.08 * sign(cmd);
}

void MockDrive::setDynamics(double velocity_tau_sec, double skid_tau_sec,
                            double side_resistance_ohm, double speed_scale) {
    if (velocity_tau_sec > 1e-3) velocity_tau_ = velocity_tau_sec;
    if (skid_tau_sec > 1e-3) skid_tau_ = skid_tau_sec;
    if (side_resistance_ohm > 1e-3) side_resistance_ = side_resistance_ohm;
    if (speed_scale > 1e-3) speed_scale_ = speed_scale;
}

void MockDrive::setAxisResponse(double lin_friction_v, double lin_vel_scale, double lin_accel_scale,
                                double ang_friction_v, double ang_vel_scale, double ang_accel_scale) {
    auto friction = [](double volts) {
        if (volts < 0.0) return 0.0;
        if (volts > 24.0) return 24.0;
        return volts;
    };
    auto scale = [](double value) {
        if (value < 0.05) return 0.05;
        if (value > 8.0) return 8.0;
        return value;
    };
    lin_friction_v_ = friction(lin_friction_v);
    ang_friction_v_ = friction(ang_friction_v);
    lin_vel_scale_ = scale(lin_vel_scale);
    ang_vel_scale_ = scale(ang_vel_scale);
    lin_accel_scale_ = scale(lin_accel_scale);
    ang_accel_scale_ = scale(ang_accel_scale);
}

bool MockDrive::axisResponseIsIdentity() const {
    return lin_friction_v_ <= 1e-9 && ang_friction_v_ <= 1e-9
        && std::fabs(lin_vel_scale_ - 1.0) <= 1e-9
        && std::fabs(ang_vel_scale_ - 1.0) <= 1e-9
        && std::fabs(lin_accel_scale_ - 1.0) <= 1e-9
        && std::fabs(ang_accel_scale_ - 1.0) <= 1e-9;
}

double MockDrive::responseSpeedLimit() const {
    const double vmax = Robot::MAX_VEL_LIN * speed_scale_;
    const double axis = (lin_vel_scale_ > ang_vel_scale_) ? lin_vel_scale_ : ang_vel_scale_;
    return vmax * axis;
}

void MockDrive::setVoltageDelayTicks(int ticks) {
    if (ticks < 0) ticks = 0;
    if (ticks >= kVoltageDelayCap) ticks = kVoltageDelayCap - 1;
    voltage_delay_ticks_ = ticks;
}

void MockDrive::delayCommand(double& left_v, double& right_v) {
    delay_left_[delay_head_] = left_v;
    delay_right_[delay_head_] = right_v;
    if (voltage_delay_ticks_ > 0) {
        int back = delay_head_ - voltage_delay_ticks_;
        if (back < 0) back += kVoltageDelayCap;
        left_v = delay_left_[back];
        right_v = delay_right_[back];
    }
    delay_head_ += 1;
    if (delay_head_ >= kVoltageDelayCap) delay_head_ = 0;
}

void MockDrive::settleContacts(double dt_seconds) {
    // Wheel voltage has already become left/right speed. A wall then stops
    // the corner that crossed it. The goal snap runs after that, in every
    // mode, and seats the front center on the point.
    const double half_track = Robot::Chassis::TRACK_WIDTH_M * 0.5;
    const double v_fwd0 = (left_mps_ + right_mps_) * 0.5;
    const double omega0 = (right_mps_ - left_mps_) / Robot::Chassis::TRACK_WIDTH_M;

    ContactBody body;
    body.x_m = true_x_;
    body.y_m = true_y_;
    body.theta = heading_rad_ + imu_offset_;
    body.v_fwd = v_fwd0;
    body.v_lat = lat_mps_;
    body.omega = omega0;

    const double wheel_cap = responseSpeedLimit();
    const ContactStep stepped = stepContacts(
        body, dt_seconds, skid_tau_, wheel_cap);

    true_x_ = body.x_m;
    true_y_ = body.y_m;
    if (dt_seconds > 0.0) {
        heading_rad_ += stepped.yaw_rad;
        // Wheels already counted the pre-contact roll. Put back the forward
        // travel and the yaw the contact removed, so an encoder reading
        // matches the pose that stayed inside. The localization step reads
        // that same encoder delta, so it stops with the wheels.
        const double dropped = v_fwd0 * dt_seconds - stepped.forward_m;
        const double extra_yaw = stepped.yaw_rad - omega0 * dt_seconds;
        left_m_  -= dropped + extra_yaw * half_track;
        right_m_ -= dropped - extra_yaw * half_track;
    }

    lat_mps_ = clamp_val(body.v_lat, -wheel_cap, wheel_cap);
    left_mps_ = clamp_val(body.v_fwd - body.omega * half_track, -wheel_cap, wheel_cap);
    right_mps_ = clamp_val(body.v_fwd + body.omega * half_track, -wheel_cap, wheel_cap);
}

void MockDrive::stepSimulation(double dt_seconds) {
    if (dt_seconds <= 0.0) return;

    double full_scale = (max_voltage_ > MockPlant::MIN_FULL_SCALE_V) ? max_voltage_ : System::MAX_VOLTS;
    const double vmax = Robot::MAX_VEL_LIN * speed_scale_;
    double blend = 1.0 - std::exp(-dt_seconds / velocity_tau_);
    auto approach = [&](double mps, double volts) {
        double target = (volts / full_scale) * vmax;
        return clamp_val(mps + (target - mps) * blend, -vmax, vmax);
    };
    // The lag integrates the voltage from voltage_delay_ticks_ ago. Zero
    // delay keeps this tick's command, which is the compiled plant.
    double left_v = plant_left_v_;
    double right_v = plant_right_v_;
    delayCommand(left_v, right_v);
    if (axisResponseIsIdentity()) {
        left_mps_  = approach(left_mps_,  left_v);
        right_mps_ = approach(right_mps_, right_v);
    } else {
        // Forward voltage is the average. Yaw voltage is V_R - V_L, so a
        // full opposite pair is two full-scale rails and the yaw cap is
        // twice the wheel cap divided by the track.
        const double track = Robot::Chassis::TRACK_WIDTH_M;
        const double half = track * 0.5;
        auto tauFor = [&](double accel_scale) {
            double tau = velocity_tau_ / accel_scale;
            if (tau < 1e-3) tau = 1e-3;
            return tau;
        };
        auto chase = [&](double speed, double volts, double cap, double volt_span,
                         double tau, double friction) {
            double used = volts;
            if (friction > 0.0) {
                if (std::fabs(speed) <= 1e-3) {
                    if (std::fabs(volts) <= friction) used = 0.0;
                    else used = volts - friction * sign(volts);
                } else {
                    used = volts - friction * sign(speed);
                }
            }
            const double target = (volt_span > 1e-6) ? (used / volt_span) * cap : 0.0;
            const double rise = 1.0 - std::exp(-dt_seconds / tau);
            double next = speed + (target - speed) * rise;
            // Friction may stop the axis. It may not reverse it. A command
            // larger than friction and aimed the other way is allowed through.
            const bool driven_through = (volts * speed < 0.0) && (std::fabs(volts) > friction);
            if (speed * next < 0.0 && !driven_through) next = 0.0;
            if (std::fabs(speed) <= 1e-3 && std::fabs(volts) <= friction) next = 0.0;
            return clamp_val(next, -cap, cap);
        };
        const double v_cap = vmax * lin_vel_scale_;
        const double w_cap = (half > 1e-6) ? (vmax / half) * ang_vel_scale_ : 0.0;
        double v = (left_mps_ + right_mps_) * 0.5;
        double w = (track > 1e-6) ? (right_mps_ - left_mps_) / track : 0.0;
        v = chase(v, (left_v + right_v) * 0.5, v_cap, full_scale,
                  tauFor(lin_accel_scale_), lin_friction_v_);
        w = chase(w, right_v - left_v, w_cap, 2.0 * full_scale,
                  tauFor(ang_accel_scale_), ang_friction_v_);
        left_mps_ = v - w * half;
        right_mps_ = v + w * half;
    }

    left_m_  += left_mps_  * dt_seconds;
    right_m_ += right_mps_ * dt_seconds;

    // Omni rollers, the goal-lock orbit, and wall contact all run in here,
    // after the voltage has become wheel speed. A left turn while driving
    // forward still throws the robot to its right.
    settleContacts(dt_seconds);

    const double v_sat = responseSpeedLimit();
    auto amps = [&](double volts, double mps) {
        double emf = (v_sat > 1e-6) ? (mps / v_sat) * full_scale : 0.0;
        return (volts - emf) / side_resistance_;
    };
    left_amps_  = amps(left_v,  left_mps_);
    right_amps_ = amps(right_v, right_mps_);
    stepCascade(dt_seconds);
}

void MockDrive::printTelemetry() const {
    std::cout << "[MockDrive] "
              << "V(L: " << plant_left_v_ << ", R: " << plant_right_v_ << ") | "
              << "Vel(L: " << left_mps_ << ", R: " << right_mps_ << " m/s) | "
              << "Pos(L: " << left_m_ << ", R: " << right_m_ << " m) | "
              << "Heading: " << (heading_rad_ + imu_offset_) << " rad | "
              << "Skid: " << lat_mps_ << " m/s | "
              << "Intake: " << subsystems_.intake_pct
              << "% Cascade: " << subsystems_.cascade_pct << "%\n";
}


void MockDrive::pollKeyboard() {
    fieldwindow::pollInput(joystick_, buttons_);
}

void MockDrive::service(double dt, const Pose2d& display_pose,
                        const Pose2d* particles, int particle_count) {
    // Host domain randomization. Training calls stepSimulation directly and
    // keeps whatever setDynamics stored, so a search is not pulled back here.
    const fieldwindow::PlantOverride over = fieldwindow::plantOverride();
    setDynamics(over.velocity_tau_sec, over.skid_tau_sec, side_resistance_, over.speed_scale);
    setVoltageDelayTicks(over.voltage_delay_ticks);
    setAxisResponse(over.lin_friction_v, over.lin_vel_scale, over.lin_accel_scale,
                    over.ang_friction_v, over.ang_vel_scale, over.ang_accel_scale);

    // Truth at sensor time, before this tick's integration, so it lines up
    // with the estimate the pipeline just published.
    // The plant stores the geometric center. The field view and the
    // estimate both use the front middle.
    const Pose2d truth = poseMetersToInches(tracking::trackingFromCenter(
        Pose2d{true_x_, true_y_, heading_rad_ + imu_offset_}));
    stepSimulation(dt);
    const CascadeReading cascade = readCascade();
    const char* phase = "Idle";
#if !defined(PLANT_MODULE)
    phase = matchLoaderRunning() ? "Loader" : goalScorePhaseName();
#endif
    fieldwindow::setMechanisms(subsystems_, cascade.position_deg / kCascadeTravelDeg,
                               cascade, phase);
    fieldwindow::render(truth, display_pose, particles, particle_count);
}
