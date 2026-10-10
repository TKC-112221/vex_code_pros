/* ---------------------------------------------------------------------------
 * MockDrive.cpp  --  differential-drive forward/inverse kinematics with the
 * same per-actuator safety as TankDrive, backed by a simulated wheel plant
 * instead of the PROS V5 API.
 *
 * Each side is a first-order lag toward (voltage / max_volts) * MAX_VEL_LIN,
 * so a held voltage settles at a finite speed and zero voltage coasts to a
 * stop. service() is the desktop stand-in for the 10 ms of real time that
 * elapses between pipeline ticks: it integrates the plant and redraws the
 * estimator pose. The keyboard is the V5 controller.
 * ------------------------------------------------------------------------- */
#include "hardware/MockDrive.h"
#include "config/Settings.h"
#include "core/Specs.h"
#include "core/Tuning.h"
#include "math/MathUtils.h"
#include "math/WallRange.h"
#include "host/FieldWindow.h"
#include <algorithm>
#include <cmath>
#include <iostream>

using mathutil::clamp_val;
using mathutil::sign;

MockDrive::MockDrive(const IFeedForward& feedforward, double max_volts, bool scale_battery)
    : feedforward_(&feedforward), max_voltage_(max_volts), use_battery_scaling_(scale_battery) {}

void MockDrive::initialize() {
    ensureSensorScales();
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
    true_x_ = true_y_ = 0.0;
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
    prev_v_fwd_ = (left_mps_ + right_mps_) * 0.5;
    prev_v_lat_ = lat_mps_;
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

void MockDrive::resetSensors() {
    left_m_ = right_m_ = 0.0;
    left_mps_ = right_mps_ = 0.0;
    plant_left_v_ = plant_right_v_ = 0.0;
    left_amps_ = right_amps_ = 0.0;
    prev_l_ = prev_r_ = 0.0;
    heading_rad_ = 0.0;
    imu_offset_ = 0.0;
    lat_mps_ = 0.0;
    prev_v_fwd_ = prev_v_lat_ = 0.0;
    true_x_ = true_y_ = 0.0;
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
    double delta_l = qL - prev_l_;
    double delta_r = qR - prev_r_;
    prev_l_ = qL;
    prev_r_ = qR;
    o.forward_delta = (delta_l + delta_r) / 2.0;
    // Encoders do not measure the omni skid, so the localizer sees no strafe.
    o.strafe_delta = 0.0;

    const double vel_sigma = Robot::Encoder::VEL_SIGMA_RPM * Robot::RPM_TO_MPS;
    const double l_mps = left_mps_ + normal_() * vel_sigma;
    const double r_mps = right_mps_ + normal_() * vel_sigma;
    double l_vel = l_mps / Robot::RPM_TO_MPS;
    double r_vel = r_mps / Robot::RPM_TO_MPS;
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

    o.left_pos_deg  = (qL / Robot::METERS_PER_TURN) * 360.0;
    o.right_pos_deg = (qR / Robot::METERS_PER_TURN) * 360.0;
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

    const Pose2d pose{true_x_, true_y_, heading_rad_ + imu_offset_};
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

    // Inverse kinematics: chassis effort -> per-bank voltage.
    double left_v  = cmd.linear - cmd.angular;
    double right_v = cmd.linear + cmd.angular;

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
}

void MockDrive::setDynamics(double velocity_tau_sec, double skid_tau_sec,
                            double side_resistance_ohm, double speed_scale) {
    if (velocity_tau_sec > 1e-3) velocity_tau_ = velocity_tau_sec;
    if (skid_tau_sec > 1e-3) skid_tau_ = skid_tau_sec;
    if (side_resistance_ohm > 1e-3) side_resistance_ = side_resistance_ohm;
    if (speed_scale > 1e-3) speed_scale_ = speed_scale;
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
    left_mps_  = approach(left_mps_,  plant_left_v_);
    right_mps_ = approach(right_mps_, plant_right_v_);

    left_m_  += left_mps_  * dt_seconds;
    right_m_ += right_mps_ * dt_seconds;

    // Omni rollers free the lateral axis. Yaw swings forward speed sideways:
    //   d v_lat / dt = -ω * v_fwd - v_lat / tau
    // Positive lat is to the left. A left turn while driving forward throws
    // the robot to its right, which is the momentum it already had.
    const double v_fwd = (left_mps_ + right_mps_) * 0.5;
    const double omega = (right_mps_ - left_mps_) / Robot::Chassis::TRACK_WIDTH_M;
    const double decay = std::exp(-dt_seconds / skid_tau_);
    const double lat1 = clamp_val(
        lat_mps_ * decay + (-omega * v_fwd) * skid_tau_ * (1.0 - decay),
        -vmax, vmax);
    const double lat_avg = 0.5 * (lat_mps_ + lat1);
    lat_mps_ = lat1;

    const double h0 = heading_rad_ + imu_offset_;
    heading_rad_ += omega * dt_seconds;
    const double h1 = heading_rad_ + imu_offset_;
    const double dth = mathutil::normalize_angle(h1 - h0);
    const double mid = h0 + dth * 0.5;
    const double forward = v_fwd * dt_seconds;
    const double lateral = lat_avg * dt_seconds;
    true_x_ += forward * std::cos(mid) - lateral * std::sin(mid);
    true_y_ += forward * std::sin(mid) + lateral * std::cos(mid);

    auto amps = [&](double volts, double mps) {
        double emf = (vmax > 1e-6) ? (mps / vmax) * full_scale : 0.0;
        return (volts - emf) / side_resistance_;
    };
    left_amps_  = amps(plant_left_v_,  left_mps_);
    right_amps_ = amps(plant_right_v_, right_mps_);
}

void MockDrive::printTelemetry() const {
    std::cout << "[MockDrive] "
              << "V(L: " << plant_left_v_ << ", R: " << plant_right_v_ << ") | "
              << "Vel(L: " << left_mps_ << ", R: " << right_mps_ << " m/s) | "
              << "Pos(L: " << left_m_ << ", R: " << right_m_ << " m) | "
              << "Heading: " << (heading_rad_ + imu_offset_) << " rad | "
              << "Skid: " << lat_mps_ << " m/s | "
              << "Intake: " << subsystems_.intake_pct << "%\n";
}


void MockDrive::pollKeyboard() {
    fieldwindow::pollInput(joystick_, buttons_);
}

void MockDrive::service(double dt, const Pose2d& display_pose,
                        const Pose2d* particles, int particle_count) {
    // Truth at sensor time, before this tick's integration, so it lines up
    // with the estimate the pipeline just published.
    const Pose2d truth = poseMetersToInches(
        Pose2d{true_x_, true_y_, heading_rad_ + imu_offset_});
    stepSimulation(dt);
    fieldwindow::render(truth, display_pose, particles, particle_count);
}
