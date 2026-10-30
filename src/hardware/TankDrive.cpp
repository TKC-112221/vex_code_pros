/* ---------------------------------------------------------------------------
 * TankDrive.cpp  --  differential-drive forward/inverse kinematics + hardware
 * safety on the PROS V5 API. All left/right specifics from v1 are consolidated
 * here, behind the drivebase-agnostic IDriveBase contract.
 *
 * PROS API notes vs VEXcode:
 *   - motor voltage is millivolts (move_voltage), so volts are * 1000
 *   - get_position() is degrees; turns = degrees / 360
 *   - get_current_draw() is milliamps; amps = mA / 1000
 *   - analog sticks are -127..127; buttons via get_digital
 *   - motor reversal is encoded by negating the port at construction
 * ------------------------------------------------------------------------- */
#include "hardware/TankDrive.h"
#include "hardware/ChassisVoltage.h"
#include "hardware/DeviceRead.h"
#include "hardware/DifferentialMix.h"
#include "logic/CascadeLift.h"
#include "config/Ports.h"
#include "config/Settings.h"
#include "core/Specs.h"
#include "core/Tuning.h"
#include "math/MathUtils.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

using mathutil::clamp_val;
using mathutil::sign;

static constexpr int kBankMotors = 3;

TankDrive::TankDrive(const IFeedForward& feedforward, double max_volts, bool scale_battery)
    : feedforward_(&feedforward), max_voltage(max_volts), use_battery_scaling(scale_battery) {
    pilot = new pros::Controller(pros::E_CONTROLLER_MASTER);
    imu   = new pros::Imu(Ports::IMU);

    // Left bank reversed (negative ports); right bank forward.
    left_drive = new pros::MotorGroup(
        { (std::int8_t)-Ports::L1, (std::int8_t)-Ports::L2, (std::int8_t)-Ports::L3 },
        pros::MotorGears::blue);
    right_drive = new pros::MotorGroup(
        { (std::int8_t)Ports::R1, (std::int8_t)Ports::R2, (std::int8_t)Ports::R3 },
        pros::MotorGears::blue);

    main1 = new pros::Motor(Ports::MAIN1, pros::MotorGears::blue);
    main2 = new pros::Motor(Ports::MAIN2, pros::MotorGears::blue);
    sub1  = new pros::Motor(Ports::SUB1, pros::MotorGears::green);
    sub2  = new pros::Motor(Ports::SUB2, pros::MotorGears::green);

    pneumatics_g = new pros::adi::DigitalOut(Ports::PNEUMATICS_G);
    pneumatics_b = new pros::adi::DigitalOut(Ports::PNEUMATICS_B);
}

TankDrive::~TankDrive() {
    delete left_drive; delete right_drive; delete imu; delete pilot;
    delete main1; delete main2; delete sub1; delete sub2;
    delete pneumatics_g; delete pneumatics_b;
}

void TankDrive::initialize() {
    left_drive->set_brake_mode(pros::MotorBrake::coast);
    right_drive->set_brake_mode(pros::MotorBrake::coast);
    // Hold keeps the cascade up when both motors are idle. Voltage still wins
    // while a command is applied. SUB1/SUB2 stay free.
    main1->set_brake_mode(pros::MotorBrake::hold);
    main2->set_brake_mode(pros::MotorBrake::hold);
    // Height 0 is the cascade position at startup.
    cascadeLift().zero(readCascade());
    sub1->set_brake_mode(pros::MotorBrake::coast);
    sub2->set_brake_mode(pros::MotorBrake::coast);
    ranges_.probe();
    std::printf("subsys: init MAIN1 port %u (const %d) MAIN2 port %u (const %d) "
                "SUB1 port %u SUB2 port %u\n",
                main1->get_port(), Ports::MAIN1, main2->get_port(), Ports::MAIN2,
                sub1->get_port(), sub2->get_port());
    std::fflush(stdout);
}

void TankDrive::beginSensorCalibration() {
    imu_offset_ = 0.0;
}

void TankDrive::waitSensorCalibration() {
    if (Settings::ControlToggles::IMU_FALLBACK) {
        imu_wheels_ = true;
        return;
    }
    // Blocking reset, with one retry. A failed reset latches wheel heading
    // so later reads do not integrate PROS_ERR_F as a heading.
    imu_wheels_ = !devread::imuCalibrated(*imu);
    if (imu_wheels_) have_heading_ = false;
}

void TankDrive::calibrateSensors() {
    beginSensorCalibration();
    waitSensorCalibration();
}

void TankDrive::stop() {
    left_drive->move_voltage(0);
    right_drive->move_voltage(0);
    volt_surge_ = 0.0;
    volt_yaw_ = 0.0;
    volt_primed_ = false;
}

JoystickData TankDrive::readJoystick() {
    JoystickData joy;
    joy.left_y  = devread::stick(*pilot, pros::E_CONTROLLER_ANALOG_LEFT_Y);
    joy.left_x  = devread::stick(*pilot, pros::E_CONTROLLER_ANALOG_LEFT_X);
    joy.right_y = devread::stick(*pilot, pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    joy.right_x = devread::stick(*pilot, pros::E_CONTROLLER_ANALOG_RIGHT_X);
    return joy;
}

ButtonState TankDrive::readButtons() {
    ButtonState b;
    b.l1 = devread::button(*pilot, pros::E_CONTROLLER_DIGITAL_L1);
    b.l2 = devread::button(*pilot, pros::E_CONTROLLER_DIGITAL_L2);
    b.r1 = devread::button(*pilot, pros::E_CONTROLLER_DIGITAL_R1);
    b.r2 = devread::button(*pilot, pros::E_CONTROLLER_DIGITAL_R2);
    b.up = devread::button(*pilot, pros::E_CONTROLLER_DIGITAL_UP);
    b.down = devread::button(*pilot, pros::E_CONTROLLER_DIGITAL_DOWN);
    b.left = devread::button(*pilot, pros::E_CONTROLLER_DIGITAL_LEFT);
    b.right = devread::button(*pilot, pros::E_CONTROLLER_DIGITAL_RIGHT);
    b.a = devread::button(*pilot, pros::E_CONTROLLER_DIGITAL_A);
    b.b = devread::button(*pilot, pros::E_CONTROLLER_DIGITAL_B);
    b.x = devread::button(*pilot, pros::E_CONTROLLER_DIGITAL_X);
    b.y = devread::button(*pilot, pros::E_CONTROLLER_DIGITAL_Y);
    return b;
}

void TankDrive::alignHeadingTo(double target_rad) {
    if (Settings::ControlToggles::IMU_FALLBACK || imu_wheels_) { imu_offset_ = 0.0; return; }
    const double rot = imu->get_rotation();
    if (!devread::finite(rot)) return;
    const double raw = -rot * (M_PI / 180.0);
    imu_offset_ = target_rad - raw;
}

ChassisObservation TankDrive::readObservation(double prev_heading_rad) {
    namespace CT = Settings::ControlToggles;
    ChassisObservation o;

    bool gap = false;
    const double dt = devread::sampleDt(last_sense_ms_, have_sense_ms_, gap);
    o.sample_dt = dt;
    if (gap) { have_l_ = false; have_r_ = false; }

    // Both banks, every motor. A failed read holds that bank's baseline.
    double ldeg = 0.0, rdeg = 0.0;
    const bool lok = devread::groupDegrees(*left_drive, kBankMotors, ldeg);
    const bool rok = devread::groupDegrees(*right_drive, kBankMotors, rdeg);
    const double delta_l = lok ? devread::travelMeters(ldeg, prev_l_, have_l_) : 0.0;
    const double delta_r = rok ? devread::travelMeters(rdeg, prev_r_, have_r_) : 0.0;
    o.forward_delta = (delta_l + delta_r) / 2.0;
    o.left_pos_deg = lok ? ldeg : (have_l_ ? prev_l_ / Robot::METERS_PER_TURN * 360.0 : 0.0);
    o.right_pos_deg = rok ? rdeg : (have_r_ ? prev_r_ / Robot::METERS_PER_TURN * 360.0 : 0.0);

    const double v_l = (dt > 0.0) ? delta_l / dt : 0.0;
    const double v_r = (dt > 0.0) ? delta_r / dt : 0.0;
    o.v_lin = (dt > 0.0) ? o.forward_delta / dt : 0.0;
    o.left_vel_rpm = v_l / Robot::RPM_TO_MPS;
    o.right_vel_rpm = v_r / Robot::RPM_TO_MPS;

    if (CT::IMU_FALLBACK || imu_wheels_) {
        o.heading_rad = prev_heading_rad + (delta_r - delta_l) / Robot::Chassis::TRACK_WIDTH_M;
        o.v_ang = (v_r - v_l) / Robot::Chassis::TRACK_WIDTH_M;
        o.imu_rate_z_dps = -o.v_ang * (180.0 / M_PI);
        o.imu_accel_x = 0.0; o.imu_accel_y = 0.0;
        held_heading_ = o.heading_rad;
        have_heading_ = true;
    } else {
        const double rot = imu->get_rotation();
        if (devread::finite(rot)) {
            o.heading_rad = -rot * (M_PI / 180.0) + imu_offset_;
            held_heading_ = o.heading_rad;
            have_heading_ = true;
        } else {
            o.heading_rad = have_heading_ ? held_heading_ : prev_heading_rad;
        }
        const pros::imu_gyro_s_t rate = imu->get_gyro_rate();
        if (devread::finite(rate.z)) {
            o.v_ang = -rate.z * (M_PI / 180.0);
            o.imu_rate_z_dps = rate.z;
        }
        const pros::imu_accel_s_t accel = imu->get_accel();
        if (devread::finite(accel.x)) o.imu_accel_x = accel.x;
        if (devread::finite(accel.y)) o.imu_accel_y = accel.y;
    }

    o.left_current_amps = devread::groupAmps(*left_drive, kBankMotors);
    o.right_current_amps = devread::groupAmps(*right_drive, kBankMotors);
    o.left_temp_c = devread::groupTempC(*left_drive, kBankMotors);
    o.right_temp_c = devread::groupTempC(*right_drive, kBankMotors);
    o.battery_voltage = devread::batteryVolts();
    return o;
}

void TankDrive::readDistances(DistanceReading (&out)[kDistanceChannels]) {
    ranges_.read(out);
}

// Back-EMF current limit: clamp each side to its predicted EMF +/- max drop.
void TankDrive::applyCurrentLimit(double& left_v, double& right_v, double v_lin, double v_ang) const {
    double turn_r = (std::fabs(v_ang) < Physics::OMEGA_EPS) ? Physics::TURN_RADIUS_MAX_M
                  : std::min(std::fabs(v_lin) / (std::fabs(v_ang) + Physics::OMEGA_EPS), Physics::TURN_RADIUS_MAX_M);
    double v_avg_emf = feedforward_->linearVolts(v_lin, 0.0, std::fabs(v_lin), 0.0, v_ang, turn_r);
    double v_diff_emf = feedforward_->angularVolts(v_ang, 0.0, std::fabs(v_lin), 0.0, turn_r);
    double left_emf  = v_avg_emf - v_diff_emf / 2.0;
    double right_emf = v_avg_emf + v_diff_emf / 2.0;
    left_v  = clamp_val(left_v,  left_emf  - System::MAX_VOLTAGE_DROP, left_emf  + System::MAX_VOLTAGE_DROP);
    right_v = clamp_val(right_v, right_emf - System::MAX_VOLTAGE_DROP, right_emf + System::MAX_VOLTAGE_DROP);
}

ChassisCommand TankDrive::applyDrive(const ChassisCommand& cmd, const ChassisObservation& obs) {
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
        double fwd  = devread::stick(*pilot, pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double turn = -devread::stick(*pilot, pros::E_CONTROLLER_ANALOG_RIGHT_X);
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
            left_v *= scale; right_v *= scale;
        }
    }

    double batt_scale = 1.0;
    if (use_battery_scaling) {
        double batt = devread::batteryVolts();
        batt_scale = (batt > Controls::BATTERY_VALID_V) ? (System::MAX_VOLTS / batt) : 1.0;
    }

    left_drive->move_voltage(devread::voltsToMv(left_v * batt_scale));
    right_drive->move_voltage(devread::voltsToMv(right_v * batt_scale));

    // Report the applied effort (post-safety, pre battery scaling) as chassis.
    ChassisCommand applied;
    applied.linear  = (left_v + right_v) / 2.0;
    applied.angular = (right_v - left_v) / 2.0;
    return applied;
}

CascadeReading TankDrive::readCascade() const {
    return readMainPair(*main1, *main2);
}

void TankDrive::applySubsystems(const SubsystemCommand& sub) {
    SubsystemCommand cmd = sub;
    cascadeLift().writePercent(cmd, readCascade());
    const MainPairPct pair = mixMainPair(cmd.cascade_pct, cmd.intake_pct);
    const std::int32_t m1_mv = devread::pctToMv(pair.main1_pct);
    const std::int32_t m2_mv = devread::pctToMv(pair.main2_pct);
    const std::int32_t m1_rc = main1->move_voltage(m1_mv);
    const std::int32_t m2_rc = main2->move_voltage(m2_mv);
    const std::int32_t s1_rc = sub1->move_voltage(0);
    const std::int32_t s2_rc = sub2->move_voltage(0);
    pneumatics_g->set_value(cmd.pneumatics_g_state);
    pneumatics_b->set_value(cmd.pneumatics_b_state);

    // Change-only, plus a line every 500 ms while a shoulder command is live.
    // rc is 1 on success and PROS_ERR (2147483647) when the port is not a motor.
    static int prev_cascade = 0, prev_intake = 0, prev_height = 0;
    static std::uint32_t prev_ms = 0;
    const std::uint32_t now = pros::millis();
    const int height_flag = sub.cascade_height ? 1 : 0;
    const bool changed = sub.cascade_pct != prev_cascade || sub.intake_pct != prev_intake
                      || height_flag != prev_height;
    const bool live = cmd.cascade_pct != 0 || cmd.intake_pct != 0 || sub.cascade_height;
    if (changed || (live && now - prev_ms >= 500)) {
        prev_cascade = sub.cascade_pct;
        prev_intake = sub.intake_pct;
        prev_height = height_flag;
        prev_ms = now;
        std::printf("subsys: cascade %d (req %d h=%d tgt=%.1f) intake %d -> "
                    "M1 p%u %d%% %ldmV rc=%ld | M2 p%u %d%% %ldmV rc=%ld | "
                    "S1 rc=%ld S2 rc=%ld\n",
                    cmd.cascade_pct, sub.cascade_pct, height_flag, sub.cascade_target_in,
                    cmd.intake_pct,
                    main1->get_port(), pair.main1_pct, static_cast<long>(m1_mv), static_cast<long>(m1_rc),
                    main2->get_port(), pair.main2_pct, static_cast<long>(m2_mv), static_cast<long>(m2_rc),
                    static_cast<long>(s1_rc), static_cast<long>(s2_rc));
        std::fflush(stdout);
    }
}