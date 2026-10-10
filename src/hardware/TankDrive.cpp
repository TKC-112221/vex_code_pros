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
#include "config/Ports.h"
#include "config/Settings.h"
#include "core/Specs.h"
#include "core/Tuning.h"
#include "math/MathUtils.h"
#include <algorithm>
#include <cmath>

using mathutil::clamp_val;
using mathutil::sign;

static inline std::int32_t voltsToMv(double volts) {
    double mv = clamp_val(volts * 1000.0, -12000.0, 12000.0);
    return static_cast<std::int32_t>(mv);
}
static inline std::int32_t pctToMv(int pct) {
    int p = (pct > 100) ? 100 : (pct < -100 ? -100 : pct);
    return static_cast<std::int32_t>(p * 120);   // 100% -> 12000 mV
}

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

    intake        = new pros::Motor(Ports::INTAKE, pros::MotorGears::green);
    roller_top    = new pros::Motor(Ports::ROLLER_TOP, pros::MotorGears::green);
    roller_bottom = new pros::Motor(Ports::ROLLER_BOTTOM, pros::MotorGears::green);

    pneumatics_g = new pros::adi::DigitalOut(Ports::PNEUMATICS_G);
    pneumatics_b = new pros::adi::DigitalOut(Ports::PNEUMATICS_B);
}

TankDrive::~TankDrive() {
    delete left_drive; delete right_drive; delete imu; delete pilot;
    delete intake; delete roller_top; delete roller_bottom;
    delete pneumatics_g; delete pneumatics_b;
}

void TankDrive::initialize() {
    left_drive->set_brake_mode(pros::MotorBrake::coast);
    right_drive->set_brake_mode(pros::MotorBrake::coast);
}

void TankDrive::beginSensorCalibration() {
    imu_offset_ = 0.0;
}

void TankDrive::waitSensorCalibration() {
    if (Settings::ControlToggles::IMU_FALLBACK) return;
    // Blocking reset. The kernel gives up after 3 s, and it never polls
    // is_calibrating(), which stays true on an error status and froze
    // initialize() the moment the startup clip returned.
    imu->reset(true);
}

void TankDrive::calibrateSensors() {
    beginSensorCalibration();
    waitSensorCalibration();
}

void TankDrive::stop() {
    left_drive->move_voltage(0);
    right_drive->move_voltage(0);
}

JoystickData TankDrive::readJoystick() {
    JoystickData joy;
    joy.left_y  =  pilot->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)  / Controller::FULL_SCALE;
    joy.left_x  =  pilot->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)  / Controller::FULL_SCALE;
    joy.right_y =  pilot->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) / Controller::FULL_SCALE;
    joy.right_x =  pilot->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / Controller::FULL_SCALE;
    return joy;
}

ButtonState TankDrive::readButtons() {
    ButtonState b;
    b.l1 = pilot->get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    b.l2 = pilot->get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    b.r1 = pilot->get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    b.r2 = pilot->get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    b.up = pilot->get_digital(pros::E_CONTROLLER_DIGITAL_UP);
    b.down = pilot->get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    b.left = pilot->get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
    b.right = pilot->get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    b.a = pilot->get_digital(pros::E_CONTROLLER_DIGITAL_A);
    b.b = pilot->get_digital(pros::E_CONTROLLER_DIGITAL_B);
    b.x = pilot->get_digital(pros::E_CONTROLLER_DIGITAL_X);
    b.y = pilot->get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    return b;
}

void TankDrive::alignHeadingTo(double target_rad) {
    if (Settings::ControlToggles::IMU_FALLBACK) { imu_offset_ = 0.0; return; }
    double raw = -imu->get_rotation() * (M_PI / 180.0);
    imu_offset_ = target_rad - raw;
}

ChassisObservation TankDrive::readObservation(double prev_heading_rad) {
    namespace CT = Settings::ControlToggles;
    ChassisObservation o;

    // Forward kinematics: wheel travel -> forward increment (deg -> turns -> m).
    double cl = (left_drive->get_position()  / 360.0) * Robot::METERS_PER_TURN;
    double cr = (right_drive->get_position() / 360.0) * Robot::METERS_PER_TURN;
    double delta_l = cl - prev_l_;
    double delta_r = cr - prev_r_;
    prev_l_ = cl; prev_r_ = cr;
    o.forward_delta = (delta_l + delta_r) / 2.0;

    //double l_vel = left_drive->get_actual_velocity();
    //double r_vel = right_drive->get_actual_velocity();
    //o.v_lin = (l_vel + r_vel) / 2.0 * Robot::RPM_TO_MPS;
    double l_vel = delta_l / System::DT_SEC;
    double r_vel = delta_r / System::DT_SEC;
    o.v_lin = o.forward_delta / System::DT_SEC * Robot::RPM_TO_MPS;


    if (CT::IMU_FALLBACK) {
        o.heading_rad = prev_heading_rad + (delta_r - delta_l) / Robot::Chassis::TRACK_WIDTH_M;
        o.v_ang = (r_vel - l_vel) * Robot::RPM_TO_MPS / Robot::Chassis::TRACK_WIDTH_M;
        o.imu_rate_z_dps  = -(r_vel - l_vel) * Robot::RPM_TO_MPS / Robot::Chassis::TRACK_WIDTH_M * (180.0 / M_PI);
        o.imu_accel_x = 0.0; o.imu_accel_y = 0.0;
    } else {
        o.heading_rad = -imu->get_rotation() * (M_PI / 180.0) + imu_offset_;
        o.v_ang = -imu->get_gyro_rate().z * (M_PI / 180.0);
        o.imu_rate_z_dps  = imu->get_gyro_rate().z;
        o.imu_accel_x = imu->get_accel().x;
        o.imu_accel_y = imu->get_accel().y;
    }

    o.left_pos_deg = left_drive->get_position();
    o.right_pos_deg = right_drive->get_position();
    o.left_vel_rpm = l_vel;  o.right_vel_rpm = r_vel;
    o.left_current_amps  = left_drive->get_current_draw()  / 1000.0;
    o.right_current_amps = right_drive->get_current_draw() / 1000.0;
    {
        const auto l_temps = left_drive->get_temperature_all();
        const auto r_temps = right_drive->get_temperature_all();
        double l_sum = 0.0, r_sum = 0.0;
        for (double t : l_temps) l_sum += t;
        for (double t : r_temps) r_sum += t;
        o.left_temp_c  = l_temps.empty() ? 0.0 : l_sum / static_cast<double>(l_temps.size());
        o.right_temp_c = r_temps.empty() ? 0.0 : r_sum / static_cast<double>(r_temps.size());
    }
    o.battery_voltage = pros::battery::get_voltage() / 1000.0;
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

    // Inverse kinematics: chassis effort -> per-bank voltage.
    double left_v  = cmd.linear - cmd.angular;
    double right_v = cmd.linear + cmd.angular;

    // Stick-zero brake: gently oppose residual motion when the sticks are idle.
    if (CT::STICK_ZERO_BRAKE) {
        double fwd  = pilot->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)  / Controller::FULL_SCALE;
        double turn = -pilot->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / Controller::FULL_SCALE;
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
        double batt = pros::battery::get_voltage() / 1000.0;
        batt_scale = (batt > Controls::BATTERY_VALID_V) ? (System::MAX_VOLTS / batt) : 1.0;
    }

    left_drive->move_voltage(voltsToMv(left_v * batt_scale));
    right_drive->move_voltage(voltsToMv(right_v * batt_scale));

    // Report the applied effort (post-safety, pre battery scaling) as chassis.
    ChassisCommand applied;
    applied.linear  = (left_v + right_v) / 2.0;
    applied.angular = (right_v - left_v) / 2.0;
    return applied;
}

void TankDrive::applySubsystems(const SubsystemCommand& sub) {
    intake->move_voltage(pctToMv(sub.intake_pct));
    roller_top->move_voltage(pctToMv(sub.roller_top_pct));
    roller_bottom->move_voltage(pctToMv(sub.roller_bottom_pct));
    pneumatics_g->set_value(sub.pneumatics_g_state);
    pneumatics_b->set_value(sub.pneumatics_b_state);
}
