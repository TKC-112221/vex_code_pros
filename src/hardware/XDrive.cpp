/* ---------------------------------------------------------------------------
 * XDrive.cpp  --  four-omni holonomic forward/inverse kinematics on the PROS
 * V5 API. A corner omni at 45 deg has driving-direction components +/- 1/sqrt(2),
 * so the chassis<->wheel maps carry a sqrt(2) factor:
 *   chassis forward = sqrt(2) * (average wheel travel).
 * ------------------------------------------------------------------------- */
#include "hardware/XDrive.h"
#include "config/Ports.h"
#include "config/Settings.h"
#include "core/Specs.h"
#include "math/MathUtils.h"
#include <algorithm>
#include <cmath>

static constexpr double SQRT2 = 1.4142135623730951;

static inline std::int32_t voltsToMv(double volts) {
    double mv = mathutil::clamp_val(volts * 1000.0, -12000.0, 12000.0);
    return static_cast<std::int32_t>(mv);
}
static inline std::int32_t pctToMv(int pct) {
    int p = (pct > 100) ? 100 : (pct < -100 ? -100 : pct);
    return static_cast<std::int32_t>(p * 120);
}

XDrive::XDrive(double max_volts, bool scale_battery)
    : max_voltage_(max_volts), use_battery_scaling_(scale_battery) {
    pilot_ = new pros::Controller(pros::E_CONTROLLER_MASTER);
    imu_   = new pros::Imu(Ports::IMU);

    // Reversal via negated port: FL/BL reversed, FR/BR forward.
    fl_ = new pros::Motor((std::int8_t)-Ports::FL, pros::MotorGears::blue);
    fr_ = new pros::Motor((std::int8_t) Ports::FR, pros::MotorGears::blue);
    bl_ = new pros::Motor((std::int8_t)-Ports::BL, pros::MotorGears::blue);
    br_ = new pros::Motor((std::int8_t) Ports::BR, pros::MotorGears::blue);

    intake_        = new pros::Motor(Ports::INTAKE, pros::MotorGears::green);
    roller_top_    = new pros::Motor(Ports::ROLLER_TOP, pros::MotorGears::green);
    roller_bottom_ = new pros::Motor(Ports::ROLLER_BOTTOM, pros::MotorGears::green);
    pneumatics_g_  = new pros::adi::DigitalOut(Ports::PNEUMATICS_G);
    pneumatics_b_  = new pros::adi::DigitalOut(Ports::PNEUMATICS_B);
}

XDrive::~XDrive() {
    delete fl_; delete fr_; delete bl_; delete br_;
    delete imu_; delete pilot_;
    delete intake_; delete roller_top_; delete roller_bottom_;
    delete pneumatics_g_; delete pneumatics_b_;
}

void XDrive::initialize() {
    fl_->set_brake_mode(pros::MotorBrake::coast); fr_->set_brake_mode(pros::MotorBrake::coast);
    bl_->set_brake_mode(pros::MotorBrake::coast); br_->set_brake_mode(pros::MotorBrake::coast);
}

void XDrive::calibrateSensors() {
    imu_offset_ = 0.0;
    if (!Settings::ControlToggles::IMU_FALLBACK) {
        imu_->reset();
        while (imu_->is_calibrating()) pros::delay(10);
    }
}

void XDrive::stop() {
    fl_->move_voltage(0); fr_->move_voltage(0);
    bl_->move_voltage(0); br_->move_voltage(0);
}

JoystickData XDrive::readJoystick() {
    JoystickData joy;
    joy.forward =  pilot_->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)  / 127.0;
    joy.turn    = -pilot_->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0;
    joy.strafe  =  pilot_->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)  / 127.0;   // holonomic
    return joy;
}

ButtonState XDrive::readButtons() {
    ButtonState b;
    b.l1 = pilot_->get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    b.l2 = pilot_->get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    b.r1 = pilot_->get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    b.r2 = pilot_->get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    b.up = pilot_->get_digital(pros::E_CONTROLLER_DIGITAL_UP);
    b.down = pilot_->get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    b.left = pilot_->get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
    b.right = pilot_->get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    b.a = pilot_->get_digital(pros::E_CONTROLLER_DIGITAL_A);
    b.b = pilot_->get_digital(pros::E_CONTROLLER_DIGITAL_B);
    b.x = pilot_->get_digital(pros::E_CONTROLLER_DIGITAL_X);
    b.y = pilot_->get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    return b;
}

void XDrive::alignHeadingTo(double target_rad) {
    if (Settings::ControlToggles::IMU_FALLBACK) { imu_offset_ = 0.0; return; }
    double raw = -imu_->get_rotation() * (M_PI / 180.0);
    imu_offset_ = target_rad - raw;
}

double XDrive::batteryScale() const {
    if (!use_battery_scaling_) return 1.0;
    double batt = pros::battery::get_voltage() / 1000.0;
    return (batt > 5.0) ? (System::MAX_VOLTS / batt) : 1.0;
}

double XDrive::clampScale(const double* v, int n, double max_v) {
    double max_abs = 0.0;
    for (int i = 0; i < n; ++i) max_abs = std::max(max_abs, std::fabs(v[i]));
    return (max_abs > max_v) ? (max_v / max_abs) : 1.0;
}

// IMU-derived heading/omega + battery/accel log channels (shared w/ parenthesis).
void XDrive::fillImuChannels(double prev_heading_rad, ChassisObservation& o) const {
    if (Settings::ControlToggles::IMU_FALLBACK) {
        // Wheel-derived heading isn't well-posed for a holonomic base; hold last.
        o.heading_rad = prev_heading_rad;
        o.v_ang = 0.0;
        o.imu_heading_deg = prev_heading_rad * (180.0 / M_PI);
        o.imu_rate_z_dps = 0.0;
        o.imu_accel_x = 0.0; o.imu_accel_y = 0.0;
    } else {
        o.heading_rad = -imu_->get_rotation() * (M_PI / 180.0) + imu_offset_;
        o.v_ang = -imu_->get_gyro_rate().z * (M_PI / 180.0);
        o.imu_heading_deg = imu_->get_rotation();
        o.imu_rate_z_dps = imu_->get_gyro_rate().z;
        o.imu_accel_x = imu_->get_accel().x;
        o.imu_accel_y = imu_->get_accel().y;
    }
    o.battery_voltage = pros::battery::get_voltage() / 1000.0;
}

ChassisObservation XDrive::readObservation(double prev_heading_rad) {
    ChassisObservation o;

    // Wheel travel in meters of wheel surface (deg -> turns -> m).
    double cfl = (fl_->get_position() / 360.0) * Robot::METERS_PER_TURN;
    double cfr = (fr_->get_position() / 360.0) * Robot::METERS_PER_TURN;
    double cbl = (bl_->get_position() / 360.0) * Robot::METERS_PER_TURN;
    double cbr = (br_->get_position() / 360.0) * Robot::METERS_PER_TURN;
    double dfl = cfl - prev_fl_, dfr = cfr - prev_fr_, dbl = cbl - prev_bl_, dbr = cbr - prev_br_;
    prev_fl_ = cfl; prev_fr_ = cfr; prev_bl_ = cbl; prev_br_ = cbr;

    // Forward kinematics (sqrt(2) accounts for the 45 deg roller geometry).
    o.forward_delta = SQRT2 * (dfl + dfr + dbl + dbr) / 4.0;
    o.strafe_delta  = SQRT2 * (dfr + dbl - dfl - dbr) / 4.0;

    double rfl = fl_->get_actual_velocity(), rfr = fr_->get_actual_velocity();
    double rbl = bl_->get_actual_velocity(), rbr = br_->get_actual_velocity();
    o.v_lin = SQRT2 * (rfl + rfr + rbl + rbr) / 4.0 * Robot::RPM_TO_MPS;

    fillImuChannels(prev_heading_rad, o);

    // Log channels: fold corners into representative left/right pairs.
    o.left_pos_deg  = (fl_->get_position() + bl_->get_position()) / 2.0;
    o.right_pos_deg = (fr_->get_position() + br_->get_position()) / 2.0;
    o.left_vel_rpm  = (rfl + rbl) / 2.0;
    o.right_vel_rpm = (rfr + rbr) / 2.0;
    o.left_current_amps  = (fl_->get_current_draw() + bl_->get_current_draw()) / 2000.0;
    o.right_current_amps = (fr_->get_current_draw() + br_->get_current_draw()) / 2000.0;
    return o;
}

ChassisCommand XDrive::applyDrive(const ChassisCommand& cmd, const ChassisObservation& /*obs*/) {
    // Inverse kinematics: chassis effort -> four corner voltages.
    double v[4] = {
        cmd.linear - cmd.lateral - cmd.angular,   // FL
        cmd.linear + cmd.lateral + cmd.angular,   // FR
        cmd.linear + cmd.lateral - cmd.angular,   // BL
        cmd.linear - cmd.lateral + cmd.angular    // BR
    };

    double scale = 1.0;
    if (Settings::ControlToggles::VOLTAGE_CLAMP) scale = clampScale(v, 4, System::MAX_VOLTS);
    double bscale = batteryScale();

    fl_->move_voltage(voltsToMv(v[0] * scale * bscale));
    fr_->move_voltage(voltsToMv(v[1] * scale * bscale));
    bl_->move_voltage(voltsToMv(v[2] * scale * bscale));
    br_->move_voltage(voltsToMv(v[3] * scale * bscale));

    ChassisCommand applied;
    applied.linear  = cmd.linear  * scale;
    applied.lateral = cmd.lateral * scale;
    applied.angular = cmd.angular * scale;
    return applied;
}

void XDrive::applySubsystems(const SubsystemCommand& sub) {
    intake_->move_voltage(pctToMv(sub.intake_pct));
    roller_top_->move_voltage(pctToMv(sub.roller_top_pct));
    roller_bottom_->move_voltage(pctToMv(sub.roller_bottom_pct));
    pneumatics_g_->set_value(sub.pneumatics_g_state);
    pneumatics_b_->set_value(sub.pneumatics_b_state);
}
