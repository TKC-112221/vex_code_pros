/* ---------------------------------------------------------------------------
 * XDrive.cpp  --  four-omni holonomic forward/inverse kinematics on the PROS
 * V5 API. A corner omni at 45 deg has driving-direction components +/- 1/sqrt(2),
 * so the chassis<->wheel maps carry a sqrt(2) factor:
 *   chassis forward = sqrt(2) * (average wheel travel).
 * ------------------------------------------------------------------------- */
#include "hardware/XDrive.h"
#include "hardware/DeviceRead.h"
#include "hardware/DifferentialMix.h"
#include "config/Ports.h"
#include "config/Settings.h"
#include "core/Specs.h"
#include "core/Tuning.h"
#include <algorithm>
#include <cmath>

static constexpr double SQRT2 = 1.4142135623730951;

static inline std::int32_t voltsToMv(double volts) { return devread::voltsToMv(volts); }
static inline std::int32_t pctToMv(int pct) { return devread::pctToMv(pct); }

XDrive::XDrive(double max_volts, bool scale_battery)
    : max_voltage_(max_volts), use_battery_scaling_(scale_battery) {
    pilot_ = new pros::Controller(pros::E_CONTROLLER_MASTER);
    imu_   = new pros::Imu(Ports::IMU);

    // Reversal via negated port: FL/BL reversed, FR/BR forward.
    fl_ = new pros::Motor((std::int8_t)-Ports::FL, pros::MotorGears::blue);
    fr_ = new pros::Motor((std::int8_t) Ports::FR, pros::MotorGears::blue);
    bl_ = new pros::Motor((std::int8_t)-Ports::BL, pros::MotorGears::blue);
    br_ = new pros::Motor((std::int8_t) Ports::BR, pros::MotorGears::blue);

    main1_ = new pros::Motor(Ports::MAIN1, pros::MotorGears::blue);
    main2_ = new pros::Motor(Ports::MAIN2, pros::MotorGears::blue);
    sub1_  = new pros::Motor(Ports::SUB1, pros::MotorGears::green);
    sub2_  = new pros::Motor(Ports::SUB2, pros::MotorGears::green);
    pneumatics_g_  = new pros::adi::DigitalOut(Ports::PNEUMATICS_G);
    pneumatics_b_  = new pros::adi::DigitalOut(Ports::PNEUMATICS_B);
}

XDrive::~XDrive() {
    delete fl_; delete fr_; delete bl_; delete br_;
    delete imu_; delete pilot_;
    delete main1_; delete main2_; delete sub1_; delete sub2_;
    delete pneumatics_g_; delete pneumatics_b_;
}

void XDrive::initialize() {
    fl_->set_brake_mode(pros::MotorBrake::coast); fr_->set_brake_mode(pros::MotorBrake::coast);
    bl_->set_brake_mode(pros::MotorBrake::coast); br_->set_brake_mode(pros::MotorBrake::coast);
    main1_->set_brake_mode(pros::MotorBrake::hold);
    main2_->set_brake_mode(pros::MotorBrake::hold);
    sub1_->set_brake_mode(pros::MotorBrake::coast);
    sub2_->set_brake_mode(pros::MotorBrake::coast);
}

void XDrive::beginSensorCalibration() {
    imu_offset_ = 0.0;
}

void XDrive::waitSensorCalibration() {
    if (Settings::ControlToggles::IMU_FALLBACK) {
        imu_wheels_ = true;
        return;
    }
    imu_wheels_ = !devread::imuCalibrated(*imu_);
    if (imu_wheels_) have_heading_ = false;
}

double XDrive::takeSampleDt(bool& gap) {
    const double dt = devread::sampleDt(last_sense_ms_, have_sense_ms_, gap);
    if (gap) {
        have_fl_ = false;
        have_fr_ = false;
        have_bl_ = false;
        have_br_ = false;
    }
    return dt;
}

void XDrive::calibrateSensors() {
    beginSensorCalibration();
    waitSensorCalibration();
}

void XDrive::stop() {
    fl_->move_voltage(0); fr_->move_voltage(0);
    bl_->move_voltage(0); br_->move_voltage(0);
}

JoystickData XDrive::readJoystick() {
    JoystickData joy;
    joy.left_y  = devread::stick(*pilot_, pros::E_CONTROLLER_ANALOG_LEFT_Y);
    joy.left_x  = devread::stick(*pilot_, pros::E_CONTROLLER_ANALOG_LEFT_X);
    joy.right_y = devread::stick(*pilot_, pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    joy.right_x = devread::stick(*pilot_, pros::E_CONTROLLER_ANALOG_RIGHT_X);
    return joy;
}

ButtonState XDrive::readButtons() {
    ButtonState b;
    b.l1 = devread::button(*pilot_, pros::E_CONTROLLER_DIGITAL_L1);
    b.l2 = devread::button(*pilot_, pros::E_CONTROLLER_DIGITAL_L2);
    b.r1 = devread::button(*pilot_, pros::E_CONTROLLER_DIGITAL_R1);
    b.r2 = devread::button(*pilot_, pros::E_CONTROLLER_DIGITAL_R2);
    b.up = devread::button(*pilot_, pros::E_CONTROLLER_DIGITAL_UP);
    b.down = devread::button(*pilot_, pros::E_CONTROLLER_DIGITAL_DOWN);
    b.left = devread::button(*pilot_, pros::E_CONTROLLER_DIGITAL_LEFT);
    b.right = devread::button(*pilot_, pros::E_CONTROLLER_DIGITAL_RIGHT);
    b.a = devread::button(*pilot_, pros::E_CONTROLLER_DIGITAL_A);
    b.b = devread::button(*pilot_, pros::E_CONTROLLER_DIGITAL_B);
    b.x = devread::button(*pilot_, pros::E_CONTROLLER_DIGITAL_X);
    b.y = devread::button(*pilot_, pros::E_CONTROLLER_DIGITAL_Y);
    return b;
}

void XDrive::alignHeadingTo(double target_rad) {
    if (Settings::ControlToggles::IMU_FALLBACK || imu_wheels_) { imu_offset_ = 0.0; return; }
    const double rot = imu_->get_rotation();
    if (!devread::finite(rot)) return;
    const double raw = -rot * (M_PI / 180.0);
    imu_offset_ = target_rad - raw;
}

double XDrive::batteryScale() const {
    if (!use_battery_scaling_) return 1.0;
    double batt = devread::batteryVolts();
    return (batt > Controls::BATTERY_VALID_V) ? (System::MAX_VOLTS / batt) : 1.0;
}

double XDrive::clampScale(const double* v, int n, double max_v) {
    double max_abs = 0.0;
    for (int i = 0; i < n; ++i) max_abs = std::max(max_abs, std::fabs(v[i]));
    return (max_abs > max_v) ? (max_v / max_abs) : 1.0;
}

// IMU-derived heading/omega + battery/accel log channels (shared w/ parenthesis).
void XDrive::fillImuChannels(double prev_heading_rad, ChassisObservation& o) {
    if (Settings::ControlToggles::IMU_FALLBACK || imu_wheels_) {
        // Wheel-derived heading isn't well-posed for a holonomic base; hold last.
        o.heading_rad = prev_heading_rad;
        o.v_ang = 0.0;
        o.imu_rate_z_dps = 0.0;
        o.imu_accel_x = 0.0; o.imu_accel_y = 0.0;
        held_heading_ = o.heading_rad;
        have_heading_ = true;
    } else {
        const double rot = imu_->get_rotation();
        if (devread::finite(rot)) {
            o.heading_rad = -rot * (M_PI / 180.0) + imu_offset_;
            held_heading_ = o.heading_rad;
            have_heading_ = true;
        } else {
            o.heading_rad = have_heading_ ? held_heading_ : prev_heading_rad;
        }
        const pros::imu_gyro_s_t rate = imu_->get_gyro_rate();
        if (devread::finite(rate.z)) {
            o.v_ang = -rate.z * (M_PI / 180.0);
            o.imu_rate_z_dps = rate.z;
        }
        const pros::imu_accel_s_t accel = imu_->get_accel();
        if (devread::finite(accel.x)) o.imu_accel_x = accel.x;
        if (devread::finite(accel.y)) o.imu_accel_y = accel.y;
    }
    o.battery_voltage = devread::batteryVolts();
}

static double wheelDelta(pros::Motor& motor, double& prev_m, bool& have) {
    double deg = 0.0;
    if (!devread::motorDegrees(motor, deg)) return 0.0;
    return devread::travelMeters(deg, prev_m, have);
}

static double finiteRpm(pros::Motor& motor) {
    const double rpm = motor.get_actual_velocity();
    return devread::finite(rpm) ? rpm : 0.0;
}

ChassisObservation XDrive::readObservation(double prev_heading_rad) {
    ChassisObservation o;
    bool gap = false;
    o.sample_dt = takeSampleDt(gap);
    (void)gap;

    const double dfl = wheelDelta(*fl_, prev_fl_, have_fl_);
    const double dfr = wheelDelta(*fr_, prev_fr_, have_fr_);
    const double dbl = wheelDelta(*bl_, prev_bl_, have_bl_);
    const double dbr = wheelDelta(*br_, prev_br_, have_br_);

    // Forward kinematics (sqrt(2) accounts for the 45 deg roller geometry).
    o.forward_delta = SQRT2 * (dfl + dfr + dbl + dbr) / 4.0;
    o.strafe_delta  = SQRT2 * (dfr + dbl - dfl - dbr) / 4.0;

    const double rfl = finiteRpm(*fl_), rfr = finiteRpm(*fr_);
    const double rbl = finiteRpm(*bl_), rbr = finiteRpm(*br_);
    o.v_lin = SQRT2 * (rfl + rfr + rbl + rbr) / 4.0 * Robot::RPM_TO_MPS;

    fillImuChannels(prev_heading_rad, o);

    o.left_pos_deg  = (have_fl_ && have_bl_)
        ? (prev_fl_ + prev_bl_) * 0.5 / Robot::METERS_PER_TURN * 360.0 : 0.0;
    o.right_pos_deg = (have_fr_ && have_br_)
        ? (prev_fr_ + prev_br_) * 0.5 / Robot::METERS_PER_TURN * 360.0 : 0.0;
    o.left_vel_rpm  = (rfl + rbl) / 2.0;
    o.right_vel_rpm = (rfr + rbr) / 2.0;
    o.left_current_amps  = (devread::motorAmps(*fl_) + devread::motorAmps(*bl_)) / 2.0;
    o.right_current_amps = (devread::motorAmps(*fr_) + devread::motorAmps(*br_)) / 2.0;
    o.left_temp_c  = (devread::motorTempC(*fl_) + devread::motorTempC(*bl_)) / 2.0;
    o.right_temp_c = (devread::motorTempC(*fr_) + devread::motorTempC(*br_)) / 2.0;
    return o;
}

void XDrive::readDistances(DistanceReading (&out)[kDistanceChannels]) {
    ranges_.read(out);
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

CascadeReading XDrive::readCascade() const {
    return readMainPair(*main1_, *main2_);
}

void XDrive::applySubsystems(const SubsystemCommand& sub) {
    const MainPairPct pair = mixMainPair(sub.cascade_pct, sub.intake_pct);
    main1_->move_voltage(pctToMv(pair.main1_pct));
    main2_->move_voltage(pctToMv(pair.main2_pct));
    sub1_->move_voltage(0);
    sub2_->move_voltage(0);
    pneumatics_g_->set_value(sub.pneumatics_g_state);
    pneumatics_b_->set_value(sub.pneumatics_b_state);
}
