/* ---------------------------------------------------------------------------
 * ParenthesisDrive.cpp  --  six-motor X + forward side-wheel kinematics on the
 * PROS V5 API. Extends XDrive: reuses its peripherals, IMU handling, joystick/
 * buttons, subsystems, battery/clamp helpers, and corner motors; adds the two
 * side traction wheels and overrides the 6-wheel mix and the odometry source.
 * ------------------------------------------------------------------------- */
#include "hardware/ParenthesisDrive.h"
#include "hardware/ChassisVoltage.h"
#include "hardware/DeviceRead.h"
#include "config/Ports.h"
#include "config/Settings.h"
#include "core/Specs.h"
#include <cmath>

static constexpr double SQRT2 = 1.4142135623730951;

static inline std::int32_t voltsToMv(double volts) { return devread::voltsToMv(volts); }

ParenthesisDrive::ParenthesisDrive(double max_volts, bool scale_battery)
    : XDrive(max_volts, scale_battery) {
    side_l_ = new pros::Motor((std::int8_t)-Ports::SIDE_L, pros::MotorGears::blue);
    side_r_ = new pros::Motor((std::int8_t) Ports::SIDE_R, pros::MotorGears::blue);
}

ParenthesisDrive::~ParenthesisDrive() {
    delete side_l_; delete side_r_;
}

void ParenthesisDrive::initialize() {
    XDrive::initialize();
    side_l_->set_brake_mode(pros::MotorBrake::coast);
    side_r_->set_brake_mode(pros::MotorBrake::coast);
}

void ParenthesisDrive::stop() {
    XDrive::stop();
    side_l_->move_voltage(0);
    side_r_->move_voltage(0);
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

ChassisObservation ParenthesisDrive::readObservation(double prev_heading_rad) {
    ChassisObservation o;
    bool gap = false;
    o.sample_dt = takeSampleDt(gap);
    if (gap) { have_sl_ = false; have_sr_ = false; }

    const double dsl = wheelDelta(*side_l_, prev_sl_, have_sl_);
    const double dsr = wheelDelta(*side_r_, prev_sr_, have_sr_);
    o.forward_delta = (dsl + dsr) / 2.0;
    const double rpm_l = finiteRpm(*side_l_);
    const double rpm_r = finiteRpm(*side_r_);
    o.v_lin = (rpm_l + rpm_r) / 2.0 * Robot::RPM_TO_MPS;

    const double dfl = wheelDelta(*fl_, prev_fl_, have_fl_);
    const double dfr = wheelDelta(*fr_, prev_fr_, have_fr_);
    const double dbl = wheelDelta(*bl_, prev_bl_, have_bl_);
    const double dbr = wheelDelta(*br_, prev_br_, have_br_);
    o.strafe_delta = SQRT2 * (dfr + dbl - dfl - dbr) / 4.0;

    fillImuChannels(prev_heading_rad, o);

    o.left_pos_deg  = have_sl_ ? prev_sl_ / Robot::METERS_PER_TURN * 360.0 : 0.0;
    o.right_pos_deg = have_sr_ ? prev_sr_ / Robot::METERS_PER_TURN * 360.0 : 0.0;
    o.left_vel_rpm  = rpm_l;
    o.right_vel_rpm = rpm_r;
    o.left_current_amps  = devread::motorAmps(*side_l_);
    o.right_current_amps = devread::motorAmps(*side_r_);
    o.left_temp_c  = devread::motorTempC(*side_l_);
    o.right_temp_c = devread::motorTempC(*side_r_);
    return o;
}

ChassisCommand ParenthesisDrive::applyDrive(const ChassisCommand& cmd, const ChassisObservation& obs) {
    ChassisVoltageState held;
    held.v = volt_surge_;
    held.w = volt_yaw_;
    held.primed = volt_primed_;
    const ChassisCommand volts = chassisVoltage(cmd, obs, nullptr, held);
    volt_surge_ = held.v;
    volt_yaw_ = held.w;
    volt_primed_ = held.primed;

    // Inverse kinematics: four holonomic corners + a forward-only side pair.
    double corner[4] = {
        volts.linear - volts.lateral - volts.angular,   // FL
        volts.linear + volts.lateral + volts.angular,   // FR
        volts.linear + volts.lateral - volts.angular,   // BL
        volts.linear - volts.lateral + volts.angular    // BR
    };
    double side[2] = {
        volts.linear - volts.angular,                 // SideL (no lateral term)
        volts.linear + volts.angular                  // SideR
    };

    double all[6] = { corner[0], corner[1], corner[2], corner[3], side[0], side[1] };
    double scale = 1.0;
    if (Settings::ControlToggles::VOLTAGE_CLAMP) scale = clampScale(all, 6, System::MAX_VOLTS);
    double bscale = batteryScale();

    fl_->move_voltage(voltsToMv(corner[0] * scale * bscale));
    fr_->move_voltage(voltsToMv(corner[1] * scale * bscale));
    bl_->move_voltage(voltsToMv(corner[2] * scale * bscale));
    br_->move_voltage(voltsToMv(corner[3] * scale * bscale));
    side_l_->move_voltage(voltsToMv(side[0] * scale * bscale));
    side_r_->move_voltage(voltsToMv(side[1] * scale * bscale));

    ChassisCommand applied;
    applied.linear  = volts.linear  * scale;
    applied.lateral = volts.lateral * scale;
    applied.angular = volts.angular * scale;
    applied.request = DriveRequest::Volts;
    return applied;
}
