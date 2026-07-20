/* ---------------------------------------------------------------------------
 * ParenthesisDrive.cpp  --  six-motor X + forward side-wheel kinematics on the
 * PROS V5 API. Extends XDrive: reuses its peripherals, IMU handling, joystick/
 * buttons, subsystems, battery/clamp helpers, and corner motors; adds the two
 * side traction wheels and overrides the 6-wheel mix and the odometry source.
 * ------------------------------------------------------------------------- */
#include "hardware/ParenthesisDrive.h"
#include "config/Ports.h"
#include "config/Settings.h"
#include "core/Specs.h"
#include "math/MathUtils.h"
#include <cmath>

static constexpr double SQRT2 = 1.4142135623730951;

static inline std::int32_t voltsToMv(double volts) {
    double mv = mathutil::clamp_val(volts * 1000.0, -12000.0, 12000.0);
    return static_cast<std::int32_t>(mv);
}

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

ChassisObservation ParenthesisDrive::readObservation(double prev_heading_rad) {
    ChassisObservation o;

    // Forward distance/speed from the forward-facing side wheels (no 45 deg factor).
    double csl = (side_l_->get_position() / 360.0) * Robot::METERS_PER_TURN;
    double csr = (side_r_->get_position() / 360.0) * Robot::METERS_PER_TURN;
    double dsl = csl - prev_sl_, dsr = csr - prev_sr_;
    prev_sl_ = csl; prev_sr_ = csr;
    o.forward_delta = (dsl + dsr) / 2.0;
    o.v_lin = (side_l_->get_actual_velocity() + side_r_->get_actual_velocity()) / 2.0 * Robot::RPM_TO_MPS;

    // Strafe from the corner omnis (advance corner baselines to stay consistent).
    double cfl = (fl_->get_position() / 360.0) * Robot::METERS_PER_TURN;
    double cfr = (fr_->get_position() / 360.0) * Robot::METERS_PER_TURN;
    double cbl = (bl_->get_position() / 360.0) * Robot::METERS_PER_TURN;
    double cbr = (br_->get_position() / 360.0) * Robot::METERS_PER_TURN;
    double dfl = cfl - prev_fl_, dfr = cfr - prev_fr_, dbl = cbl - prev_bl_, dbr = cbr - prev_br_;
    prev_fl_ = cfl; prev_fr_ = cfr; prev_bl_ = cbl; prev_br_ = cbr;
    o.strafe_delta = SQRT2 * (dfr + dbl - dfl - dbr) / 4.0;

    fillImuChannels(prev_heading_rad, o);

    // Log channels: the side wheels are the natural left/right references.
    o.left_pos_deg  = side_l_->get_position();
    o.right_pos_deg = side_r_->get_position();
    o.left_vel_rpm  = side_l_->get_actual_velocity();
    o.right_vel_rpm = side_r_->get_actual_velocity();
    o.left_current_amps  = side_l_->get_current_draw() / 1000.0;
    o.right_current_amps = side_r_->get_current_draw() / 1000.0;
    return o;
}

ChassisCommand ParenthesisDrive::applyDrive(const ChassisCommand& cmd, const ChassisObservation& /*obs*/) {
    // Inverse kinematics: four holonomic corners + a forward-only side pair.
    double corner[4] = {
        cmd.linear - cmd.lateral - cmd.angular,   // FL
        cmd.linear + cmd.lateral + cmd.angular,   // FR
        cmd.linear + cmd.lateral - cmd.angular,   // BL
        cmd.linear - cmd.lateral + cmd.angular    // BR
    };
    double side[2] = {
        cmd.linear - cmd.angular,                 // SideL (no lateral term)
        cmd.linear + cmd.angular                  // SideR
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
    applied.linear  = cmd.linear  * scale;
    applied.lateral = cmd.lateral * scale;
    applied.angular = cmd.angular * scale;
    return applied;
}
