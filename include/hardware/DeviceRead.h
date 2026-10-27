#pragma once
/* ---------------------------------------------------------------------------
 * DeviceRead  --  V5 sensor reads that stay finite when a call fails.
 *
 * PROS returns PROS_ERR (INT32_MAX) or PROS_ERR_F (infinity) for a missed
 * device transaction. The host stand-in never does. Callers treat a failed
 * sample as "no new information" instead of integrating it.
 * ------------------------------------------------------------------------- */

#include "api.h"
#include "core/Specs.h"
#include "core/Tuning.h"
#include <cmath>
#include <cstdint>

namespace devread {

inline bool finite(double v) { return std::isfinite(v); }

// Analog sticks are -127..127. PROS_ERR is INT32_MAX, which would clamp to
// a full-scale voltage if it were divided and passed through the drive.
inline double stick(pros::Controller& pilot, pros::controller_analog_e_t channel) {
    const std::int32_t raw = pilot.get_analog(channel);
    if (raw < -127 || raw > 127) return 0.0;
    return static_cast<double>(raw) / Controller::FULL_SCALE;
}

// get_digital returns 1, 0, or PROS_ERR. Storing PROS_ERR in a bool is true.
inline bool button(pros::Controller& pilot, pros::controller_digital_e_t id) {
    return pilot.get_digital(id) == 1;
}

inline std::int32_t voltsToMv(double volts) {
    if (!finite(volts)) return 0;
    double mv = volts * 1000.0;
    if (mv > 12000.0) mv = 12000.0;
    if (mv < -12000.0) mv = -12000.0;
    return static_cast<std::int32_t>(mv);
}

inline std::int32_t pctToMv(int pct) {
    if (pct > 100) pct = 100;
    if (pct < -100) pct = -100;
    return static_cast<std::int32_t>(pct * 120);
}

// Time since the previous sample. A gap above 200 ms is not a velocity
// sample: gap is set and the caller rebases its encoder memory.
inline double sampleDt(std::uint32_t& last_ms, bool& have_last, bool& gap) {
    gap = false;
    const std::uint32_t now = pros::millis();
    double dt = System::DT_SEC;
    if (have_last) {
        const std::uint32_t elapsed = now - last_ms;
        if (elapsed == 0) dt = System::DT_SEC;
        else if (elapsed > 200) gap = true;
        else dt = static_cast<double>(elapsed) * 0.001;
    }
    last_ms = now;
    have_last = true;
    return dt;
}

// Average of every motor in the bank, in degrees. One failed read rejects
// the bank so a single PROS_ERR_F cannot move the mean.
inline bool groupDegrees(pros::MotorGroup& group, int count, double& degrees) {
    if (count < 1) return false;
    double sum = 0.0;
    for (int i = 0; i < count; ++i) {
        const double deg = group.get_position(static_cast<std::uint8_t>(i));
        if (!finite(deg)) return false;
        sum += deg;
    }
    degrees = sum / static_cast<double>(count);
    return true;
}

inline bool motorDegrees(pros::Motor& motor, double& degrees) {
    const double deg = motor.get_position();
    if (!finite(deg)) return false;
    degrees = deg;
    return true;
}

// Meters of wheel travel since the last good sample. The first good sample
// only arms the baseline.
inline double travelMeters(double degrees, double& prev_m, bool& have) {
    const double meters = (degrees / 360.0) * Robot::METERS_PER_TURN;
    if (!have) {
        prev_m = meters;
        have = true;
        return 0.0;
    }
    const double delta = meters - prev_m;
    prev_m = meters;
    return delta;
}

inline double groupAmps(pros::MotorGroup& group, int count) {
    if (count < 1) return 0.0;
    double sum = 0.0;
    int n = 0;
    for (int i = 0; i < count; ++i) {
        const std::int32_t ma = group.get_current_draw(static_cast<std::uint8_t>(i));
        if (ma == PROS_ERR || ma < 0) continue;
        sum += static_cast<double>(ma);
        ++n;
    }
    if (n == 0) return 0.0;
    return (sum / static_cast<double>(n)) / 1000.0;
}

inline double motorAmps(pros::Motor& motor) {
    const std::int32_t ma = motor.get_current_draw();
    if (ma == PROS_ERR || ma < 0) return 0.0;
    return static_cast<double>(ma) / 1000.0;
}

// Indexed reads. get_temperature_all() allocates a vector on every call.
inline double groupTempC(pros::MotorGroup& group, int count) {
    if (count < 1) return 0.0;
    double sum = 0.0;
    int n = 0;
    for (int i = 0; i < count; ++i) {
        const double t = group.get_temperature(static_cast<std::uint8_t>(i));
        if (!finite(t) || t < -20.0 || t > 120.0) continue;
        sum += t;
        ++n;
    }
    if (n == 0) return 0.0;
    return sum / static_cast<double>(n);
}

inline double motorTempC(pros::Motor& motor) {
    const double t = motor.get_temperature();
    if (!finite(t) || t < -20.0 || t > 120.0) return 0.0;
    return t;
}

inline double batteryVolts() {
    const std::int32_t mv = pros::battery::get_voltage();
    if (mv == PROS_ERR || mv < 0) return 0.0;
    return static_cast<double>(mv) / 1000.0;
}

// Blocking reset. The kernel gives up after 3 s. One retry covers a sensor
// that was already busy. False means the caller should stop trusting it.
inline bool imuCalibrated(pros::Imu& imu) {
    if (imu.reset(true) == 1) return true;
    return imu.reset(true) == 1;
}

}  // namespace devread
