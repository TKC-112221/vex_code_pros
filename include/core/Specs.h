#pragma once
/* ---------------------------------------------------------------------------
 * Specs.h  --  The Physics: wheel geometry, gearing, track width, loop timing.
 * Only changes when the robot is physically rebuilt. Ported verbatim from v1
 * (constants/specs.h) but exposed exclusively through scoped names.
 * ------------------------------------------------------------------------- */

namespace Robot {

    constexpr double PI = 3.14159265358979323846;
    constexpr double G_MPS2 = 9.80665;

    namespace Chassis {
        constexpr double WHEEL_DIAMETER_IN = 3.25;
        constexpr double WHEEL_DIAMETER_M  = WHEEL_DIAMETER_IN * 0.0254;
        constexpr double GEAR_RATIO        = 48.0 / 36.0;
        constexpr double TRACK_WIDTH_M     = 0.27051;
    }

    namespace Drive {
        constexpr double MOTOR_RPM_CART     = 600.0;
        constexpr double VELOCITY_EFFICIENCY = 0.90;
    }

    // XY of Field::Point::Align. Heading is not stored; the boomerang caller
    // passes it when the drive starts.
    namespace Align {
        constexpr double POINT_X_IN = -11.5;
        constexpr double POINT_Y_IN = -2.0;
    }
    constexpr double ODOM_IMU_WEIGHT = 1.0;

    // Four outward-facing distance sensors. Chassis frame: x forward, y left,
    // yaw CCW from forward. D1 front, D2 right, D3 rear, D4 left.
    // Edit MOUNT when the sensors move on the robot. Field walls are the
    // perimeter of a 12 ft field centered on the origin.
    //
    // VEX publishes the accuracy as about ±15 mm below 200 mm and ±5% above
    // that, over 20–2000 mm (VEXcode distance-sensor API and the V5 knowledge
    // base). PROS returns confidence 10 at or below 200 mm; that value is a
    // sentinel, and real confidence (0–63) exists only past 200 mm. James
    // Pearman measured the beam at about 30 Hz (33 ms is also the VL53L0X
    // default timing budget).
    namespace Range {
        constexpr int COUNT = 4;
        constexpr double FIELD_HALF_M = 6.0 * 0.3048;
        constexpr double MIN_M = 0.020;
        constexpr double MAX_M = 2.000;
        constexpr double NEAR_M = 0.200;
        constexpr double ACCURACY_NEAR_M = 0.015;
        constexpr double ACCURACY_FRAC = 0.0288;
        constexpr int CONF_NEAR = 10;
        constexpr double UPDATE_SEC = 0.033;
        struct Mount { double x; double y; double yaw; };
        constexpr Mount MOUNT[COUNT] = {
            { 0.16,  0.00,  0.0 },       // D1 front
            { 0.00, -0.14, -PI / 2.0 },  // D2 right
            {-0.16,  0.00,  PI },        // D3 rear
            { 0.00,  0.14,  PI / 2.0 },  // D4 left
        };
    }

    // Published accuracy, used as the 1-sigma range error. Past 200 mm a
    // confidence below 63 widens it, up to 4x. The sentinel 10 is not applied.
    inline double rangeAccuracyM(double range_m) {
        if (range_m <= Range::NEAR_M) return Range::ACCURACY_NEAR_M;
        return Range::ACCURACY_FRAC * range_m;
    }
    inline double rangeSigmaM(double range_m, int confidence) {
        double sigma = rangeAccuracyM(range_m);
        if (range_m > Range::NEAR_M && confidence > 0 && confidence < 63) {
            double scale = 63.0 / static_cast<double>(confidence);
            if (scale > 4.0) scale = 4.0;
            sigma *= scale;
        }
        return sigma;
    }

    // Blue (600 rpm, 6:1) V5 cartridge, which is what TankDrive constructs.
    // VEX lists 300 ticks per output revolution for that cartridge (1800 for
    // 36:1, 900 for 18:1). get_position() in degrees is that tick converted.
    // The knowledge-base line "0.02 degree accuracy" is finer than one tick,
    // so the model follows the tick table.
    namespace Encoder {
        constexpr double TICKS_PER_REV = 300.0;
        // No published velocity-noise figure. One tick in a 10 ms sample is
        // 20 RPM, and VEXos filters that difference, so the report jitters by
        // a few RPM. 5 RPM is a quarter of the single-sample step.
        constexpr double VEL_SIGMA_RPM = 5.0;
    }

    // V5 Inertial Sensor, ICM-20602, ranges set to ±4 g and ±1000 deg/s
    // (VEX knowledge base). Calibration removes the resting gyro bias. It does
    // not remove the ±1% sensitivity tolerance; that is the scale error teams
    // measure as about a degree or two per revolution. The sigmas below are
    // half of that tolerance, so a typical unit lands inside the spec. Rate
    // and accel noise are the datasheet RMS at a 100 Hz bandwidth. Heading is
    // reported to 0.01 degree (the 0–359.99 heading value).
    namespace Imu {
        constexpr double GYRO_FS_DPS = 1000.0;
        constexpr double ACCEL_FS_G = 4.0;
        constexpr double SCALE_SIGMA = 0.005;
        constexpr double ACCEL_SCALE_SIGMA = 0.005;
        constexpr double BIAS_SIGMA_DPS = 0.02;
        constexpr double RATE_SIGMA_DPS = 0.04;
        constexpr double ACCEL_SIGMA_G = 0.001;
        constexpr double HEADING_LSB_DEG = 0.01;
    }

    // Derived values used throughout the codebase.
    constexpr double METERS_PER_TURN     = Chassis::WHEEL_DIAMETER_M * PI / Chassis::GEAR_RATIO;
    constexpr double RPM_TO_MPS          = METERS_PER_TURN / 60.0;
    constexpr double THEORETICAL_MAX_VEL = (Drive::MOTOR_RPM_CART / 60.0) * PI
                                           * Chassis::WHEEL_DIAMETER_M * Chassis::GEAR_RATIO;
    constexpr double MAX_VEL_LIN         = THEORETICAL_MAX_VEL * Drive::VELOCITY_EFFICIENCY;
    constexpr double MAX_VEL_ANG         = MAX_VEL_LIN / (Chassis::TRACK_WIDTH_M / 2.0);
}

namespace System {
    constexpr double LOOP_TIME_MS   = 10.0;
    constexpr int    MAX_RECORD_FRAMES = 32000;
    constexpr double MAX_VOLTS      = 12.0;
    constexpr double MAX_VOLTAGE_DROP = 12.0;
    constexpr double DT_SEC         = LOOP_TIME_MS / 1000.0;

    // Brain ticks per distance-sensor sample. The sensor's timing budget is
    // Robot::Range::UPDATE_SEC (33 ms); the loop is LOOP_TIME_MS. Fusing on
    // the ticks in between repeats one reading and spends the ray casts
    // inside the 10 ms clock for no new measurement. ceil(33 / 10) = 4.
    constexpr int RANGE_STRIDE = (
        static_cast<int>(Robot::Range::UPDATE_SEC * 1000.0 + 0.5) +
        static_cast<int>(LOOP_TIME_MS) - 1
    ) / static_cast<int>(LOOP_TIME_MS);
}
