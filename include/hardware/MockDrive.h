#pragma once
/* ---------------------------------------------------------------------------
 * MockDrive  --  simulated differential chassis. Concrete IDriveBase.
 *
 * Same contract and inverse/forward kinematics as TankDrive, so the estimator,
 * controller, and pipeline are reused unchanged. main.cpp constructs this when
 * ROBOT_HOST is defined. There is no PROS motor, IMU, or controller here:
 * applyDrive stores the post-safety wheel voltages, and service() integrates
 * them into wheel speed and encoder position. Reported encoders, IMU, and
 * distance beams follow the V5 hardware specs in Robot::Encoder, Robot::Imu,
 * and Robot::Range. The field window draws the plant and the localizer's pose.
 *
 * All wheels are omnis, so the rollers do not hold the robot sideways.
 * While it yaws, forward momentum swings into a lateral skid. Encoders
 * still report only wheel rolling. The reported accelerometer is the
 * plant's chassis-frame specific force, which does include that skid.
 *
 * Inverse kinematics match TankDrive (positive angular is CCW):
 *   left = linear - angular,  right = linear + angular
 * ------------------------------------------------------------------------- */

#include "interfaces/IDriveBase.h"
#include "interfaces/IFeedForward.h"
#include "hardware/MockDriveParams.h"
#include <cstdint>

class MockDrive : public IDriveBase {
public:
    explicit MockDrive(const IFeedForward& feedforward,
                       double max_volts = 12.0, bool scale_battery = false);

    void initialize() override;
    void beginSensorCalibration() override;
    void waitSensorCalibration() override;
    void calibrateSensors() override;

    JoystickData readJoystick() override;
    ButtonState  readButtons() override;

    ChassisObservation readObservation(double prev_heading_rad) override;
    void readDistances(DistanceReading (&out)[kDistanceChannels]) override;

    ChassisCommand applyDrive(const ChassisCommand& cmd, const ChassisObservation& obs) override;
    void applySubsystems(const SubsystemCommand& sub) override;
    void stop() override;

    void alignHeadingTo(double target_rad) override;

    void service(double dt, const Pose2d& display_pose,
                 const Pose2d* particles = nullptr, int particle_count = 0) override;

    // Inject the driver inputs readJoystick / readButtons will return.
    void setJoystick(const JoystickData& joy);
    void setButtons(const ButtonState& buttons);
    void setBatteryVoltage(double volts);

    // True plant, the pose a perfect localizer would publish. x and y are
    // inches. theta is radians. v_lin and v_lat are m/s in the body frame
    // (forward, left). v_ang is rad/s, CCW. Training reads this instead of
    // the noisy sensor path.
    struct PlantSnapshot {
        double x_in = 0;
        double y_in = 0;
        double theta = 0;
        double v_lin = 0;
        double v_lat = 0;
        double v_ang = 0;
        double left_amps = 0;
        double right_amps = 0;
    };

    PlantSnapshot plantSnapshot() const;

    // Place the plant. Wheel speeds match v_lin and v_ang. Does not draw a
    // new sensor-error sample, so a training reset stays deterministic.
    void setPlant(double x_in, double y_in, double theta,
                  double v_lin, double v_lat, double v_ang);

    // Advance the plant. dt_seconds is the control period (System::DT_SEC).
    // service() calls this and then redraws the field.
    void stepSimulation(double dt_seconds);

    // Runtime plant. Defaults are MockPlant. The tuner writes these; the
    // firmware build does not call this.
    void setDynamics(double velocity_tau_sec, double skid_tau_sec,
                     double side_resistance_ohm, double speed_scale);

    void resetSensors();
    void printTelemetry() const;

private:
    const IFeedForward* feedforward_;
    double max_voltage_;
    bool   use_battery_scaling_;

    // Wheel plant (meters, m/s) and the voltage the motors are currently receiving.
    double left_m_ = 0.0, right_m_ = 0.0;
    double left_mps_ = 0.0, right_mps_ = 0.0;
    double lat_mps_ = 0.0;   // body frame, positive to the left
    double plant_left_v_ = 0.0, plant_right_v_ = 0.0;
    double left_amps_ = 0.0, right_amps_ = 0.0;
    double velocity_tau_ = MockPlant::VELOCITY_TAU_SEC;
    double skid_tau_ = MockPlant::SKID_TAU_SEC;
    double side_resistance_ = MockPlant::SIDE_RESISTANCE_OHM;
    double speed_scale_ = MockPlant::SPEED_SCALE;

    // Forward-kinematics memory (meters) and simulated IMU.
    double prev_l_ = 0.0, prev_r_ = 0.0;
    double prev_v_fwd_ = 0.0, prev_v_lat_ = 0.0;
    double heading_rad_ = 0.0;
    double imu_offset_ = 0.0;
    // Plant pose the simulated distance sensors raycast from.
    double true_x_ = 0.0, true_y_ = 0.0;
    double battery_voltage_ = 12.0;

    // Sensor-error state. The plant above stays clean; only the readings move.
    // Gyro scale is a property of the unit. Calibration redraws the residual
    // bias and restarts the integrated heading.
    double imu_scale_ = 1.0;
    double imu_accel_scale_ = 1.0;
    double imu_bias_dps_ = 0.0;
    double imu_rotation_deg_ = 0.0;   // get_rotation(), clockwise positive
    double imu_plant_heading_ = 0.0;
    bool imu_scaled_ = false;

    // Last distance sample. A new one is taken every kRangePeriodTicks.
    int dist_mm_[kDistanceChannels]{};
    int dist_conf_[kDistanceChannels]{};
    bool dist_valid_[kDistanceChannels]{};
    int dist_tick_ = 0;

    std::uint64_t rng_ = 0xC0FFEE1234ull;

    JoystickData joystick_{};
    ButtonState buttons_{};
    SubsystemCommand subsystems_{};

    void applyCurrentLimit(double& left_v, double& right_v, double v_lin, double v_ang) const;
    void ensureSensorScales();
    void resetImuIntegration();
    double reportedRotationDeg() const;
    int distanceConfidence(int mm);

    // Sample the desktop keyboard into joystick_ / buttons_.
    void pollKeyboard();

    double uniform_();
    double normal_();
};
