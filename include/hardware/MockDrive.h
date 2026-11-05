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
#include "core/Specs.h"
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
    CascadeReading readCascade() const override;
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

    // Place the plant. Wheel speeds match v_lin and v_ang. The reported
    // heading matches theta, with the IMU offset left at zero. Does not draw
    // a new sensor-error sample, so a training reset stays deterministic.
    void setPlant(double x_in, double y_in, double theta,
                  double v_lin, double v_lat, double v_ang);

    // Advance the plant. dt_seconds is the control period (System::DT_SEC).
    // service() calls this and then redraws the field.
    void stepSimulation(double dt_seconds);

    // Runtime plant. Defaults are MockPlant. The tuner and the field
    // window write these; the firmware build does not call this.
    // max_vel_mps is the steady forward speed at MockPlant::NOMINAL_VOLTS.
    // drive_accel_mps2 is the forward accel from a stop at that voltage,
    // after static_friction_n. yaw_friction_nm is the torque to start a
    // spin. lateral_friction_n is roller Coulomb. traction_mu is the
    // longitudinal grip before a wheel slips.
    void setDynamics(double mass_kg, double max_vel_mps, double drive_accel_mps2,
                     double static_friction_n, double skid_tau_sec,
                     double side_resistance_ohm, double yaw_inertia_scale,
                     double yaw_friction_nm, double lateral_friction_n,
                     double traction_mu);

    // Ticks between the voltage applyDrive stored and the motor that
    // integrates it. Zero is the compiled plant. The field window is what
    // moves it; training leaves it alone, so a late command can oscillate
    // a tuned loop on the host without changing a search.
    void setVoltageDelayTicks(int ticks);

    // The four the training critic is allowed to see. The actor is not.
    struct Dynamics {
        double mass_kg = 0;
        double max_vel_mps = 0;
        double drive_accel_mps2 = 0;
        double static_friction_n = 0;
    };
    Dynamics dynamics() const {
        return Dynamics{mass_kg_, max_vel_mps_, drive_accel_mps2_, static_friction_n_};
    }

    void resetSensors();

    // New IMU scale, calibration residual, and gyro integral, drawn from
    // `seed`. The plant pose is left where it is. The reported heading matches
    // that pose, and the encoder baseline matches the wheels, so the first
    // reading contains only the motion that happens after this call.
    void armLocalization(std::uint64_t seed);

    void printTelemetry() const;

private:
    const IFeedForward* feedforward_;
    double max_voltage_;
    bool   use_battery_scaling_;

    // Wheel plant (meters, m/s) and the voltage the motors are currently receiving.
    // left_mps_ / right_mps_ are encoder speeds. body_fwd_ and body_omega_
    // are the chassis. They match until a wheel exceeds traction_mu_.
    double left_m_ = 0.0, right_m_ = 0.0;
    double left_mps_ = 0.0, right_mps_ = 0.0;
    double lat_mps_ = 0.0;   // body frame, positive to the left
    double body_fwd_ = 0.0;
    double body_omega_ = 0.0;
    bool left_slip_ = false, right_slip_ = false;
    double plant_left_v_ = 0.0, plant_right_v_ = 0.0;
    double left_amps_ = 0.0, right_amps_ = 0.0;
    double mass_kg_ = MockPlant::MASS_KG;
    double max_vel_mps_ = MockPlant::MAX_VEL_MPS;
    double drive_accel_mps2_ = MockPlant::DRIVE_ACCEL_MPS2;
    double static_friction_n_ = MockPlant::STATIC_FRICTION_N;
    double yaw_inertia_scale_ = MockPlant::YAW_INERTIA_SCALE;
    double yaw_friction_nm_ = MockPlant::YAW_FRICTION_NM;
    double lateral_friction_n_ = MockPlant::LATERAL_FRICTION_N;
    double traction_mu_ = MockPlant::TRACTION_MU;
    double skid_tau_ = MockPlant::SKID_TAU_SEC;
    double side_resistance_ = MockPlant::SIDE_RESISTANCE_OHM;
    double speedCeiling() const;
    double motorCurrent(double volts, double wheel_mps) const;

    // One Newton step of the motor and the chassis. Voltages are the
    // delayed command. Pose and walls stay in settleContacts.
    void integrateMotors(double dt_seconds, double left_v, double right_v);

    // Newest command sits at delay_head_. A delay of N ticks reads N slots
    // behind that, so the lag sees an older voltage. The cap leaves one
    // slot between the write and the oldest read.
    static constexpr int kVoltageDelayCap = 16;
    int voltage_delay_ticks_ = 0;
    int delay_head_ = 0;
    double delay_left_[kVoltageDelayCap]{};
    double delay_right_[kVoltageDelayCap]{};
    void delayCommand(double& left_v, double& right_v);

    // Forward-kinematics memory (meters) and simulated IMU.
    double prev_l_ = 0.0, prev_r_ = 0.0;
    double prev_v_fwd_ = 0.0, prev_v_lat_ = 0.0;
    double heading_rad_ = 0.0;
    double imu_offset_ = 0.0;
    // Plant pose the simulated distance sensors raycast from.
    // Meters. The plant starts at the geometric center of the field, which
    // is (0, FIELD_HALF) now that the origin is the center of the bottom edge.
    double true_x_ = 0.0, true_y_ = Robot::Range::CENTER_Y_M;
    double battery_voltage_ = 12.0;

    // Wall contact and the goal-lock square-up, after this tick's wheel speeds
    // already exist. dt <= 0 only separates a pose that was placed through
    // a wall. Encoder travel that the contact cancelled is taken back off
    // the wheels.
    void settleContacts(double dt_seconds);

    // Sensor-error state. The plant above stays clean; only the readings move.
    // Gyro scale is a property of the unit. Calibration redraws the residual
    // bias and restarts the integrated heading.
    double imu_scale_ = 1.0;
    double imu_accel_scale_ = 1.0;
    double imu_bias_dps_ = 0.0;
    double imu_rotation_deg_ = 0.0;   // get_rotation(), clockwise positive
    double imu_plant_heading_ = 0.0;
    bool imu_scaled_ = false;
    // Heading last published before the domain test unplugged the IMU.
    // Cleared once the port is connected again.
    bool imu_held_ = false;
    double imu_held_heading_ = 0.0;

    // Last distance sample. A new one is taken every kRangePeriodTicks.
    int dist_mm_[kDistanceChannels]{};
    int dist_conf_[kDistanceChannels]{};
    bool dist_valid_[kDistanceChannels]{};
    int dist_tick_ = 0;

    std::uint64_t rng_ = 0xC0FFEE1234ull;

    JoystickData joystick_{};
    ButtonState buttons_{};
    SubsystemCommand subsystems_{};

    // Common-mode cascade. Positive degrees are up. The seat is where a
    // goal takes the game piece and the holding current falls off.
    double cascade_deg_ = 0;
    double cascade_rpm_ = 0;
    double cascade_amps_ = 0;
    double cascade_torque_ = 0;
    void stepCascade(double dt_seconds);

    void applyCurrentLimit(double& left_v, double& right_v, double v_lin, double v_ang) const;

    // Previous velocity command, so a caller that omitted accel still gets kA.
    // A logged step passes Volts and does not update this.
    double volt_surge_ = 0.0;
    double volt_yaw_ = 0.0;
    bool volt_primed_ = false;
    void ensureSensorScales();
    void resetImuIntegration();
    double reportedRotationDeg() const;
    int distanceConfidence(int mm);

    // Sample the desktop keyboard into joystick_ / buttons_.
    void pollKeyboard();

    double uniform_();
    double normal_();
};
