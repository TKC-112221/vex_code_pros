/* ===========================================================================
 * main.cpp  --  PROS entry points + the universal robot pipeline.
 *
 * Dataflow each 10 ms tick (teleop / record):
 *   sense -> curve sticks -> map sticks -> override -> process commands
 *        -> actuate chassis -> subsystems -> telemetry -> record
 *
 * Localization runs asynchronously on a separate pros::Task, consuming each
 * SensorDict exactly once (no dropped travel deltas). The linear delta is the
 * inertial accelerometer, integrated once here so every estimator shares it.
 *
 * Replay still uses CascadedController for recorded MotionTarget following.
 * ========================================================================= */

#include "main.h"
#include "api.h"

#include "core/Types.h"
#include "core/Specs.h"
#include "core/Gains.h"
#include "config/Settings.h"

#include "interfaces/IDriveBase.h"
#include "interfaces/IEstimator.h"
#include "interfaces/IController.h"
#include "interfaces/IFeedForward.h"
#include "interfaces/ITrajectoryManager.h"
#include "interfaces/IVelocityMapper.h"
#include "interfaces/ICommandProcessor.h"
#include "interfaces/IDriveCurve.h"

// ---- Concrete implementations (the only place types are named) ----
// ROBOT_HOST is defined by the desktop link (make host). The V5 image is not.
#if defined(ROBOT_HOST)
#include "hardware/MockDrive.h"
#include "host/FieldWindow.h"
#else
#include "hardware/TankDrive.h"
#include "hardware/XDrive.h"
#include "hardware/ParenthesisDrive.h"
#endif
#include "estimators/ImuLinearDelta.h"
#include "estimators/OdometryEstimator.h"
#include "estimators/ParticleFilterEstimator.h"
#include "estimators/SlipEnsembleEstimator.h"
#include "control/CascadedController.h"
#include "control/PassthroughController.h"
#include "control/NeuralFeedforward.h"
#include "control/VASController.h"
#include <cmath>
#include "trajectory/LogFrameReplayer.h"
#include "logic/MacroController.h"
#include "logic/UIManager.h"
#include "logic/SensorReader.h"
#include "logic/SensorCalibration.h"
#include "logic/TankDriveVelocityMapper.h"
#include "logic/UserInputOverride.h"
#include "logic/DriftCurveOverride.h"
#include "curves/CubicDriveCurve.h"
#include "curves/LinearDriveCurve.h"
#include "logic/SubsystemControl.h"
#include "control/StaticMotionLimiter.h"

// ============================ WIRING ======================================
// Swap g_feedforward for a VASController(kS, kV, kA, kS_ang, kV_ang, kA_ang)
// to use the traditional armature model instead of the learned MLP.
VASController         g_feedforward(0.2, (12.0-0.2)/4.0, 12.0/5.0, 0.2, 4.72*0.3556/2.0, 2.4*(2*0.1652)/(6*0.3556));
#if defined(ROBOT_HOST)
MockDrive             g_hardware(g_feedforward, System::MAX_VOLTS, Settings::ControlToggles::BATTERY_SCALING);
#else
TankDrive             g_hardware(g_feedforward, System::MAX_VOLTS, Settings::ControlToggles::BATTERY_SCALING);
#endif
OdometryEstimator g_estimator;
// SlipEnsembleEstimator g_estimator;
// ParticleFilterEstimator g_estimator;
LogFrameReplayer*     g_trajectory = nullptr;
MacroController       g_macros;
UIManager             g_ui;

StaticMotionLimiter       g_limiter;
CascadedController        g_replay_controller(g_feedforward, g_limiter);  // used for REPLAYING only
PassthroughController     g_command_processor;
TankDriveVelocityMapper   g_velocity_mapper(System::MAX_VOLTS);
CubicDriveCurve           g_curve(Tuning::CUBE_RATIO);
// LinearDriveCurve       g_curve;

// The pipeline talks to these abstractions only.
IFeedForward*       feedforward       = &g_feedforward;
IDriveBase*         drive             = &g_hardware;
IEstimator*         estimator         = &g_estimator;
IController*        replay_controller = &g_replay_controller;
ITrajectoryManager* trajectory        = nullptr;
IMacroController*   macros            = &g_macros;
IUIManager*         ui                = &g_ui;
IVelocityMapper*    velocity_mapper   = &g_velocity_mapper;
ICommandProcessor*  command_processor = &g_command_processor;
IDriveCurve*        drive_curve       = &g_curve;

// ============================ SHARED STATE ================================
RobotMode g_mode = RobotMode::IDLE;
RobotMode g_last_recording_mode = RobotMode::RECORDING;

pros::Mutex g_loc_mutex;
ImuLinearDelta g_imu_delta;
double      g_prev_heading = 0.0;
SensorDict  g_pending_sensors{};
bool        g_obs_pending = false;
bool        g_localization_run = true;

pros::Task* g_pipeline = nullptr;
pros::Task* g_localization = nullptr;

// Begin autonomous replay from the recorded start pose/heading.
static void startReplay() {
    if (trajectory->getFrameCount() == 0) trajectory->loadFromHeader();
    g_mode = RobotMode::REPLAYING;
    trajectory->resetPlayback();
    drive->alignHeadingTo(trajectory->getStartHeading());
    resetSensorFilters();

    g_loc_mutex.take();
    estimator->overridePose(trajectory->getStartPose());
    g_imu_delta.reset();
    g_prev_heading = trajectory->getStartHeading();
    g_obs_pending = false;
    g_loc_mutex.give();
}

static StateEstimate snapshotEstimate() {
    g_loc_mutex.take();
    StateEstimate est = estimator->getState();
    g_loc_mutex.give();
    return est;
}

static double snapshotTotalHeading() {
    g_loc_mutex.take();
    double h = estimator->getTotalHeading();
    g_loc_mutex.give();
    return h;
}

// Publish one sensor dict for the localization task. Linear travel is the
// accelerometer integral; if the previous sample was not consumed yet, the
// new travel is added so no step is dropped.
static void publishSensors(SensorDict sensors) {
    g_imu_delta.apply(sensors.chassis, System::DT_SEC,
                      !Settings::ControlToggles::IMU_FALLBACK);
    g_loc_mutex.take();
    if (g_obs_pending) {
        g_pending_sensors.chassis.forward_delta += sensors.chassis.forward_delta;
        g_pending_sensors.chassis.strafe_delta += sensors.chassis.strafe_delta;
        g_pending_sensors.chassis.heading_rad = sensors.chassis.heading_rad;
        g_pending_sensors.chassis.v_lin = sensors.chassis.v_lin;
        g_pending_sensors.chassis.v_ang = sensors.chassis.v_ang;
        for(int i = 0; i < kDistanceChannels; ++i) {
            g_pending_sensors.distance[i] = sensors.distance[i];
        }
    } else {
        g_pending_sensors = sensors;
        g_obs_pending = true;
    }
    g_loc_mutex.give();
}

// ============================ ASYNC LOCALIZATION ==========================
void localization_task() {
    while (g_localization_run) {
        g_loc_mutex.take();
        if (g_obs_pending) {
            estimator->update(g_pending_sensors);
            g_prev_heading = estimator->getState().pose.theta;
            g_obs_pending = false;
        }
        g_loc_mutex.give();
        pros::delay(5);
    }
}

// ============================ THE PIPELINE ================================
void robot_pipeline() {
    const uint32_t LOOP_MS = (uint32_t)System::LOOP_TIME_MS;

    while (true) {
        // --- 1. SENSE (unified dict API) ---
        g_loc_mutex.take();
        double prev_heading = g_prev_heading;
        g_loc_mutex.give();

        RobotReading reading = readRobotInput(*drive, prev_heading);
        publishSensors(reading.sensors);

        StateEstimate est = snapshotEstimate();

        // --- 2. UI & MODE MANAGEMENT ---
        RobotMode requested = ui->processStateToggles(reading.buttons, g_mode);
        if (requested != g_mode) {
            if (requested == RobotMode::RECORDING || requested == RobotMode::SYSID_DRIVE) {
                trajectory->beginRecording(requested, est.pose);
                replay_controller->reset();
                command_processor->reset();
                g_last_recording_mode = requested;
            }
            g_mode = requested;
        }
        ui->updateDisplay(g_mode, est.pose, snapshotTotalHeading(),
                          trajectory->getFrameCount(), g_last_recording_mode);

        // --- 2b. UTILITY BUTTONS ---
        if (ui->checkSaveRequested(reading.buttons) && g_mode == RobotMode::IDLE &&
            trajectory->getFrameCount() > 0) {
            ui->displayMessage("Dumping...");
            trajectory->saveToConsole();
            ui->displayMessage("Done!");
        }
        if (ui->checkLoadRequested(reading.buttons, g_mode)) trajectory->loadFromHeader();
        if (ui->checkCalibrationRequested(reading.buttons)) {
            g_drift_curve_abort = true;
            calibrateRobotSensors(*drive, *estimator, *command_processor, ui,
                                  &g_loc_mutex, &g_prev_heading, &g_obs_pending);
            g_imu_delta.reset();
            pros::delay(500);
        }
        if (ui->checkAutonTestRequested(reading.buttons)) {
            g_drift_curve_abort = true;
            startReplay();
        }

        // --- 3. SAFETY ABORT (replay) ---
        if (g_mode == RobotMode::REPLAYING && macros->isAbortRequested(reading.buttons)) {
            g_mode = RobotMode::IDLE;
            macros->notifyAbort(pros::millis());
            drive->stop();
        }

        // --- 4. TARGET / COMMAND / ACTUATION ---
        ChassisCommand   drive_cmd;
        SubsystemCommand sub_cmd;
        MotionTarget     record_target;
        bool recording = (g_mode == RobotMode::RECORDING || g_mode == RobotMode::SYSID_DRIVE);

        if (g_mode == RobotMode::REPLAYING) {
            MotionTarget target = trajectory->getNextTarget(est.pose, reading.sensors.chassis);
            if (trajectory->isFinished()) {
                g_mode = RobotMode::IDLE;
                drive->stop();
            } else {
                drive_cmd = replay_controller->computeCommand(est.pose, target, reading.sensors.chassis);
                sub_cmd   = trajectory->getTargetSubsystems();
                actuateSubsystems(*drive, sub_cmd);
                drive->applyDrive(drive_cmd, reading.sensors.chassis);
            }
        } else {
            // Captured before the curve so a small real deflection still aborts.
            const JoystickData raw_joy = reading.joystick;

            // curve each stick axis, then map to a velocity/command vector
            JoystickData shaped = reading.joystick;
            shaped.left_y  = drive_curve->apply(shaped.left_y);
            shaped.left_x  = drive_curve->apply(shaped.left_x);
            shaped.right_y = drive_curve->apply(shaped.right_y);
            shaped.right_x = drive_curve->apply(shaped.right_x) * 0.6;
            reading.joystick = shaped;

            CommandVector mapped = velocity_mapper->map(reading.sensors.chassis.heading_rad,
                                                        reading.joystick);

            // An override replaces this while its run flag is set.
            OverrideResult overridden = userInputOverride(reading.sensors.chassis, mapped,
                                                          reading.buttons, est, raw_joy,
                                                          g_mode, feedforward);
            reading.buttons = overridden.buttons;

            // mid-pipeline processor (default passthrough; hook for ghost/assists)
            CommandVector processed = command_processor->process(reading, overridden.commands, est);

            drive_cmd = processed.toChassis();

            // Non-chassis subsystems (intake / rollers / pneumatics / ...)
            sub_cmd = controlSubsystems(*drive, *macros, reading, processed, pros::millis());

            // Actuate chassis from the processed command vector
            ChassisCommand applied = drive->applyDrive(drive_cmd, reading.sensors.chassis);

            // Recording: pose from estimate; effort lives in command + applied
            // (chassis frame). Do not stuff volts into MotionTarget.velocity.
            record_target.pose = est.pose;

            if (recording) {
                trajectory->recordFrame(record_target, est.pose, processed, applied, sub_cmd,
                                        reading.sensors.chassis,
                                        replay_controller->lastSpatialError(),
                                        replay_controller->lastAngularError());
            }
        }

        // --- 5. SYNCHRONIZE ---
        // Sleep one control period. On the host, delay is the desktop stub and
        // service() advances the simulated plant. On the brain, service() is empty.
        pros::delay(LOOP_MS);
#if defined(ROBOT_HOST)
        Pose2d particles[50];
        int particle_count = 0;
        g_loc_mutex.take();
        const Pose2d estimate = estimator->getState().pose;
        particle_count = estimator->sampleParticles(particles, 50);
        g_loc_mutex.give();
        Pose2d curve[64];
        int curve_n = userInputOverridePath(estimate, curve, 64);
        fieldwindow::setFollowPath(curve, curve_n, curve_n >= 2);
        drive->service(System::DT_SEC, estimate, particles, particle_count);
#else
        drive->service(System::DT_SEC, snapshotEstimate().pose);
#endif
    }
}

// ============================ PROS ENTRY POINTS ===========================
void initialize() {
    feedforward->init();
    drive->initialize();
    g_trajectory = new LogFrameReplayer();
    trajectory   = g_trajectory;
    replay_controller->init();
    estimator->initialize();
    if (trajectory->getFrameCount() == 0) trajectory->loadFromHeader();

    calibrateRobotSensors(*drive, *estimator, *command_processor, ui,
                          &g_loc_mutex, &g_prev_heading, &g_obs_pending);
    g_imu_delta.reset();
    ui->displayMessage("READY");

    g_localization = new pros::Task(localization_task);
    g_pipeline     = new pros::Task(robot_pipeline);
}

void disabled() {
    g_mode = RobotMode::IDLE;
    g_drift_curve_abort = true;
    drive->stop();
}

void competition_initialize() {}

void autonomous() {
    startReplay();
    while (g_mode == RobotMode::REPLAYING) pros::delay(10);
}

void opcontrol() {
    g_mode = RobotMode::IDLE;
    while (true) pros::delay(10);
}
