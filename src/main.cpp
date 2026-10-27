/* ===========================================================================
 * main.cpp  --  PROS entry points + the universal robot pipeline.
 *
 * Dataflow each 10 ms tick (teleop / record):
 *   sense -> curve sticks -> map sticks -> override -> process commands
 *        -> actuate chassis -> subsystems -> telemetry -> record
 *
 * Localization runs asynchronously on a separate pros::Task, consuming each
 * SensorDict exactly once (no dropped travel deltas). Forward travel is the
 * wheel encoder. Sideways travel is the skid formula, both applied once here
 * so every estimator shares them. LocalizationOverride may replace that
 * sample for a wall, a bumper, or a latched goal.
 *
 * Replay still uses CascadedController for recorded MotionTarget following.
 * Command mode runs auton::CommandScheduler on this same tick. A recording
 * started in driver control keeps appending through that routine and through
 * the driver macros. Localization stays on its own task. Mechanism commands
 * only change subsystem state.
 * ========================================================================= */

#include "main.h"
#include "api.h"

#include "core/Types.h"
#include "core/Specs.h"
#include "core/Tuning.h"
#include "config/Settings.h"
#include "control/DrivePolicy.h"

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
#include "estimators/LocalizationOverride.h"
#include "math/MathUtils.h"
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
#include "logic/GoalAligner.h"
#include "curves/CubicDriveCurve.h"
#include "curves/LinearDriveCurve.h"
#include "logic/SubsystemControl.h"
#include "control/StaticMotionLimiter.h"
#include "platform/FramePlayer.h"
#include "auton/AutonCatalog.h"
#include "auton/BrainSelector.h"
#include "auton/CommandScheduler.h"
#include "auton/Mechanisms.h"

#include <atomic>

// ============================ WIRING ======================================
// Swap g_feedforward for a VASController(kS, kV, kA, kS_ang, kV_ang, kA_ang)
// to use the traditional armature model instead of the learned MLP.
VASController         g_feedforward(Feedforward::KS_LIN, Feedforward::KV_LIN, Feedforward::KA_LIN,
                                      Feedforward::KS_ANG, Feedforward::KV_ANG, Feedforward::KA_ANG);
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
// Survives COMMAND mode. Left starts and stops it; skills and the driver
// macros do not. Cleared when driver control is left for replay or disable.
bool g_recording = false;

pros::Mutex g_loc_mutex;
ImuLinearDelta g_imu_delta;
double      g_prev_heading = 0.0;
SensorDict  g_pending_sensors{};
bool        g_obs_pending = false;
bool        g_localization_run = true;

pros::Task* g_pipeline = nullptr;
pros::Task* g_localization = nullptr;
#if !defined(ROBOT_HOST)
pros::Task* g_brain_ui = nullptr;
#endif
auton::CommandScheduler g_scheduler;

// The button bits decodeRecorded understands. Skills commands store the
// same intent as the driver macros, but periodic() does not fill the bitmask
// the log replays from.
static uint8_t intentBitmask(const SubsystemIntent& intent) {
    uint8_t bits = 0;
    if (intent.intake == IntakeState::In) bits |= 1u << 0;
    else if (intent.intake == IntakeState::Out) bits |= 1u << 1;
    if (intent.cascade == CascadeState::Up) bits |= 1u << 2;
    else if (intent.cascade == CascadeState::Down) bits |= 1u << 3;
    if (intent.pneumatic_g) bits |= 1u << 4;
    if (intent.pneumatic_b) bits |= 1u << 5;
    return bits;
}

static RobotMode modeAfterRoutine() {
    return g_recording ? RobotMode::RECORDING : RobotMode::IDLE;
}

// The host has no field control. A connected competition switch reports
// disabled before the match and between autonomous and driver control.
static bool fieldDisabled() {
#if defined(ROBOT_HOST)
    return false;
#else
    return pros::competition::is_disabled() != 0;
#endif
}

static bool fieldAutonomous() {
#if defined(ROBOT_HOST)
    return false;
#else
    return pros::competition::is_autonomous() != 0;
#endif
}

// Command routines and recorded replay. stopSkills() is the abort path:
// it drops the queue and releases the mechanisms the routine owned.
// An in-progress recording is left alone; the command branch keeps appending.
static void startSkills() {
    std::unique_ptr<auton::Command> routine = auton::buildSelected(feedforward);
    if (!routine) return;
    g_drift_curve_abort = true;
    g_scheduler.cancel();
    auton::robotMechanisms().reset();
    g_scheduler.add(std::move(routine));
    g_mode = RobotMode::COMMAND;
}

static void stopSkills() {
    g_scheduler.cancel();
    auton::robotMechanisms().reset();
    actuateSubsystems(*drive, auton::robotMechanisms().periodic());
    drive->stop();
}

// Field disable and the end of the autonomous period. VEXos already ignores
// motor commands while disabled; this drops the routine so it does not
// resume when driver control starts.
static void releaseField() {
    g_drift_curve_abort = true;
    g_recording = false;
    if (g_mode == RobotMode::COMMAND) stopSkills();
    else {
        g_scheduler.cancel();
        auton::robotMechanisms().reset();
        drive->stop();
    }
    g_mode = RobotMode::IDLE;
}

static void startReplay() {
    if (trajectory->getFrameCount() == 0) trajectory->loadFromHeader();
    g_recording = false;
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

// Pose the goal touch test uses. The host plant is the real chassis.
// The brain has no separate truth, so the estimate stands in for it.
static Pose2d goalAlignBodyPose(const Pose2d& estimate) {
#if defined(ROBOT_HOST)
    (void)estimate;
    const MockDrive::PlantSnapshot plant = g_hardware.plantSnapshot();
    return Pose2d{plant.x_in, plant.y_in, plant.theta};
#else
    return estimate;
#endif
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

// One midpoint step, the same integration Odometry::update applies.
static Pose2d advancePose(Pose2d pose, double forward, double strafe, double heading) {
    const double d_theta = mathutil::normalize_angle(heading - pose.theta);
    const double mid = pose.theta + d_theta * 0.5;
    const double c = std::cos(mid);
    const double s = std::sin(mid);
    pose.x += forward * c - strafe * s;
    pose.y += forward * s + strafe * c;
    pose.theta = heading;
    return pose;
}

// Publish one sensor dict for the localization task. Forward travel is the
// wheel encoder and sideways travel is the skid formula. If the previous
// sample was not consumed yet, the new travel is added so no step is
// dropped. LocalizationOverride may replace that travel. The inertial
// velocity channels are copied out so the recording path can log them.
static void publishSensors(SensorDict sensors, ChassisObservation* inertial_out = nullptr) {
    const double v_fwd = g_imu_delta.forwardVelocity();
    const double v_lat = g_imu_delta.lateralVelocity();
    const double yaw = sensors.chassis.v_ang;
    LocalizationCue cue;
    cue.front_speed = std::hypot(v_fwd, v_lat + yaw * (Robot::Chassis::BODY_LENGTH_M * 0.5));
    cue.ax_g = sensors.chassis.imu_accel_x;
    cue.accel_g = std::hypot(cue.ax_g, sensors.chassis.imu_accel_y);
    double dt = sensors.chassis.sample_dt;
    if (!(dt >= 0.001) || dt > 0.2) dt = System::DT_SEC;
    g_imu_delta.apply(sensors.chassis, dt);
    g_loc_mutex.take();
    const Pose2d pose = poseInchesToMeters(estimator->getState().pose);
    Pose2d carried = pose;
    if (g_obs_pending) {
        const ChassisObservation& pend = g_pending_sensors.chassis;
        carried = advancePose(pose, pend.forward_delta, pend.strafe_delta, pend.heading_rad);
    }
    const LocalizationOverride over = applyLocalizationOverride(
        g_imu_delta, sensors.chassis, pose, carried, cue, pros::millis());
    if (over.snap) {
        estimator->overridePose(over.seated_in);
        g_pending_sensors = sensors;
        g_obs_pending = true;
    } else if (over.replace) {
        g_pending_sensors = sensors;
        g_obs_pending = true;
    } else if (g_obs_pending) {
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
    if (inertial_out) {
        inertial_out->v_inertial_fwd = sensors.chassis.v_inertial_fwd;
        inertial_out->v_inertial_lat = sensors.chassis.v_inertial_lat;
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
// g_imu_epoch advances each time a calibration is requested. The IMU task
// stores the same value in g_imu_done when the reset returns. This loop
// commits the pose and raises g_input_enabled on its next tick. The brain
// clip does not gate either one.
static std::atomic<std::uint32_t> g_imu_epoch{0};
static std::atomic<std::uint32_t> g_imu_done{0};
static std::atomic<bool> g_input_enabled{false};
static std::atomic<bool> g_cal_notice{false};

static bool imuSettled() {
    const std::uint32_t epoch = g_imu_epoch.load();
    return epoch != 0 && g_imu_done.load() == epoch;
}

void robot_pipeline() {
    const uint32_t LOOP_MS = (uint32_t)System::LOOP_TIME_MS;
    // Deadline for this tick. Each pass adds one period, then waits out
    // whatever time the work did not already use. A stall longer than one
    // period rebases the deadline instead of busy-looping through it.
    uint32_t target_ms = pros::millis();
    bool field_halted = false;
    auto pace = [&]() {
        const int32_t remaining_ms = static_cast<int32_t>(target_ms - pros::millis());
        if (remaining_ms < -static_cast<int32_t>(LOOP_MS)) target_ms = pros::millis();
        else if (remaining_ms > 0) pros::delay(static_cast<uint32_t>(remaining_ms));
    };

    while (true) {
        target_ms += LOOP_MS;

        if (fieldDisabled()) {
            if (!field_halted) {
                releaseField();
                field_halted = true;
            } else {
                drive->stop();
            }
            pace();
            continue;
        }
        field_halted = false;

        // Sticks, buttons, and autonomous stay dropped until the IMU task
        // has finished and this loop has reset the pose.
        if (!g_input_enabled.load()) {
            if (imuSettled()) {
                commitRobotSensorCalibration(*estimator, *command_processor,
                                             &g_loc_mutex, &g_prev_heading, &g_obs_pending);
                g_imu_delta.reset();
                g_input_enabled.store(true);
                if (g_cal_notice.exchange(false)) ui->displayMessage("RESET COMPLETE");
            } else {
                drive->stop();
                pace();
                continue;
            }
        }

        // --- 1. SENSE (unified dict API) ---
        g_loc_mutex.take();
        double prev_heading = g_prev_heading;
        g_loc_mutex.give();

        RobotReading reading = readRobotInput(*drive, prev_heading);
        publishSensors(reading.sensors, &reading.sensors.chassis);

        StateEstimate est = snapshotEstimate();
        const Pose2d body = goalAlignBodyPose(est.pose);

        // --- 2. UI & MODE MANAGEMENT ---
        RobotMode requested = ui->processStateToggles(reading.buttons, g_mode);
        if (requested != g_mode) {
            if (requested == RobotMode::RECORDING) {
                g_recording = true;
                trajectory->beginRecording(est.pose);
                replay_controller->reset();
                command_processor->reset();
            } else if (requested == RobotMode::IDLE) {
                g_recording = false;
            }
            g_mode = requested;
        }
        ui->updateDisplay(g_mode, est.pose, snapshotTotalHeading(),
                          trajectory->getFrameCount(), g_recording);
        // The auton screen is not drawn here. pros::screen and the route
        // preview block, and this tick still has to read the controller and
        // apply the sticks. brain_ui_task owns the display.

        // --- 2b. UTILITY BUTTONS ---
        if (ui->checkSaveRequested(reading.buttons) && g_mode == RobotMode::IDLE &&
            trajectory->getFrameCount() > 0) {
            ui->displayMessage("Dumping...");
            trajectory->saveToConsole();
            ui->displayMessage("Done!");
        }
        if (ui->checkLoadRequested(reading.buttons, g_mode)) trajectory->loadFromHeader();
        if (ui->checkCalibrationRequested(reading.buttons) && g_input_enabled.load()) {
            g_drift_curve_abort = true;
            if (g_mode == RobotMode::COMMAND) {
                g_mode = modeAfterRoutine();
                stopSkills();
            }
            ui->displayMessage("CALIBRATING...");
            g_cal_notice.store(true);
            g_input_enabled.store(false);
            g_imu_epoch.fetch_add(1);
            drive->stop();
            pace();
            continue;
        }
        // Y runs whichever routine competition autonomous() runs.
        if (ui->checkAutonTestRequested(reading.buttons)) {
            g_drift_curve_abort = true;
            if (Settings::Auton::USE_COMMAND_SCHEDULER) startSkills();
            else startReplay();
        }

        // --- 3. SAFETY ABORT (replay) ---
        if (g_mode == RobotMode::REPLAYING && macros->isAbortRequested(reading.buttons)) {
            g_mode = RobotMode::IDLE;
            macros->notifyAbort(pros::millis());
            drive->stop();
        }

        // --- 4. TARGET / COMMAND / ACTUATION ---
        // The goal snap lives in the plant. Autonomous and replay leave the
        // latch set; reverse is what lets go.

        ChassisCommand   drive_cmd;
        SubsystemCommand sub_cmd;

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
        } else if (g_mode == RobotMode::COMMAND) {
            if (macros->isAbortRequested(reading.buttons)) {
                macros->notifyAbort(pros::millis());
                stopSkills();
                if (g_recording) {
                    trajectory->recordFrame(est.pose, {}, {}, {},
                                            reading.sensors.chassis, reading.raw);
                }
                g_mode = modeAfterRoutine();
            } else {
                const auton::SchedulerOutput step = g_scheduler.run(est);
                sub_cmd = auton::robotMechanisms().periodic();
                sub_cmd.raw_bitmask = intentBitmask(auton::robotMechanisms().intent());
                sub_cmd = applySubsystemCommand(*drive, *macros, sub_cmd,
                                                reading.buttons, pros::millis());
                CommandVector commanded{};
                ChassisCommand applied{};
                if (!step.running) {
                    drive->stop();
                    g_mode = modeAfterRoutine();
                } else if (step.has_drive) {
                    commanded = CommandVector::fromChassis(step.drive);
                    applied = drive->applyDrive(step.drive, reading.sensors.chassis);
                } else {
                    drive->stop();
                }
                if (g_recording) {
                    trajectory->recordFrame(est.pose, commanded, applied, sub_cmd,
                                            reading.sensors.chassis, reading.raw);
                }
            }
        } else {
            // Captured before the curve so a small real deflection still aborts.
            const JoystickData raw_joy = reading.joystick;

            // curve each stick axis, then map to a velocity/command vector
            JoystickData shaped = reading.joystick;
            shaped.left_y  = drive_curve->apply(shaped.left_y);
            shaped.left_x  = drive_curve->apply(shaped.left_x);
            shaped.right_y = drive_curve->apply(shaped.right_y);
            shaped.right_x = drive_curve->apply(shaped.right_x) * Tuning::TURN_STICK_SCALE;
            reading.joystick = shaped;

            CommandVector mapped = velocity_mapper->map(reading.sensors.chassis.heading_rad,
                                                        reading.joystick);

            // An override replaces this while its run flag is set.
            OverrideResult overridden = userInputOverride(reading.sensors.chassis, mapped,
                                                          reading.buttons, est, body, raw_joy,
                                                          g_mode, feedforward);
            reading.buttons = overridden.buttons;

            // mid-pipeline processor (default passthrough; hook for ghost/assists)
            CommandVector processed = command_processor->process(reading, overridden.commands, est);

            drive_cmd = processed.toChassis();

            // Cascade, intake, claw, and the scoring macro.
            sub_cmd = controlSubsystems(*drive, *macros, reading, processed, pros::millis());

            // Actuate chassis from the processed command vector
            ChassisCommand applied = drive->applyDrive(drive_cmd, reading.sensors.chassis);

            // Unified recording: estimator pose, denoised rates, plant effort,
            // and the pre-filter sensor dict from this tick.
            if (g_recording) {
                trajectory->recordFrame(est.pose, processed, applied, sub_cmd,
                                        reading.sensors.chassis, reading.raw);
            }
        }

        // --- 5. SYNCHRONIZE ---
        // On the host, service() advances the simulated plant one period.
        // On the brain, service() is empty. Then sleep only until target_ms,
        // so the loop period stays LOOP_MS when the work finishes early.
        // A late tick does not sleep; the next pass still adds one period.
#if defined(ROBOT_HOST)
        Pose2d particles[50];
        int particle_count = 0;
        g_loc_mutex.take();
        const StateEstimate estimate = estimator->getState();
        particle_count = estimator->sampleParticles(particles, 50);
        g_loc_mutex.give();
        Pose2d curve[64];
        // The latched goal arc is drawn over the routine and the drift curve.
        int curve_n = goalAlignPath(body, curve, 64);
        if (curve_n < 2) {
            curve_n = (g_mode == RobotMode::COMMAND)
                ? g_scheduler.samplePath(estimate.pose, curve, 64, estimate.v_lin, estimate.v_ang)
                : userInputOverridePath(estimate.pose, body, curve, 64, estimate.v_lin, estimate.v_ang);
        }
        fieldwindow::setFollowPath(curve, curve_n, curve_n >= 2);
        drive->service(System::DT_SEC, estimate.pose, particles, particle_count);
#else
        drive->service(System::DT_SEC, snapshotEstimate().pose);
#endif
        pace();
    }
}

// ============================ PROS ENTRY POINTS ===========================
// Timed-elastic preview and the driver override share DriftFollower's
// Sample[64] scratch. 0x4000 words is 64 KB. The default task stack is half
// of that, which is enough on a 1 MB host thread and not on the V5.
constexpr std::uint16_t kWorkerStack = 0x4000;

static void imuCalTask() {
    std::uint32_t done = 0;
    while (true) {
        const std::uint32_t want = g_imu_epoch.load();
        if (want == done) {
            pros::delay(5);
            continue;
        }
        drive->stop();
        drive->beginSensorCalibration();
        drive->waitSensorCalibration();
        done = want;
        g_imu_done.store(done);
    }
}

#if !defined(ROBOT_HOST)
// Selector and route preview. Started after the splash so it is the next
// owner of the brain display, and kept off the control tick so a fit cannot
// freeze the sticks. Controller text is flushed here for the same reason.
static void brain_ui_task(void*) {
    while (true) {
        g_ui.serviceController();
        auton::serviceBrainSelector(g_ui.brainUpdates());
        pros::delay(20);
    }
}

static void splashThenUi(void*) {
    if (Settings::Splash::ENABLED) {
        playFrameSequence({
            Settings::Splash::PATH_FORMAT,
            Settings::Splash::FIRST_FRAME,
            Settings::Splash::FRAME_COUNT,
            Settings::Splash::FRAME_STEP,
            Settings::Splash::FPS,
        });
    }
    g_ui.setBrainUpdates(true);
    g_brain_ui = new pros::Task(brain_ui_task, nullptr, TASK_PRIORITY_DEFAULT - 1,
                                kWorkerStack, "brain-ui");
}
#endif

void initialize() {
    feedforward->init();
    DrivePolicy::init();
    drive->initialize();
    g_trajectory = new LogFrameReplayer();
    trajectory   = g_trajectory;
    replay_controller->init();
    estimator->initialize();
    if (trajectory->getFrameCount() == 0) trajectory->loadFromHeader();

    g_imu_done.store(0);
    g_input_enabled.store(false);
    g_imu_epoch.store(1);

    // Brain text stays off before the tick starts, so the first frames are
    // not fighting the splash for the screen. On the host the window is
    // created here, on this thread, before any task can touch it.
    g_ui.setBrainUpdates(false);
#if defined(ROBOT_HOST)
    fieldwindow::open();
#endif

    new pros::Task(imuCalTask, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "imu-cal");

    // Tick starts now. Sticks stay locked until imuCalTask finishes.
    g_localization = new pros::Task(localization_task, TASK_PRIORITY_DEFAULT, kWorkerStack, "localize");
    g_pipeline     = new pros::Task(robot_pipeline, TASK_PRIORITY_DEFAULT, kWorkerStack, "pipeline");
#if defined(ROBOT_HOST)
    // The window is pumped on the thread that created it.
    if (Settings::Splash::ENABLED) {
        playFrameSequence({
            Settings::Splash::PATH_FORMAT,
            Settings::Splash::FIRST_FRAME,
            Settings::Splash::FRAME_COUNT,
            Settings::Splash::FRAME_STEP,
            Settings::Splash::FPS,
        });
    }
    g_ui.setBrainUpdates(true);
#else
    new pros::Task(splashThenUi, nullptr, TASK_PRIORITY_DEFAULT - 1, kWorkerStack, "splash");
#endif
}

void disabled() {
    releaseField();
    while (fieldDisabled()) pros::delay(20);
}

void competition_initialize() {}

void autonomous() {
    // The host reports "not autonomous", so this returns and the controller
    // Y path remains the simulator's start. On a field, returning before the
    // period ends would make PROS call autonomous() again and restart it.
    while (!g_input_enabled.load()) {
        if (!fieldAutonomous() || fieldDisabled()) return;
        pros::delay(10);
    }
    if (!fieldAutonomous() || fieldDisabled()) return;
    if (Settings::Auton::USE_COMMAND_SCHEDULER) startSkills();
    else startReplay();
    while (fieldAutonomous() && !fieldDisabled()) pros::delay(10);
    if (g_mode == RobotMode::COMMAND || g_mode == RobotMode::REPLAYING) releaseField();
}

void opcontrol() {
    g_mode = RobotMode::IDLE;
    g_recording = false;
    g_drift_curve_abort = true;
    g_scheduler.cancel();
    while (!fieldDisabled() && !fieldAutonomous()) pros::delay(10);
}
