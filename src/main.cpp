/* ===========================================================================
 * main.cpp  --  PROS entry points + the universal robot pipeline.
 *
 * The pipeline loop is written entirely against abstract interfaces (IDriveBase,
 * IEstimator, IController, ITrajectoryManager) and drivebase-agnostic types
 * (Twist, ChassisCommand, MotionTarget). It never computes a left/right voltage
 * and never assumes a tank chassis. Swapping any stage -- a different drive
 * (XDrive/ParenthesisDrive), estimator (KalmanEstimator), or stick curve -- is a
 * one-line change in the WIRING block; the pipeline does not change.
 *
 * PROS lifecycle: initialize() sets everything up and launches the pipeline on a
 * background pros::Task; autonomous() and opcontrol() simply select the mode the
 * running pipeline acts on.
 *
 * Dataflow each 10 ms tick:
 *   sense -> estimate -> UI/state -> target -> control -> actuate -> record
 * ========================================================================= */

#include "main.h"

#include "core/Types.h"
#include "core/Specs.h"
#include "core/Gains.h"
#include "config/Settings.h"

#include "interfaces/IDriveBase.h"
#include "interfaces/IEstimator.h"
#include "interfaces/IController.h"
#include "interfaces/ITrajectoryManager.h"

// ---- Concrete implementations (the only place types are named) ----
#include "hardware/TankDrive.h"
#include "hardware/XDrive.h"            // holonomic swap-in
#include "hardware/ParenthesisDrive.h" // 6-motor X + forward side wheels
#include "estimators/OdometryEstimator.h"
#include "control/CascadedController.h"
#include "trajectory/LogFrameReplayer.h"
#include "curves/CubicDriveCurve.h"
#include "logic/TeleopTargetGenerator.h"
#include "logic/MacroController.h"
#include "logic/UIManager.h"
#include "control/StaticMotionLimiter.h"    // default command clamp
#include "control/PhysicsMotionLimiter.h"   // physics-predicted command clamp
#include "math/NeuralNetDynamics.h"         // the drivetrain "simulator"

// ============================ WIRING ======================================
// Choose the concrete strategy for each pipeline stage here. To change the
// chassis, swap the one line below -- nothing else in the project changes:
//     TankDrive        g_hardware(...);   // 6-motor differential (default)
//     XDrive           g_hardware(...);   // 4-omni holonomic
//     ParenthesisDrive g_hardware(...);   // 4-omni + 2 forward side wheels
CubicDriveCurve       g_curve(Tuning::CUBE_RATIO);
TankDrive             g_hardware(System::MAX_VOLTS, Settings::ControlToggles::BATTERY_SCALING);
OdometryEstimator     g_estimator;
LogFrameReplayer      g_trajectory;
MacroController       g_macros;
UIManager             g_ui;

// Command clamp (max vel/accel supplier). Default: fixed tuned ceilings, so the
// build is byte-for-byte identical to the hardcoded limits. To predict feasible
// ceilings from the drivetrain model at the current battery/velocity, swap to:
//     NeuralNetDynamics    g_dynamics;
//     PhysicsMotionLimiter g_limiter(g_dynamics);
// and pass g_limiter to g_controller and g_teleop below.
StaticMotionLimiter   g_limiter;

CascadedController    g_controller(g_limiter);
TeleopTargetGenerator g_teleop(g_curve, g_limiter);

// The pipeline talks to these abstractions only.
IDriveBase*         drive      = &g_hardware;
IEstimator*         estimator  = &g_estimator;
IController*        controller = &g_controller;
ITrajectoryManager* trajectory = &g_trajectory;
ITeleopSource*      teleop     = &g_teleop;
IMacroController*   macros     = &g_macros;
IUIManager*         ui         = &g_ui;

// ============================ SHARED STATE ================================
RobotMode g_mode = RobotMode::IDLE;
RobotMode g_last_recording_mode = RobotMode::RECORDING;
double    g_prev_heading = 0.0;   // fed back to the drivebase for IMU-fallback
pros::Task* g_pipeline = nullptr;

// Begin autonomous replay from the recorded start pose/heading.
static void startReplay() {
    if (trajectory->getFrameCount() == 0) trajectory->loadFromHeader();
    g_mode = RobotMode::REPLAYING;
    trajectory->resetPlayback();
    drive->alignHeadingTo(trajectory->getStartHeading());
    estimator->overridePose(trajectory->getStartPose());
    g_prev_heading = trajectory->getStartHeading();
}

// ============================ THE PIPELINE ================================
void robot_pipeline() {
    const uint32_t LOOP_MS = (uint32_t)System::LOOP_TIME_MS;
    uint32_t timestamp = pros::millis();

    while (true) {
        // --- 1. SENSE & ESTIMATE ---
        ButtonState        buttons = drive->readButtons();
        JoystickData       joy     = drive->readJoystick();
        ChassisObservation obs     = drive->readObservation(g_prev_heading);
        estimator->update(obs);
        StateEstimate est = estimator->getState();
        g_prev_heading = est.pose.theta;

        // --- 2. UI & MODE MANAGEMENT ---
        RobotMode requested = ui->processStateToggles(buttons, g_mode);
        if (requested != g_mode) {
            if (requested == RobotMode::RECORDING || requested == RobotMode::SYSID_DRIVE) {
                trajectory->beginRecording(requested, est.pose);
                controller->reset();
                teleop->resetToPose(est.pose);
                g_last_recording_mode = requested;
            }
            g_mode = requested;
        }
        ui->updateDisplay(g_mode, est.pose, estimator->getTotalHeading(),
                           trajectory->getFrameCount(), g_last_recording_mode);

        // --- 2b. UTILITY BUTTONS ---
        if (ui->checkSaveRequested(buttons) && g_mode == RobotMode::IDLE &&
            trajectory->getFrameCount() > 0) {
            ui->displayMessage("Dumping...");
            trajectory->saveToConsole();
            ui->displayMessage("Done!");
        }
        if (ui->checkLoadRequested(buttons, g_mode)) trajectory->loadFromHeader();
        if (ui->checkCalibrationRequested(buttons)) {
            ui->displayMessage("CALIBRATING...");
            drive->stop();
            drive->calibrateSensors();
            estimator->reset();
            teleop->resetToPose(Pose2d{});
            controller->reset();
            g_prev_heading = 0.0;
            ui->displayMessage("RESET COMPLETE");
            pros::delay(500);
        }
        if (ui->checkAutonTestRequested(buttons)) startReplay();

        // --- 3. SAFETY ABORT (replay) ---
        if (g_mode == RobotMode::REPLAYING && macros->isAbortRequested(buttons)) {
            g_mode = RobotMode::IDLE;
            macros->notifyAbort(pros::millis());
            drive->stop();
        }

        // --- 4. TARGET GENERATION, CONTROL, ACTUATION ---
        ChassisCommand   drive_cmd;
        SubsystemCommand sub_cmd;
        TeleopDrive      td;          // retained for post-actuation recording
        bool recording = (g_mode == RobotMode::RECORDING || g_mode == RobotMode::SYSID_DRIVE);

        if (g_mode == RobotMode::REPLAYING) {
            MotionTarget target = trajectory->getNextTarget(est.pose, obs);
            if (trajectory->isFinished()) {
                g_mode = RobotMode::IDLE;
                drive->stop();
            } else {
                drive_cmd = controller->computeCommand(est.pose, target, obs);
                sub_cmd   = trajectory->getTargetSubsystems();
            }
        } else {
            td = teleop->generate(g_mode, joy, buttons, est.pose, obs);
            sub_cmd = macros->processSubsystems(buttons, pros::millis());
            if (td.direct) drive_cmd = td.command;
            else           drive_cmd = controller->computeCommand(est.pose, td.target, obs);
        }

        // --- 5. ACTUATE (drivebase owns inverse kinematics + safety) ---
        ChassisCommand applied = drive->applyDrive(drive_cmd, obs);
        drive->applySubsystems(sub_cmd);

        // --- 6. RECORD (after actuation, to log the applied effort) ---
        if (recording) {
            trajectory->recordFrame(td.target, est.pose, applied, sub_cmd, obs,
                                    td.curved_forward, td.curved_turn,
                                    controller->lastSpatialError(),
                                    controller->lastAngularError());
        }

        // --- 7. SYNCHRONIZE ---
        pros::Task::delay_until(&timestamp, LOOP_MS);
    }
}

// ============================ PROS ENTRY POINTS ===========================
void initialize() {
    drive->initialize();
    controller->init();
    estimator->initialize();
    if (trajectory->getFrameCount() == 0) trajectory->loadFromHeader();

    ui->displayMessage("CALIBRATING...");
    drive->stop();
    drive->calibrateSensors();
    estimator->reset();
    teleop->resetToPose(Pose2d{});
    controller->reset();
    ui->displayMessage("READY");

    // Launch the core pipeline on a background task (runs across all modes).
    g_pipeline = new pros::Task(robot_pipeline);
}

void disabled() {
    g_mode = RobotMode::IDLE;
    drive->stop();
}

void competition_initialize() {}

void autonomous() {
    startReplay();
    // Occupy the autonomous period until the recorded path finishes or aborts.
    while (g_mode == RobotMode::REPLAYING) pros::delay(10);
}

void opcontrol() {
    g_mode = RobotMode::IDLE;
    // The background pipeline drives teleop; just hold the period open.
    while (true) pros::delay(10);
}
