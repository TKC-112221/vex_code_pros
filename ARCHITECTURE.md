# v3 Architecture

v3 is an object-oriented PROS robot stack. Stages of the control dataflow sit
behind interfaces so the chassis, estimator, stick mapping, and feedforward can
be swapped from one wiring block in `src/main.cpp`.

## The core rule

> Nothing above the drivebase ever computes a left/right voltage or assumes a
> chassis geometry.

The pipeline speaks drivebase-agnostic currencies:

- **`Twist { vx, vy, omega }`** — chassis-frame velocity. Used in motion
  targets. A differential drive ignores `vy`.
- **`CommandVector { surge, sway, yaw }`** — teleop / recording intent on the
  same axes (forward, left, CCW). Converts to `ChassisCommand` at actuation.
- **`ChassisCommand { linear, lateral, angular }`** — generalized chassis
  effort. Controllers and mappers emit this; the **drivebase** alone converts
  it to actuators (inverse kinematics) and converts encoders/IMU back to a
  `ChassisObservation` (forward kinematics).

## Platform

This is a **PROS** project (PROS 4 / V5 kernel). `src/main.cpp` provides the
entry points — `initialize()`, `disabled()`, `competition_initialize()`,
`autonomous()`, `opcontrol()`.

- `initialize()` wires components, starts IMU calibration on a background task,
  launches the **pipeline** and **localization** tasks, and optionally plays the
  splash clip (`platform/FramePlayer`).
- `autonomous()` starts recorded-path replay and waits until it finishes.
- `opcontrol()` only holds the competition mode; the pipeline task already owns
  teleop / record / replay.

### Host simulation

`make host` builds a desktop binary with `-DROBOT_HOST=1`. That link uses
`MockDrive` (simulated differential plant) and `host/FieldWindow` instead of
the physical chassis classes. The same pipeline and estimators run unchanged;
`IDriveBase::service()` advances the plant and redraws the field view.

## Two tasks

Work is split across two `pros::Task`s started from `initialize()`:

| Task | Period | Responsibility |
|------|--------|----------------|
| `robot_pipeline` | `System::LOOP_TIME_MS` (10 ms) | Sense, UI/mode, teleop or replay command path, actuate, record |
| `localization_task` | 5 ms | Consume each pending `SensorDict` exactly once via `IEstimator::update` |

Linear travel for localization is the **inertial accelerometer** integral
(`ImuLinearDelta`), applied in the pipeline before the dict is published. If a
sample is still pending when the next tick publishes, forward/strafe deltas are
**accumulated** so no travel step is dropped. Pose/velocity snapshots used by
the pipeline are mutex-protected reads of the estimator.

Sticks and buttons stay dropped until IMU calibration finishes and the pipeline
commits sensor calibration (`g_input_enabled`). The splash clip does not gate
that unlock.

## The pipeline (one control tick)

```
sense ─▶ UI / mode ─▶ [ teleop path | replay path ] ─▶ service / sleep
```

### Sense

`readRobotInput()` builds one `RobotReading` — `{ SensorDict, joystick, buttons }` —
from the drivebase. Noisy channels are denoised (Savitzky–Golay or 3-sample
Kalman per `Settings::ControlToggles::SAVITZKY_GOLAY_FILTER`); wheel deltas stay
raw. Distance beams `D1..D4` live in `SensorDict::distance[]`.

### Teleop / record path

```
curve sticks ─▶ velocity map ─▶ user-input override ─▶ command processor
            ─▶ subsystems ─▶ applyDrive ─▶ (optional) recordFrame
```

| Stage | Interface / hook | Default impl | Notes |
|-------|------------------|--------------|-------|
| Stick curve | `IDriveCurve` | `CubicDriveCurve` | `LinearDriveCurve` available |
| Stick → command | `IVelocityMapper` | `TankDriveVelocityMapper` | Relative arcade: `surge`/`yaw` as open-loop volts; `sway = 0` |
| Override | free function | `userInputOverride` → `DriftCurveOverride` | Optional; passes through when idle |
| Mid-pipeline filter | `ICommandProcessor` | `PassthroughController` | Hook for ghost/assists without touching neighbors |
| Macros / subsystems | `IMacroController` + `controlSubsystems` | `MacroController` | Intake, rollers, pneumatics |
| Chassis | `IDriveBase` | `TankDrive` (brain) / `MockDrive` (host) | Also `XDrive`, `ParenthesisDrive` |
| Record | `ITrajectoryManager` | `LogFrameReplayer` | Logs estimator pose, `CommandVector`, applied effort, sensors |

Teleop does **not** run `CascadedController`. The mapped / overridden command
vector is converted to a `ChassisCommand` and sent straight to the drivebase.

### Replay path

```
trajectory.getNextTarget ─▶ CascadedController ─▶ subsystems ─▶ applyDrive
```

Replay still uses the cascaded follower (`IController`) against a
`MotionTarget` from `LogFrameReplayer` (pose/velocity refs plus optional
recorded chassis effort / voltage passthrough per settings toggles).

### Operator UI

`IUIManager` / `UIManager` owns mode toggles (idle / record / replay), save /
load / calibrate / auton-test buttons, and brain display updates.

## Wiring (swap a stage here)

Concrete types are named only at the top of `src/main.cpp`. The rest of the
tick talks through interface pointers.

```cpp
VASController         g_feedforward(...);   // or NeuralFeedforward
TankDrive             g_hardware(...);      // or XDrive / ParenthesisDrive / MockDrive
OdometryEstimator     g_estimator;          // or SlipEnsembleEstimator / ParticleFilterEstimator
StaticMotionLimiter   g_limiter;
CascadedController    g_replay_controller(g_feedforward, g_limiter);
PassthroughController g_command_processor;
TankDriveVelocityMapper g_velocity_mapper(...);
CubicDriveCurve       g_curve(...);
```

### Holonomic drives

`XDrive` and `ParenthesisDrive` keep the chassis-frame convention
`linear` / `lateral` / `angular` with the same sign sense as `TankDrive`. With
`lateral`/`sway` at 0 they collapse onto a tank left/right mix. Full driver
strafing needs a mapper (and usually a processor) that commands lateral effort;
the supplied `TankDriveVelocityMapper` does not.

- **`XDrive`** — four omnis at 45°. IK: `FL = lin−lat−ang`, `FR = lin+lat+ang`,
  `BL = lin+lat−ang`, `BR = lin−lat+ang` (with the usual `√2` roller factor).
- **`ParenthesisDrive`** — `XDrive` plus two forward side wheels
  (`SideL = lin−ang`, `SideR = lin+ang`) for traction and forward tracking.

## Localization

`IEstimator` consumes a `SensorDict` (chassis deltas + optional distance beams),
never raw motors.

| Impl | Role |
|------|------|
| `OdometryEstimator` (default) | Trapezoidal wheel/IMU odometry + damped wall nudge from distance sensors |
| `SlipEnsembleEstimator` | Ensemble of slip hypotheses; distance nudges periodically |
| `ParticleFilterEstimator` | MCL; distance beams reweight particles; optional absolute fixes |
| `KalmanEstimator` | Present in tree; not currently wired in `main.cpp` |

Pure-math helpers (`Odometry`, `ParticleFilter`, `SlipEnsemble`,
`ExtendedKalmanFilter`) live under the estimator/math layers and are not
pipeline stages themselves. The EKF inside `CascadedController` is separate:
it smooths command derivatives for replay feedforward / PID D-term, not field
pose.

## Feedforward and motion limits

- **`IFeedForward`** — `VASController` (default wiring) or `NeuralFeedforward`
  (learned MLP). Used by replay control, drivebase current-limit helpers, and
  overrides that need a dynamics model (e.g. drift curve).
- **`IMotionLimiter`** — `StaticMotionLimiter` (default) or
  `PhysicsMotionLimiter` (inverts an `IDynamicsModel` / `NeuralNetDynamics`
  against available voltage). Injected into `CascadedController` for replay
  clamps; teleop stick mapping does not go through it today.

## Recording schema

`LogFrame` stores chassis-frame data only: estimator pose, measured rates
(including inertial velocity), command intent (`cmd_surge/sway/yaw`), applied
effort, encoder banks, electrical/thermal, and raw IMU. Wheel mixes stay inside
the concrete drivebase.

## Directory layout

```
include/
  core/         Types, LogFrame, Specs, Gains, Limits
  config/       Ports, Settings (feature flags, splash)
  interfaces/   I* abstract contracts
  hardware/     TankDrive, XDrive, ParenthesisDrive, MockDrive, DistanceBank
  estimators/   Odometry*, Kalman*, ParticleFilter*, SlipEnsemble*, ImuLinearDelta
  control/      CascadedController, PassthroughController, Drift*, feedforward, limiters
  filters/      LowPassFilter, SlewLimiter
  curves/       CubicDriveCurve, LinearDriveCurve
  logic/        SensorReader, UIManager, MacroController, SubsystemControl,
                TankDriveVelocityMapper, UserInputOverride, DriftCurveOverride, …
  trajectory/   LogFrameReplayer
  math/         MathUtils, EKF, NeuralNet*, sensor denoisers, WallRange, …
  field/        FieldPoints
  platform/     FramePlayer, PngImage, RobotFile
  host/         FieldWindow, DeviceScreens (desktop)
  data/         generated MLP weights / limits (when used)
src/            matching .cpp + main.cpp
```

## Legacy / unused-in-wiring pieces

These still compile and document an older teleop shape, but **`main.cpp` does
not construct them** in the active path:

- `ITeleopSource` / `TeleopTargetGenerator` — ghost-robot teleop that produced
  a `MotionTarget` for `CascadedController`
- `IInputFilter` consumers in that path (`LowPassFilter`, `SlewLimiter` on the
  teleop generator)

The current teleop path replaces that with curve → `IVelocityMapper` →
override → `ICommandProcessor` → drivebase. Replay keeps the cascaded
controller.

## Structural notes (vs. earlier monolithic loop)

1. **Localization is asynchronous** — a dedicated task; the control tick only
   publishes sensors and snapshots pose.
2. **Per-actuator safety lives in the drivebase** — voltage clamp, optional
   current limit / stick-zero brake / battery scaling are applied inside
   `applyDrive`, so controllers stay chassis-frame.
3. **Teleop is open-loop chassis effort by default** — feedback following is
   replay-only unless a richer `ICommandProcessor` is swapped in.
4. **Constants are namespaced** (`Robot::`, `PidGains::`, `Settings::`, …).
)
