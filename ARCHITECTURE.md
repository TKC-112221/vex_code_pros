# v3 Architecture

v3 is the object-oriented successor to `vex_tankdrive_code_ver.1`. The **effective
control logic is preserved** from v1 — same EKF, cascaded PID, neural feedforward,
ghost-robot teleop, record/replay, and subsystem behavior — but the code is
restructured so that every stage of the dataflow is a polymorphic component behind
an interface.

## The core rule

> Nothing above the drivebase ever computes a left/right voltage or assumes a
> chassis geometry.

The pipeline speaks two drivebase-agnostic currencies:

- **`Twist { vx, vy, omega }`** — a velocity in each chassis direction. Used for
  references/targets. A differential drive simply ignores `vy`.
- **`ChassisCommand { linear, lateral, angular }`** — a generalized effort in each
  chassis direction. The controller emits this; the **drivebase** alone converts it
  to actuators (inverse kinematics) and converts encoders back to a chassis
  observation (forward kinematics).

## Platform

This is a **PROS** project (PROS 4 / V5 kernel). `src/main.cpp` provides the PROS
entry points — `initialize()`, `disabled()`, `competition_initialize()`,
`autonomous()`, `opcontrol()`. `initialize()` sets everything up and launches the
pipeline on a background `pros::Task`; `autonomous()` and `opcontrol()` just
select the `RobotMode` the running pipeline acts on. See `PROS_SETUP.md` to build.

## The pipeline (one 10 ms tick, `src/main.cpp`)

```
sense ─▶ estimate ─▶ UI/state ─▶ target ─▶ control ─▶ actuate ─▶ record
```

Each arrow is an interface, each box a swappable strategy:

| Stage           | Interface             | Default impl            | Alternatives provided        |
|-----------------|-----------------------|-------------------------|------------------------------|
| Chassis         | `IDriveBase`          | `TankDrive`             | `XDrive`, `ParenthesisDrive` |
| Estimator       | `IEstimator`          | `OdometryEstimator`     | `KalmanEstimator`            |
| Controller      | `IController`         | `CascadedController`    | —                            |
| Stick curve     | `IDriveCurve`         | `CubicDriveCurve`       | `LinearDriveCurve`           |
| Signal filter   | `IInputFilter`        | `LowPassFilter` (EMA)   | `SlewLimiter`                |
| Trajectory      | `ITrajectoryManager`  | `LogFrameReplayer`      | —                            |
| Teleop source   | `ITeleopSource`       | `TeleopTargetGenerator` | —                            |
| Macros/subsys   | `IMacroController`    | `MacroController`       | —                            |
| Operator UI     | `IUIManager`          | `UIManager`             | —                            |
| Command clamp   | `IMotionLimiter`      | `StaticMotionLimiter`   | `PhysicsMotionLimiter`       |
| Dynamics model  | `IDynamicsModel`      | `NeuralNetDynamics`     | —                            |
| Feedforward     | `IFeedForward`        | `NeuralFeedforward`     | `VASController`              |

**Every** pipeline stage — including teleop drive-intent, button macros /
subsystems, and the operator UI — now sits behind a mother interface, so any
stage can be replaced without touching the others. (The pure-math helpers
`Odometry` and `ExtendedKalmanFilter` remain concrete utilities used *inside*
stages, not swappable stages themselves.)

## Swapping a stage

Everything is wired in one block at the top of `src/main.cpp`. Changing the
chassis is a single line:

```cpp
TankDrive        g_hardware(...);   // 6-motor differential (default)
XDrive           g_hardware(...);   // 4-omni holonomic
ParenthesisDrive g_hardware(...);   // 4-omni + 2 forward side wheels
```

No other file changes — the estimator, controller, trajectory, and the whole
pipeline are already blind to how the wheels are arranged. Likewise
`OdometryEstimator` → `KalmanEstimator`, or `CubicDriveCurve` → `LinearDriveCurve`.

### Holonomic drives

Both holonomic chassis use the chassis-frame convention `linear` (forward),
`lateral` (left), `angular` (CCW), and keep the sign convention compatible with
`TankDrive`, so the existing (differential) `CascadedController` drives them
correctly in arcade mode (it commands `lateral = 0`).

- **`XDrive`** — four omni wheels at 45°. IK: `FL = lin−lat−ang`,
  `FR = lin+lat+ang`, `BL = lin+lat−ang`, `BR = lin−lat+ang`. Forward/inverse
  kinematics carry the `√2` factor of a 45° roller. With `lateral = 0` the wheel
  pairs collapse exactly onto the tank left/right mix. `Odometry` was extended to
  integrate `strafe_delta` (0 for tank, so tank is unchanged) so X-drive
  dead-reckoning is correct even while strafing.
- **`ParenthesisDrive`** — an `XDrive` (it inherits from it) plus two
  forward-facing traction wheels on the sides: `SideL = lin−ang`,
  `SideR = lin+ang` (no lateral term). The side wheels add forward + turning
  torque and resist lateral slip — an *enforced linear directional torque* — and
  double as clean forward tracking wheels (odometry takes forward distance from
  them, strafe from the corner omnis).

Full holonomic *strafing under driver control* additionally needs a holonomic
controller and a joystick strafe axis (the drive classes already expose
`JoystickData::strafe` and honor `ChassisCommand::lateral`); the supplied
`CascadedController` is differential and leaves `lateral` at 0.

## Command clamp (motion limiter)

The velocity/acceleration ceilings that clamp commanded motion (stick scaling and
slew in `TeleopTargetGenerator`; the accel clamp and position-PID velocity clamp
in `CascadedController`) are supplied by an injected **`IMotionLimiter`** rather
than read from constants directly.

- **`StaticMotionLimiter`** (default) returns the fixed tuned ceilings
  (`Robot::MAX_VEL_*`, `Physics::MAX_ACCEL_*`), so the out-of-the-box build is
  byte-for-byte identical to the hardcoded limits.
- **`PhysicsMotionLimiter`** predicts the *feasible* ceiling **at the current
  operating point** by inverting an `IDynamicsModel` against the available bus
  voltage: it finds the largest accel `a` with `|volts(v_now, a)| ≤ V_available`
  (and likewise for velocity). As the robot speeds up (back-EMF eats headroom) or
  the battery sags, the predicted accel/vel ceilings drop. Inversion is a bounded
  bisection; the model is `NeuralNetDynamics` (the same learned MLP the controller
  uses for feedforward), itself behind the swappable `IDynamicsModel` interface.

Swap it in `main.cpp`'s wiring block:

```cpp
NeuralNetDynamics    g_dynamics(g_feedforward);
PhysicsMotionLimiter g_limiter(g_dynamics);   // replaces StaticMotionLimiter
// g_controller and g_teleop already take g_limiter by reference
```

Because it re-evaluates the MLP each tick (a bounded bisection per query), the
physics limiter costs more CPU than the static one — reasonable on V5, but it is
opt-in for that reason.

## Directory layout

```
include/
  core/        Types, LogFrame, Specs, Gains, Limits          (vocabulary + constants)
  config/      Ports, Settings                                (wiring + feature flags)
  interfaces/  I* abstract superclasses
  hardware/    TankDrive, XDrive, ParenthesisDrive
  estimators/  Odometry, OdometryEstimator, KalmanEstimator
  control/     CascadedController, NeuralFeedforward, VASController
  filters/     LowPassFilter, SlewLimiter
  curves/      CubicDriveCurve, LinearDriveCurve
  logic/       TeleopTargetGenerator, MacroController, UIManager
  trajectory/  LogFrameReplayer
  math/        MathUtils, ExtendedKalmanFilter, NeuralNetDynamics
  data/        auto-generated MLP weights, recorded paths, dynamics limits (reused from v1)
src/           matching .cpp for the non-trivial classes + main.cpp
```

## Behavior-preserving liberties

The logic matches v1; these are the deliberate structural changes ("same behavior
with some liberty"):

1. **Odometry runs in the main loop**, not a separate RTOS task. One 10 ms tick
   integrates odometry and control together, removing v1's `odom_lock` mutex.
2. **Per-actuator safety moved to the drivebase.** v1 mixed to left/right inside
   the controller and then applied stick-zero brake, back-EMF current limit, and
   the ratio-preserving voltage clamp. Those are now in `TankDrive` (they are
   inherently per-wheel), leaving the controller purely chassis-frame. The active
   config (`VOLTAGE_CLAMP` on, others off) is byte-for-byte equivalent.
3. **The EKF stays inside `CascadedController`**, exactly as v1, because it smooths
   the command derivative used for feedforward and the PID D-term. `IEstimator`
   therefore owns *pose* estimation; `KalmanEstimator` shows the EKF can also drive
   that stage.
4. **Replay reference effort is reconstructed in chassis frame.** Recorded per-wheel
   volts become `(linear, angular)` effort; the disabled wheel-position corrections
   fold in as chassis additions. Exact for the active `REPLAY_VOLTAGE_FALLBACK` path.
5. **Constants are scoped** (`Robot::`, `PidGains::`, `Settings::` …); v1's legacy
   flat aliases are dropped.

Minor, documented in-code: pneumatics logging has a one-tick phase relative to v1's
read-before-toggle order; controller tracking diagnostics are stale during a pivot
override (the controller isn't run). Both affect only logged diagnostics under rare
input combinations, never the drive output.
