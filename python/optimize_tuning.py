"""Fit the MockDrive plant, then CMA-ES the controller knobs it can excite.

Stage 1 replays a LogFrame recording through the C++ plant (python/plant.pyd,
the same MockDrive the path-follower env steps). It fits the velocity lag,
the skid lag, the speed scale, and, when the log has current, the side
resistance. Short windows are restarted on the logged state so open-loop
drift does not dominate. Wheel speed is the measurement. Pose is not.

Stage 2 searches include/core/Tuning.h section by section on that plant.
The order is the dependency order: armature feedforward, then the accel
the planner is allowed to use, then the slip model, then heading trim,
then lateral trim, then the turn-radius clamp. Each section starts from
the best point so far. The hand-tuned value is scored first and is kept
whenever a generation does not beat it.

Parameters the closed-loop rollout never reads are copied through and
listed as held. Searching them would move numbers that do not change the
cost. The result is tuning_paste.txt in the current directory. Tuning.h
is not modified.

Rebuild the plant after C++ changes, from this repo:

    py -3 python/build_plant.py
    py -3 python/optimize_tuning.py --log path/to/skills_auto.bin
    py -3 python/optimize_tuning.py --quick
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plant


# Fixed poses, inches and radians. Hermite is the follower's shape.
# Exit speed is m/s. Curved goals excite slip, heading, and the radius cap.
GOALS = (
    (48.0, 0.0, 0.0, 0.0, "Hermite"),
    (36.0, 24.0, 1.2, 0.0, "Hermite"),
    (24.0, -18.0, -0.6, 0.0, "Hermite"),
    (30.0, 10.0, 0.4, 0.4, "Hermite"),
)

# A candidate that leaves the field or goes non-finite. Still grows with
# distance so CMA-ES has a slope back toward the field.
FAIL = 1.0e5


def cpp_num(value):
    text = f"{float(value):.8g}"
    if "e" not in text and "." not in text:
        text += ".0"
    return text


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def nominal():
    """Compiled Tuning.h / MockDriveParams.h values exported by the plant."""
    return {
        "VELOCITY_TAU": float(plant.VELOCITY_TAU),
        "SKID_TAU": float(plant.SKID_TAU),
        "SIDE_RESISTANCE": float(plant.SIDE_RESISTANCE),
        "SPEED_SCALE": float(plant.SPEED_SCALE),
        "KS_LIN": float(plant.KS_LIN),
        "LIN_SPEED_AT_SAT": float(plant.LIN_SPEED_AT_SAT),
        "LIN_ACCEL_AT_FULL_V": float(plant.LIN_ACCEL_AT_FULL_V),
        "KS_ANG": float(plant.KS_ANG),
        "ANG_RATE_PER_VOLT": float(plant.ANG_RATE_PER_VOLT),
        "ANG_LEVER_M": float(plant.ANG_LEVER_M),
        "ANG_KA_COEFF": float(plant.ANG_KA_COEFF),
        "ANG_KA_LENGTH_M": float(plant.ANG_KA_LENGTH_M),
        "PATH_ACCEL": float(plant.PATH_ACCEL),
        "MIN_ACCEL": float(plant.MIN_ACCEL),
        "PRACTICE_ACCEL": float(plant.PRACTICE_ACCEL),
        "K_SLIP": float(plant.K_SLIP),
        "A_MAX": float(plant.A_MAX),
        "MAX_SLIP_RAD": float(plant.MAX_SLIP_RAD),
        "CLOSE_IN": float(plant.CLOSE_IN),
        "HEAD_KP": float(plant.HEAD_KP),
        "HEAD_KI": float(plant.HEAD_KI),
        "HEAD_KD": float(plant.HEAD_KD),
        "HEAD_WINDUP": float(plant.HEAD_WINDUP),
        "LAT_KP": float(plant.LAT_KP),
        "LAT_KI": float(plant.LAT_KI),
        "LAT_KD": float(plant.LAT_KD),
        "LAT_KD2": float(plant.LAT_KD2),
        "LAT_I_LIMIT": float(plant.LAT_I_LIMIT),
        "TURN_RADIUS_MAX": float(plant.TURN_RADIUS_MAX),
        "KAPPA_EPS": float(plant.KAPPA_EPS),
    }


# Physically plausible boxes. Heterogeneous units are mapped to [-1, 1]
# before CMA-ES, so one step size covers every knob. A zero nominal uses
# an absolute box; a multiplicative box around zero cannot move.
BOUNDS = {
    "VELOCITY_TAU": (0.03, 0.60),
    "SKID_TAU": (0.05, 1.20),
    "SIDE_RESISTANCE": (0.40, 6.00),
    "SPEED_SCALE": (0.50, 1.50),
    "KS_LIN": (0.0, 1.5),
    "LIN_SPEED_AT_SAT": (1.0, 8.0),
    "LIN_ACCEL_AT_FULL_V": (1.0, 15.0),
    "KS_ANG": (0.0, 1.5),
    "ANG_RATE_PER_VOLT": (1.0, 12.0),
    "ANG_LEVER_M": (0.15, 0.60),
    "ANG_KA_COEFF": (0.4, 8.0),
    "ANG_KA_LENGTH_M": (0.05, 0.40),
    "PATH_ACCEL": (0.4, 8.0),
    "MIN_ACCEL": (0.02, 0.50),
    "PRACTICE_ACCEL": (1.0, 12.0),
    "K_SLIP": (0.0, 10.0),
    "A_MAX": (1.0, 16.0),
    "MAX_SLIP_RAD": (0.30, math.pi / 2.0),
    "CLOSE_IN": (2.0, 16.0),
    "HEAD_KP": (0.0, 8.0),
    "HEAD_KI": (0.0, 2.0),
    "HEAD_KD": (0.0, 4.0),
    "HEAD_WINDUP": (0.0, 20.0),
    "LAT_KP": (0.0, 20.0),
    "LAT_KI": (0.0, 10.0),
    "LAT_KD": (0.0, 5.0),
    "LAT_KD2": (0.0, 2.0),
    "LAT_I_LIMIT": (0.0, 6.0),
    "TURN_RADIUS_MAX": (2.0, 20.0),
    "KAPPA_EPS": (1.0e-4, 5.0e-3),
}

# Later sections read earlier ones. Feedforward turns planned speed into
# volts. Accel limits set that speed. Slip and the PIDs then trim it.
SECTIONS = (
    ("feedforward", (
        "KS_LIN", "LIN_SPEED_AT_SAT", "LIN_ACCEL_AT_FULL_V",
        "KS_ANG", "ANG_RATE_PER_VOLT", "ANG_LEVER_M",
        "ANG_KA_COEFF", "ANG_KA_LENGTH_M",
    )),
    ("motion", ("PATH_ACCEL", "MIN_ACCEL", "PRACTICE_ACCEL")),
    ("drift", ("K_SLIP", "A_MAX", "MAX_SLIP_RAD", "CLOSE_IN")),
    ("heading", ("HEAD_KP", "HEAD_KI", "HEAD_KD", "HEAD_WINDUP")),
    ("lateral", ("LAT_KP", "LAT_KI", "LAT_KD", "LAT_KD2", "LAT_I_LIMIT")),
    ("geometry", ("TURN_RADIUS_MAX", "KAPPA_EPS")),
)


def require_plant():
    missing = []
    if not hasattr(plant.PlantSim, "set_dynamics"):
        missing.append("PlantSim.set_dynamics")
    if not hasattr(plant.DriftGuide, "configure"):
        missing.append("DriftGuide.configure")
    if not hasattr(plant, "read_log"):
        missing.append("read_log")
    if missing:
        raise SystemExit(
            "python/plant.pyd is older than this tuner (missing "
            + ", ".join(missing)
            + "). Rebuild it with: py -3 python/build_plant.py"
        )
    for key, (lo, hi) in BOUNDS.items():
        value = nominal()[key]
        if value < lo or value > hi:
            raise SystemExit(f"{key} compiled value {value} is outside [{lo}, {hi}]")


def feedforward(values):
    """Same formulas as Feedforward:: in include/core/Tuning.h."""
    volts = float(plant.MAX_VOLTS)
    motors = float(plant.ANG_KA_MOTOR_COUNT)
    ks_lin = values["KS_LIN"]
    kv_lin = (volts - ks_lin) / values["LIN_SPEED_AT_SAT"]
    ka_lin = volts / values["LIN_ACCEL_AT_FULL_V"]
    ks_ang = values["KS_ANG"]
    kv_ang = values["ANG_RATE_PER_VOLT"] * values["ANG_LEVER_M"] / 2.0
    ka_ang = (values["ANG_KA_COEFF"] * (2.0 * values["ANG_KA_LENGTH_M"])
              / (motors * values["ANG_LEVER_M"]))
    return ks_lin, kv_lin, ka_lin, ks_ang, kv_ang, ka_ang


def configure(guide, values, practice_accel):
    ks_lin, kv_lin, ka_lin, ks_ang, kv_ang, ka_ang = feedforward(values)
    guide.configure(
        ks_lin, kv_lin, ka_lin, ks_ang, kv_ang, ka_ang,
        float(plant.EXP_RATE),
        values["CLOSE_IN"] * float(plant.METERS_PER_INCH),
        float(plant.DRIFT_ACCEL),
        values["MAX_SLIP_RAD"],
        values["PATH_ACCEL"],
        values["K_SLIP"],
        values["A_MAX"],
        values["LAT_KP"], values["LAT_KI"], values["LAT_KD"],
        values["LAT_KD2"], values["LAT_I_LIMIT"],
        values["HEAD_KP"], values["HEAD_KI"], values["HEAD_KD"], values["HEAD_WINDUP"],
        float(plant.SETTLE_SPEED),
        values["MIN_ACCEL"],
        values["TURN_RADIUS_MAX"],
        values["KAPPA_EPS"],
        float(plant.SETTLE_IN),
        float(plant.SETTLE_DEG),
        values["PRACTICE_ACCEL"] if practice_accel else -1.0,
    )


def track_cost(guide, sim, values, ticks):
    """Mean tracking cost. Both accel sources are rolled so each is visible."""
    total = 0.0
    count = 0
    for practice in (True, False):
        configure(guide, values, practice)
        for tx, ty, heading, exit_speed, shape in GOALS:
            guide.begin(tx, ty, heading, exit_speed, False, False, shape)
            sim.reset(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            acc = 0.0
            failed = False
            state = sim.state()
            for i in range(ticks):
                if not all(math.isfinite(v) for v in (
                    state.x_in, state.y_in, state.theta, state.v_lin, state.v_ang
                )):
                    failed = True
                    break
                if abs(state.x_in) > 90.0 or abs(state.y_in) > 90.0:
                    failed = True
                    break
                surge, sway, yaw = guide.command(
                    state.x_in, state.y_in, state.theta, state.v_lin, state.v_ang)
                dx = (state.x_in - tx) * float(plant.METERS_PER_INCH)
                dy = (state.y_in - ty) * float(plant.METERS_PER_INCH)
                along = dx * math.cos(heading) + dy * math.sin(heading)
                cross = -dx * math.sin(heading) + dy * math.cos(heading)
                head = wrap(heading - state.theta)
                weight = (i + 1) / ticks
                acc += weight * (cross * cross + 0.25 * along * along + 0.15 * head * head)
                volts = float(plant.MAX_VOLTS)
                acc += 0.002 * (surge * surge + yaw * yaw) / (volts * volts)
                state = sim.step(surge, sway, yaw)
            if failed or not math.isfinite(acc):
                dist = math.hypot(state.x_in - tx, state.y_in - ty)
                total += FAIL + 10.0 * dist
            else:
                dx = (state.x_in - tx) * float(plant.METERS_PER_INCH)
                dy = (state.y_in - ty) * float(plant.METERS_PER_INCH)
                head = wrap(heading - state.theta)
                speed = state.v_lin - exit_speed
                acc += 8.0 * (dx * dx + dy * dy) + 4.0 * head * head + 2.0 * speed * speed
                total += acc / ticks
            count += 1
    return total / count


def load_log(path):
    rows = plant.read_log(str(path))
    if len(rows) < 8:
        raise SystemExit(f"{path} has {len(rows)} frames. Need a LogFrame recording.")
    times = [float(row["time"]) for row in rows[:8]]
    if times != sorted(times) or times[-1] <= times[0]:
        raise SystemExit(
            f"{path} times do not increase. The file is not a LogFrame from this layout "
            f"({int(plant.LOG_FRAME_BYTES)} bytes per frame)."
        )
    return rows


def plant_cost(sim, rows, values, window, windows, use_current):
    n = len(rows)
    if n <= window:
        starts = [0]
        span = n
    else:
        span = window
        last = n - span
        starts = [int(round(i * last / max(windows - 1, 1))) for i in range(windows)]
    total = 0.0
    samples = 0
    sim.set_dynamics(
        values["VELOCITY_TAU"], values["SKID_TAU"],
        values["SIDE_RESISTANCE"], values["SPEED_SCALE"])
    for start in starts:
        frame = rows[start]
        sim.reset(frame["x_in"], frame["y_in"], frame["theta"],
                  frame["v_lin"], frame["v_lat"], frame["v_ang"])
        for k in range(span):
            frame = rows[start + k]
            state = sim.step(frame["applied_linear"], frame["applied_lateral"],
                             frame["applied_angular"])
            if not math.isfinite(state.v_lin):
                return FAIL
            err = (state.v_lin - frame["v_lin"]) ** 2
            err += 0.15 * (state.v_ang - frame["v_ang"]) ** 2
            err += 0.25 * (state.v_lat - frame["v_lat"]) ** 2
            if use_current:
                err += 0.02 * ((state.left_amps - frame["left_amps"]) ** 2
                               + (state.right_amps - frame["right_amps"]) ** 2)
            total += err
            samples += 1
    return total / max(samples, 1)


def log_is_excited(rows):
    linear = np.array([float(row["applied_linear"]) for row in rows])
    angular = np.array([float(row["applied_angular"]) for row in rows])
    return float(np.std(linear)) > 0.05 or float(np.std(angular)) > 0.05


def log_has_current(rows):
    amps = [max(abs(float(row["left_amps"])), abs(float(row["right_amps"]))) for row in rows]
    return max(amps) > 0.05


def to_unit(raw, lo, hi):
    return 2.0 * (raw - lo) / (hi - lo) - 1.0


def from_unit(unit, lo, hi):
    unit = float(np.clip(unit, -1.0, 1.0))
    return lo + (unit + 1.0) * 0.5 * (hi - lo)


def cma_minimize(keys, values, cost, generations, seed):
    """CMA-ES in [-1, 1]. The incoming values are retained if nothing beats them."""
    import cma

    lo = np.array([BOUNDS[key][0] for key in keys])
    hi = np.array([BOUNDS[key][1] for key in keys])
    start = np.array([values[key] for key in keys], dtype=float)
    # A nominal sitting on the bound has no room to sample. Shift only the
    # initial mean. The scored elite is still the real nominal.
    span = hi - lo
    mean = np.clip(start, lo + 0.05 * span, hi - 0.05 * span)
    x0 = [to_unit(value, lo[i], hi[i]) for i, value in enumerate(mean)]

    def objective(unit):
        trial = dict(values)
        for i, key in enumerate(keys):
            trial[key] = from_unit(unit[i], lo[i], hi[i])
        score = float(cost(trial))
        if not math.isfinite(score):
            return FAIL
        return score

    elite_score = objective([to_unit(value, lo[i], hi[i]) for i, value in enumerate(start)])
    elite = start.copy()
    print(f"  start {elite_score:.6g}")

    options = {
        "bounds": [-1.0, 1.0],
        "seed": seed,
        "maxiter": generations,
        "verb_disp": 0,
        "verb_log": 0,
        "CMA_elitist": True,
        "tolfun": 1e-4,
        "tolx": 1e-3,
    }
    strategy = cma.CMAEvolutionStrategy(x0, 0.3, options)
    generation = 0
    while not strategy.stop():
        generation += 1
        points = strategy.ask()
        scores = [objective(point) for point in points]
        strategy.tell(points, scores)
        best_i = int(np.argmin(scores))
        if scores[best_i] < elite_score:
            elite_score = scores[best_i]
            for i, key in enumerate(keys):
                elite[i] = from_unit(points[best_i][i], lo[i], hi[i])
        print(f"  gen {generation:3d}  best {elite_score:.6g}  pop {min(scores):.6g}")
        if generation >= generations:
            break

    updated = dict(values)
    for i, key in enumerate(keys):
        updated[key] = float(elite[i])
    return updated, elite_score


def write_paste(path, values, notes, held_reason):
    ks_lin, kv_lin, ka_lin, ks_ang, kv_ang, ka_ang = feedforward(values)
    lateral = plant.LATERAL_AXIS
    angular = (
        values["HEAD_KP"], values["HEAD_KI"], values["HEAD_KD"], values["HEAD_WINDUP"],
        plant.ANGULAR_AXIS[4], plant.ANGULAR_AXIS[5], plant.ANGULAR_AXIS[6],
        plant.ANGULAR_AXIS[7], plant.ANGULAR_AXIS[8],
    )

    def axis(fields):
        return ", ".join(cpp_num(field) for field in fields)

    lines = [
        "// Written by python/optimize_tuning.py. Copy each assignment over the",
        "// matching line. Do not append this file to a header.",
        "//",
        "// KV_LIN, KA_LIN, KV_ANG, and KA_ANG are formulas in Tuning.h. They",
        "// update when the knobs below change. With these knobs they evaluate to",
        f"//   KV_LIN {cpp_num(kv_lin)}   KA_LIN {cpp_num(ka_lin)}",
        f"//   KV_ANG {cpp_num(kv_ang)}   KA_ANG {cpp_num(ka_ang)}",
        "//",
        "// Physics::TURN_RADIUS_MAX_M aliases data/generated_limits.h when that",
        "// header is on the include path. Replace the alias with the literal",
        "// below, in both branches, or edit generated_limits.h. Editing only the",
        "// #else fallback does nothing while the generated header exists.",
        "//",
        "// Boomerang::CLOSE_IN is also DrivePolicy::kPositionToleranceIn.",
        "//",
    ]
    for note in notes:
        lines.append("// " + note)
    lines += [
        "",
        "// include/hardware/MockDriveParams.h",
        f"constexpr double VELOCITY_TAU_SEC = {cpp_num(values['VELOCITY_TAU'])};",
        f"constexpr double SKID_TAU_SEC = {cpp_num(values['SKID_TAU'])};",
        f"constexpr double SIDE_RESISTANCE_OHM = {cpp_num(values['SIDE_RESISTANCE'])};",
        f"constexpr double SPEED_SCALE = {cpp_num(values['SPEED_SCALE'])};",
        "",
        "// include/core/Tuning.h  namespace Feedforward",
        f"constexpr double KS_LIN = {cpp_num(values['KS_LIN'])};",
        f"constexpr double KS_ANG = {cpp_num(values['KS_ANG'])};",
        f"constexpr double LIN_SPEED_AT_SAT = {cpp_num(values['LIN_SPEED_AT_SAT'])};",
        f"constexpr double LIN_ACCEL_AT_FULL_V = {cpp_num(values['LIN_ACCEL_AT_FULL_V'])};",
        f"constexpr double ANG_RATE_PER_VOLT = {cpp_num(values['ANG_RATE_PER_VOLT'])};",
        f"constexpr double ANG_LEVER_M = {cpp_num(values['ANG_LEVER_M'])};",
        f"constexpr double ANG_KA_COEFF = {cpp_num(values['ANG_KA_COEFF'])};",
        f"constexpr double ANG_KA_LENGTH_M = {cpp_num(values['ANG_KA_LENGTH_M'])};",
        "",
        "// include/core/Tuning.h  namespace Drift",
        f"constexpr double PATH_ACCEL = {cpp_num(values['PATH_ACCEL'])};",
        f"constexpr double K_SLIP = {cpp_num(values['K_SLIP'])};",
        f"constexpr double A_MAX = {cpp_num(values['A_MAX'])};",
        f"constexpr double MAX_SLIP_RAD = {cpp_num(values['MAX_SLIP_RAD'])};",
        f"constexpr double LAT_KP = {cpp_num(values['LAT_KP'])};",
        f"constexpr double LAT_KI = {cpp_num(values['LAT_KI'])};",
        f"constexpr double LAT_KD = {cpp_num(values['LAT_KD'])};",
        f"constexpr double LAT_KD2 = {cpp_num(values['LAT_KD2'])};",
        f"constexpr double LAT_I_LIMIT = {cpp_num(values['LAT_I_LIMIT'])};",
        "",
        "// include/core/Tuning.h  namespace Motion",
        f"constexpr double MIN_ACCEL_MPS2 = {cpp_num(values['MIN_ACCEL'])};",
        f"constexpr double KAPPA_EPS = {cpp_num(values['KAPPA_EPS'])};",
        "",
        "// include/core/Tuning.h  namespace Practice",
        f"constexpr double MAX_ACCEL_MPS2 = {cpp_num(values['PRACTICE_ACCEL'])};",
        "",
        "// include/core/Tuning.h  namespace Physics  (see the generated-limits note)",
        f"constexpr double TURN_RADIUS_MAX_M = {cpp_num(values['TURN_RADIUS_MAX'])};",
        "",
        "// include/core/Tuning.h  namespace Boomerang",
        f"constexpr Axis Lateral = {{{axis(lateral)}}};",
        f"constexpr Axis Angular = {{{axis(angular)}}};",
        f"constexpr double CLOSE_IN = {cpp_num(values['CLOSE_IN'])};",
        "",
        "// Held. These stay at the compiled values. The rollout below does not",
        "// read them, so a search would only decorate the paste.",
    ]
    for reason in held_reason:
        lines.append("// " + reason)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


HELD = (
    "PidGains, Ramsete, FF_BOOST, SPATIAL_WARP: CascadedController replay, not DriftGuide.",
    "Tuning slew/cube/turn stick, Face, Pivot, Replay wheel KP, Controls deadbands: teleop.",
    "Drift::EXP_RATE: Exponential curves only. DriftFollower's shape is Hermite.",
    "Drift::DRIFT_ACCEL: the atan slip model. DriftFollower uses arcsin (A_MAX).",
    "Drift::PATH_ACCEL is searched. It applies when the target leaves max_accel unset.",
    "Practice::MAX_ACCEL_MPS2 is searched. DriftGuide writes it onto the target.",
    "SETTLE_SPEED, SETTLE_IN, SETTLE_DEG: they only raise the settled flag.",
    "Boomerang Lateral PID, slew, and timing: DriftFollower's lateral loop is Drift::LAT_*.",
    "BOOMERANG_LEAD, BOOMERANG_MAX_SLIP, PROBE_M, CARROT_FLOOR, CURVE_STEPS: other types.",
    "driftpath lookahead and graph limits: DriftCurve, not this follower.",
    "DrivePolicy scales: they belong to the trained network, which this search does not run.",
    "Particles, Slip, RangeLik, Ekf, ImuBias, SensorFilter, WallSolve, WallNudge: estimators.",
    "The guide is scored on ground-truth pose, so estimator gains have no effect here.",
    "ANG_KA_MOTOR_COUNT is the motor count from the fit, not a gain.",
    "MockDrive confidence, range period, and reported temperature do not move the chassis.",
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, help="LogFrame file, e.g. skills_auto.bin")
    parser.add_argument("--quick", action="store_true", help="two generations, short rollouts")
    parser.add_argument("--generations", type=int, default=12)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--stage", choices=("all", "plant", "gains"), default="all")
    parser.add_argument("--out", type=Path, default=Path.cwd() / "tuning_paste.txt")
    args = parser.parse_args()

    try:
        import cma  # noqa: F401
    except ImportError as error:
        raise SystemExit("CMA-ES needs the cma package: py -3 -m pip install cma") from error

    require_plant()
    values = nominal()
    notes = []
    generations = 2 if args.quick else args.generations
    ticks = 80 if args.quick else 200
    windows = 4 if args.quick else 8
    window = 25 if args.quick else 50

    sim = plant.PlantSim()
    guide = plant.DriftGuide()

    if args.stage in ("all", "plant"):
        if args.log is None:
            notes.append(
                "Stage 1 kept the compiled MockDrive plant. Pass --log a LogFrame "
                "recording to fit tau, skid, resistance, and speed scale."
            )
            print(notes[-1])
        else:
            rows = load_log(args.log)
            if not log_is_excited(rows):
                notes.append(
                    f"Stage 1 skipped {args.log}: applied volts barely move, so the "
                    "plant parameters are not observable. Compiled plant kept."
                )
                print(notes[-1])
            else:
                use_current = log_has_current(rows)
                keys = ["VELOCITY_TAU", "SKID_TAU", "SPEED_SCALE"]
                if use_current:
                    keys.append("SIDE_RESISTANCE")
                    notes.append("Stage 1 fit resistance against logged motor current.")
                else:
                    notes.append(
                        "Stage 1 held SIDE_RESISTANCE. The log current is ~0, and with "
                        "the current limit off, resistance does not change the motion."
                    )
                print("stage 1 plant  " + ", ".join(keys))
                values, score = cma_minimize(
                    keys, values,
                    lambda trial: plant_cost(sim, rows, trial, window, windows, use_current),
                    generations, args.seed)
                notes.append(f"Stage 1 cost {score:.6g}.")
                sim.set_dynamics(
                    values["VELOCITY_TAU"], values["SKID_TAU"],
                    values["SIDE_RESISTANCE"], values["SPEED_SCALE"])

    if args.stage in ("all", "gains"):
        sim.set_dynamics(
            values["VELOCITY_TAU"], values["SKID_TAU"],
            values["SIDE_RESISTANCE"], values["SPEED_SCALE"])
        for name, keys in SECTIONS:
            print(f"stage 2 {name}  " + ", ".join(keys))
            values, score = cma_minimize(
                keys, values,
                lambda trial: track_cost(guide, sim, trial, ticks),
                generations, args.seed)
            notes.append(f"Stage 2 {name} cost {score:.6g}.")
            write_paste(args.out, values, notes, HELD)
            print(f"  wrote {args.out}")

    write_paste(args.out, values, notes, HELD)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
