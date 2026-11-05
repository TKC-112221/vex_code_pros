"""Fit the MockDrive plant, then CMA-ES the controller knobs it can excite.

Stage 1 replays a LogFrame recording through the C++ plant (python/plant.pyd,
the same MockDrive the path-follower env steps). It fits the velocity lag,
the skid lag, the speed scale, and, when the log has current, the side
resistance. Short windows are restarted on the logged state so open-loop
drift does not dominate. Wheel speed is the measurement. Pose is not.

Stage 2 searches the knobs that move a rollout, all in one CMA-ES run:
    feedforward, path and practice accel, the slip model, the lateral
    loop, the heading trim, TurnTo's speed and accel caps, and the
    segment follower's Stanley gains and pivot threshold. DriveTo's
    curve leg is the drift rollout. DriveTo with segmentPath() is the
    segment rollout. TurnTo is a pivot in place. The volts for all
    three come from the same feedforward, written onto the drivebase
    before each rollout. The follower commands a yaw rate, and that
    drivebase model turns it into volts. The search does not fit a
    second kV or kA for the pivot. The saturation speed is not searched. It
    is the plant's top speed, so the volts for a hold match the chassis
    that is actually rolling. A slower model cannot brake, and the plant
    runs through the goal. The cost is the mean of three families, so a
    pivot is not drowned by the longer drives. A drive family scores time
    until the true plant pose is inside FINISH_RADIUS_IN of the target,
    the yaw rate is within FINISH_YAW_RAD_S of zero, and the distance to
    the target has started to grow, plus the angle between the path
    tangent and the direction the chassis is moving. A pivot scores time
    until the heading is inside FINISH_HEAD_DEG and the yaw rate is
    inside that same band. That radius is much wider than the firmware settle
    band (SETTLE_IN, or SETTLE_LARGE_IN if it holds), which made the search
    creep in order to sit inside it. A stop and a fly-by use the same
    gate: the clock stops when the range opens inside the radius, not when
    a settle window has been held. Cross-track and
    yaw-rail time are not in the cost. Each moving tick scores the angle
    between the path tangent and the direction the chassis is traveling.
    Below settle speed that direction is noise, and a stop is aiming the
    body at the exit heading, so those ticks are left out. The exit
    heading itself stays the body heading. Leaving the field is a failure.
    Timed elastic deforms the base curve once, when the plan is committed.
    The follower is commanded from the firmware
    odometry estimate (noisy IMU, accelerometer travel, and the wall
    nudge). Goal alignment is off, so a bumper touch does not snap the
    chassis onto a goal. The cost is the true plant pose. Each goal
    redraws that IMU from a fixed seed, so every candidate sees the same
    sensor errors. --shape picks the base
curve (Hermite by default). Timed elastic is on by default, matching
DriftFollower, and --no-timed-elastic scores the same curve with the
band off. The band's stored speed is the follower's target while it is
on. The turn-radius clamp stays
at the compiled value. The first restart starts at the compiled tune.
Later restarts start from a random
mean. The budget is --generations per restart. tolfun does not end the run.

Parameters the closed-loop rollout never reads are copied through and
listed as held. Searching them would move numbers that do not change the
cost. The result is python/tuning_paste.txt. Set USE_TUNING_PASTE to 1
in include/config/Settings.h to compile those values over the hand-tuned
knobs. The headers themselves are left unchanged.

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


# Fixed poses, inches and radians. Exit speed is m/s. The shape field is the
# base curve, planned once. Names match DriftCurveShape. timed_elastic is
# the local planner on that curve, on by default, the same as DriftFollower.
# Curved goals excite slip, heading, and the radius cap.
#
# The origin is the center of the bottom edge: +x toward the right wall,
# +y toward the far wall. The field is 144 in square, so x is -72..72 and
# y is 0..144. The rollout starts at the geometric center, (0, 72), which
# is where (0, 0) used to be. Goal coordinates are the same offsets from
# that center.
SHAPES = (
    "Arc", "Exponential", "Quartic", "Quintic",
    "Hermite", "QuarticHermite",
)
START_X_IN = 0.0
START_Y_IN = 72.0
# A rollout this far past a wall is off the field. 18 in is the old
# |x| > 90, |y| > 90 box measured from the center origin.
_FIELD_ABORT_MARGIN_IN = 18.0
_FIELD_X_MIN_IN = -72.0
_FIELD_X_MAX_IN = 72.0
_FIELD_Y_MIN_IN = 0.0
_FIELD_Y_MAX_IN = 144.0
GOALS = (
    # x in, y in, exit heading rad, exit speed m/s, base shape, timed elastic
    (48.0, 72.0, -math.pi / 2, 0.0, "Hermite", True),          # straight stop, facing +x
    (36.0, 96.0, 1.2 - math.pi / 2, 0.0, "Hermite", True),    # forward-left
    (24.0, 54.0, -0.6 - math.pi / 2, 0.0, "Hermite", True),   # forward-right
    (30.0, 82.0, 0.4 - math.pi / 2, 0.4, "Hermite", True),    # fly-by
    (8.0, 114.0, 0.0, 0.0, "Hermite", True),                  # left, exit facing +Y
    (6.0, 32.0, -math.pi, 0.0, "Hermite", True),              # right, exit facing -Y
    (42.0, 50.0, 2.4 - math.pi / 2, 0.5, "Hermite", True),    # forward, large heading, fly-by
    (20.0, 100.0, math.pi / 2, 0.0, "Hermite", True),         # ahead, arrive facing back
    (-36.0, 72.0, math.pi / 2, 0.0, "Hermite", True),         # behind, facing back
    (-24.0, 102.0, 0.0, 0.0, "Hermite", True),                # behind-left, facing +Y
    (-18.0, 44.0, -2.2 - math.pi / 2, 0.0, "Hermite", True),  # behind-right
    (-30.0, 84.0, 0.3 - math.pi / 2, 0.0, "Hermite", True),   # behind, arrive facing forward
)

# DriveTo(segmentPath()). Same pose tuple as GOALS, plus an optional
# keep-out (x in, y in, radius in) the polyline has to leave. The start
# pose faces +x, so a target up the field makes the follower pivot first.
SEGMENT_CASES = (
    ((36.0, 72.0, -math.pi / 2, 0.0, "Hermite", True), None),
    ((24.0, 108.0, 0.0, 0.0, "Hermite", True), None),
    ((40.0, 84.0, -math.pi / 2, 0.35, "Hermite", True), None),
    ((-20.0, 96.0, math.pi / 2, 0.0, "Hermite", True), None),
    ((48.0, 72.0, -math.pi / 2, 0.0, "Hermite", True), (24.0, 72.0, 10.0)),
)

# TurnTo headings, radians, from the same start facing +x. Short, quarter,
# and half turns. The chassis is not asked to drive.
TURN_HEADINGS = (
    -math.pi / 2 + 0.45,
    0.0,
    math.pi,
    math.pi / 2,
)


def goals_with_shape(shape, timed_elastic=True):
    """The fixed poses, one base shape, and one timed-elastic setting."""
    if shape == "TimedElastic":
        raise SystemExit(
            "TimedElastic is the local planner, not a base shape. "
            "Pick a shape from "
            + ", ".join(SHAPES)
            + ", and leave the band on (or pass --no-timed-elastic)."
        )
    if shape not in SHAPES:
        raise SystemExit(
            f"unknown path shape {shape!r}. Choose from: {', '.join(SHAPES)}"
        )
    band = bool(timed_elastic)
    return tuple(
        (tx, ty, heading, exit_speed, shape, band)
        for tx, ty, heading, exit_speed, _shape, _band in GOALS
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
        "MASS_KG": float(plant.MASS_KG),
        "MAX_VEL_MPS": float(plant.MAX_VEL_MPS),
        "DRIVE_ACCEL": float(plant.DRIVE_ACCEL),
        "STATIC_FRICTION_N": float(plant.STATIC_FRICTION_N),
        "YAW_INERTIA_SCALE": float(plant.YAW_INERTIA_SCALE),
        "YAW_FRICTION_NM": float(plant.YAW_FRICTION_NM),
        "SKID_TAU": float(plant.SKID_TAU),
        "LATERAL_FRICTION_N": float(plant.LATERAL_FRICTION_N),
        "TRACTION_MU": float(plant.TRACTION_MU),
        "SIDE_RESISTANCE": float(plant.SIDE_RESISTANCE),
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
        "TURN_MAX_VEL": float(plant.TURN_MAX_VEL),
        "TURN_MAX_ACCEL": float(plant.TURN_MAX_ACCEL),
        "STANLEY_K": float(plant.STANLEY_K),
        "STANLEY_SOFT": float(plant.STANLEY_SOFT),
        "PIVOT_RAD": float(plant.PIVOT_RAD),
    }


# Physically plausible boxes. Heterogeneous units are mapped to [-1, 1]
# before CMA-ES, so one step size covers every knob. A zero nominal uses
# an absolute box; a multiplicative box around zero cannot move.
BOUNDS = {
    "MASS_KG": (2.0, 14.0),
    "MAX_VEL_MPS": (0.5, 6.0),
    "DRIVE_ACCEL": (0.5, 24.0),
    "STATIC_FRICTION_N": (0.0, 40.0),
    "YAW_INERTIA_SCALE": (0.25, 3.0),
    "YAW_FRICTION_NM": (0.0, 5.0),
    "SKID_TAU": (0.05, 1.20),
    "LATERAL_FRICTION_N": (0.0, 30.0),
    "TRACTION_MU": (0.2, 2.0),
    "SIDE_RESISTANCE": (0.40, 6.00),
    "KS_LIN": (0.0, 1.5),
    "LIN_SPEED_AT_SAT": (1.0, 8.0),
    "LIN_ACCEL_AT_FULL_V": (1.0, 15.0),
    # V_R - V_L. 24 V is both sides on opposite rails, the most static
    # yaw the motors can apply. Past that, the chassis clamp is identical.
    "KS_ANG": (0.0, 24.0),
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
    "TURN_MAX_VEL": (0.8, 12.0),
    "TURN_MAX_ACCEL": (0.5, 24.0),
    "STANLEY_K": (0.2, 8.0),
    "STANLEY_SOFT": (0.05, 1.5),
    "PIVOT_RAD": (0.25, 1.6),
}

# One search. These knobs all change the same rollout: volts for a planned
# speed, how hard that speed may rise, the slip the curve is built with,
# and the lateral loop that steers back onto it. Heading trim is in the
# same run because the clock stops when the yaw rate is near zero, and the
# cost scores travel direction against the path tangent.
# The radius clamp is held; these goals do not reach it.
TRACK_KEYS = (
    "KS_LIN", "LIN_ACCEL_AT_FULL_V",
    "KS_ANG", "ANG_RATE_PER_VOLT", "ANG_LEVER_M",
    "ANG_KA_COEFF", "ANG_KA_LENGTH_M",
    "PATH_ACCEL", "MIN_ACCEL", "PRACTICE_ACCEL",
    "K_SLIP", "A_MAX", "MAX_SLIP_RAD", "CLOSE_IN",
    "HEAD_KP", "HEAD_KI", "HEAD_KD", "HEAD_WINDUP",
    "LAT_KP", "LAT_KI", "LAT_KD", "LAT_KD2", "LAT_I_LIMIT",
    "TURN_MAX_VEL", "TURN_MAX_ACCEL",
    "STANLEY_K", "STANLEY_SOFT", "PIVOT_RAD",
)

# Voltage is applied this many ticks late, so a derivative gain is chasing
# a pose the chassis has not reached yet.
COMMAND_DELAY_TICKS = 2

# Time is a fraction of the horizon. An unfinished move costs one full
# horizon plus the pose miss. The direction term is added on top, so a
# large aim error can outweigh a late arrival. The tie-break is half a
# tick at most, so waiting another tick to sit closer to the center of
# the window loses.
_W_TIME = 1.0
# Mean squared angle, in radians, between the path tangent and the
# chassis velocity. Counted only while ground speed is at least the
# settle speed. Weight 0.5 makes a 30 degree miss about 0.14 of a
# horizon, so a small aim error does not outweigh arriving. A right
# angle for the whole move costs about one horizon, which can outweigh
# a late arrival.
_W_DIR = 0.5


def require_plant():
    missing = []
    if not hasattr(plant.PlantSim, "set_dynamics"):
        missing.append("PlantSim.set_dynamics")
    if not hasattr(plant.PlantSim, "set_feedforward"):
        missing.append("PlantSim.set_feedforward")
    if not hasattr(plant.PlantSim, "set_goal_align"):
        missing.append("PlantSim.set_goal_align")
    if not hasattr(plant.DriftGuide, "configure"):
        missing.append("DriftGuide.configure")
    if not hasattr(plant, "read_log"):
        missing.append("read_log")
    if not hasattr(plant.DriftGuide, "path"):
        missing.append("DriftGuide.path")
    if not hasattr(plant.DriftGuide, "base_path"):
        missing.append("DriftGuide.base_path")
    if not hasattr(plant, "TurnGuide"):
        missing.append("TurnGuide")
    if not hasattr(plant, "SegmentGuide"):
        missing.append("SegmentGuide")
    if not hasattr(plant.PlantSim, "localize_reset"):
        missing.append("PlantSim.localize_reset")
    for name in ("SETTLE_LARGE_IN", "SETTLE_LARGE_DEG", "SETTLE_SMALL_S", "SETTLE_LARGE_S",
                 "MASS_KG", "MAX_VEL_MPS", "DRIVE_ACCEL", "STATIC_FRICTION_N",
                 "YAW_INERTIA_SCALE", "YAW_FRICTION_NM", "LATERAL_FRICTION_N", "TRACTION_MU"):
        if not hasattr(plant, name):
            missing.append(name)
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


def apply_plant(sim, values):
    """Write the rigid-body motor onto the simulator."""
    sim.set_dynamics(
        values["MASS_KG"], values["MAX_VEL_MPS"], values["DRIVE_ACCEL"],
        values["STATIC_FRICTION_N"], values["SKID_TAU"], values["SIDE_RESISTANCE"],
        values["YAW_INERTIA_SCALE"], values["YAW_FRICTION_NM"],
        values["LATERAL_FRICTION_N"], values["TRACTION_MU"],
    )


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


def apply_feedforward(sim, values):
    """Write kS, kV, and kA onto the drivebase that turns rad/s into volts."""
    sim.set_feedforward(*feedforward(values))


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


def configure_turn(guide, values):
    ks_lin, kv_lin, ka_lin, ks_ang, kv_ang, ka_ang = feedforward(values)
    guide.configure(
        ks_lin, kv_lin, ka_lin, ks_ang, kv_ang, ka_ang,
        values["TURN_MAX_VEL"], values["TURN_MAX_ACCEL"],
    )


def configure_segment(guide, values, practice_accel):
    ks_lin, kv_lin, ka_lin, ks_ang, kv_ang, ka_ang = feedforward(values)
    guide.configure(
        ks_lin, kv_lin, ka_lin, ks_ang, kv_ang, ka_ang,
        values["PATH_ACCEL"], values["MIN_ACCEL"],
        values["HEAD_KP"], values["HEAD_KI"], values["HEAD_KD"], values["HEAD_WINDUP"],
        values["STANLEY_K"], values["STANLEY_SOFT"], values["PIVOT_RAD"],
        float(plant.SETTLE_SPEED),
        float(plant.SETTLE_IN),
        float(plant.SETTLE_DEG),
        values["PRACTICE_ACCEL"] if practice_accel else -1.0,
    )


def localization_seed(practice, tx, ty, heading):
    """Fixed per goal, so every candidate is scored on the same sensor errors."""
    acc = 0x5EED1234
    acc ^= 0x9E3779B9 if practice else 0x85EBCA6B
    for value in (tx, ty, heading):
        acc ^= (int(round(value * 1000.0)) + 0x10000) & 0xFFFFFFFF
        acc = (acc * 1664525 + 1013904223) & 0xFFFFFFFF
    return acc or 1


def delayed_step(sim, pending, surge, sway, yaw):
    """Apply the command from COMMAND_DELAY_TICKS ago. `pending` starts full of zeros."""
    pending.append((float(surge), float(sway), float(yaw)))
    old_surge, old_sway, old_yaw = pending.pop(0)
    return sim.step_velocity(old_surge, old_sway, old_yaw)


def _polyline(points):
    px = np.asarray([p[0] for p in points], dtype=float)
    py = np.asarray([p[1] for p in points], dtype=float)
    return px, py, np.diff(px), np.diff(py)


def _project(poly, x, y):
    """Tangent of the closest path segment, radians."""
    px, py, dx, dy = poly
    length2 = np.maximum(dx * dx + dy * dy, 1e-12)
    t = np.clip(((x - px[:-1]) * dx + (y - py[:-1]) * dy) / length2, 0.0, 1.0)
    dist2 = (x - (px[:-1] + t * dx)) ** 2 + (y - (py[:-1] + t * dy)) ** 2
    i = int(np.argmin(dist2))
    return math.atan2(float(dy[i]), float(dx[i])) - math.pi / 2.0


def _travel_bearing(state, slow_mps):
    """Direction the chassis is moving, or None while that angle is noise.

    v_lin is forward in the body frame and v_lat is left. Below settle
    speed the velocity heading chatters, and a stop is turning the body
    onto the exit heading rather than the path tangent.
    """
    c = -math.sin(state.theta)
    s = math.cos(state.theta)
    vx = state.v_lin * c - state.v_lat * s
    vy = state.v_lin * s + state.v_lat * c
    if math.hypot(vx, vy) < slow_mps:
        return None
    return math.atan2(vy, vx) - math.pi / 2.0


# Episode end. The firmware band is 1 in and 1 deg, or 3 in and 3 deg if
# it holds. Waiting on that band made every knob conservative, because
# only a creep could sit inside it. A rollout ends once the chassis is
# inside this radius, the yaw rate is near zero, and the range to
# the target has opened from its closest point. The radius is four times
# the wide settle band.
FINISH_RADIUS_IN = 12.0
# True-plant yaw rate, rad/s. Half a radian per second is a slow turn.
FINISH_YAW_RAD_S = 0.5
# Pivot window, degrees. Wider than TurnTo's 1 degree settle, for the
# same reason the drive radius is wider than the firmware band.
FINISH_HEAD_DEG = 3.0
# An opening smaller than this is a one-tick bobble, not a departure.
_FINISH_GROW_IN = 0.05


class PoseSettle:
    """Episode end on a loose arrival, not the firmware settle band.

    Finished when the true pose is inside FINISH_RADIUS_IN, the yaw rate
    is within FINISH_YAW_RAD_S of zero, and the distance to the target has
    grown from the closest point so far. A fly-by ends on the way out. A
    stop ends once the range opens, instead of holding the tight window.
    The wide settle window is kept only to scale the unfinished pose miss.
    """

    def __init__(self):
        small_in = float(plant.SETTLE_IN)
        small_deg = float(plant.SETTLE_DEG)
        self.radius_in = FINISH_RADIUS_IN
        self.yaw_rad_s = FINISH_YAW_RAD_S
        self.grow_in = _FINISH_GROW_IN
        self.large_in = max(float(plant.SETTLE_LARGE_IN), small_in)
        self.large_deg = max(float(plant.SETTLE_LARGE_DEG), small_deg)
        self.speed = float(plant.SETTLE_SPEED)
        self.closest_in = None

    def update(self, dist_in, yaw_rad_s):
        if self.closest_in is None or dist_in < self.closest_in:
            self.closest_in = dist_in
        growing = dist_in > self.closest_in + self.grow_in
        near = dist_in <= self.radius_in
        quiet = abs(yaw_rad_s) <= self.yaw_rad_s
        return near and quiet and growing


def _pose_cost(dist_in, goal_head, speed_err, settle):
    """Miss scaled by the wide window, so just outside it is order 1."""
    return ((dist_in / settle.large_in) ** 2
            + (abs(math.degrees(goal_head)) / settle.large_deg) ** 2
            + (speed_err / settle.speed) ** 2)


def _score_goal(guide, sim, goal, practice, ticks):
    tx, ty, heading, exit_speed, shape, timed_elastic = goal
    guide.begin(tx, ty, heading, exit_speed, False, False, shape, timed_elastic)
    # Planned once. A curve that fails to build has nothing to drive, and
    # nothing to compare the travel direction against.
    sample = guide.path if timed_elastic else guide.base_path
    points = list(sample(START_X_IN, START_Y_IN, -math.pi / 2.0, 0.0, 0.0))
    sim.localize_reset(START_X_IN, START_Y_IN, -math.pi / 2.0, 0.0, 0.0, 0.0,
                       localization_seed(practice, tx, ty, heading))
    state = sim.state()
    dist_in = math.hypot(state.x_in - tx, state.y_in - ty)
    goal_head = wrap(heading - state.theta)
    speed_err = state.v_lin - exit_speed
    row = {
        "kind": "drift",
        "practice": practice,
        "tx": tx,
        "ty": ty,
        "heading": heading,
        "exit_speed": exit_speed,
        "done": False,
        "seconds": ticks * float(plant.DT_SEC),
        "miss_in": dist_in,
        "head_deg": abs(math.degrees(goal_head)),
        "course_deg": 0.0,
        "speed_err": speed_err,
        "cost": FAIL + 10.0 * dist_in,
    }
    if len(points) < 2:
        return row
    poly = _polyline(points)
    pending = [(0.0, 0.0, 0.0) for _ in range(COMMAND_DELAY_TICKS)]
    settle = PoseSettle()
    measured = 0
    dir_acc = 0.0
    dir_n = 0
    done = False
    for _ in range(ticks):
        est = sim.estimate()
        if not all(math.isfinite(v) for v in (
            state.x_in, state.y_in, state.theta, state.v_lin, state.v_lat, state.v_ang,
            est.x_in, est.y_in, est.theta, est.v_lin, est.v_ang,
        )):
            return row
        surge, sway, yaw = guide.command(
            est.x_in, est.y_in, est.theta, est.v_lin, est.v_ang)
        state = delayed_step(sim, pending, surge, sway, yaw)
        if not all(math.isfinite(v) for v in (
            state.x_in, state.y_in, state.theta, state.v_lin, state.v_lat, state.v_ang
        )):
            return row
        if (state.x_in < _FIELD_X_MIN_IN - _FIELD_ABORT_MARGIN_IN
                or state.x_in > _FIELD_X_MAX_IN + _FIELD_ABORT_MARGIN_IN
                or state.y_in < _FIELD_Y_MIN_IN - _FIELD_ABORT_MARGIN_IN
                or state.y_in > _FIELD_Y_MAX_IN + _FIELD_ABORT_MARGIN_IN):
            row["miss_in"] = math.hypot(state.x_in - tx, state.y_in - ty)
            row["head_deg"] = abs(math.degrees(wrap(heading - state.theta)))
            row["speed_err"] = state.v_lin - exit_speed
            row["cost"] = FAIL + 10.0 * row["miss_in"]
            return row
        measured += 1
        bearing = _travel_bearing(state, settle.speed)
        if bearing is not None:
            dir_acc += wrap(_project(poly, state.x_in, state.y_in) - bearing) ** 2
            dir_n += 1
        dist_in = math.hypot(state.x_in - tx, state.y_in - ty)
        goal_head = wrap(heading - state.theta)
        speed_err = state.v_lin - exit_speed
        if settle.update(dist_in, state.v_ang):
            done = True
            break
    if measured == 0:
        return row
    if dir_n:
        mean_dir = dir_acc / dir_n
        direction = _W_DIR * mean_dir
        course_deg = math.degrees(math.sqrt(mean_dir))
    else:
        direction = 0.0
        course_deg = 0.0
    pose = _pose_cost(dist_in, goal_head, speed_err, settle)
    if done:
        # Half a tick at most. One more tick of travel costs more than this,
        # so the search does not wait inside the radius for a tighter pose.
        tie = 0.5 * (_W_TIME / ticks) * min(pose, 1.0)
        cost = _W_TIME * (measured / ticks) + tie + direction
    else:
        cost = _W_TIME + pose + direction
    if not math.isfinite(cost):
        return row
    row.update({
        "done": done,
        "seconds": measured * float(plant.DT_SEC),
        "miss_in": dist_in,
        "head_deg": abs(math.degrees(goal_head)),
        "course_deg": course_deg,
        "speed_err": speed_err,
        "cost": cost,
    })
    return row


def _score_turn(guide, sim, heading, ticks):
    """One TurnTo. The clock stops on a loose heading, not the 1 degree window."""
    guide.begin(heading)
    sim.localize_reset(START_X_IN, START_Y_IN, -math.pi / 2.0, 0.0, 0.0, 0.0,
                       localization_seed(False, 0.0, 0.0, heading))
    state = sim.state()
    head = wrap(heading - state.theta)
    slide_in = math.hypot(state.x_in - START_X_IN, state.y_in - START_Y_IN)
    settle = PoseSettle()
    row = {
        "kind": "turn",
        "practice": False,
        "tx": START_X_IN,
        "ty": START_Y_IN,
        "heading": heading,
        "exit_speed": 0.0,
        "done": False,
        "seconds": ticks * float(plant.DT_SEC),
        "miss_in": slide_in,
        "head_deg": abs(math.degrees(head)),
        "course_deg": 0.0,
        "speed_err": state.v_lin,
        "cost": FAIL + abs(math.degrees(head)),
    }
    pending = [(0.0, 0.0, 0.0) for _ in range(COMMAND_DELAY_TICKS)]
    measured = 0
    done = False
    for _ in range(ticks):
        est = sim.estimate()
        if not all(math.isfinite(v) for v in (
            state.x_in, state.y_in, state.theta, state.v_lin, state.v_ang,
            est.x_in, est.y_in, est.theta, est.v_lin, est.v_ang,
        )):
            return row
        surge, sway, yaw = guide.command(
            est.x_in, est.y_in, est.theta, est.v_lin, est.v_ang)
        state = delayed_step(sim, pending, surge, sway, yaw)
        if not all(math.isfinite(v) for v in (
            state.x_in, state.y_in, state.theta, state.v_lin, state.v_ang
        )):
            return row
        if (state.x_in < _FIELD_X_MIN_IN - _FIELD_ABORT_MARGIN_IN
                or state.x_in > _FIELD_X_MAX_IN + _FIELD_ABORT_MARGIN_IN
                or state.y_in < _FIELD_Y_MIN_IN - _FIELD_ABORT_MARGIN_IN
                or state.y_in > _FIELD_Y_MAX_IN + _FIELD_ABORT_MARGIN_IN):
            row["miss_in"] = math.hypot(state.x_in - START_X_IN, state.y_in - START_Y_IN)
            row["head_deg"] = abs(math.degrees(wrap(heading - state.theta)))
            row["speed_err"] = state.v_lin
            row["cost"] = FAIL + row["head_deg"]
            return row
        measured += 1
        head = wrap(heading - state.theta)
        slide_in = math.hypot(state.x_in - START_X_IN, state.y_in - START_Y_IN)
        if abs(math.degrees(head)) <= FINISH_HEAD_DEG and abs(state.v_ang) <= FINISH_YAW_RAD_S:
            done = True
            break
    if measured == 0:
        return row
    pose = ((abs(math.degrees(head)) / FINISH_HEAD_DEG) ** 2
            + (state.v_ang / FINISH_YAW_RAD_S) ** 2
            + (slide_in / settle.large_in) ** 2
            + (state.v_lin / settle.speed) ** 2)
    if done:
        tie = 0.5 * (_W_TIME / ticks) * min(pose, 1.0)
        cost = _W_TIME * (measured / ticks) + tie
    else:
        cost = _W_TIME + pose
    if not math.isfinite(cost):
        return row
    row.update({
        "done": done,
        "seconds": measured * float(plant.DT_SEC),
        "miss_in": slide_in,
        "head_deg": abs(math.degrees(head)),
        "speed_err": state.v_lin,
        "cost": cost,
    })
    return row


def _family_mean(rows):
    return sum(row["cost"] for row in rows) / len(rows)


def score_tracking(guide, segment, turn, sim, values, ticks, goals=None):
    """Mean of the drift, segment, and turn families, plus every row."""
    goals = GOALS if goals is None else goals
    apply_feedforward(sim, values)
    drift_rows = []
    segment_rows = []
    for practice in (True, False):
        configure(guide, values, practice)
        configure_segment(segment, values, practice)
        for goal in goals:
            drift_rows.append(_score_goal(guide, sim, goal, practice, ticks))
        for goal, block in SEGMENT_CASES:
            if block is None:
                segment.clear_block()
            else:
                segment.set_block(*block)
            row = _score_goal(segment, sim, goal, practice, ticks)
            row["kind"] = "segment"
            segment_rows.append(row)
    configure_turn(turn, values)
    turn_rows = [_score_turn(turn, sim, heading, ticks) for heading in TURN_HEADINGS]
    rows = drift_rows + segment_rows + turn_rows
    cost = (_family_mean(drift_rows) + _family_mean(segment_rows) + _family_mean(turn_rows)) / 3.0
    return cost, rows


def track_cost(guide, segment, turn, sim, values, ticks, goals=None):
    """Mean of drift, segment, and turn. Both drive accel sources are rolled."""
    cost, _rows = score_tracking(guide, segment, turn, sim, values, ticks, goals)
    return cost


def print_tracking(guide, segment, turn, sim, values, ticks, goals=None):
    cost, rows = score_tracking(guide, segment, turn, sim, values, ticks, goals)
    print(f"  rollout cost {cost:.6g}")
    for row in rows:
        if row["kind"] == "turn":
            mode = "turn"
            where = f"head {math.degrees(row['heading']):6.1f} deg"
        else:
            family = "segment" if row["kind"] == "segment" else "drift"
            accel = "practice" if row["practice"] else "path"
            mode = f"{family}/{accel}"
            where = f"({row['tx']:g}, {row['ty']:g}) in"
        state = "done" if row["done"] else "open"
        print(
            f"  {mode:16} {where}  {state} {row['seconds']:.2f}s"
            f"  miss {row['miss_in']:.1f} in  head {row['head_deg']:.1f} deg"
            f"  course {row['course_deg']:.1f} deg"
            f"  speed {row['speed_err']:+.2f} m/s"
        )
    return cost


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
    apply_plant(sim, values)
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


def cma_minimize(keys, values, cost, generations, seed, *,
                 popsize=None, restarts=1, sigma=0.3,
                 tolfun=1e-4, tolx=1e-3, tolstagnation=None,
                 random_mean=False):
    """CMA-ES in [-1, 1]. The incoming values are retained if nothing beats them.

    The first restart is centered on the incoming values. Further restarts,
    when random_mean is set, are centered on a fresh point in the box.
    tolfun 0 and tolx 0 leave the generation count as the budget. A flat
    population can still end one restart early.
    """
    import cma

    lo = np.array([BOUNDS[key][0] for key in keys])
    hi = np.array([BOUNDS[key][1] for key in keys])
    start = np.array([values[key] for key in keys], dtype=float)
    # A nominal sitting on the bound has no room to sample. Shift only the
    # initial mean. The scored elite is still the real nominal.
    span = hi - lo
    mean = np.clip(start, lo + 0.05 * span, hi - 0.05 * span)

    def objective(unit):
        trial = dict(values)
        for i, key in enumerate(keys):
            trial[key] = from_unit(unit[i], lo[i], hi[i])
        score = float(cost(trial))
        if not math.isfinite(score):
            return FAIL
        return score

    def adopt(unit, score):
        nonlocal elite_score, elite
        if score < elite_score:
            elite_score = score
            elite = np.array([from_unit(unit[i], lo[i], hi[i]) for i in range(len(keys))])

    elite_score = objective([to_unit(value, lo[i], hi[i]) for i, value in enumerate(start)])
    elite = start.copy()
    print(f"  start {elite_score:.6g}")

    rng = np.random.default_rng(seed)
    compiled = [to_unit(value, lo[i], hi[i]) for i, value in enumerate(mean)]
    for restart in range(restarts):
        if restart == 0 or not random_mean:
            x0 = compiled
            label = "compiled"
        else:
            x0 = rng.uniform(-0.85, 0.85, size=len(keys)).tolist()
            adopt(x0, objective(x0))
            label = "random"
        options = {
            "bounds": [-1.0, 1.0],
            "seed": seed + restart,
            "maxiter": generations,
            "verb_disp": 0,
            "verb_log": 0,
            "verbose": -9,
            "CMA_elitist": True,
            "tolfun": tolfun,
            "tolx": tolx,
            "tolflatfitness": 20,
        }
        if tolfun == 0.0:
            options["tolfunhist"] = 0
        if tolx == 0.0:
            options["tolxstagnation"] = False
        if tolstagnation is not None:
            options["tolstagnation"] = tolstagnation
        if popsize is not None:
            options["popsize"] = popsize
        strategy = cma.CMAEvolutionStrategy(x0, sigma, options)
        print(
            f"  restart {restart + 1}/{restarts}  {label}"
            f"  pop {strategy.popsize}  sigma {sigma:g}"
        )
        generation = 0
        while generation < generations:
            reason = strategy.stop()
            if reason:
                print(f"  stop {dict(reason)}")
                break
            generation += 1
            points = strategy.ask()
            scores = [objective(point) for point in points]
            strategy.tell(points, scores)
            best_i = int(np.argmin(scores))
            adopt(points[best_i], scores[best_i])
            print(f"  gen {generation:3d}  best {elite_score:.6g}  pop {min(scores):.6g}")

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
        "// Written by python/optimize_tuning.py.",
        "// Included from Tuning.h and MockDriveParams.h when USE_TUNING_PASTE is 1.",
        "// The includer sets TUNING_PASTE_PART. This file has no include guard.",
        "//",
        "// KV_LIN, KA_LIN, KV_ANG, and KA_ANG stay formulas in Tuning.h. They",
        "// follow the knobs below. With these knobs they evaluate to",
        f"//   KV_LIN {cpp_num(kv_lin)}   KA_LIN {cpp_num(ka_lin)}",
        f"//   KV_ANG {cpp_num(kv_ang)}   KA_ANG {cpp_num(ka_ang)}",
        "//",
        "// Part 6 replaces Physics::TURN_RADIUS_MAX_M, including its alias to",
        "// data/generated_limits.h.",
        "//",
        "// Boomerang::CLOSE_IN is also DrivePolicy::kPositionToleranceIn.",
        "//",
        "// Part 8 is Turn::MAX_VEL_RAD_S and MAX_ACCEL_RAD_S2. The pivot's",
        "// volts stay the feedforward in part 2.",
        "//",
        "// Part 9 is Segment::STANLEY_K, STANLEY_SOFT, and PIVOT_RAD.",
        "//",
    ]
    for note in notes:
        lines.append("// " + note)
    lines += [
        "",
        "#ifndef TUNING_PASTE_PART",
        '#error "Include python/tuning_paste.txt with TUNING_PASTE_PART set to 1..9."',
        "#endif",
        "",
        "#if TUNING_PASTE_PART == 1",
        "// MockPlant  (include/hardware/MockDriveParams.h)",
        f"constexpr double MASS_KG = {cpp_num(values['MASS_KG'])};",
        f"constexpr double MAX_VEL_MPS = {cpp_num(values['MAX_VEL_MPS'])};",
        f"constexpr double DRIVE_ACCEL_MPS2 = {cpp_num(values['DRIVE_ACCEL'])};",
        f"constexpr double STATIC_FRICTION_N = {cpp_num(values['STATIC_FRICTION_N'])};",
        f"constexpr double YAW_INERTIA_SCALE = {cpp_num(values['YAW_INERTIA_SCALE'])};",
        f"constexpr double YAW_FRICTION_NM = {cpp_num(values['YAW_FRICTION_NM'])};",
        f"constexpr double SKID_TAU_SEC = {cpp_num(values['SKID_TAU'])};",
        f"constexpr double LATERAL_FRICTION_N = {cpp_num(values['LATERAL_FRICTION_N'])};",
        f"constexpr double TRACTION_MU = {cpp_num(values['TRACTION_MU'])};",
        f"constexpr double SIDE_RESISTANCE_OHM = {cpp_num(values['SIDE_RESISTANCE'])};",
        "#elif TUNING_PASTE_PART == 2",
        "// Feedforward  (include/core/Tuning.h)",
        f"constexpr double KS_LIN = {cpp_num(values['KS_LIN'])};",
        f"constexpr double KS_ANG = {cpp_num(values['KS_ANG'])};",
        f"constexpr double LIN_SPEED_AT_SAT = {cpp_num(values['LIN_SPEED_AT_SAT'])};",
        f"constexpr double LIN_ACCEL_AT_FULL_V = {cpp_num(values['LIN_ACCEL_AT_FULL_V'])};",
        f"constexpr double ANG_RATE_PER_VOLT = {cpp_num(values['ANG_RATE_PER_VOLT'])};",
        f"constexpr double ANG_LEVER_M = {cpp_num(values['ANG_LEVER_M'])};",
        f"constexpr double ANG_KA_COEFF = {cpp_num(values['ANG_KA_COEFF'])};",
        f"constexpr double ANG_KA_LENGTH_M = {cpp_num(values['ANG_KA_LENGTH_M'])};",
        "#elif TUNING_PASTE_PART == 3",
        "// Drift  (include/core/Tuning.h)",
        f"constexpr double PATH_ACCEL = {cpp_num(values['PATH_ACCEL'])};",
        f"constexpr double K_SLIP = {cpp_num(values['K_SLIP'])};",
        f"constexpr double A_MAX = {cpp_num(values['A_MAX'])};",
        f"constexpr double MAX_SLIP_RAD = {cpp_num(values['MAX_SLIP_RAD'])};",
        f"constexpr double LAT_KP = {cpp_num(values['LAT_KP'])};",
        f"constexpr double LAT_KI = {cpp_num(values['LAT_KI'])};",
        f"constexpr double LAT_KD = {cpp_num(values['LAT_KD'])};",
        f"constexpr double LAT_KD2 = {cpp_num(values['LAT_KD2'])};",
        f"constexpr double LAT_I_LIMIT = {cpp_num(values['LAT_I_LIMIT'])};",
        "#elif TUNING_PASTE_PART == 4",
        "// Motion  (include/core/Tuning.h)",
        f"constexpr double MIN_ACCEL_MPS2 = {cpp_num(values['MIN_ACCEL'])};",
        f"constexpr double KAPPA_EPS = {cpp_num(values['KAPPA_EPS'])};",
        "#elif TUNING_PASTE_PART == 5",
        "// Practice  (include/core/Tuning.h)",
        f"constexpr double MAX_ACCEL_MPS2 = {cpp_num(values['PRACTICE_ACCEL'])};",
        "#elif TUNING_PASTE_PART == 6",
        "// Physics::TURN_RADIUS_MAX_M  (include/core/Tuning.h)",
        f"constexpr double TURN_RADIUS_MAX_M = {cpp_num(values['TURN_RADIUS_MAX'])};",
        "#elif TUNING_PASTE_PART == 7",
        "// Boomerang  (include/core/Tuning.h)",
        f"constexpr Axis Lateral = {{{axis(lateral)}}};",
        f"constexpr Axis Angular = {{{axis(angular)}}};",
        f"constexpr double CLOSE_IN = {cpp_num(values['CLOSE_IN'])};",
        "#elif TUNING_PASTE_PART == 8",
        "// Turn  (include/core/Tuning.h)",
        f"constexpr double MAX_VEL_RAD_S = {cpp_num(values['TURN_MAX_VEL'])};",
        f"constexpr double MAX_ACCEL_RAD_S2 = {cpp_num(values['TURN_MAX_ACCEL'])};",
        "#elif TUNING_PASTE_PART == 9",
        "// Segment  (include/core/Tuning.h)",
        f"constexpr double STANLEY_K = {cpp_num(values['STANLEY_K'])};",
        f"constexpr double STANLEY_SOFT = {cpp_num(values['STANLEY_SOFT'])};",
        f"constexpr double PIVOT_RAD = {cpp_num(values['PIVOT_RAD'])};",
        "#else",
        '#error "TUNING_PASTE_PART must be 1 through 9."',
        "#endif",
        "",
        "#undef TUNING_PASTE_PART",
        "",
        "// Held. These stay at the compiled values. The rollout does not read",
        "// them, or these goals give them no slope.",
    ]
    for reason in held_reason:
        lines.append("// " + reason)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


HELD = (
    "PidGains, Ramsete, FF_BOOST, SPATIAL_WARP: CascadedController replay, not DriftGuide.",
    "Tuning slew/cube/turn stick, Face, Pivot, Replay wheel KP, Controls deadbands: teleop.",
    "Drift::EXP_RATE: Exponential curves only. The other shapes do not read it.",
    "Drift::DRIFT_ACCEL: the atan slip model. DriftFollower uses arcsin (A_MAX).",
    "Drift::PATH_ACCEL is searched. It applies when the target leaves max_accel unset.",
    "Practice::MAX_ACCEL_MPS2 is searched. DriftGuide writes it onto the target.",
    "TURN_RADIUS_MAX, KAPPA_EPS: held. None of the four goals reaches the radius clamp.",
    "SETTLE_SPEED, SETTLE_IN, SETTLE_DEG, and the wide window: the follower's square-up, and the scale on an unfinished pose miss. The episode clock is the loose arrival (radius, quiet yaw, opening range), not this band.",
    "Boomerang Lateral PID, slew, and timing: DriftFollower's lateral loop is Drift::LAT_*.",
    "BOOMERANG_LEAD, BOOMERANG_MAX_SLIP, PROBE_M, CARROT_FLOOR, CURVE_STEPS: other types.",
    "driftpath lookahead and graph limits: DriftCurve, not this follower.",
    "DrivePolicy scales: they belong to the trained network, which this search does not run.",
    "Particles, Slip, RangeLik, Ekf, ImuBias, SensorFilter, WallSolve, WallNudge: estimators.",
    "The guide is scored on the true plant pose and commanded from odometry. Estimator gains are not in this search.",
    "Turn settle window, fallback kV/kA, and the leftover-speed brake. The pivot plan uses the searched caps, and the volts are the shared feedforward.",
    "Segment pivot_done_rad. Stanley k, the soft speed, and the pivot threshold are searched. Path accel and the heading trim are the shared knobs.",
    "LIN_SPEED_AT_SAT is the plant top speed, MAX_VEL_MPS. Searching it made the feedforward hold a slower robot than the plant, so full volts could not brake and the chassis ran through the goal.",
    "ANG_KA_MOTOR_COUNT is the motor count from the fit, not a gain.",
    "MockDrive confidence, range period, and reported temperature do not move the chassis.",
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, help="LogFrame file, e.g. skills_auto.bin")
    parser.add_argument("--quick", action="store_true", help="two generations, one restart, short rollouts")
    parser.add_argument(
        "--generations", type=int, default=40,
        help="Generations per tracking restart. tolfun does not end that search.",
    )
    parser.add_argument("--popsize", type=int, default=30, help="Candidates per tracking generation.")
    parser.add_argument(
        "--restarts", type=int, default=4,
        help="Tracking restarts. The first is the compiled tune. The rest use a random mean.",
    )
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--stage", choices=("all", "plant", "gains"), default="all")
    parser.add_argument(
        "--out", type=Path,
        default=Path(__file__).resolve().parent / "tuning_paste.txt",
        help="Paste consumed when USE_TUNING_PASTE is 1. Default: python/tuning_paste.txt.",
    )
    parser.add_argument("--sigma", type=float, default=0.45, help="CMA-ES sigma parameter.")
    parser.add_argument(
        "--shape", choices=SHAPES, default="Hermite",
        help="Base curve, planned once. Same names as DriftCurveShape.",
    )
    parser.add_argument(
        "--timed-elastic", action=argparse.BooleanOptionalAction, default=True,
        help="Deform the base curve once, when the plan is committed. "
             "On by default, matching DriftFollower. Travel direction is "
             "scored against that committed path.",
    )
    args = parser.parse_args()

    try:
        import cma  # noqa: F401
    except ImportError as error:
        raise SystemExit("CMA-ES needs the cma package: py -3 -m pip install cma") from error

    require_plant()
    values = nominal()
    notes = []
    if args.quick:
        generations = 2
        ticks = 80
        popsize = 8
        restarts = 1
        random_mean = False
        sigma = 0.3
    else:
        generations = args.generations
        ticks = 200
        popsize = args.popsize
        restarts = args.restarts
        random_mean = restarts > 1
        sigma = args.sigma
    windows = 4 if args.quick else 8
    window = 25 if args.quick else 50

    sim = plant.PlantSim()
    # A bumper touch must not snap the chassis onto a goal while the follower
    # is being scored.
    sim.set_goal_align(False)
    guide = plant.DriftGuide()
    segment = plant.SegmentGuide()
    turn = plant.TurnGuide()

    if args.stage in ("all", "plant"):
        if args.log is None:
            notes.append(
                "Stage 1 kept the compiled MockDrive plant. Pass --log a LogFrame "
                "recording to fit mass, top speed, acceleration, and friction."
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
                keys = [
                    "MASS_KG", "MAX_VEL_MPS", "DRIVE_ACCEL", "STATIC_FRICTION_N",
                    "YAW_INERTIA_SCALE", "YAW_FRICTION_NM", "SKID_TAU",
                    "LATERAL_FRICTION_N", "TRACTION_MU",
                ]
                if use_current:
                    keys.append("SIDE_RESISTANCE")
                    notes.append(
                        "Stage 1 fit resistance against logged motor current. "
                        "It also caps stall torque once current hits the V5 limit."
                    )
                else:
                    notes.append(
                        "Stage 1 held SIDE_RESISTANCE. The log current is ~0. "
                        "Resistance does not change the motion until stall current "
                        "would pass the V5 limit."
                    )
                print("stage 1 plant  " + ", ".join(keys))
                values, score = cma_minimize(
                    keys, values,
                    lambda trial: plant_cost(sim, rows, trial, window, windows, use_current),
                    generations, args.seed)
                notes.append(f"Stage 1 cost {score:.6g}.")
                apply_plant(sim, values)

    # Volts for a measured speed have to mean that speed on this plant.
    # Leaving the saturation speed free lets the search describe a slower
    # robot. The hold voltage then sits on the rail and the plant, which is
    # still below its real top speed, cannot be braked.
    values["LIN_SPEED_AT_SAT"] = values["MAX_VEL_MPS"]

    if args.stage in ("all", "gains"):
        apply_plant(sim, values)
        goals = goals_with_shape(args.shape, args.timed_elastic)
        print(f"stage 2 pose  shape {args.shape}")
        print("  " + ", ".join(TRACK_KEYS))
        print(
            f"  {restarts} restarts x {generations} generations x {popsize} candidates"
        )
        if args.timed_elastic:
            print("  timed elastic band deforms the base curve once, at plan time")
        print("  cost is the mean of drift, segment, and turn")
        print("  a drive ends when the range opens inside the finish radius")
        print("  with yaw rate within 0.5 rad/s, plus travel direction along the path")
        print("  a pivot ends inside 3 deg with that same yaw rate")
        print("  goal alignment is off")
        values, score = cma_minimize(
            TRACK_KEYS, values,
            lambda trial: track_cost(guide, segment, turn, sim, trial, ticks, goals),
            generations, args.seed,
            popsize=popsize, restarts=restarts, sigma=sigma,
            tolfun=0.0, tolx=0.0, tolstagnation=0,
            random_mean=random_mean)
        band = " + timed elastic" if args.timed_elastic else ""
        notes.append(
            f"Stage 2 cost {score:.6g} on {args.shape}{band} drives, segment drives, and turns"
            f" (pop {popsize}, restarts {restarts}, generations {generations})."
        )
        print_tracking(guide, segment, turn, sim, values, ticks, goals)
        write_paste(args.out, values, notes, HELD)
        print(f"  wrote {args.out}")

    write_paste(args.out, values, notes, HELD)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
