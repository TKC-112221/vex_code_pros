"""Gymnasium env: follow a generated path and its thrust schedule.

One action is one 10 ms plant step. The actor observation is
DrivePolicy::read:

    six strided frames of body velocity and the surge/yaw command,
    newest first, at the lags published by the plant binding,
    the path tangent 0.2 m ahead against the chassis heading,
    body-frame path points at 0.2, 0.5, 1.0, and 1.5 m,
    the thrust the schedule wants right now,
    signed cross-track.

The action is surge and yaw. This chassis cannot command sway, so the
plant step sends that axis as zero. reset() returns a dict: "obs" is the
actor vector, "priv" is the ground-truth pose, slip, and the friction
parameters the critic sees during training.

The thrust schedule is full throttle except on clock ramps. A ramp falls
to zero and rises back at a constant slope in time, whether or not the
robot is moving.

Every step is scored. The thrust term is the weight times the negative
absolute error. The path term is the user's asymmetric scale applied to
how fast the distance to the path is closing, minus a small weight on
that distance:

    0.8 * x if x > 0 else x
    - DIST_WEIGHT * distance

x is the closing rate, in units of CROSS_RATE_IN per step. An approach
is paid at 0.8 and a departure is charged in full, so leaving and coming
back cannot profit. The distance term is the rent for staying off the
line after that excursion. The raw distance derivative (positive when
the gap is opening) would flip the sign and pay the robot to leave.

The episode ends when the cross-track passes DIVERGE_IN, when the chassis
is inside the same arrival the tuner uses (within FINISH_RADIUS_IN of the
path end, yaw rate within FINISH_YAW_RAD_S of zero, and the distance to
that end has started to grow), or when the clock for this path runs out.
The last stage samples two field poses and connects them with a
turn-straight-turn at the tightest planned radius. That arrival also
requires the nose to be within POSE_HEAD_DEG of the exit heading.
"""

import sys
from pathlib import Path

import gymnasium as gym
import numpy as np
from gymnasium import spaces

sys.path.insert(0, str(Path(__file__).resolve().parent))
import path_params as P
import plant
from optimize_tuning import PoseSettle


def _wrap_angle(angle):
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


_DT = float(plant.DT_SEC)
_MAX_VEL_LIN = float(plant.MAX_VEL_LIN)
_MAX_VEL_ANG = float(plant.MAX_VEL_ANG)

# Ordered easy to hard. Training stays on a stage until the path is finished.
STAGES = P.STAGES


class _Path:
    def __init__(self, s, x, y, theta, kappa, dk, dips):
        self.s = s
        self.x = x
        self.y = y
        self.theta = theta
        self.kappa = kappa
        self.dk = dk
        self.straight_in = _longest_straight(s, kappa, dk)
        self.dips = int(dips)


class _ClockSchedule:
    """Full throttle, with V-shaped stops on the episode clock.

    Each event is (center_sec, half_sec). Thrust is |t − center| / half
    inside the window and 1 outside, and overlapping windows take the
    lower value. The slope on each side is constant.
    """

    def __init__(self, events):
        self.events = tuple(events)
        self.done_time = 0.0 if not events else max(center + half for center, half in events)

    def at(self, t):
        value = 1.0
        for center, half in self.events:
            if half <= 1e-6:
                continue
            value = min(value, abs(float(t) - center) / half)
        return float(min(1.0, value))


def _longest_straight(s, kappa, dk):
    longest = 0.0
    run = 0.0
    for i in range(len(s) - 1):
        straight = abs(kappa[i]) < P.KAPPA_STRAIGHT and abs(dk[i]) < P.DK_STRAIGHT and abs(kappa[i + 1]) < P.KAPPA_STRAIGHT
        if straight:
            run += float(s[i + 1] - s[i])
            longest = max(longest, run)
        else:
            run = 0.0
    return longest


def _stage_spec(stage, rng):
    if stage == "straight":
        return {"bends": 0, "sharp": False, "dips": 0}
    if stage == "straight-stop":
        return {"bends": 0, "sharp": False, "dips": 1}
    if stage == "gentle":
        return {"bends": 1, "sharp": False, "dips": 0}
    if stage == "mixed":
        return {"bends": int(rng.integers(*P.MIXED_BENDS)), "sharp": False, "dips": 1}
    return {"bends": int(rng.integers(*P.FULL_BENDS)), "sharp": True, "dips": int(rng.integers(*P.FULL_DIPS))}


def _append(samples, state, length, d2, straight):
    """Integrate one span. state is [kappa, theta, x, y, s], mutated in place."""
    length = max(float(length), P.DS_IN)
    n = max(1, int(round(length / P.DS_IN)))
    step = length / n
    kappa, theta, x, y, s = state
    for _ in range(n):
        if straight:
            k_next = 0.0
            d2_i = 0.0
        else:
            k_next = float(np.clip(kappa + d2 * step, -P.K_MAX, P.K_MAX))
            d2_i = (k_next - kappa) / step
        k_avg = 0.5 * (kappa + k_next)
        theta_next = theta + k_avg * step
        mid = theta + 0.5 * (theta_next - theta)
        x_next = x + (-np.sin(mid)) * step
        y_next = y + np.cos(mid) * step
        samples.append((s, x, y, theta, 0.0 if straight else kappa, d2_i))
        kappa = 0.0 if straight else k_next
        theta = theta if straight else theta_next
        x, y, s = x_next, y_next, s + step
    if straight:
        kappa = 0.0
    state[:] = (kappa, theta, x, y, s)


def _snap_time(t):
    return float(round(float(t) / _DT) * _DT)


def _clock_schedule(rng, dips, horizon_sec, length_in):
    """Place stops on the clock so a full-throttle pass is still on the path.

    From rest the teacher covers about 100 in/s, so the last ramp ends
    before 72% of length/105 s. Each side of a stop has one constant slope.
    """
    if dips <= 0 or horizon_sec <= P.MIN_CLOCK_SEC:
        return _ClockSchedule(())
    deadline = min(horizon_sec - P.DEADLINE_MARGIN_SEC,
                   max(P.DEADLINE_FLOOR_SEC, P.DEADLINE_LENGTH_FRAC * float(length_in) / P.TEACHER_SPEED_IPS))
    gap = P.EVENT_GAP_SEC
    span = deadline - P.EVENT_GAP_SEC
    room = span - (dips - 1) * gap
    max_half = max(_DT, room / (2.0 * dips))
    prefer = float(rng.uniform(*P.RAMP_HALF_SEC))
    half = max(_DT, _snap_time(min(prefer, max_half)))
    events = []
    cursor = P.EVENT_GAP_SEC
    for _ in range(dips):
        center = _snap_time(cursor + half)
        events.append((center, half))
        cursor = center + half + gap
    return _ClockSchedule(events)


def _generate_local(rng, stage):
    spec = _stage_spec(stage, rng)
    samples = []
    state = [0.0, 0.0, 0.0, 0.0, 0.0]
    _append(samples, state, float(rng.uniform(*P.STRAIGHT_OPEN_IN)), 0.0, True)
    for _ in range(spec["bends"]):
        sign = float(rng.choice([-1.0, 1.0]))
        peak = P.SHARP_K_PEAK if spec["sharp"] else P.GENTLE_K_PEAK
        k_peak = float(rng.uniform(P.K_PEAK_MIN, peak)) * sign
        d2_mag = float(rng.uniform(*P.D2_RANGE))
        _append(samples, state, abs(k_peak) / d2_mag, d2_mag * sign, False)
        _append(samples, state, float(rng.uniform(*P.BEND_HOLD_IN)), 0.0, False)
        kappa = state[0]
        if abs(kappa) > P.KAPPA_FLAT:
            _append(samples, state, abs(kappa) / d2_mag, -np.sign(kappa) * d2_mag, False)
        state[0] = 0.0
        _append(samples, state, float(rng.uniform(*P.STRAIGHT_BETWEEN_IN)), 0.0, True)
    if spec["bends"] == 0:
        _append(samples, state, float(rng.uniform(*P.NO_BEND_STRAIGHT_IN)), 0.0, True)
    # Full-throttle passes from rest cover about 100 in/s. A stop on the
    # clock has to finish while that pass is still on the path.
    dip_floor = P.DIP_BASE_IN + P.DIP_EXTRA_IN * (spec["dips"] - 1)
    if spec["dips"] > 0 and state[4] < dip_floor:
        _append(samples, state, dip_floor - state[4], 0.0, True)
    kappa, theta, x, y, s = state
    samples.append((s, x, y, theta, 0.0, 0.0))
    cols = np.asarray(samples, dtype=np.float64)
    path = _Path(cols[:, 0], cols[:, 1], cols[:, 2], cols[:, 3], cols[:, 4], cols[:, 5], spec["dips"])
    if path.straight_in < P.MIN_STRAIGHT_IN:
        return None
    if spec["bends"] > 0 and float(np.max(np.abs(path.kappa))) < P.MIN_BEND_KAPPA:
        return None
    return path


def _field_box():
    margin = P.WALL_MARGIN_IN
    return (
        float(plant.FIELD_X_MIN_IN) + margin,
        float(plant.FIELD_X_MAX_IN) - margin,
        float(plant.FIELD_Y_MIN_IN) + margin,
        float(plant.FIELD_Y_MAX_IN) - margin,
    )


def _place_on_field(rng, path):
    ang = float(rng.uniform(-np.pi, np.pi))
    c = np.cos(ang)
    s = np.sin(ang)
    x = path.x * c - path.y * s
    y = path.x * s + path.y * c
    x_min, x_max, y_min, y_max = _field_box()
    minx, maxx = float(x.min()), float(x.max())
    miny, maxy = float(y.min()), float(y.max())
    if maxx - minx > x_max - x_min or maxy - miny > y_max - y_min:
        return False
    ox = float(rng.uniform(x_min - minx, x_max - maxx))
    oy = float(rng.uniform(y_min - miny, y_max - maxy))
    path.x = np.ascontiguousarray(x + ox)
    path.y = np.ascontiguousarray(y + oy)
    path.theta = np.ascontiguousarray(path.theta + ang)
    return True


def _mod2pi(angle):
    return float(angle - 2.0 * np.pi * np.floor(angle / (2.0 * np.pi)))


def _dubins_segments(alpha, beta, d):
    """Turn-straight-turn families. Lengths are in units of the turn radius."""
    ca = float(np.cos(alpha))
    sa = float(np.sin(alpha))
    cb = float(np.cos(beta))
    sb = float(np.sin(beta))
    cab = float(np.cos(alpha - beta))
    found = []

    tmp0 = d + sa - sb
    p2 = 2.0 + d * d - 2.0 * cab + 2.0 * d * (sa - sb)
    if p2 >= 0.0:
        tmp1 = float(np.arctan2(cb - ca, tmp0))
        found.append((_mod2pi(-alpha + tmp1), float(np.sqrt(p2)), _mod2pi(beta - tmp1), "LSL"))

    tmp0 = d - sa + sb
    p2 = 2.0 + d * d - 2.0 * cab + 2.0 * d * (sb - sa)
    if p2 >= 0.0:
        tmp1 = float(np.arctan2(ca - cb, tmp0))
        found.append((_mod2pi(alpha - tmp1), float(np.sqrt(p2)), _mod2pi(-beta + tmp1), "RSR"))

    p2 = -2.0 + d * d + 2.0 * cab + 2.0 * d * (sa + sb)
    if p2 >= 0.0:
        p = float(np.sqrt(p2))
        tmp2 = float(np.arctan2(-ca - cb, d + sa + sb) - np.arctan2(-2.0, p))
        found.append((_mod2pi(-alpha + tmp2), p, _mod2pi(-beta + tmp2), "LSR"))

    p2 = d * d - 2.0 + 2.0 * cab - 2.0 * d * (sa + sb)
    if p2 >= 0.0:
        p = float(np.sqrt(p2))
        tmp2 = float(np.arctan2(ca + cb, d - sa - sb) - np.arctan2(2.0, p))
        found.append((_mod2pi(alpha - tmp2), p, _mod2pi(beta - tmp2), "RSL"))
    return found


_DUBINS_KIND = {
    "LSL": ("L", "S", "L"),
    "RSR": ("R", "S", "R"),
    "LSR": ("L", "S", "R"),
    "RSL": ("R", "S", "L"),
}


def _segment_pose(x, y, theta, amount, kind, radius):
    if kind == "L":
        return (
            x + radius * (np.cos(theta + amount) - np.cos(theta)),
            y + radius * (np.sin(theta + amount) - np.sin(theta)),
            theta + amount,
            P.K_MAX,
        )
    if kind == "R":
        return (
            x + radius * (np.cos(theta) - np.cos(theta - amount)),
            y + radius * (np.sin(theta) - np.sin(theta - amount)),
            theta - amount,
            -P.K_MAX,
        )
    return (
        x + radius * amount * (-np.sin(theta)),
        y + radius * amount * np.cos(theta),
        theta,
        0.0,
    )


def _sample_dubins(x0, y0, h0, segments, radius):
    samples = []
    x, y, theta = float(x0), float(y0), float(h0)
    arc = 0.0
    last_kappa = 0.0
    for amount, kind in segments:
        amount = float(amount)
        length = amount * radius
        if length < 1e-6:
            continue
        n = max(1, int(round(length / P.DS_IN)))
        step_a = amount / n
        step_s = length / n
        for i in range(n):
            px, py, pth, kappa = _segment_pose(x, y, theta, step_a * i, kind, radius)
            samples.append((arc, px, py, pth, kappa, 0.0))
            arc += step_s
        x, y, theta, last_kappa = _segment_pose(x, y, theta, amount, kind, radius)
    samples.append((arc, x, y, theta, last_kappa, 0.0))
    return samples, (x, y, theta)


def _inside_field(samples, box):
    x0, x1, y0, y1 = box
    for _s, x, y, _theta, _kappa, _dk in samples:
        if x < x0 or x > x1 or y < y0 or y > y1:
            return False
    return True


def _sample_pose(rng, box):
    x0, x1, y0, y1 = box
    return (
        float(rng.uniform(x0, x1)),
        float(rng.uniform(y0, y1)),
        float(rng.uniform(-np.pi, np.pi)),
    )


def _pose_fallback():
    n = P.FALLBACK_SAMPLES
    s = np.linspace(0.0, P.FALLBACK_LENGTH_IN, n)
    return _Path(
        s,
        s - P.FALLBACK_LENGTH_IN * 0.5,
        np.full(n, float(plant.FIELD_CENTER_Y_IN)),
        np.zeros(n),
        np.zeros(n),
        np.zeros(n),
        0,
    )


def _pose_path(rng):
    """Shortest in-bounds turn-straight-turn between two random field poses."""
    box = _field_box()
    radius = 1.0 / P.K_MAX
    for _ in range(P.GENERATE_TRIES):
        x0, y0, h0 = _sample_pose(rng, box)
        x1, y1, h1 = _sample_pose(rng, box)
        dx = x1 - x0
        dy = y1 - y0
        chord = float(np.hypot(dx, dy))
        if chord < P.POSE_MIN_IN or chord > P.POSE_MAX_IN:
            continue
        bearing = float(np.arctan2(dy, dx))
        alpha = _mod2pi(h0 - bearing)
        beta = _mod2pi(h1 - bearing)
        best = None
        for t, p, q, name in _dubins_segments(alpha, beta, chord / radius):
            length = (t + p + q) * radius
            if length > P.POSE_MAX_LENGTH_IN:
                continue
            if best is not None and length >= best[0]:
                continue
            kinds = _DUBINS_KIND[name]
            samples, end = _sample_dubins(
                x0, y0, h0,
                ((t, kinds[0]), (p, kinds[1]), (q, kinds[2])),
                radius,
            )
            ex, ey, eh = end
            if np.hypot(ex - x1, ey - y1) > 1.0 or abs(_wrap_angle(eh - h1)) > 0.05:
                continue
            if not _inside_field(samples, box):
                continue
            best = (length, samples)
        if best is None:
            continue
        cols = np.asarray(best[1], dtype=np.float64)
        return _Path(cols[:, 0], cols[:, 1], cols[:, 2], cols[:, 3], cols[:, 4], cols[:, 5], 0)
    return _pose_fallback()


def _fallback_path(rng, stage):
    spec = _stage_spec(stage, rng)
    n = P.FALLBACK_SAMPLES
    s = np.linspace(0.0, P.FALLBACK_LENGTH_IN, n)
    return _Path(
        s,
        s - P.FALLBACK_LENGTH_IN * 0.5,
        np.full(n, float(plant.FIELD_CENTER_Y_IN)),
        np.zeros(n),
        np.zeros(n),
        np.zeros(n),
        spec["dips"],
    )


# Estimator noise on the actor's velocity history. The privileged critic
# vector keeps the clean plant.
_VEL_NOISE_MPS = 0.04
_YAW_NOISE_RAD = 0.08


def _dynamics_bounds():
    """Uniform window around the nominal plant. Shared by reset and the critic."""
    tau = float(plant.VELOCITY_TAU)
    skid = float(plant.SKID_TAU)
    resistance = float(plant.SIDE_RESISTANCE)
    scale = float(plant.SPEED_SCALE)
    return (
        (0.5 * tau, 2.0 * tau),
        (0.35 * skid, 3.0 * skid),
        (0.65 * resistance, 1.6 * resistance),
        (0.75 * scale, 1.15 * scale),
    )


def _sample_dynamics(rng):
    """Uniform draw. Skid is the wide axis: tiles, dust, and roller wear."""
    return tuple(float(rng.uniform(lo, hi)) for lo, hi in _dynamics_bounds())


def _span_unit(value, bounds):
    lo, hi = bounds
    if hi <= lo:
        return 0.0
    return float(np.clip(2.0 * (value - lo) / (hi - lo) - 1.0, -1.0, 1.0))


def generate_path(rng, stage):
    if stage == "pose":
        return _pose_path(rng)
    for _ in range(P.GENERATE_TRIES):
        path = _generate_local(rng, stage)
        if path is not None and _place_on_field(rng, path):
            return path
    return _fallback_path(rng, stage)


class PathFollowEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, gamma=P.DEFAULT_GAMMA):
        super().__init__()
        self.plant = plant.PlantSim()
        # The grab is process-global and defaults on. One touch seats every
        # later episode, including the other env, on that goal.
        self.plant.set_goal_align(False)
        # Discount lives on the trainer. The step reward is undiscounted.
        self.gamma = float(gamma)
        self._stage = STAGES[-1]
        if not hasattr(plant, "HISTORY_LAG") or not hasattr(plant, "LOOKAHEAD_M"):
            raise RuntimeError("plant module is stale; rebuild with python python/build_plant.py")
        self._lags = np.asarray(plant.HISTORY_LAG, dtype=np.int64)
        self._hist_depth = int(self._lags.max()) + 1
        actor = spaces.Box(low=-np.inf, high=np.inf, shape=(int(plant.FEATURE_COUNT),), dtype=np.float32)
        # pose (x, y, sin, cos), body velocity, and the four dynamics knobs.
        priv = spaces.Box(low=-np.inf, high=np.inf, shape=(11,), dtype=np.float32)
        self.observation_space = spaces.Dict({"obs": actor, "priv": priv})
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(int(plant.ACTION_COUNT),), dtype=np.float32
        )
        self._path = None
        self._hint = 0
        self._prev_dist = 0.0
        self._prev_action = np.zeros(int(plant.ACTION_COUNT), dtype=np.float64)
        self._hist = np.zeros((self._hist_depth, 5), dtype=np.float64)
        self._steps = 0
        self._horizon = P.MAX_HORIZON
        self._proj = None
        self._theta = 0.0
        self._thrust_err_sum = 0.0
        self._peak_cross = 0.0
        self._arrival = PoseSettle()
        self._schedule = _ClockSchedule(())

    def set_stage(self, name):
        if name not in STAGES:
            raise ValueError(f"unknown curriculum stage {name}")
        self._stage = name

    def set_progress(self, fraction):
        last = len(STAGES) - 1
        index = min(int(np.floor(float(np.clip(fraction, 0.0, 1.0)) * last + 1e-9)), last)
        self._stage = STAGES[index]

    def curriculum_stage(self):
        return self._stage

    def path_polyline(self):
        path = self._path
        return path.x.copy(), path.y.copy()

    def thrust_at(self, t):
        return self._schedule.at(t)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.plant.set_goal_align(False)
        velocity_tau, skid_tau, resistance, speed_scale = _sample_dynamics(self.np_random)
        self.plant.set_dynamics(velocity_tau, skid_tau, resistance, speed_scale)
        self._path = generate_path(self.np_random, self._stage)
        self._hint = 0
        self._steps = 0
        self._prev_action = np.zeros(int(plant.ACTION_COUNT), dtype=np.float64)
        self._hist = np.zeros((self._hist_depth, 5), dtype=np.float64)
        self._thrust_err_sum = 0.0
        self._peak_cross = 0.0
        self._arrival = PoseSettle()
        length = float(self._path.s[-1] - self._path.s[0])
        # 20 in/s is a slow crawl. Stops add two seconds each. A policy
        # that never leaves the start runs out the clock.
        crawl_steps = length / P.CRAWL_IPS / _DT
        self._horizon = int(np.clip(crawl_steps + P.DIP_EXTRA_STEPS * max(self._path.dips, 0) + P.HORIZON_PAD_STEPS,
                                     P.MIN_HORIZON, P.MAX_HORIZON))
        self._schedule = _clock_schedule(
            self.np_random, self._path.dips, self._horizon * _DT, length
        )

        phi = float(self._path.theta[0])
        offset = float(self.np_random.uniform(-P.START_OFFSET_IN, P.START_OFFSET_IN))
        x = float(self._path.x[0] - np.cos(phi) * offset)
        y = float(self._path.y[0] - np.sin(phi) * offset)
        self._theta = phi + float(self.np_random.uniform(-P.START_HEADING_RAD, P.START_HEADING_RAD))
        self.plant.reset(x, y, self._theta, 0.0, 0.0, 0.0)
        state = self.plant.state()
        self._proj = self._project(state.x_in, state.y_in)
        self._prev_dist = abs(self._proj["cross"])
        self._peak_cross = self._prev_dist
        return self._obs(state), {
            "cross_in": self._prev_dist,
            "path_s": float(self._path.s[-1]),
            "straight_in": self._path.straight_in,
        }

    def step(self, action):
        command = np.clip(np.asarray(action, dtype=np.float64).reshape(-1), -1.0, 1.0)
        # The schedule the policy just saw. Scoring the later clock time
        # would charge a copy of that schedule for the step just taken.
        target = self.thrust_at(self._steps * _DT)
        surge = float(command[0]) * _MAX_VEL_LIN
        yaw = float(command[1]) * _MAX_VEL_ANG if command.size > 1 else 0.0
        self._prev_action[:] = 0.0
        self._prev_action[0] = float(command[0])
        if command.size > 1 and self._prev_action.size > 1:
            self._prev_action[1] = float(command[1])
        self.plant.step_velocity(surge, 0.0, yaw)
        state = self.plant.state()
        self._theta = float(state.theta)
        proj = self._project(state.x_in, state.y_in)
        self._proj = proj
        dist = abs(proj["cross"])
        closing = self._prev_dist - dist
        rate = closing / P.CROSS_RATE_IN
        path_term = P.APPROACH_GAIN * rate if rate > 0.0 else rate
        thrust = float(command[0])
        reward = (
            P.THRUST_WEIGHT * (-abs(thrust - target))
            + path_term
            - P.DIST_WEIGHT * dist
        )

        self._prev_dist = dist
        self._steps += 1
        self._thrust_err_sum += abs(thrust - target)
        self._peak_cross = max(self._peak_cross, dist)

        terminated = False
        truncated = False
        reason = ""
        goal_dist = float(np.hypot(
            state.x_in - self._path.x[-1], state.y_in - self._path.y[-1]))
        goal_head_deg = abs(float(np.degrees(_wrap_angle(
            float(self._path.theta[-1]) - float(state.theta)))))
        settled = self._arrival.update(goal_dist, float(state.v_ang))
        if self._stage == "pose" and goal_head_deg > P.POSE_HEAD_DEG:
            settled = False
        if dist > P.DIVERGE_IN:
            terminated = True
            reason = "diverged"
        elif settled:
            terminated = True
            reason = "finished"
        elif self._steps >= self._horizon:
            truncated = True
            reason = "timeout"

        info = {
            "cross_in": dist,
            "signed_cross_in": float(proj["cross"]),
            "target_thrust": target,
            "thrust": thrust,
            "path_s": float(proj["s"]),
            "reason": reason,
            "goal_dist_in": goal_dist,
            "goal_head_deg": goal_head_deg,
        }
        if terminated or truncated:
            info["is_success"] = reason == "finished"
            info["thrust_err"] = self._thrust_err_sum / float(self._steps)
            info["peak_cross_in"] = self._peak_cross
        return self._obs(state), float(reward), terminated, truncated, info

    def tracking_action(self):
        """Heading-and-cross-track tracker. The cloning teacher, not the plant."""
        proj = self._proj
        ahead = self._sample_at(min(proj["s"] + P.LOOKAHEAD_IN, float(self._path.s[-1])))
        err = _wrap_angle(ahead["phi"] - self._theta)
        state = self.plant.state()
        speed_ips = float(state.v_lin) / 0.0254
        # v * κ is the yaw rate the path wants. The 0.09 turns that into a
        # normalized yaw command; the heading and cross-track terms correct.
        yaw = P.YAW_HEADING * err - P.YAW_CROSS * proj["cross"] + P.YAW_CURVATURE * speed_ips * ahead["kappa"]
        action = np.array([self.thrust_at(self._steps * _DT), yaw], dtype=np.float32)
        return np.clip(action, -1.0, 1.0)

    def _project(self, x, y):
        path = self._path
        last = len(path.s) - 2
        i0 = max(0, self._hint - P.HINT_BACK)
        i1 = min(last, self._hint + P.HINT_AHEAD)
        best_i = i0
        best_t = 0.0
        best_d2 = 1e18
        best_cx = float(path.x[i0])
        best_cy = float(path.y[i0])
        for i in range(i0, i1 + 1):
            ax = float(path.x[i])
            ay = float(path.y[i])
            vx = float(path.x[i + 1] - ax)
            vy = float(path.y[i + 1] - ay)
            seg2 = vx * vx + vy * vy
            if seg2 < 1e-8:
                t = 0.0
                cx, cy = ax, ay
            else:
                t = ((x - ax) * vx + (y - ay) * vy) / seg2
                t = float(np.clip(t, 0.0, 1.0))
                cx = ax + t * vx
                cy = ay + t * vy
            d2 = (x - cx) ** 2 + (y - cy) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
                best_t = t
                best_cx, best_cy = cx, cy
        self._hint = best_i

        def mix(values):
            return (1.0 - best_t) * float(values[best_i]) + best_t * float(values[best_i + 1])

        phi = mix(path.theta)
        dx = float(x) - best_cx
        dy = float(y) - best_cy
        cross = -np.cos(phi) * dx - np.sin(phi) * dy
        return {
            "s": mix(path.s),
            "phi": phi,
            "kappa": mix(path.kappa),
            "dk": mix(path.dk),
            "cross": float(cross),
        }

    def _sample_at(self, s_query):
        s = self._path.s
        s_query = float(np.clip(s_query, s[0], s[-1]))
        index = int(np.searchsorted(s, s_query, side="right") - 1)
        index = min(max(index, 0), len(s) - 2)
        span = float(s[index + 1] - s[index])
        t = 0.0 if span < 1e-6 else (s_query - float(s[index])) / span

        def mix(values):
            return (1.0 - t) * float(values[index]) + t * float(values[index + 1])

        return {
            "x": mix(self._path.x),
            "y": mix(self._path.y),
            "phi": mix(self._path.theta),
            "kappa": mix(self._path.kappa),
            "dk": mix(self._path.dk),
        }

    def _push_history(self, state):
        self._hist[1:] = self._hist[:-1]
        self._hist[0, 0] = float(state.v_lin) + float(self.np_random.normal(0.0, _VEL_NOISE_MPS))
        self._hist[0, 1] = float(state.v_lat) + float(self.np_random.normal(0.0, _VEL_NOISE_MPS))
        self._hist[0, 2] = float(state.v_ang) + float(self.np_random.normal(0.0, _YAW_NOISE_RAD))
        self._hist[0, 3] = float(self._prev_action[0])
        self._hist[0, 4] = float(self._prev_action[1]) if self._prev_action.size > 1 else 0.0

    def _lookahead(self, state):
        theta = float(state.theta)
        c = float(-np.sin(theta))
        s = float(np.cos(theta))
        inch = float(plant.METERS_PER_INCH)
        s_end = float(self._path.s[-1])
        xs = []
        ys = []
        for dist_m in plant.LOOKAHEAD_M:
            sample = self._sample_at(min(self._proj["s"] + float(dist_m) / inch, s_end))
            dx = (float(sample["x"]) - float(state.x_in)) * inch
            dy = (float(sample["y"]) - float(state.y_in)) * inch
            xs.append(c * dx + s * dy)
            ys.append(-s * dx + c * dy)
        return xs, ys

    def _privileged(self, state):
        half = float(plant.FIELD_HALF_IN)
        center_y = float(plant.FIELD_CENTER_Y_IN)
        lin = float(plant.MAX_VEL_LIN)
        ang = float(plant.MAX_VEL_ANG)
        tau, skid, resistance, scale = self.plant.dynamics()
        bounds = _dynamics_bounds()
        values = (
            float(state.x_in) / half,
            (float(state.y_in) - center_y) / half,
            float(np.cos(state.theta)),
            float(-np.sin(state.theta)),
            float(state.v_lin) / lin,
            float(state.v_lat) / lin,
            float(state.v_ang) / ang,
            _span_unit(tau, bounds[0]),
            _span_unit(skid, bounds[1]),
            _span_unit(resistance, bounds[2]),
            _span_unit(scale, bounds[3]),
        )
        return np.asarray(values, dtype=np.float32)

    def _obs(self, state):
        self._push_history(state)
        look_x, look_y = self._lookahead(state)
        near = self._sample_at(
            min(self._proj["s"] + float(plant.LOOKAHEAD_M[0]) / float(plant.METERS_PER_INCH),
                float(self._path.s[-1]))
        )
        rows = self._hist[self._lags]
        reading = plant.read_policy(
            rows[:, 0].tolist(),
            rows[:, 1].tolist(),
            rows[:, 2].tolist(),
            rows[:, 3].tolist(),
            rows[:, 4].tolist(),
            look_x,
            look_y,
            float(near["phi"]),
            float(state.theta),
            float(self.thrust_at(self._steps * _DT)),
            float(self._proj["cross"]),
        )
        return {
            "obs": np.asarray(reading.features, dtype=np.float32),
            "priv": self._privileged(state),
        }


# The trainer and the viewer imported this name when the task was a pose.
DriveToPoseEnv = PathFollowEnv
