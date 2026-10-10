"""Gymnasium env: follow a generated path and its thrust schedule.

One action is one 10 ms plant step. The observation is DrivePolicy::read:

    previous chassis command,
    direction of the velocity relative to the path tangent,
    dφ/ds and d²φ/ds² of that tangent,
    the thrust the schedule wants right now,
    signed cross-track.

φ is the path heading a short distance ahead. Its arc-length derivatives
are zero on a straight. The action is still surge, sway, and yaw: this
actor replaces DriftFollower, so the steer is part of the command.

The thrust schedule is full throttle except on clock ramps. A ramp falls
to zero and rises back at a constant slope in time, whether or not the
robot is moving.

Every step is scored. There is no terminal bonus. The thrust term is the
weight times the negative absolute error. The path term is the user's
asymmetric scale applied to how fast the distance to the path is closing,
minus a small weight on that distance:

    0.8 * x if x > 0 else x
    - DIST_WEIGHT * distance

x is the closing rate, in units of CROSS_RATE_IN per step. An approach
is paid at 0.8 and a departure is charged in full, so leaving and coming
back cannot profit. The distance term is the rent for staying off the
line after that excursion. The raw distance derivative (positive when
the gap is opening) would flip the sign and pay the robot to leave.

The episode ends when the cross-track passes DIVERGE_IN, when the robot
reaches the end of the path still inside that band, or when the clock
for this path runs out.
"""

import sys
from pathlib import Path

import gymnasium as gym
import numpy as np
from gymnasium import spaces

sys.path.insert(0, str(Path(__file__).resolve().parent))
import path_params as P
import plant


def _wrap_angle(angle):
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


_DT = float(plant.DT_SEC)
_MAX_VOLTS = float(plant.MAX_VOLTS)

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
        x_next = x + np.cos(mid) * step
        y_next = y + np.sin(mid) * step
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


def _place_on_field(rng, path):
    ang = float(rng.uniform(-np.pi, np.pi))
    c = np.cos(ang)
    s = np.sin(ang)
    x = path.x * c - path.y * s
    y = path.x * s + path.y * c
    half = float(plant.FIELD_HALF_IN) - P.WALL_MARGIN_IN
    minx, maxx = float(x.min()), float(x.max())
    miny, maxy = float(y.min()), float(y.max())
    if maxx - minx > 2.0 * half or maxy - miny > 2.0 * half:
        return False
    ox = float(rng.uniform(-half - minx, half - maxx))
    oy = float(rng.uniform(-half - miny, half - maxy))
    path.x = np.ascontiguousarray(x + ox)
    path.y = np.ascontiguousarray(y + oy)
    path.theta = np.ascontiguousarray(path.theta + ang)
    return True


def _fallback_path(rng, stage):
    spec = _stage_spec(stage, rng)
    n = P.FALLBACK_SAMPLES
    s = np.linspace(0.0, P.FALLBACK_LENGTH_IN, n)
    return _Path(
        s,
        s - P.FALLBACK_LENGTH_IN * 0.5,
        np.zeros(n),
        np.zeros(n),
        np.zeros(n),
        np.zeros(n),
        spec["dips"],
    )


def generate_path(rng, stage):
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
        # Discount lives on the trainer. The step reward is undiscounted.
        self.gamma = float(gamma)
        self._stage = STAGES[-1]
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(plant.FEATURE_COUNT,), dtype=np.float32
        )
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(plant.ACTION_COUNT,), dtype=np.float32
        )
        self._path = None
        self._hint = 0
        self._prev_dist = 0.0
        self._prev_action = np.zeros(plant.ACTION_COUNT, dtype=np.float64)
        self._steps = 0
        self._horizon = P.MAX_HORIZON
        self._proj = None
        self._theta = 0.0
        self._thrust_err_sum = 0.0
        self._peak_cross = 0.0
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
        self._path = generate_path(self.np_random, self._stage)
        self._hint = 0
        self._steps = 0
        self._prev_action = np.zeros(plant.ACTION_COUNT, dtype=np.float64)
        self._thrust_err_sum = 0.0
        self._peak_cross = 0.0
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
        x = float(self._path.x[0] - np.sin(phi) * offset)
        y = float(self._path.y[0] + np.cos(phi) * offset)
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
        command = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        # The schedule the policy just saw. Scoring the later clock time
        # would charge a copy of that schedule for the step just taken.
        target = self.thrust_at(self._steps * _DT)
        volts = command * _MAX_VOLTS
        self.plant.step(float(volts[0]), float(volts[1]), float(volts[2]))
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
        self._prev_action = command
        self._steps += 1
        self._thrust_err_sum += abs(thrust - target)
        self._peak_cross = max(self._peak_cross, dist)

        terminated = False
        truncated = False
        reason = ""
        if dist > P.DIVERGE_IN:
            terminated = True
            reason = "diverged"
        elif proj["s"] >= float(self._path.s[-1]) - P.FINISH_IN:
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
        action = np.array([self.thrust_at(self._steps * _DT), 0.0, yaw], dtype=np.float32)
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
        cross = -np.sin(phi) * dx + np.cos(phi) * dy
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

        return {"phi": mix(self._path.theta), "kappa": mix(self._path.kappa), "dk": mix(self._path.dk)}

    def _velocity_heading(self, state):
        vx = state.v_lin * np.cos(state.theta) - state.v_lat * np.sin(state.theta)
        vy = state.v_lin * np.sin(state.theta) + state.v_lat * np.cos(state.theta)
        if float(np.hypot(vx, vy)) < P.STOPPED_SPEED_MPS:
            return float(state.theta)
        return float(np.arctan2(vy, vx))

    def _obs(self, state):
        ahead = self._sample_at(min(self._proj["s"] + P.LOOKAHEAD_IN, float(self._path.s[-1])))
        reading = plant.read_policy(
            float(self._prev_action[0]),
            float(self._prev_action[1]),
            float(self._prev_action[2]),
            float(ahead["phi"]),
            self._velocity_heading(state),
            float(ahead["kappa"]),
            float(ahead["dk"]),
            float(self.thrust_at(self._steps * _DT)),
            float(self._proj["cross"]),
        )
        return np.asarray(reading.features, dtype=np.float32)


# The trainer and the viewer imported this name when the task was a pose.
DriveToPoseEnv = PathFollowEnv
