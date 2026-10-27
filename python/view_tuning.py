"""Play the tuning search's rollout on the field.

Same plant, goals, and tick count as python/optimize_tuning.py. The square
is the region around those goals. The orange line is the base curve,
planned once. --shape selects that curve. The timed elastic band stays
on unless --no-timed-elastic is set; it is not the orange line.
The blue trail is the chassis. With --compare, the compiled hand tune is
drawn in gray under the paste.

    python/.venv/Scripts/python.exe python/view_tuning.py
    python/.venv/Scripts/python.exe python/view_tuning.py --compare
    python/.venv/Scripts/python.exe python/view_tuning.py --nominal
    python/.venv/Scripts/python.exe python/view_tuning.py --save python/tuning_rollout.png --no-show
"""

import argparse
import math
import re
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon

sys.path.insert(0, str(Path(__file__).resolve().parent))

import plant
from optimize_tuning import (
    COMMAND_DELAY_TICKS, SHAPES, START_X_IN, START_Y_IN, PoseSettle,
    configure, delayed_step, goals_with_shape, localization_seed,
    nominal, require_plant, wrap,
)


HERE = Path(__file__).resolve().parent
DEFAULT_PASTE = HERE / "tuning_paste.txt"

# Same body the host field window draws.
_INCHES_PER_METER = 1.0 / 0.0254
_BODY_LENGTH_IN = 0.38 * _INCHES_PER_METER
_TRACK_WIDTH_IN = 0.27051 * _INCHES_PER_METER


def chassis_corners(x, y, theta):
    half_l = _BODY_LENGTH_IN / 2.0
    half_w = _TRACK_WIDTH_IN / 2.0
    local = np.array(
        [[half_l, half_w], [half_l, -half_w], [-half_l, -half_w], [-half_l, half_w]],
        dtype=float,
    )
    c = np.cos(theta)
    s = np.sin(theta)
    world = np.empty_like(local)
    world[:, 0] = x + local[:, 0] * c - local[:, 1] * s
    world[:, 1] = y + local[:, 0] * s + local[:, 1] * c
    return world

_NUM = r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?"
_ASSIGN = re.compile(rf"constexpr double (\w+) = ({_NUM});")
_ANGULAR = re.compile(rf"constexpr Axis Angular = \{{({_NUM}(?:\s*,\s*{_NUM})*)\}};")

# Paste names that differ from the tuner's dictionary.
_RENAME = {
    "VELOCITY_TAU_SEC": "VELOCITY_TAU",
    "SKID_TAU_SEC": "SKID_TAU",
    "SIDE_RESISTANCE_OHM": "SIDE_RESISTANCE",
    "MIN_ACCEL_MPS2": "MIN_ACCEL",
    "MAX_ACCEL_MPS2": "PRACTICE_ACCEL",
    "TURN_RADIUS_MAX_M": "TURN_RADIUS_MAX",
}


def load_paste(path):
    """Compiled nominal, with every assignment in the paste written over it."""
    text = path.read_text(encoding="utf-8")
    values = nominal()
    for name, raw in _ASSIGN.findall(text):
        values[_RENAME.get(name, name)] = float(raw)
    angular = _ANGULAR.search(text)
    if angular:
        parts = [float(part) for part in angular.group(1).split(",")]
        values["HEAD_KP"], values["HEAD_KI"], values["HEAD_KD"], values["HEAD_WINDUP"] = parts[:4]
    return values


def apply_plant(sim, values):
    sim.set_dynamics(
        values["VELOCITY_TAU"], values["SKID_TAU"],
        values["SIDE_RESISTANCE"], values["SPEED_SCALE"])


def planned_path(guide):
    sample = getattr(guide, "path", None) or getattr(guide, "base_path", None)
    if sample is None:
        return [], []
    points = sample(START_X_IN, START_Y_IN, 0.0, 0.0, 0.0)
    if len(points) < 2:
        return [], []
    return [p[0] for p in points], [p[1] for p in points]


def segment_distance(px, py, x, y):
    ax = px[:-1]
    ay = py[:-1]
    bx = px[1:]
    by = py[1:]
    dx = bx - ax
    dy = by - ay
    length2 = dx * dx + dy * dy
    length2 = np.where(length2 < 1e-12, 1.0, length2)
    t = np.clip(((x - ax) * dx + (y - ay) * dy) / length2, 0.0, 1.0)
    return np.hypot(x - (ax + t * dx), y - (ay + t * dy))


def peak_cross_in(states, path_x, path_y):
    if len(path_x) < 2:
        return 0.0
    px = np.asarray(path_x, dtype=float)
    py = np.asarray(path_y, dtype=float)
    peak = 0.0
    for state in states:
        dist = float(np.min(segment_distance(px, py, state.x_in, state.y_in)))
        if dist > peak:
            peak = dist
    return peak


def rollout(guide, sim, values, goal, practice, ticks):
    tx, ty, heading, exit_speed, shape, timed_elastic = goal
    apply_plant(sim, values)
    configure(guide, values, practice)
    guide.begin(tx, ty, heading, exit_speed, False, False, shape, timed_elastic)
    path_x, path_y = planned_path(guide)
    sim.localize_reset(
        START_X_IN, START_Y_IN, 0.0, 0.0, 0.0, 0.0,
        localization_seed(practice, tx, ty, heading),
    )
    states = [sim.state()]
    yaw = []
    # Same voltage delay, odometry estimate, and pose gate the tuner scores.
    # Past that gate the plant is no longer driving the move.
    pending = [(0.0, 0.0, 0.0) for _ in range(COMMAND_DELAY_TICKS)]
    settle = PoseSettle()
    for _ in range(ticks):
        est = sim.estimate()
        surge, sway, yaw_v = guide.command(
            est.x_in, est.y_in, est.theta, est.v_lin, est.v_ang)
        yaw.append(yaw_v)
        state = delayed_step(sim, pending, surge, sway, yaw_v)
        states.append(state)
        dist_in = math.hypot(state.x_in - tx, state.y_in - ty)
        if settle.update(dist_in, state.v_ang):
            break
    end = states[-1]
    miss = math.hypot(end.x_in - tx, end.y_in - ty)
    head_deg = abs(math.degrees(wrap(heading - end.theta)))
    return {
        "states": states,
        "path_x": path_x,
        "path_y": path_y,
        "goal_x": tx,
        "goal_y": ty,
        "heading": heading,
        "exit_speed": exit_speed,
        "practice": practice,
        "peak_cross_in": peak_cross_in(states, path_x, path_y),
        "miss_in": miss,
        "head_deg": head_deg,
        "peak_yaw": max((abs(v) for v in yaw), default=0.0),
        "seconds": (len(states) - 1) * float(plant.DT_SEC),
    }


def frame_limits(episodes):
    xs = [START_X_IN]
    ys = [START_Y_IN]
    for episode in episodes:
        xs.extend(state.x_in for state in episode["states"])
        ys.extend(state.y_in for state in episode["states"])
        xs.extend(episode["path_x"])
        ys.extend(episode["path_y"])
        xs.append(episode["goal_x"])
        ys.append(episode["goal_y"])
        other = episode.get("other")
        if other is not None:
            xs.extend(state.x_in for state in other["states"])
            ys.extend(state.y_in for state in other["states"])
            xs.extend(other["path_x"])
            ys.extend(other["path_y"])
    pad = 14.0
    lo_x, hi_x = min(xs) - pad, max(xs) + pad
    lo_y, hi_y = min(ys) - pad, max(ys) + pad
    span = max(hi_x - lo_x, hi_y - lo_y)
    cx = 0.5 * (lo_x + hi_x)
    cy = 0.5 * (lo_y + hi_y)
    half = 0.5 * span
    return cx - half, cx + half, cy - half, cy + half


def draw_field(ax, limits):
    lo_x, hi_x, lo_y, hi_y = limits
    ax.set_aspect("equal")
    ax.set_xlim(lo_x, hi_x)
    ax.set_ylim(lo_y, hi_y)
    ax.set_xlabel("x (in)")
    ax.set_ylabel("y (in)")
    ax.grid(True, color="#d5d5d5", linewidth=0.6)
    ax.add_patch(plt.Rectangle(
        (float(plant.FIELD_X_MIN_IN), float(plant.FIELD_Y_MIN_IN)),
        float(plant.FIELD_SIZE_IN),
        float(plant.FIELD_SIZE_IN),
        fill=False, edgecolor="#222222", linewidth=1.2,
    ))


def draw_robot(ax, pose, face, edge, nose):
    corners = chassis_corners(pose.x_in, pose.y_in, pose.theta)
    ax.add_patch(Polygon(
        corners, closed=True, facecolor=face, edgecolor=edge,
        linewidth=1.0, zorder=4, alpha=0.95,
    ))
    ax.plot(
        corners[:2, 0], corners[:2, 1], color=nose, linewidth=2.4,
        zorder=5, solid_capstyle="round",
    )


def draw_trail(ax, states, step, color, width):
    step = min(step, len(states) - 1)
    xs = [state.x_in for state in states[: step + 1]]
    ys = [state.y_in for state in states[: step + 1]]
    ax.plot(xs, ys, color=color, linewidth=width, zorder=2)
    return states[step]


def goal_label(episode):
    accel = "practice" if episode["practice"] else "path accel"
    return (
        f"({episode['goal_x']:.0f}, {episode['goal_y']:.0f}) in  "
        f"heading {episode['heading']:.2f}  exit {episode['exit_speed']:.1f} m/s  {accel}"
    )


def draw_episode(ax, episode, step, limits):
    draw_field(ax, limits)
    other = episode.get("other")
    if other is not None and other["path_x"]:
        ax.plot(other["path_x"], other["path_y"], color="#b9770e", linewidth=1.2, zorder=1, alpha=0.7)
    if episode["path_x"]:
        ax.plot(episode["path_x"], episode["path_y"], color="#d35400", linewidth=1.6, zorder=1)
    ax.plot(episode["goal_x"], episode["goal_y"], marker="x", color="#922b21", markersize=8, zorder=3)

    pose = draw_trail(ax, episode["states"], step, "#1b3a6b", 1.4)
    if other is not None:
        other_pose = draw_trail(ax, other["states"], step, "#7f8c8d", 1.2)
        draw_robot(ax, other_pose, "#95a5a6", "#2c3e50", "#f4d35e")
    draw_robot(ax, pose, "#2c6bed", "#102040", "#f4d35e")

    accel = "practice" if episode["practice"] else "path"
    ax.set_title(
        f"({episode['goal_x']:.0f}, {episode['goal_y']:.0f}) {accel}   "
        f"miss {episode['miss_in']:.1f} in   "
        f"head {episode['head_deg']:.1f} deg   "
        f"{episode['seconds']:.2f} s   "
        f"{pose.v_lin:.2f} m/s",
        fontsize=9,
    )


def play(episodes, rate, limits):
    stride = max(1, int(round(rate * 0.033 / float(plant.DT_SEC))))
    timeline = []
    for index, episode in enumerate(episodes):
        last = len(episode["states"]) - 1
        timeline.extend((index, step) for step in range(0, last + 1, stride))
        timeline.extend((index, last) for _ in range(15))

    fig, ax = plt.subplots(figsize=(7.2, 7.2))

    def update(frame):
        ax.clear()
        index, step = timeline[frame]
        draw_episode(ax, episodes[index], step, limits)
        return []

    anim = FuncAnimation(fig, update, frames=len(timeline), interval=33, blit=False, repeat=True)
    return fig, anim


def save_still(path, episodes, limits):
    count = len(episodes)
    cols = min(4, count)
    rows = (count + cols - 1) // cols
    fig, axes = plt.subplots(
        rows, cols, figsize=(6.2 * cols, 6.2 * rows), squeeze=False, layout="constrained",
    )
    for ax, episode in zip(axes.ravel(), episodes):
        draw_episode(ax, episode, len(episode["states"]) - 1, limits)
    for ax in axes.ravel()[count:]:
        ax.axis("off")
    fig.savefig(path, dpi=120)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Watch the tuning rollout on the field")
    parser.add_argument("--paste", type=Path, default=DEFAULT_PASTE)
    parser.add_argument("--nominal", action="store_true", help="compiled hand tune, ignore the paste")
    parser.add_argument("--compare", action="store_true", help="draw the compiled hand tune under the paste")
    parser.add_argument("--ticks", type=int, default=200, help="plant steps per goal; the search uses 200")
    parser.add_argument("--accel", choices=("both", "practice", "path"), default="both")
    parser.add_argument(
        "--shape", choices=SHAPES, default="Hermite",
        help="Base curve, planned once. Same names as DriftCurveShape.",
    )
    parser.add_argument(
        "--timed-elastic", action=argparse.BooleanOptionalAction, default=True,
        help="Local planner on that curve. On by default. The drawn path stays the base curve.",
    )
    parser.add_argument("--rate", type=float, default=1.0, help="playback speed relative to the 10 ms plant")
    parser.add_argument("--save", type=Path, default=None, help="png still of every goal, or gif of the playback")
    parser.add_argument("--show", action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args()

    if args.ticks < 1:
        raise SystemExit("--ticks must be at least 1")
    require_plant()

    if args.nominal:
        values = nominal()
        label = "compiled"
    else:
        if not args.paste.is_file():
            raise SystemExit(f"no paste at {args.paste}")
        values = load_paste(args.paste)
        label = args.paste.name
    hand = nominal() if args.compare and not args.nominal else None

    practice_flags = {"both": (True, False), "practice": (True,), "path": (False,)}[args.accel]
    sim = plant.PlantSim()
    if hasattr(sim, "set_goal_align"):
        sim.set_goal_align(False)
    guide = plant.DriftGuide()
    episodes = []
    for practice in practice_flags:
        for goal in goals_with_shape(args.shape, args.timed_elastic):
            episode = rollout(guide, sim, values, goal, practice, args.ticks)
            if hand is not None:
                episode["other"] = rollout(guide, sim, hand, goal, practice, args.ticks)
            episodes.append(episode)
            print(
                f"{label}: {goal_label(episode)}  "
                f"miss {episode['miss_in']:.1f} in, "
                f"head {episode['head_deg']:.1f} deg, "
                f"{episode['seconds']:.2f} s"
            )
            if hand is not None:
                other = episode["other"]
                print(
                    f"  compiled: miss {other['miss_in']:.1f} in, "
                    f"head {other['head_deg']:.1f} deg, {other['seconds']:.2f} s"
                )

    limits = frame_limits(episodes)
    if args.save is not None:
        suffix = args.save.suffix.lower()
        if suffix == ".gif":
            fig, anim = play(episodes, args.rate, limits)
            anim.save(args.save, writer=PillowWriter(fps=30))
            plt.close(fig)
        else:
            save_still(args.save, episodes, limits)
        print(f"wrote {args.save}")

    if args.show:
        fig, anim = play(episodes, args.rate, limits)
        fig._tuning_anim = anim
        plt.show()


if __name__ == "__main__":
    main()
