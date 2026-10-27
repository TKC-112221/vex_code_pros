"""Play the trained path policy on the field.

Uses the same plant and observation as training. The square is the 12 ft
field. The orange line is the path. Red marks on the driven trail are
the clock times when the thrust schedule was near zero. The robot
outline matches the host field window.

    python/.venv/Scripts/python.exe python/view_policy.py
    python/.venv/Scripts/python.exe python/view_policy.py --episodes 3 --seed 1
    python/.venv/Scripts/python.exe python/view_policy.py --save python/policy_rollout.png --no-show
"""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon, Rectangle
from stable_baselines3 import SAC

sys.path.insert(0, str(Path(__file__).resolve().parent))

import plant
from drive_env import PathFollowEnv


HERE = Path(__file__).resolve().parent
DEFAULT_CHECKPOINT = HERE / "drive_policy.zip"

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


def rollout(model, seed):
    env = PathFollowEnv()
    obs, _ = env.reset(seed=seed)
    path_x, path_y = env.path_polyline()
    states = [env.plant.state()]
    info = {"reason": "", "cross_in": 0.0, "thrust_err": 0.0, "peak_cross_in": 0.0}
    done = False
    while not done:
        action, _ = model.predict(obs, deterministic=True)
        obs, _, terminated, truncated, info = env.step(action)
        states.append(env.plant.state())
        done = terminated or truncated
    dt = float(plant.DT_SEC)
    thrust = np.array([env.thrust_at(i * dt) for i in range(len(states))], dtype=np.float64)
    return {
        "states": states,
        "path_x": path_x,
        "path_y": path_y,
        "thrust": thrust,
        "reason": info["reason"] or "running",
        "cross_in": info.get("peak_cross_in", info["cross_in"]),
        "thrust_err": info.get("thrust_err", 0.0),
        "seconds": (len(states) - 1) * dt,
    }


def draw_field(ax):
    ax.set_aspect("equal")
    pad = 8.0
    x0 = float(plant.FIELD_X_MIN_IN)
    y0 = float(plant.FIELD_Y_MIN_IN)
    size = float(plant.FIELD_SIZE_IN)
    ax.set_xlim(x0 - pad, x0 + size + pad)
    ax.set_ylim(y0 - pad, y0 + size + pad)
    ax.set_xlabel("x (in)")
    ax.set_ylabel("y (in)")
    ax.set_xticks(np.arange(x0, x0 + size + 1.0, 24.0))
    ax.set_yticks(np.arange(y0, y0 + size + 1.0, 24.0))
    ax.grid(True, color="#d5d5d5", linewidth=0.6)
    ax.add_patch(
        Rectangle(
            (x0, y0),
            size,
            size,
            fill=False,
            edgecolor="#222222",
            linewidth=1.5,
        )
    )


def draw_episode(ax, episode, step):
    draw_field(ax)
    states = episode["states"]
    step = min(step, len(states) - 1)
    xs = [s.x_in for s in states[: step + 1]]
    ys = [s.y_in for s in states[: step + 1]]
    ax.plot(episode["path_x"], episode["path_y"], color="#d35400", linewidth=1.6, zorder=1)
    thrust = episode["thrust"][: step + 1]
    slow = thrust < 0.3
    if np.any(slow):
        ax.plot(
            np.asarray(xs)[slow],
            np.asarray(ys)[slow],
            color="#c0392b",
            linewidth=3.0,
            zorder=3,
            linestyle="none",
            marker=".",
            markersize=3,
        )
    ax.plot(xs, ys, color="#1b3a6b", linewidth=1.4, zorder=2)

    pose = states[step]
    corners = chassis_corners(pose.x_in, pose.y_in, pose.theta)
    ax.add_patch(
        Polygon(corners, closed=True, facecolor="#2c6bed", edgecolor="#102040", linewidth=1.0, zorder=4)
    )
    ax.plot(corners[:2, 0], corners[:2, 1], color="#f4d35e", linewidth=2.4, zorder=5, solid_capstyle="round")

    ax.set_title(
        f"{episode['reason']}   {episode['seconds']:.2f} s   "
        f"peak cross {episode['cross_in']:.1f} in   "
        f"thrust err {episode['thrust_err']:.3f}   "
        f"now {pose.v_lin:.2f} m/s"
    )


def play(episodes, rate):
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
        draw_episode(ax, episodes[index], step)
        return []

    anim = FuncAnimation(fig, update, frames=len(timeline), interval=33, blit=False, repeat=True)
    return fig, anim


def save_still(path, episodes):
    count = len(episodes)
    fig, axes = plt.subplots(1, count, figsize=(7.2 * count, 7.2), squeeze=False)
    for ax, episode in zip(axes[0], episodes):
        draw_episode(ax, episode, len(episode["states"]) - 1)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Watch the trained drive policy on the field")
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--rate", type=float, default=1.0, help="playback speed relative to the 10 ms plant")
    parser.add_argument("--save", type=Path, default=None, help="png still of the full path, or gif of the playback")
    parser.add_argument("--show", action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args()

    if not args.checkpoint.is_file():
        raise SystemExit(f"no checkpoint at {args.checkpoint}; train with python/train_drive_policy.py first")
    if args.episodes < 1:
        raise SystemExit("--episodes must be at least 1")

    import train_drive_policy  # noqa: F401  (checkpoint names this policy class)
    model = SAC.load(args.checkpoint, device="cpu")
    episodes = []
    for index in range(args.episodes):
        episode = rollout(model, args.seed + index)
        episodes.append(episode)
        print(
            f"seed {args.seed + index}: {episode['reason']} in {episode['seconds']:.2f} s, "
            f"peak cross {episode['cross_in']:.1f} in, thrust err {episode['thrust_err']:.3f}"
        )

    if args.save is not None:
        suffix = args.save.suffix.lower()
        if suffix == ".gif":
            fig, anim = play(episodes, args.rate)
            anim.save(args.save, writer=PillowWriter(fps=30))
            plt.close(fig)
        else:
            save_still(args.save, episodes)
        print(f"wrote {args.save}")

    if args.show:
        fig, anim = play(episodes, args.rate)
        fig._policy_anim = anim
        plt.show()


if __name__ == "__main__":
    main()
