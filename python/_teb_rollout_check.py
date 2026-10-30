import math

from optimize_tuning import (
    COMMAND_DELAY_TICKS, START_X_IN, START_Y_IN, configure, delayed_step, nominal, wrap,
)
import plant


def rollout(goal, timed, practice, ticks=500):
    tx, ty, heading, exit_speed = goal
    guide = plant.DriftGuide()
    sim = plant.PlantSim()
    configure(guide, nominal(), practice)
    guide.begin(tx, ty, heading, exit_speed, False, False, "Hermite", timed)
    sim.localize_reset(START_X_IN, START_Y_IN, -math.pi / 2.0, 0.0, 0.0, 0.0, 1)
    pending = [(0.0, 0.0, 0.0) for _ in range(COMMAND_DELAY_TICKS)]
    yaws, speeds, thetas = [], [], []
    state = sim.state()
    done = False
    for _ in range(ticks):
        est = sim.estimate()
        surge, sway, yaw = guide.command(est.x_in, est.y_in, est.theta, est.v_lin, est.v_ang)
        yaws.append(yaw)
        state = delayed_step(sim, pending, surge, sway, yaw)
        speeds.append(state.v_lin)
        thetas.append(state.theta)
        dist = math.hypot(state.x_in - tx, state.y_in - ty)
        head = abs(wrap(heading - state.theta))
        speed_gate = 0.25 if exit_speed < 1e-3 else 0.15
        if dist < 2.0 and head < math.radians(8.0) and abs(state.v_lin - exit_speed) < speed_gate:
            done = True
            break
    flips = 0
    prev = 0
    for y in yaws:
        sign = 1 if y > 1.5 else (-1 if y < -1.5 else 0)
        if sign and prev and sign != prev:
            flips += 1
        if sign:
            prev = sign
    span = 0.0
    if thetas:
        acc = 0.0
        lo = hi = 0.0
        for th in thetas:
            acc += wrap(th - (thetas[0] + acc))
            lo = min(lo, acc)
            hi = max(hi, acc)
        span = math.degrees(hi - lo)
    dist = math.hypot(state.x_in - tx, state.y_in - ty)
    head = abs(math.degrees(wrap(heading - state.theta)))
    sat = sum(abs(y) > 10.0 for y in yaws) / max(1, len(yaws))
    mode = "practice" if practice else "path"
    band = "teb" if timed else "base"
    print(
        f"{band:4} {mode:8} ({tx:g},{ty:g}) h={heading:.2f} "
        f"{'done' if done else 'open':4} {len(yaws) * 0.01:4.2f}s "
        f"miss {dist:5.1f}in head {head:5.1f}deg v {state.v_lin:5.2f} "
        f"flips {flips:3d} sat {100 * sat:4.0f}% peakYaw {max(abs(y) for y in yaws):5.1f} "
        f"peakV {max(abs(s) for s in speeds):4.2f} nose {span:5.1f}deg"
    )


def trace(timed, ticks=40):
    tx, ty = 48.0, 72.0
    guide = plant.DriftGuide()
    sim = plant.PlantSim()
    configure(guide, nominal(), True)
    guide.begin(tx, ty, -math.pi / 2.0, 0.0, False, False, "Hermite", timed)
    sim.localize_reset(START_X_IN, START_Y_IN, -math.pi / 2.0, 0.0, 0.0, 0.0, 1)
    pending = [(0.0, 0.0, 0.0) for _ in range(COMMAND_DELAY_TICKS)]
    label = "teb" if timed else "base"
    print(f"--- {label} trace ---")
    for i in range(ticks):
        est = sim.estimate()
        surge, sway, yaw = guide.command(est.x_in, est.y_in, est.theta, est.v_lin, est.v_ang)
        pts = list(guide.path(est.x_in, est.y_in, est.theta, est.v_lin, est.v_ang))
        chord = 0.0
        if len(pts) >= 2:
            chord = math.degrees(math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0]) - math.pi / 2.0)
        state = delayed_step(sim, pending, surge, sway, yaw)
        if i % 5 == 0:
            print(
                f"{i:3d} th {math.degrees(state.theta):6.1f} surge {surge:6.2f} "
                f"yaw {yaw:6.2f} v {state.v_lin:5.2f} x {state.x_in:5.1f} chord {chord:6.1f}"
            )


trace(True)
trace(False)
print("--- goals ---")
goals = [
    (48.0, 72.0, -math.pi / 2.0, 0.0),
    (36.0, 96.0, 1.2 - math.pi / 2.0, 0.0),
    (24.0, 54.0, -0.6 - math.pi / 2.0, 0.0),
    (30.0, 82.0, 0.4 - math.pi / 2.0, 0.4),
    (-36.0, 72.0, math.pi / 2.0, 0.0),
    (20.0, 100.0, math.pi / 2.0, 0.0),
]
for goal in goals:
    for practice in (True, False):
        rollout(goal, True, practice)
        rollout(goal, False, practice)
