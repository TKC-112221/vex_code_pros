"""Preview the arc the timed elastic band plans from a start pose to a goal.

The orange dashed line is the base curve, planned once. The teal line is
the band's one deformation of that curve, with a dot on each sample. A
magenta segment is a reversal. Drag either pose. Scroll on a pose to turn
it. Press p to print the samples.

    python python/view_teb.py
    python python/view_teb.py --start 0 72 0 --goal 36 96 69 --exit-speed 0
    python python/view_teb.py --start 0 72 90 --goal -24 102 90 --no-show

Headings on the command line are degrees. Positions are inches, in the
same frame as the tuner: x is -72..72, y is 0..144.
"""

import argparse
import math
import sys
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.backend_bases import MouseButton
from matplotlib.patches import Polygon
from matplotlib.widgets import CheckButtons, RadioButtons, Slider

sys.path.insert(0, str(Path(__file__).resolve().parent))

import plant
from optimize_tuning import SHAPES, configure, nominal, require_plant, wrap


_INCHES_PER_METER = 1.0 / 0.0254
_BODY_LENGTH_IN = 0.38 * _INCHES_PER_METER
_TRACK_WIDTH_IN = 0.27051 * _INCHES_PER_METER
_PICK_IN = 14.0
_TURN_STEP_DEG = 5.0
_REVERSAL_RAD = math.radians(120.0)
# Slider rows are short. The label and the readout sit outside the axes, so a
# grab this many pixels above or below the track still counts.
_SLIDER_PAD_PX = 28.0


def wrap_deg(degrees):
    return math.degrees(wrap(math.radians(degrees)))


def chassis_corners(x, y, theta):
    half_l = _BODY_LENGTH_IN / 2.0
    half_w = _TRACK_WIDTH_IN / 2.0
    local = (
        (half_l, half_w),
        (half_l, -half_w),
        (-half_l, -half_w),
        (-half_l, half_w),
    )
    c = -math.sin(theta)
    s = math.cos(theta)
    return [
        (x + px * c - py * s, y + px * s + py * c)
        for px, py in local
    ]


def segment_headings(points):
    """Travel heading of each segment. A join shorter than 0.05 in keeps the last heading."""
    headings = []
    reversals = []
    prev = None
    for i in range(len(points) - 1):
        dx = points[i + 1][0] - points[i][0]
        dy = points[i + 1][1] - points[i][1]
        if math.hypot(dx, dy) < 0.05:
            headings.append(prev)
            reversals.append(False)
            continue
        heading = math.atan2(dy, dx) - math.pi / 2.0
        reversals.append(prev is not None and abs(wrap(heading - prev)) > _REVERSAL_RAD)
        headings.append(heading)
        prev = heading
    return headings, reversals


def polyline_stats(points):
    """Length in inches, tightest radius in inches, and reversal count."""
    if len(points) < 2:
        return 0.0, math.inf, 0
    length = 0.0
    peak = 0.0
    for i in range(len(points) - 1):
        length += math.hypot(points[i + 1][0] - points[i][0], points[i + 1][1] - points[i][1])
        if i == 0:
            continue
        ax, ay = points[i - 1][0], points[i - 1][1]
        bx, by = points[i][0], points[i][1]
        cx, cy = points[i + 1][0], points[i + 1][1]
        ab = math.hypot(bx - ax, by - ay)
        bc = math.hypot(cx - bx, cy - by)
        ac = math.hypot(cx - ax, cy - ay)
        if ab * bc * ac < 1e-6:
            continue
        area = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        peak = max(peak, abs(2.0 * area / (ab * bc * ac)))
    _headings, reversals = segment_headings(points)
    radius = math.inf if peak < 1e-4 else 1.0 / peak
    return length, radius, sum(reversals)


def radius_text(radius_in):
    if not math.isfinite(radius_in) or radius_in > 1000.0:
        return "straight"
    return f"tightest radius {radius_in:.1f} in"


def near_polyline(points, path, tol):
    if len(points) == 0 or len(path) < 2:
        return False
    for px, py, *_rest in points:
        best = math.inf
        for i in range(len(path) - 1):
            ax, ay = path[i][0], path[i][1]
            bx, by = path[i + 1][0], path[i + 1][1]
            dx = bx - ax
            dy = by - ay
            length2 = dx * dx + dy * dy
            if length2 < 1e-12:
                dist = math.hypot(px - ax, py - ay)
            else:
                t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
                dist = math.hypot(px - (ax + t * dx), py - (ay + t * dy))
            if dist < best:
                best = dist
        if best > tol:
            return False
    return True


class BandProbe:
    def __init__(self, start, goal, speed, yaw, exit_speed, shape, practice):
        require_plant()
        self.start = [float(start[0]), float(start[1])]
        self.goal = [float(goal[0]), float(goal[1])]
        self.start_deg = wrap_deg(float(start[2]))
        self.goal_deg = wrap_deg(float(goal[2]))
        self.speed = float(speed)
        self.yaw = float(yaw)
        self.exit_speed = float(exit_speed)
        self.shape = shape
        self.practice = bool(practice)
        self.guide = plant.DriftGuide()
        self.values = nominal()
        self._configure()

    def _configure(self):
        configure(self.guide, self.values, self.practice)

    def curves(self):
        # path() and base_path() before command() preview and do not commit.
        self.guide.begin(
            self.goal[0], self.goal[1], math.radians(self.goal_deg), self.exit_speed,
            False, False, self.shape, True,
        )
        args = (
            self.start[0], self.start[1], math.radians(self.start_deg),
            self.speed, self.yaw,
        )
        base = list(self.guide.base_path(*args))
        band = list(self.guide.path(*args))
        return base, band

    def summary(self, base, band):
        base_len, base_radius, _base_rev = polyline_stats(base)
        band_len, band_radius, reversals = polyline_stats(band)
        if len(band) < 2:
            relation = "band did not build"
        elif near_polyline(band, base, 0.15) and near_polyline(base, band, 0.15):
            relation = "band kept the base curve"
        else:
            relation = "band deformed the base curve"
        if band:
            miss = math.hypot(band[-1][0] - self.goal[0], band[-1][1] - self.goal[1])
            relation += f"; end is {miss:.1f} in from the goal"
        lines = [
            (
                f"{self.shape}  start ({self.start[0]:.1f}, {self.start[1]:.1f}) in"
                f"  heading {self.start_deg:.1f} deg"
                f"  v {self.speed:.2f} m/s  yaw {self.yaw:.2f} rad/s"
            ),
            (
                f"goal ({self.goal[0]:.1f}, {self.goal[1]:.1f}) in"
                f"  heading {self.goal_deg:.1f} deg"
                f"  exit {self.exit_speed:.2f} m/s"
                f"  {'practice' if self.practice else 'path'} accel"
            ),
            f"base  {len(base):2d} pts  {base_len:6.1f} in  {radius_text(base_radius)}",
            (
                f"band  {len(band):2d} pts  {band_len:6.1f} in  {radius_text(band_radius)}"
                f"  reversals {reversals}"
            ),
            relation,
        ]
        return "\n".join(lines)

    def report(self, base=None, band=None):
        if base is None or band is None:
            base, band = self.curves()
        print(self.summary(base, band))
        print("band samples, inches:")
        for index, point in enumerate(band):
            print(f"  {index:2d}  {point[0]:7.2f}  {point[1]:7.2f}")
        print()
        return base, band


def draw_heading(ax, x, y, theta, color, length):
    ax.annotate(
        "",
        xy=(x + length * (-math.sin(theta)), y + length * math.cos(theta)),
        xytext=(x, y),
        arrowprops={"arrowstyle": "->", "color": color, "lw": 1.6},
        zorder=6,
    )


def draw_band(ax, points):
    if len(points) < 2:
        return
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    ax.plot(xs, ys, color="#0e6b6b", linewidth=2.0, zorder=2, label="timed elastic")
    ax.plot(xs, ys, linestyle="none", marker="o", color="#0e6b6b", markersize=3.5, zorder=3)
    headings, reversals = segment_headings(points)
    for i, (heading, reversal) in enumerate(zip(headings, reversals)):
        x0, y0 = points[i][0], points[i][1]
        x1, y1 = points[i + 1][0], points[i + 1][1]
        if reversal:
            ax.plot([x0, x1], [y0, y1], color="#8e1b4d", linewidth=2.6, zorder=3)
        if heading is None or i % 2 != 0:
            continue
        ax.plot(
            [x0, x0 + 3.0 * (-math.sin(heading))],
            [y0, y0 + 3.0 * math.cos(heading)],
            color="#148f77", linewidth=0.8, zorder=3,
        )


def draw(ax, probe, base, band):
    ax.set_aspect("equal")
    ax.set_xlim(float(plant.FIELD_X_MIN_IN) - 8.0, float(plant.FIELD_X_MIN_IN) + float(plant.FIELD_SIZE_IN) + 8.0)
    ax.set_ylim(float(plant.FIELD_Y_MIN_IN) - 8.0, float(plant.FIELD_Y_MIN_IN) + float(plant.FIELD_SIZE_IN) + 8.0)
    ax.set_xlabel("x (in)")
    ax.set_ylabel("y (in)")
    ax.grid(True, color="#d5d5d5", linewidth=0.6)
    ax.add_patch(plt.Rectangle(
        (float(plant.FIELD_X_MIN_IN), float(plant.FIELD_Y_MIN_IN)),
        float(plant.FIELD_SIZE_IN),
        float(plant.FIELD_SIZE_IN),
        fill=False, edgecolor="#222222", linewidth=1.2,
    ))
    draw_band(ax, band)
    if len(base) >= 2:
        ax.plot(
            [p[0] for p in base], [p[1] for p in base],
            color="#d35400", linewidth=1.3, linestyle=(0, (5, 3)),
            zorder=4, label="base curve",
        )
    start_theta = math.radians(probe.start_deg)
    corners = chassis_corners(probe.start[0], probe.start[1], start_theta)
    ax.add_patch(Polygon(
        corners, closed=True, facecolor="#2c6bed", edgecolor="#102040",
        linewidth=1.0, zorder=4,
    ))
    ax.plot(
        [corners[0][0], corners[1][0]], [corners[0][1], corners[1][1]],
        color="#f4d35e", linewidth=2.4, zorder=5, solid_capstyle="round",
    )
    nose = 0.5 * _BODY_LENGTH_IN + 2.0
    draw_heading(
        ax,
        probe.start[0] + nose * (-math.sin(start_theta)),
        probe.start[1] + nose * math.cos(start_theta),
        start_theta, "#1a365d", 10.0,
    )
    ax.plot(probe.goal[0], probe.goal[1], marker="x", color="#922b21", markersize=9, zorder=5)
    draw_heading(ax, probe.goal[0], probe.goal[1], math.radians(probe.goal_deg), "#922b21", 12.0)
    ax.legend(loc="upper right", fontsize=8)


def _is_left(event):
    button = getattr(event, "button", None)
    if button in (1, MouseButton.LEFT):
        return True
    buttons = getattr(event, "buttons", None) or ()
    return MouseButton.LEFT in buttons


class BandWindow:
    def __init__(self, probe):
        self.probe = probe
        self.drag = None
        self._grabbed = None
        # The navigation toolbar takes the left-button drag for itself once a
        # tool is armed, and it is not needed to read the arc.
        mpl.rcParams["toolbar"] = "none"
        self.fig = plt.figure(figsize=(12.2, 8.2))
        self.fig._band_window = self
        self.ax = self.fig.add_axes((0.05, 0.36, 0.60, 0.60))
        self.status = self.fig.text(
            0.05, 0.30, "", va="top", ha="left", fontsize=8, family="monospace",
        )
        self.sliders = {}
        specs = (
            ("start", "start heading deg", -180.0, 180.0, probe.start_deg),
            ("goal", "goal heading deg", -180.0, 180.0, probe.goal_deg),
            ("speed", "start speed m/s", 0.0, 2.5, probe.speed),
            ("yaw", "start yaw rad/s", -12.0, 12.0, probe.yaw),
            ("exit", "exit speed m/s", 0.0, 2.5, probe.exit_speed),
        )
        top = 0.88
        for index, (key, label, lo, hi, value) in enumerate(specs):
            axes = self.fig.add_axes((0.74, top - index * 0.075, 0.22, 0.05))
            slider = Slider(
                axes, label, lo, hi, valinit=value,
                valstep=0.5 if key in ("start", "goal") else 0.05,
                handle_style={"size": 14},
            )
            slider.label.set_size(8)
            slider.valtext.set_size(8)
            slider.on_changed(self._on_slider)
            self.sliders[key] = slider
        radio_ax = self.fig.add_axes((0.74, 0.28, 0.18, 0.24))
        radio_ax.set_title("base curve", fontsize=8)
        self.radio = RadioButtons(radio_ax, SHAPES, active=SHAPES.index(probe.shape))
        for label in self.radio.labels:
            label.set_fontsize(8)
        self.radio.on_clicked(self._on_shape)
        check_ax = self.fig.add_axes((0.74, 0.16, 0.18, 0.08))
        self.check = CheckButtons(check_ax, ["practice accel"], [probe.practice])
        self.check.on_clicked(self._on_practice)
        self.fig.text(
            0.74, 0.06,
            "Drag the robot or the goal.\nScroll on a pose to turn it.\np prints the samples.",
            fontsize=8, color="#333333", va="bottom",
        )
        self.fig.canvas.mpl_connect("button_press_event", self._on_press)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_motion)
        self.fig.canvas.mpl_connect("button_release_event", self._on_release)
        self.fig.canvas.mpl_connect("scroll_event", self._on_scroll)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.refresh(report=False)
        # Hit tests use the axes bounding boxes, which a deferred draw has not
        # filled in yet.
        self.fig.canvas.draw()

    def refresh(self, report):
        base, band = self.probe.curves()
        self.ax.clear()
        draw(self.ax, self.probe, base, band)
        self.status.set_text(self.probe.summary(base, band))
        self.fig.canvas.draw_idle()
        if report:
            self.probe.report(base, band)

    def _read_sliders(self):
        self.probe.start_deg = wrap_deg(self.sliders["start"].val)
        self.probe.goal_deg = wrap_deg(self.sliders["goal"].val)
        self.probe.speed = self.sliders["speed"].val
        self.probe.yaw = self.sliders["yaw"].val
        self.probe.exit_speed = self.sliders["exit"].val

    def _on_slider(self, _value):
        self._read_sliders()
        self.refresh(report=False)

    def _on_shape(self, label):
        self.probe.shape = label
        self.refresh(report=True)

    def _on_practice(self, _label):
        self.probe.practice = not self.probe.practice
        self.probe._configure()
        self.refresh(report=True)

    def _pixel(self, event):
        if event.x is None or event.y is None:
            return None
        return float(event.x), float(event.y)

    def _data_xy(self, event):
        pixel = self._pixel(event)
        if pixel is None:
            return None
        try:
            x, y = self.ax.transData.inverted().transform(pixel)
        except ValueError:
            return None
        return float(x), float(y)

    def _pose_at(self, event):
        # Scroll and press both use this. A point on the slider row is not a pose.
        pixel = self._pixel(event)
        data = self._data_xy(event)
        if pixel is None or data is None:
            return None
        box = self.ax.bbox
        if pixel[0] < box.x0 or pixel[0] > box.x1 or pixel[1] < box.y0 or pixel[1] > box.y1:
            return None
        start = math.hypot(data[0] - self.probe.start[0], data[1] - self.probe.start[1])
        goal = math.hypot(data[0] - self.probe.goal[0], data[1] - self.probe.goal[1])
        if start <= goal and start <= _PICK_IN:
            return "start"
        if goal <= _PICK_IN:
            return "goal"
        return None

    def _slider_at(self, event):
        pixel = self._pixel(event)
        if pixel is None:
            return None
        px, py = pixel
        best = None
        best_d = None
        for key, slider in self.sliders.items():
            box = slider.ax.bbox
            if box.width <= 0 or px < box.x0 - 170 or px > box.x1 + 80:
                continue
            center = 0.5 * (box.y0 + box.y1)
            dist = abs(py - center)
            if dist > max(_SLIDER_PAD_PX, 0.5 * box.height + 8):
                continue
            if best_d is None or dist < best_d:
                best, best_d = key, dist
        return best

    def _slider_value(self, key, event):
        slider = self.sliders[key]
        pixel = self._pixel(event)
        if pixel is None:
            return None
        try:
            xdata, _ydata = slider.ax.transData.inverted().transform(pixel)
        except ValueError:
            return None
        lo, hi = slider.valmin, slider.valmax
        val = min(hi, max(lo, float(xdata)))
        step = slider.valstep
        if isinstance(step, (int, float)) and step > 0:
            val = lo + round((val - lo) / step) * step
            val = min(hi, max(lo, val))
        return val

    def _grab(self, axes):
        canvas = self.fig.canvas
        if canvas.mouse_grabber not in (None, axes):
            return
        if canvas.mouse_grabber is None:
            canvas.grab_mouse(axes)
        self._grabbed = axes

    def _release_grab(self):
        canvas = self.fig.canvas
        if self._grabbed is not None and canvas.mouse_grabber is self._grabbed:
            canvas.release_mouse(self._grabbed)
        self._grabbed = None

    def _on_press(self, event):
        if not _is_left(event):
            return
        slider_key = self._slider_at(event)
        if slider_key is not None:
            self.drag = ("slider", slider_key)
            self._grab(self.sliders[slider_key].ax)
            box = self.sliders[slider_key].ax.bbox
            # A press on the label, which sits left of the track, should not
            # snap the value. Motion after that press still drags.
            if box.x0 <= event.x <= box.x1:
                self._move_slider(slider_key, event)
            return
        pose = self._pose_at(event)
        if pose is None:
            return
        self.drag = ("pose", pose)
        self._grab(self.ax)

    def _move_slider(self, key, event):
        val = self._slider_value(key, event)
        slider = self.sliders[key]
        if val is None or val == slider.val:
            return
        slider.set_val(val)

    def _on_motion(self, event):
        if self.drag is None:
            return
        kind, which = self.drag
        if kind == "slider":
            self._move_slider(which, event)
            return
        data = self._data_xy(event)
        if data is None:
            return
        target = self.probe.start if which == "start" else self.probe.goal
        target[0], target[1] = data
        self.refresh(report=False)

    def _on_release(self, _event):
        moved = self.drag is not None and self.drag[0] == "pose"
        self.drag = None
        self._release_grab()
        if moved:
            self.refresh(report=True)

    def _on_scroll(self, event):
        which = self._pose_at(event)
        if which is None:
            return
        step = _TURN_STEP_DEG if event.button == "up" else -_TURN_STEP_DEG
        slider = self.sliders[which]
        slider.set_val(wrap_deg(slider.val + step))

    def _on_key(self, event):
        if event.key == "p":
            self.probe.report()


def main():
    parser = argparse.ArgumentParser(description="Preview the timed elastic band from a start pose to a goal")
    parser.add_argument(
        "--start", nargs=3, type=float, metavar=("X", "Y", "DEG"), default=(0.0, 72.0, -90.0),
        help="start x in, y in, heading deg. Default is the field center, facing +x.",
    )
    parser.add_argument(
        "--goal", nargs=3, type=float, metavar=("X", "Y", "DEG"), default=(36.0, 96.0, -21.2),
        help="goal x in, y in, exit heading deg",
    )
    parser.add_argument("--v", type=float, default=0.0, help="start speed, m/s")
    parser.add_argument("--yaw", type=float, default=0.0, help="start yaw rate, rad/s")
    parser.add_argument("--exit-speed", type=float, default=0.0, help="goal exit speed, m/s")
    parser.add_argument("--shape", choices=SHAPES, default="Hermite")
    parser.add_argument("--practice", action="store_true", help="use the practice accel cap")
    parser.add_argument("--save", type=Path, default=None, help="write a png of this pose")
    parser.add_argument("--show", action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args()

    probe = BandProbe(args.start, args.goal, args.v, args.yaw, args.exit_speed, args.shape, args.practice)
    base, band = probe.report()
    if args.save is not None or not args.show:
        fig = plt.figure(figsize=(7.4, 8.6))
        ax = fig.add_axes((0.12, 0.36, 0.82, 0.60))
        draw(ax, probe, base, band)
        fig.text(0.12, 0.30, probe.summary(base, band), va="top", ha="left", fontsize=8, family="monospace")
        if args.save is not None:
            fig.savefig(args.save, dpi=120)
            print(f"wrote {args.save}")
        if not args.show:
            plt.close(fig)
            return
        plt.close(fig)

    BandWindow(probe)
    plt.show()


if __name__ == "__main__":
    main()
