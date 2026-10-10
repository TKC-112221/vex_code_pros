"""Tunable parameters for the path-following trainer.

The plant (volt limits, timestep, field size) comes from the C++ binding.
Feature scales for the policy observation come from the same binding.
Everything that shapes the generated path, the reward, and the teacher
lives here.
"""

# Discount shared by the env default and the SAC trainer.
DEFAULT_GAMMA = 0.999

# Sample spacing for the generated polyline, inches.
DS_IN = 0.5
# Tightest planned radius is about 22 inches.
K_MAX = 0.045
# Path tangent is read this far ahead of the closest point, inches.
LOOKAHEAD_IN = 8.0
# Inches of closing, per step, that the path term treats as 1.
CROSS_RATE_IN = 0.25
# Weight on -|thrust - target|. Thrust is the normalized surge command.
THRUST_WEIGHT = 1.0
# Positive closing is scaled by this. Negative closing is not.
APPROACH_GAIN = 0.8
# Per inch of cross-track, per step.
DIST_WEIGHT = 0.03
# Clock-ramp half duration, seconds. Thrust falls and rises in this long.
RAMP_HALF_SEC = (0.20, 0.45)
# Off the path by more than this, the episode is over.
DIVERGE_IN = 12.0
# Reaching the polyline within this many inches of its end counts as done.
FINISH_IN = 3.0
# Clock bounds, in steps.
MIN_HORIZON = 500
MAX_HORIZON = 1600

# A span counts as straight when curvature and its rate are under these.
KAPPA_STRAIGHT = 1e-5
DK_STRAIGHT = 1e-6

# Clock stops. TEACHER_SPEED_IPS is the full-throttle pass used to place them.
MIN_CLOCK_SEC = 0.4
DEADLINE_MARGIN_SEC = 0.25
DEADLINE_FLOOR_SEC = 0.45
DEADLINE_LENGTH_FRAC = 0.72
TEACHER_SPEED_IPS = 105.0
EVENT_GAP_SEC = 0.12

# Local path pieces, inches and curvature.
STRAIGHT_OPEN_IN = (18.0, 36.0)
GENTLE_K_PEAK = 0.022
SHARP_K_PEAK = K_MAX
K_PEAK_MIN = 0.012
D2_RANGE = (0.0015, 0.0055)
BEND_HOLD_IN = (8.0, 22.0)
STRAIGHT_BETWEEN_IN = (16.0, 40.0)
NO_BEND_STRAIGHT_IN = (36.0, 80.0)
DIP_BASE_IN = 85.0
DIP_EXTRA_IN = 30.0
MIN_STRAIGHT_IN = 12.0
MIN_BEND_KAPPA = 0.01
KAPPA_FLAT = 1e-4

WALL_MARGIN_IN = 8.0
FALLBACK_SAMPLES = 160
FALLBACK_LENGTH_IN = 80.0
GENERATE_TRIES = 40

# Where a reset places the robot relative to the path start.
START_OFFSET_IN = 1.5
START_HEADING_RAD = 0.12

# Horizon from a slow crawl, plus extra steps per stop.
CRAWL_IPS = 20.0
DIP_EXTRA_STEPS = 200.0
HORIZON_PAD_STEPS = 200.0

# Cloning teacher. Yaw is a normalized command.
YAW_HEADING = 1.3
YAW_CROSS = 0.07
YAW_CURVATURE = 0.09
# Below this, the observation uses the chassis nose as the velocity heading.
# Same threshold as DrivePolicy::kStoppedSpeedMps.
STOPPED_SPEED_MPS = 0.05

# Projection search window around the previous hint, in samples.
HINT_BACK = 4
HINT_AHEAD = 40

# Curriculum, easy to hard.
STAGES = (
    "straight",
    "straight-stop",
    "gentle",
    "mixed",
    "full",
)
MIXED_BENDS = (1, 3)
FULL_BENDS = (1, 4)
FULL_DIPS = (0, 3)
