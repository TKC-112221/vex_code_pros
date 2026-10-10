"""Roll a geometric path tracker and store the runs that finish.

The teacher holds the path tangent and the cross-track, and it copies the
thrust schedule. A run is kept only when the episode ends at the end of
the path. Diverged and timed-out runs are dropped.

The replay buffer is a FIFO, so a batch injected at a stage boundary is
gone before the actor has copied it. DemoSet keeps those steps by stage
as one contiguous array, and the trainer uploads that array to the GPU
once per injection instead of copying a fresh sample every update.
"""

import numpy as np


class DemoSet:
    """Successful tracker steps, kept per stage so later stages cannot evict the early ones.

    Each stage is two contiguous arrays. Sampling indexes those arrays, and
    the trainer can keep the same storage on the GPU across updates.
    """

    def __init__(self, cap=2000):
        self.cap = cap
        self._obs = {}
        self._actions = {}
        # Bumps when the arrays change, so a GPU copy can be reused.
        self.generation = 0

    def add(self, stage, transitions):
        if not transitions:
            return
        obs = np.stack([np.asarray(step[0], dtype=np.float32) for step in transitions])
        actions = np.stack([np.asarray(step[2], dtype=np.float32) for step in transitions])
        if stage in self._obs:
            obs = np.concatenate((self._obs[stage], obs), axis=0)
            actions = np.concatenate((self._actions[stage], actions), axis=0)
        if len(obs) > self.cap:
            obs = np.ascontiguousarray(obs[-self.cap :])
            actions = np.ascontiguousarray(actions[-self.cap :])
        else:
            obs = np.ascontiguousarray(obs)
            actions = np.ascontiguousarray(actions)
        self._obs[stage] = obs
        self._actions[stage] = actions
        self.generation += 1

    def __len__(self):
        return int(sum(obs.shape[0] for obs in self._obs.values()))

    def stages(self):
        """Observation and action arrays for every stage that still has steps."""
        return [
            (self._obs[name], self._actions[name])
            for name, obs in self._obs.items()
            if obs.shape[0] > 0
        ]


def rollout_teacher(env):
    obs, _ = env.reset()
    steps = []
    done = False
    reason = ""
    info = {}
    while not done:
        action = env.tracking_action()
        next_obs, reward, terminated, truncated, info = env.step(action)
        steps.append((obs, next_obs, action, float(reward), bool(terminated), bool(truncated)))
        obs = next_obs
        done = terminated or truncated
        reason = info["reason"]
    return steps, reason, info


def collect_arrivals(env, n_keep, max_tries):
    kept = []
    got = 0
    tried = 0
    while got < n_keep and tried < max_tries:
        tried += 1
        steps, reason, info = rollout_teacher(env)
        if reason != "finished" or not info.get("is_success", False):
            continue
        kept.extend(steps)
        got += 1
    return kept, {"tracker": (got, tried)}


def store_transitions(replay_buffer, transitions):
    for obs, next_obs, action, reward, terminated, truncated in transitions:
        replay_buffer.add(
            np.asarray(obs, dtype=np.float32).reshape(1, -1),
            np.asarray(next_obs, dtype=np.float32).reshape(1, -1),
            np.asarray(action, dtype=np.float32).reshape(1, -1),
            np.array([reward], dtype=np.float32),
            np.array([terminated or truncated], dtype=np.float32),
            [{"TimeLimit.truncated": bool(truncated and not terminated)}],
        )


def inject_demonstrations(model, env, n_keep, max_tries, demo_set=None):
    transitions, report = collect_arrivals(env, n_keep, max_tries)
    if transitions:
        store_transitions(model.replay_buffer, transitions)
        if demo_set is not None:
            demo_set.add(env.curriculum_stage(), transitions)
    stage = env.curriculum_stage()
    arrivals = sum(got for got, _tried in report.values())
    tries = sum(tried for _got, tried in report.values())
    detail = ", ".join(f"{name} {got}/{tried}" for name, (got, tried) in report.items())
    held = len(demo_set) if demo_set is not None else 0
    print(
        f"demos {stage}: kept {arrivals}/{tries} runs that finished the path, "
        f"{len(transitions)} steps, {held} steps held for cloning ({detail})"
    )
    return arrivals
