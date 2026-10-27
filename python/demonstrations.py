"""Roll a geometric path tracker and store the runs that finish.

The teacher holds the path tangent and the cross-track, and it copies the
thrust schedule. A run is kept only when the episode ends at the end of
the path. Diverged and timed-out runs are dropped.

The replay buffer is a FIFO, so a batch injected at a stage boundary is
gone before the actor has copied it. DemoSet keeps those steps by stage
as one contiguous array, and the trainer uploads that array to the GPU
once per injection instead of copying a fresh sample every update.

Each kept step is stored as the discounted return from that step to the
end of the run, with the bootstrap cut. Leaving the one-step reward in
the buffer would rescore the run with whatever the live policy does
next, and a policy that bails at the first corner would paint that
bailout onto the teacher's states.
"""

import numpy as np


def _obs_rows(obs):
    if isinstance(obs, dict):
        return int(next(iter(obs.values())).shape[0])
    return int(obs.shape[0])


def _stack_obs(transitions):
    sample = transitions[0][0]
    if isinstance(sample, dict):
        return {
            key: np.stack([np.asarray(step[0][key], dtype=np.float32) for step in transitions])
            for key in sample
        }
    return np.stack([np.asarray(step[0], dtype=np.float32) for step in transitions])


def _concat_obs(older, newer):
    if isinstance(older, dict):
        return {key: np.concatenate((older[key], newer[key]), axis=0) for key in older}
    return np.concatenate((older, newer), axis=0)


def _tail_obs(obs, cap):
    if isinstance(obs, dict):
        if _obs_rows(obs) <= cap:
            return {key: np.ascontiguousarray(value) for key, value in obs.items()}
        return {key: np.ascontiguousarray(value[-cap:]) for key, value in obs.items()}
    if _obs_rows(obs) <= cap:
        return np.ascontiguousarray(obs)
    return np.ascontiguousarray(obs[-cap:])


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
        obs = _stack_obs(transitions)
        actions = np.stack([np.asarray(step[2], dtype=np.float32) for step in transitions])
        if stage in self._obs:
            obs = _concat_obs(self._obs[stage], obs)
            actions = np.concatenate((self._actions[stage], actions), axis=0)
        obs = _tail_obs(obs, self.cap)
        if _obs_rows(obs) < actions.shape[0]:
            actions = np.ascontiguousarray(actions[-_obs_rows(obs) :])
        else:
            actions = np.ascontiguousarray(actions)
        self._obs[stage] = obs
        self._actions[stage] = actions
        self.generation += 1

    def __len__(self):
        return int(sum(_obs_rows(obs) for obs in self._obs.values()))

    def stages(self):
        """Observation and action arrays for every stage that still has steps."""
        return [
            (self._obs[name], self._actions[name])
            for name, obs in self._obs.items()
            if _obs_rows(obs) > 0
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


def _buffer_obs(obs):
    if isinstance(obs, dict):
        return {key: np.asarray(value, dtype=np.float32).reshape(1, -1) for key, value in obs.items()}
    return np.asarray(obs, dtype=np.float32).reshape(1, -1)


def _return_to_go(rewards, gamma):
    targets = [0.0] * len(rewards)
    running = 0.0
    for index in range(len(rewards) - 1, -1, -1):
        running = rewards[index] + gamma * running
        targets[index] = running
    return targets


def _episodes(transitions):
    episode = []
    for step in transitions:
        episode.append(step)
        if step[4] or step[5]:
            yield episode
            episode = []
    if episode:
        yield episode


def store_transitions(replay_buffer, transitions, gamma):
    """Write teacher runs as return-to-go targets.

    A finished run is one target per step and a cut bootstrap, so the
    critic learns the return of the action that was taken. A timeout
    keeps the one-step reward and the timeout flag.
    """
    gamma = float(gamma)
    for episode in _episodes(transitions):
        terminated = bool(episode[-1][4])
        truncated = bool(episode[-1][5])
        rewards = [step[3] for step in episode]
        if terminated and not truncated:
            targets = _return_to_go(rewards, gamma)
            for step, target in zip(episode, targets):
                obs, next_obs, action = step[0], step[1], step[2]
                replay_buffer.add(
                    _buffer_obs(obs),
                    _buffer_obs(next_obs),
                    np.asarray(action, dtype=np.float32).reshape(1, -1),
                    np.array([target], dtype=np.float32),
                    np.array([1.0], dtype=np.float32),
                    [{"TimeLimit.truncated": False}],
                )
            continue
        for obs, next_obs, action, reward, step_terminated, step_truncated in episode:
            replay_buffer.add(
                _buffer_obs(obs),
                _buffer_obs(next_obs),
                np.asarray(action, dtype=np.float32).reshape(1, -1),
                np.array([reward], dtype=np.float32),
                np.array([step_terminated or step_truncated], dtype=np.float32),
                [{"TimeLimit.truncated": bool(step_truncated and not step_terminated)}],
            )


def inject_demonstrations(model, env, n_keep, max_tries, demo_set=None):
    transitions, report = collect_arrivals(env, n_keep, max_tries)
    if transitions:
        store_transitions(model.replay_buffer, transitions, model.gamma)
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
