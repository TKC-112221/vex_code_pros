"""Train a soft actor-critic to follow a path and its thrust schedule.

The actor is the firmware network: a short velocity history, a spatial
lookahead, two hidden layers of 64, and 2 outputs (surge and yaw). The
critic is wider, sees the actor features plus the plant's true pose,
slip, and friction, and is not exported. After training,
the actor mean is written to include/control/DrivePolicyWeights.h and
data/drive_policy.csv. Rebuild the V5 image to pick up the header, and
set Settings::ControlToggles::USE_DRIVE_POLICY to run it in place of
DriftFollower. The override still has no path, so that switch does not
yet feed the features this actor was trained on.

    python python/build_plant.py
    python python/train_drive_policy.py

The curriculum does not advance on a clock. A stage changes only after
recent episodes finish the path. Example runs from the geometric tracker
stay in a side buffer and pull the actor mean toward them. That pull
starts at bc_coef and falls linearly to zero across the training budget,
so the clone can warm the policy and then get out of the way. It is
applied to the deterministic mean, so it does not shrink the noise.
The critic backup is the task return. The entropy term stays in the
actor loss only: once the action density exceeds 1, log π is positive,
and folding -alpha log π into the backup charges every step the episode
continues. The log std of the actor cannot fall below -2.2. The
temperature starts very low and is capped. Its target is the SAC
default, -dim(A). The dual optimizer raises the temperature as the
clone weight fades, and the cap stops that temperature from staying
high after the entropy has recovered.

The plant is one CPU step, and the nets are small. Training therefore
collects a streak of steps and then runs the same number of updates
back to back, with the tracker loss inside the actor backward. Losses
stay on the device until the streak ends, so a host read does not stall
the GPU between minibatches.
"""

import argparse
import math
import sys
from collections import deque
from pathlib import Path

from gymnasium import spaces
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3.common.utils import polyak_update
from stable_baselines3.sac.policies import LOG_STD_MAX, Actor, SACPolicy
import torch
from torch import nn
from torch.nn import functional as F

print(f"Is CUDA available?: {torch.cuda.is_available()}")
print(f"CUDA Devices: {torch.cuda.device_count()}")
print(f"Current GPU Index: {torch.cuda.current_device()}")
print(f"Current GPU Name: {torch.cuda.get_device_name(torch.cuda.current_device())}")


sys.path.insert(0, str(Path(__file__).resolve().parent))

from demonstrations import DemoSet, inject_demonstrations
from drive_env import STAGES, PathFollowEnv
from path_params import DEFAULT_GAMMA
from export_policy import export


HERE = Path(__file__).resolve().parent
CHECKPOINT = HERE / "drive_policy.zip"

# log std of about 0.14. Wide enough to try a nearby command, tight enough
# that a cloned run still stays inside the cross-track band.
_BC_LOG_STD = -2.0
# Most demo steps are straight, where the teacher yaw is near zero. An
# unweighted mean can sit near 0.006 while the yaw through a corner, the
# command that keeps the episode alive, is wrong. This raises the weight
# of a step in proportion to that yaw, then normalizes so the average
# clone loss keeps its scale.
_BC_YAW_EMPHASIS = 8.0
# Steps gathered before a GPU streak. gradient_steps=-1 repeats the
# update this many times, so the ratio stays one gradient step per plant step.
_COLLECT_STEPS = 1024
_BATCH_SIZE = 1024
# The dual only raises the temperature. Once the entropy sits on the
# target the gradient is zero, so a temperature that climbed while the
# policy was peaked stays there. 0.25 keeps the actor's entropy weight
# on the scale of the path reward.
_ENT_COEF_MAX = 0.25
_LOG_ENT_COEF_MAX = math.log(_ENT_COEF_MAX)
# Library floor is -20, which lets the action density spike and log π go
# largely positive. -2.2 is a std of about 0.11.
_LOG_STD_MIN = -2.2
# Applied to the bootstrapped next value only. A real return fits inside
# this. The clip stops a spike from being summed across the horizon.
_Q_BOOTSTRAP_MIN = -2000.0
_Q_BOOTSTRAP_MAX = 200.0
_GRAD_CLIP = 10.0


class ActorObsExtractor(BaseFeaturesExtractor):
    """The firmware vector only. Privileged keys never reach the actor."""

    def __init__(self, observation_space: spaces.Dict, **_kwargs):
        dim = int(observation_space["obs"].shape[0])
        super().__init__(observation_space, features_dim=dim)

    def forward(self, observations):
        return observations["obs"]


class CriticObsExtractor(BaseFeaturesExtractor):
    """Actor features plus the ground-truth plant the simulator can read."""

    def __init__(self, observation_space: spaces.Dict, **_kwargs):
        dim = int(observation_space["obs"].shape[0] + observation_space["priv"].shape[0])
        super().__init__(observation_space, features_dim=dim)

    def forward(self, observations):
        return torch.cat((observations["obs"], observations["priv"]), dim=1)


class FlooredActor(Actor):
    """Same firmware actor, with a floor on the log std."""

    def get_action_dist_params(self, obs):
        mean_actions, log_std, kwargs = super().get_action_dist_params(obs)
        log_std = torch.clamp(log_std, _LOG_STD_MIN, LOG_STD_MAX)
        return mean_actions, log_std, kwargs


class AsymmetricSACPolicy(SACPolicy):
    """Same SAC update, with a critic that sees more than the deployed actor."""

    def __init__(self, *args, **kwargs):
        kwargs["features_extractor_class"] = ActorObsExtractor
        kwargs["share_features_extractor"] = False
        super().__init__(*args, **kwargs)

    def make_actor(self, features_extractor=None):
        actor_kwargs = self._update_features_extractor(self.actor_kwargs, features_extractor)
        return FlooredActor(**actor_kwargs).to(self.device)

    def make_critic(self, features_extractor=None):
        if features_extractor is None:
            features_extractor = CriticObsExtractor(self.observation_space)
        return super().make_critic(features_extractor)


def _as_torch(obs, device):
    if isinstance(obs, dict):
        return {key: torch.as_tensor(value, device=device) for key, value in obs.items()}
    return torch.as_tensor(obs, device=device)


def _rows(obs):
    if isinstance(obs, dict):
        return int(obs["obs"].shape[0])
    return int(obs.shape[0])


def _take(obs, index):
    if isinstance(obs, dict):
        return {key: value.index_select(0, index) for key, value in obs.items()}
    return obs.index_select(0, index)


class DriveSAC(SAC):
    """SAC plus a pull toward tracker runs that finished the path.

    The clone loss is added to the actor loss and stepped once. A second
    optimizer step, and a host read of that loss, was a GPU sync after
    every plant step.
    """

    def __init__(self, *args, demo_set=None, bc_coef=8.0, **kwargs):
        super().__init__(*args, **kwargs)
        self.demo_set = demo_set if demo_set is not None else DemoSet()
        self.bc_coef = float(bc_coef)
        self._demo_generation = -1
        self._demo_tensors = []

    def _bc_coef_now(self):
        """Linear decay from bc_coef at step 0 to zero at the end of learn()."""
        initial = self.bc_coef
        total = int(getattr(self, "_total_timesteps", 0))
        if total <= 0 or initial <= 0.0:
            return initial
        done = min(1.0, float(self.num_timesteps) / float(total))
        return initial * (1.0 - done)

    def _ensure_demo_tensors(self):
        generation = self.demo_set.generation
        if generation == self._demo_generation:
            return
        self._demo_generation = generation
        device = self.device
        self._demo_tensors = [
            (
                _as_torch(obs, device),
                torch.as_tensor(actions, device=device),
            )
            for obs, actions in self.demo_set.stages()
        ]

    def _demo_batch(self, batch):
        """Even draw across stages. Indexes stay on the device."""
        self._ensure_demo_tensors()
        stages = self._demo_tensors
        if not stages or batch <= 0:
            return None, None
        per = max(1, batch // len(stages))
        observations = []
        actions = []
        for obs, act in stages:
            index = torch.randint(0, _rows(obs), (per,), device=act.device)
            observations.append(_take(obs, index))
            actions.append(act.index_select(0, index))
        if isinstance(observations[0], dict):
            keys = observations[0].keys()
            return {key: torch.cat([item[key] for item in observations], dim=0) for key in keys}, torch.cat(actions, dim=0)
        return torch.cat(observations, dim=0), torch.cat(actions, dim=0)

    def _behavior_clone_loss(self, batch, match_std=False):
        obs, actions = self._demo_batch(batch)
        if obs is None:
            return None
        mean, log_std, _kwargs = self.actor.get_action_dist_params(obs)
        # Deterministic deploy action is tanh(mean). log_std is not in this
        # square, so the clone gradient does not shrink the noise. The std
        # target is only for the warmup: after that, the entropy term owns it.
        sq = (torch.tanh(mean) - actions).square()
        yaw_w = 1.0 + _BC_YAW_EMPHASIS * actions[:, 1].abs()
        yaw_w = yaw_w / yaw_w.mean().clamp(min=1e-6)
        loss = 0.5 * (sq[:, 0].mean() + (sq[:, 1] * yaw_w).mean())
        if match_std:
            target = torch.full_like(log_std, _BC_LOG_STD)
            loss = loss + 0.05 * F.mse_loss(log_std, target)
        return loss

    def clone_demonstrations(self, batch=_BATCH_SIZE, match_std=False):
        """Copy arrivals into the actor before any plant step. Returns a device tensor."""
        self.policy.set_training_mode(True)
        coef = self._bc_coef_now()
        if coef <= 0.0:
            return None
        loss = self._behavior_clone_loss(batch, match_std=match_std)
        if loss is None:
            return None
        self.actor.optimizer.zero_grad(set_to_none=True)
        (coef * loss).backward()
        self.actor.optimizer.step()
        return loss.detach()

    def train(self, gradient_steps, batch_size=64):
        # Same update as stable-baselines3 SAC, with two GPU changes: the
        # follower loss shares the actor backward, and the logged means are
        # reduced on device so .item() runs once per streak.
        self.policy.set_training_mode(True)
        optimizers = [self.actor.optimizer, self.critic.optimizer]
        if self.ent_coef_optimizer is not None:
            optimizers += [self.ent_coef_optimizer]
        self._update_learning_rate(optimizers)

        device = self.device
        ent_coef_total = torch.zeros((), device=device)
        ent_total = torch.zeros((), device=device)
        actor_total = torch.zeros((), device=device)
        critic_total = torch.zeros((), device=device)
        bc_total = torch.zeros((), device=device)
        ent_coef_count = 0
        bc_count = 0
        bc_coef = self._bc_coef_now()

        for gradient_step in range(gradient_steps):
            replay_data = self.replay_buffer.sample(batch_size, env=self._vec_normalize_env)
            discounts = replay_data.discounts if replay_data.discounts is not None else self.gamma

            if self.use_sde:
                self.actor.reset_noise()

            actions_pi, log_prob = self.actor.action_log_prob(replay_data.observations)
            log_prob = log_prob.reshape(-1, 1)

            ent_coef_loss = None
            if self.ent_coef_optimizer is not None and self.log_ent_coef is not None:
                ent_coef = torch.exp(self.log_ent_coef.detach())
                ent_coef_loss = -(self.log_ent_coef * (log_prob + self.target_entropy).detach()).mean()
                ent_coef_total = ent_coef_total + ent_coef_loss.detach()
                ent_coef_count += 1
            else:
                ent_coef = self.ent_coef_tensor

            if ent_coef_loss is not None and self.ent_coef_optimizer is not None:
                self.ent_coef_optimizer.zero_grad(set_to_none=True)
                ent_coef_loss.backward()
                self.ent_coef_optimizer.step()
                self.log_ent_coef.data.clamp_(max=_LOG_ENT_COEF_MAX)
                ent_coef = torch.exp(self.log_ent_coef.detach())

            ent_total = ent_total + ent_coef.detach()

            with torch.no_grad():
                # Task return. Entropy stays in the actor loss below. A sharp
                # policy has positive log π, and subtracting alpha * log π here
                # would bill every step the episode continues.
                next_actions = self.actor(replay_data.next_observations)
                next_q_values = torch.cat(self.critic_target(replay_data.next_observations, next_actions), dim=1)
                next_q_values, _ = torch.min(next_q_values, dim=1, keepdim=True)
                next_q_values = next_q_values.clamp(_Q_BOOTSTRAP_MIN, _Q_BOOTSTRAP_MAX)
                target_q_values = replay_data.rewards + (1 - replay_data.dones) * discounts * next_q_values

            current_q_values = self.critic(replay_data.observations, replay_data.actions)
            critic_loss = 0.5 * sum(F.mse_loss(current_q, target_q_values) for current_q in current_q_values)
            critic_total = critic_total + critic_loss.detach()

            self.critic.optimizer.zero_grad(set_to_none=True)
            critic_loss.backward()
            nn.utils.clip_grad_norm_(self.critic.parameters(), _GRAD_CLIP)
            self.critic.optimizer.step()

            q_values_pi = torch.cat(self.critic(replay_data.observations, actions_pi), dim=1)
            min_qf_pi, _ = torch.min(q_values_pi, dim=1, keepdim=True)
            actor_loss = (ent_coef * log_prob - min_qf_pi).mean()
            actor_total = actor_total + actor_loss.detach()
            bc_loss = None
            if bc_coef > 0.0:
                bc_loss = self._behavior_clone_loss(batch_size)
            if bc_loss is not None:
                # One backward, so the follower gradient is not a second
                # launch and a sync. The weight falls to zero over learn().
                actor_loss = actor_loss + bc_coef * bc_loss
                bc_total = bc_total + bc_loss.detach()
                bc_count += 1

            self.actor.optimizer.zero_grad(set_to_none=True)
            actor_loss.backward()
            nn.utils.clip_grad_norm_(self.actor.parameters(), _GRAD_CLIP)
            self.actor.optimizer.step()

            if gradient_step % self.target_update_interval == 0:
                polyak_update(self.critic.parameters(), self.critic_target.parameters(), self.tau)
                polyak_update(self.batch_norm_stats, self.batch_norm_stats_target, 1.0)

        self._n_updates += gradient_steps
        steps = float(gradient_steps)
        self.logger.record("train/n_updates", self._n_updates, exclude="tensorboard")
        self.logger.record("train/ent_coef", float(ent_total.item()) / steps)
        self.logger.record("train/actor_loss", float(actor_total.item()) / steps)
        self.logger.record("train/critic_loss", float(critic_total.item()) / steps)
        if ent_coef_count:
            self.logger.record("train/ent_coef_loss", float(ent_coef_total.item()) / ent_coef_count)
        self.logger.record("train/bc_coef", bc_coef)
        if bc_count:
            self.logger.record("train/bc_loss", float(bc_total.item()) / bc_count)


class CurriculumCallback(BaseCallback):
    """Hold the stage until arrivals stick, and keep refreshing follower steps.

    The demo env is separate, so collecting them does not reset the episode
    the trainer is in the middle of.
    """

    def __init__(self, demo_env, demo_set, arrivals_per_stage=16):
        super().__init__()
        self._demo_env = demo_env
        self._demo_set = demo_set
        self._arrivals = arrivals_per_stage
        self._index = 0
        self._reasons = deque(maxlen=32)
        self._goal_dist = deque(maxlen=32)
        self._goal_head = deque(maxlen=32)
        self._stage_steps = 0
        self._min_steps = 6_000
        self._advance_rate = 0.80
        self._refresh_every = 4_000

    def _stage(self):
        return STAGES[self._index]

    def _apply(self):
        name = self._stage()
        self.training_env.env_method("set_stage", name)
        self._demo_env.set_stage(name)

    def _inject(self, n_keep):
        self._demo_env.set_stage(self._stage())
        inject_demonstrations(
            self.model,
            self._demo_env,
            n_keep,
            max_tries=n_keep * 4,
            demo_set=self._demo_set,
        )

    def _rate(self):
        if not self._reasons:
            return 0.0
        return sum(reason == "finished" for reason in self._reasons) / len(self._reasons)

    def _on_training_start(self):
        self._apply()
        self._inject(self._arrivals)
        # Copy the arrivals into the actor before any random step can fill
        # the buffer. learning_starts is 0, so the first action is this mean.
        last = None
        for step in range(1200):
            last = self.model.clone_demonstrations(match_std=True)
            # Reading the loss syncs the GPU. Do it on the print, not every step.
            if last is not None and step % 300 == 0:
                print(f"clone warmup {step}: loss {float(last):.4f}")
        if last is not None:
            print(f"clone warmup done: loss {float(last):.4f}")
        return True

    def _on_step(self):
        self._stage_steps += 1
        dones = self.locals["dones"]
        infos = self.locals["infos"]
        for done, info in zip(dones, infos):
            if done:
                self._reasons.append(info.get("reason", ""))
                self._goal_dist.append(float(info.get("goal_dist_in", 0.0)))
                self._goal_head.append(float(info.get("goal_head_deg", 0.0)))

        rate = self._rate()
        full = len(self._reasons) == self._reasons.maxlen
        if (
            self._index < len(STAGES) - 1
            and self._stage_steps >= self._min_steps
            and full
            and rate >= self._advance_rate
        ):
            previous = self._stage()
            self._index += 1
            self._reasons.clear()
            self._goal_dist.clear()
            self._goal_head.clear()
            self._stage_steps = 0
            print(
                f"curriculum {previous} -> {self._stage()} at step {self.num_timesteps} "
                f"after finish rate {rate:.2f}"
            )
            self._apply()
            self._inject(self._arrivals)
        elif self.num_timesteps % self._refresh_every == 0:
            rate = self._rate()
            n = len(self._goal_dist)
            dist = sum(self._goal_dist) / n if n else 0.0
            angle = sum(self._goal_head) / n if n else 0.0
            print(
                f"stage {self._stage()} step {self.num_timesteps}: "
                f"finish {rate:.2f} over {len(self._reasons)} episodes, "
                f"dist {dist:.1f} in, angle {angle:.1f} deg"
            )
            self._inject(8)
        return True


def evaluate(model, episodes=16):
    """Deterministic finish rate on every stage, using a fresh env."""
    env = PathFollowEnv()
    print("eval (deterministic):")
    for name in STAGES:
        env.set_stage(name)
        reasons = {}
        crosses = []
        thrust_errs = []
        for index in range(episodes):
            obs, _info = env.reset(seed=10_000 + index)
            done = False
            info = {}
            while not done:
                action, _ = model.predict(obs, deterministic=True)
                obs, _reward, terminated, truncated, info = env.step(action)
                done = terminated or truncated
            reasons[info["reason"]] = reasons.get(info["reason"], 0) + 1
            crosses.append(info.get("peak_cross_in", info["cross_in"]))
            thrust_errs.append(info.get("thrust_err", 0.0))
        finished = reasons.get("finished", 0)
        mean_cross = sum(crosses) / len(crosses)
        mean_thrust = sum(thrust_errs) / len(thrust_errs)
        print(
            f"  {name:14} {finished}/{episodes} finished, "
            f"peak cross {mean_cross:.1f} in, thrust err {mean_thrust:.3f}, {reasons}"
        )


def main():
    parser = argparse.ArgumentParser(description="Train the 10 ms drive policy")
    parser.add_argument("--steps", type=int, default=500_000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--resume",
        action="store_true",
        help="continue from drive_policy.zip on the last stage, with a fresh replay buffer",
    )
    args = parser.parse_args()

    env = PathFollowEnv(gamma=DEFAULT_GAMMA)
    env.set_stage(STAGES[0])
    demo_env = PathFollowEnv(gamma=DEFAULT_GAMMA)
    demo_env.set_stage(STAGES[0])
    if args.resume:
        if not CHECKPOINT.is_file():
            raise SystemExit(f"no checkpoint at {CHECKPOINT}")
        model = DriveSAC.load(CHECKPOINT, env=env, device="cuda", gamma=DEFAULT_GAMMA)
        # Demo rows are numpy. Drop whatever tensors the zip pickled so the
        # next update uploads them to this device.
        model._demo_generation = -1
        model._demo_tensors = []
        demo_set = model.demo_set
        curriculum = CurriculumCallback(demo_env, demo_set)
        curriculum._index = len(STAGES) - 1
        print(f"resumed {CHECKPOINT} on {STAGES[-1]}, gamma {DEFAULT_GAMMA}")
    else:
        demo_set = DemoSet()
        curriculum = CurriculumCallback(demo_env, demo_set)
        model = DriveSAC(
            AsymmetricSACPolicy,
            env,
            demo_set=demo_set,
            learning_rate=3e-4,
            buffer_size=150_000,
            batch_size=_BATCH_SIZE,
            gamma=DEFAULT_GAMMA,
            tau=0.005,
            # Temperature starts at 0.01 so early noise does not kick a cloned
            # command off the path. "auto" is -dim(A) = -2. The dual optimizer
            # raises the temperature as the behavior-clone weight falls away.
            ent_coef="auto_0.01",
            target_entropy="auto",
            learning_starts=0,
            # Collect a streak, then run one update per collected step. -1
            # means "match the steps just collected".
            train_freq=_COLLECT_STEPS,
            gradient_steps=-1,
            policy_kwargs=dict(
                # Actor stays the firmware net. The critic is training-only,
                # and wide enough that a 1024-row batch is real matmul work.
                net_arch=dict(pi=[64, 64], qf=[256, 256]),
                activation_fn=nn.SiLU,
            ),
            stats_window_size=40,
            seed=args.seed,
            verbose=1,
            device="cuda",
            #device="cpu",
        )
    model.learn(total_timesteps=args.steps, callback=curriculum, progress_bar=False)
    model.save(CHECKPOINT)
    print(f"saved {CHECKPOINT}")
    evaluate(model)
    export(CHECKPOINT)


if __name__ == "__main__":
    # Checkpoints name this module, so a later viewer can import the policy
    # class. Running the file directly would otherwise pickle it as __main__.
    import sys
    sys.modules["train_drive_policy"] = sys.modules["__main__"]
    ActorObsExtractor.__module__ = "train_drive_policy"
    CriticObsExtractor.__module__ = "train_drive_policy"
    FlooredActor.__module__ = "train_drive_policy"
    AsymmetricSACPolicy.__module__ = "train_drive_policy"
    main()
