"""Train a soft actor-critic to follow a path and its thrust schedule.

The actor is the firmware network: 9 features, two hidden layers of 32,
and 3 outputs. The critic is wider and is not exported. After training,
the actor mean is written to include/control/DrivePolicyWeights.h and
data/drive_policy.csv. Rebuild the V5 image to pick up the header, and
set Settings::ControlToggles::USE_DRIVE_POLICY to run it in place of
DriftFollower. The override still has no path, so that switch does not
yet feed the features this actor was trained on.

    python python/build_plant.py
    python python/train_drive_policy.py

The curriculum does not advance on a clock. A stage changes only after
recent episodes finish the path. Example runs from the geometric tracker
stay in a side buffer and pull the actor mean toward them on every
gradient step. The replay FIFO alone was dropping those runs before the
32-unit net copied them, and the entropy target is sharper than the SAC
default so that noise does not kick a cloned command off the path.

The plant is one CPU step, and the nets are small. Training therefore
collects a streak of steps and then runs the same number of updates
back to back, with the tracker loss inside the actor backward. Losses
stay on the device until the streak ends, so a host read does not stall
the GPU between minibatches.
"""

import argparse
import sys
from collections import deque
from pathlib import Path

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.utils import polyak_update
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
# Steps gathered before a GPU streak. gradient_steps=-1 repeats the
# update this many times, so the ratio stays one gradient step per plant step.
_COLLECT_STEPS = 1024
_BATCH_SIZE = 1024


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

    def _ensure_demo_tensors(self):
        generation = self.demo_set.generation
        if generation == self._demo_generation:
            return
        self._demo_generation = generation
        device = self.device
        self._demo_tensors = [
            (
                torch.as_tensor(obs, device=device),
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
            index = torch.randint(0, obs.shape[0], (per,), device=obs.device)
            observations.append(obs.index_select(0, index))
            actions.append(act.index_select(0, index))
        return torch.cat(observations, dim=0), torch.cat(actions, dim=0)

    def _behavior_clone_loss(self, batch, match_std=False):
        obs, actions = self._demo_batch(batch)
        if obs is None:
            return None
        mean, log_std, _kwargs = self.actor.get_action_dist_params(obs)
        # Deterministic deploy action is tanh(mean). The std target is only
        # for the warmup: after that, the entropy coefficient owns the noise.
        loss = F.mse_loss(torch.tanh(mean), actions)
        if match_std:
            target = torch.full_like(log_std, _BC_LOG_STD)
            loss = loss + 0.05 * F.mse_loss(log_std, target)
        return loss

    def clone_demonstrations(self, batch=_BATCH_SIZE, match_std=False):
        """Copy arrivals into the actor before any plant step. Returns a device tensor."""
        self.policy.set_training_mode(True)
        loss = self._behavior_clone_loss(batch, match_std=match_std)
        if loss is None:
            return None
        self.actor.optimizer.zero_grad(set_to_none=True)
        (self.bc_coef * loss).backward()
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

            ent_total = ent_total + ent_coef.detach()

            if ent_coef_loss is not None and self.ent_coef_optimizer is not None:
                self.ent_coef_optimizer.zero_grad(set_to_none=True)
                ent_coef_loss.backward()
                self.ent_coef_optimizer.step()

            with torch.no_grad():
                next_actions, next_log_prob = self.actor.action_log_prob(replay_data.next_observations)
                next_q_values = torch.cat(self.critic_target(replay_data.next_observations, next_actions), dim=1)
                next_q_values, _ = torch.min(next_q_values, dim=1, keepdim=True)
                next_q_values = next_q_values - ent_coef * next_log_prob.reshape(-1, 1)
                target_q_values = replay_data.rewards + (1 - replay_data.dones) * discounts * next_q_values

            current_q_values = self.critic(replay_data.observations, replay_data.actions)
            critic_loss = 0.5 * sum(F.mse_loss(current_q, target_q_values) for current_q in current_q_values)
            critic_total = critic_total + critic_loss.detach()

            self.critic.optimizer.zero_grad(set_to_none=True)
            critic_loss.backward()
            self.critic.optimizer.step()

            q_values_pi = torch.cat(self.critic(replay_data.observations, actions_pi), dim=1)
            min_qf_pi, _ = torch.min(q_values_pi, dim=1, keepdim=True)
            actor_loss = (ent_coef * log_prob - min_qf_pi).mean()
            actor_total = actor_total + actor_loss.detach()
            bc_loss = self._behavior_clone_loss(batch_size)
            if bc_loss is not None:
                # Same coefficient as the old extra step. One backward, so
                # the follower gradient is not a second launch and a sync.
                actor_loss = actor_loss + self.bc_coef * bc_loss
                bc_total = bc_total + bc_loss.detach()
                bc_count += 1

            self.actor.optimizer.zero_grad(set_to_none=True)
            actor_loss.backward()
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
            self._stage_steps = 0
            print(
                f"curriculum {previous} -> {self._stage()} at step {self.num_timesteps} "
                f"after finish rate {rate:.2f}"
            )
            self._apply()
            self._inject(self._arrivals)
        elif self.num_timesteps % self._refresh_every == 0:
            rate = self._rate()
            print(
                f"stage {self._stage()} step {self.num_timesteps}: "
                f"finish {rate:.2f} over {len(self._reasons)} episodes"
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
    args = parser.parse_args()

    env = PathFollowEnv(gamma=DEFAULT_GAMMA)
    env.set_stage(STAGES[0])
    demo_env = PathFollowEnv(gamma=DEFAULT_GAMMA)
    demo_env.set_stage(STAGES[0])
    demo_set = DemoSet()
    curriculum = CurriculumCallback(demo_env, demo_set)
    model = DriveSAC(
        "MlpPolicy",
        env,
        demo_set=demo_set,
        learning_rate=3e-4,
        buffer_size=150_000,
        batch_size=_BATCH_SIZE,
        gamma=DEFAULT_GAMMA,
        tau=0.005,
        # Start modest. The default auto coef begins at 1 and drowns the
        # tracking return. Target -5 (default is -3) keeps the 3-axis noise
        # from kicking a cloned command off the path.
        ent_coef="auto_0.1",
        target_entropy=-5.0,
        learning_starts=0,
        # Collect a streak, then run one update per collected step. -1
        # means "match the steps just collected".
        train_freq=_COLLECT_STEPS,
        gradient_steps=-1,
        policy_kwargs=dict(
            # Actor stays the firmware net. The critic is training-only,
            # and wide enough that a 1024-row batch is real matmul work.
            net_arch=dict(pi=[32, 32], qf=[256, 256]),
            activation_fn=nn.ReLU,
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
    main()
