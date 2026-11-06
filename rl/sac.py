# Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.
"""Soft actor-critic on CPU, as in RL-augmented MPC: actor and critic MLPs (512, 256, 128) with
ELU, a tanh-squashed Gaussian policy, automatic entropy tuning, and a squared-action penalty
in the actor loss against saturated actions. The critic sees the same (on-board) observation."""

from dataclasses import dataclass

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F


@dataclass
class SacConfig:
    hidden: tuple = (512, 256, 128)
    lr: float = 3e-4
    gamma: float = 0.99
    tau: float = 0.005
    batch_size: int = 512
    buffer_size: int = 500_000
    action_penalty: float = 0.01
    init_alpha: float = 0.1
    obs_clip: float = 5.0
    # Against critic overestimation, which the actor otherwise exploits: an ensemble of critics whose
    # target is the minimum of a random subset (REDQ), the actor on the ensemble mean (the minimum
    # of the two with the default pair), updated every `actor_delay` critic updates (TD3), and a pull
    # of the deterministic action towards zero, i.e. the MPC alone, with the actor's objective in
    # units of the critic's own scale (TD3+BC). The defaults are plain SAC.
    n_critics: int = 2
    n_target_min: int = 2
    actor_delay: int = 1
    prior_weight: float = 0.0


def mlp(sizes, out):
    layers = []
    for a, b in zip(sizes[:-1], sizes[1:]):
        layers += [nn.Linear(a, b), nn.ELU()]
    layers.append(nn.Linear(sizes[-1], out))
    return nn.Sequential(*layers)


class RunningNorm:
    """Welford mean / variance of the observations; frozen into the exported policy."""

    def __init__(self, dim):
        self.mean = np.zeros(dim)
        self.var = np.ones(dim)
        self.count = 1e-4

    def update(self, x):
        x = np.asarray(x, dtype=np.float64).reshape(-1, self.mean.size)
        if len(x) == 0:
            return
        batch_mean, batch_var, n = x.mean(0), x.var(0), len(x)
        delta = batch_mean - self.mean
        total = self.count + n
        self.mean = self.mean + delta * n / total
        self.var = (self.var * self.count + batch_var * n + delta ** 2 * self.count * n / total) / total
        self.count = total

    @property
    def std(self):
        return np.sqrt(np.maximum(self.var, 1e-8))


class Actor(nn.Module):
    def __init__(self, obs_dim, act_dim, hidden, init_std=0.3):
        super().__init__()
        self.net = mlp((obs_dim,) + tuple(hidden), 2 * act_dim)
        self.act_dim = act_dim
        # A residual on a working controller: start as the MPC alone (zero mean action), with
        # state-independent exploration of init_std.
        last = self.net[-1]
        with torch.no_grad():
            last.weight.zero_()
            last.bias[:act_dim].zero_()
            last.bias[act_dim:].fill_(float(np.log(init_std)))

    def forward(self, obs):
        mean, log_std = self.net(obs).split(self.act_dim, dim=-1)
        return mean, log_std.clamp(-5.0, 2.0)

    def sample(self, obs):
        mean, log_std = self(obs)
        std = log_std.exp()
        pre = mean + std * torch.randn_like(mean)
        action = torch.tanh(pre)
        log_prob = (-0.5 * ((pre - mean) / std) ** 2 - log_std - 0.5 * np.log(2 * np.pi)).sum(-1)
        log_prob = log_prob - torch.log(1 - action ** 2 + 1e-6).sum(-1)
        return action, log_prob, torch.tanh(mean)


class Critic(nn.Module):
    def __init__(self, obs_dim, act_dim, hidden, n=2):
        super().__init__()
        self.qs = nn.ModuleList(mlp((obs_dim + act_dim,) + tuple(hidden), 1) for _ in range(n))

    def forward(self, obs, act):
        """(n_critics, batch)"""
        x = torch.cat([obs, act], -1)
        return torch.stack([q(x).squeeze(-1) for q in self.qs])

    @staticmethod
    def upgrade(state):
        """A checkpoint from the two-critic layout (q1, q2)."""
        return {(k.replace("q1.", "qs.0.", 1) if k.startswith("q1.") else k.replace("q2.", "qs.1.", 1) if k.startswith("q2.") else k): v
                for k, v in state.items()}


class ReplayBuffer:
    def __init__(self, capacity, obs_dim, act_dim):
        self.obs = np.zeros((capacity, obs_dim), np.float32)
        self.next_obs = np.zeros((capacity, obs_dim), np.float32)
        self.act = np.zeros((capacity, act_dim), np.float32)
        self.rew = np.zeros(capacity, np.float32)
        self.done = np.zeros(capacity, np.float32)
        self.capacity, self.size, self.ptr = capacity, 0, 0

    def add(self, obs, act, rew, next_obs, done):
        for o, a, r, n, d in zip(obs, act, rew, next_obs, done):
            self.obs[self.ptr], self.act[self.ptr], self.rew[self.ptr] = o, a, r
            self.next_obs[self.ptr], self.done[self.ptr] = n, d
            self.ptr = (self.ptr + 1) % self.capacity
            self.size = min(self.size + 1, self.capacity)

    def sample(self, n, rng):
        i = rng.integers(0, self.size, n)
        return self.obs[i], self.act[i], self.rew[i], self.next_obs[i], self.done[i]


class Sac:
    def __init__(self, obs_dim, act_dim, config=SacConfig(), seed=0):
        torch.manual_seed(seed)
        self.cfg = config
        self.rng = np.random.default_rng(seed)
        self.actor = Actor(obs_dim, act_dim, config.hidden)
        self.critic = Critic(obs_dim, act_dim, config.hidden, config.n_critics)
        self.critic_target = Critic(obs_dim, act_dim, config.hidden, config.n_critics)
        self.critic_target.load_state_dict(self.critic.state_dict())
        self.log_alpha = torch.tensor(np.log(config.init_alpha), requires_grad=True)
        self.target_entropy = -float(act_dim)
        self.actor_opt = torch.optim.Adam(self.actor.parameters(), lr=config.lr)
        self.critic_opt = torch.optim.Adam(self.critic.parameters(), lr=config.lr)
        self.alpha_opt = torch.optim.Adam([self.log_alpha], lr=config.lr)
        self.norm = RunningNorm(obs_dim)
        self.buffer = ReplayBuffer(config.buffer_size, obs_dim, act_dim)
        self.updates = 0
        self.actor_stats = {}

    def _normalize(self, obs):
        x = (np.asarray(obs, np.float64) - self.norm.mean) / self.norm.std
        return torch.as_tensor(np.clip(x, -self.cfg.obs_clip, self.cfg.obs_clip), dtype=torch.float32)

    @torch.no_grad()
    def act(self, obs, deterministic=False):
        action, _, mean = self.actor.sample(self._normalize(obs))
        return (mean if deterministic else action).numpy()

    def update(self):
        cfg = self.cfg
        if self.buffer.size < cfg.batch_size:
            return {}
        o, a, r, o2, d = self.buffer.sample(cfg.batch_size, self.rng)
        o, o2 = self._normalize(o), self._normalize(o2)
        a, r, d = torch.as_tensor(a), torch.as_tensor(r), torch.as_tensor(d)
        alpha = self.log_alpha.exp().detach()
        with torch.no_grad():
            a2, logp2, _ = self.actor.sample(o2)
            q_next = self.critic_target(o2, a2)
            subset = torch.randperm(cfg.n_critics)[:cfg.n_target_min]
            target = r + cfg.gamma * (1 - d) * (q_next[subset].min(0).values - alpha * logp2)
        qs = self.critic(o, a)
        critic_loss = ((qs - target) ** 2).mean(1).sum()
        self.critic_opt.zero_grad()
        critic_loss.backward()
        self.critic_opt.step()

        if self.updates % cfg.actor_delay == 0:
            pi, logp, mean = self.actor.sample(o)
            q_all = self.critic(o, pi)
            q_pi = q_all.min(0).values if cfg.n_critics == 2 else q_all.mean(0)
            scale = q_pi.abs().mean().detach().clamp_min(1.0) if cfg.prior_weight > 0 else 1.0
            actor_loss = (alpha * logp - q_pi).mean() / scale + cfg.action_penalty * (pi ** 2).sum(-1).mean() \
                + cfg.prior_weight * (mean ** 2).sum(-1).mean()
            self.actor_opt.zero_grad()
            actor_loss.backward()
            self.actor_opt.step()

            alpha_loss = -(self.log_alpha * (logp.detach() + self.target_entropy)).mean()
            self.alpha_opt.zero_grad()
            alpha_loss.backward()
            self.alpha_opt.step()
            self.actor_stats = {"actor_loss": actor_loss.item(), "entropy": -logp.mean().item(),
                                "action_rms": mean.pow(2).mean().sqrt().item()}

        with torch.no_grad():
            for p, pt in zip(self.critic.parameters(), self.critic_target.parameters()):
                pt.mul_(1 - cfg.tau).add_(cfg.tau * p)
        self.updates += 1
        return {"critic_loss": critic_loss.item(), "alpha": alpha.item(), "q": qs.mean().item(),
                "q_spread": qs.std(0).mean().item(), **self.actor_stats}

    def state_dict(self):
        return {"actor": self.actor.state_dict(), "critic": self.critic.state_dict(),
                "critic_target": self.critic_target.state_dict(), "log_alpha": self.log_alpha.detach(),
                "norm": (self.norm.mean, self.norm.var, self.norm.count), "updates": self.updates,
                "actor_opt": self.actor_opt.state_dict(), "critic_opt": self.critic_opt.state_dict(),
                "alpha_opt": self.alpha_opt.state_dict()}

    def load_state_dict(self, state):
        self.actor.load_state_dict(state["actor"])
        self.critic.load_state_dict(Critic.upgrade(state["critic"]))
        self.critic_target.load_state_dict(Critic.upgrade(state["critic_target"]))
        self.log_alpha.data.copy_(state["log_alpha"])
        self.norm.mean, self.norm.var, self.norm.count = state["norm"]
        self.updates = state["updates"]
        self.actor_opt.load_state_dict(state["actor_opt"])
        self.critic_opt.load_state_dict(state["critic_opt"])
        self.alpha_opt.load_state_dict(state["alpha_opt"])

    def export(self, path):
        """The deterministic actor for the controller (GaitAdaptationPolicy): normalization, layers, tanh."""
        linears = [m for m in self.actor.net if isinstance(m, nn.Linear)]
        act_dim = self.actor.act_dim
        with open(path, "w") as f:
            f.write(f"gait_adaptation_policy 1\nobs_dim {self.norm.mean.size} act_dim {act_dim} obs_clip {self.cfg.obs_clip}\n")
            f.write("obs_mean " + " ".join(f"{v:.9g}" for v in self.norm.mean) + "\n")
            f.write("obs_std " + " ".join(f"{v:.9g}" for v in self.norm.std) + "\n")
            f.write(f"layers {len(linears)}\n")
            for k, layer in enumerate(linears):
                weight, bias = layer.weight.detach().numpy(), layer.bias.detach().numpy()
                if k == len(linears) - 1:  # the action mean only, not the log std
                    weight, bias = weight[:act_dim], bias[:act_dim]
                f.write(f"layer {weight.shape[1]} {weight.shape[0]} {'linear' if k == len(linears) - 1 else 'elu'}\n")
                f.write(" ".join(f"{v:.9g}" for v in weight.reshape(-1)) + "\n")
                f.write(" ".join(f"{v:.9g}" for v in bias) + "\n")
