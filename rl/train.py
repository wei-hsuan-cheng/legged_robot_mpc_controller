#!/usr/bin/env python3
# Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.
"""Train the H2 WB-ID MPC gait-adaptation policy with SAC on N lockstep simulations (CPU).

  cd ~/ocs2_ros2_ws && source install/setup.bash
  python3 <repo>/rl/train.py --num-envs 8 --steps 600000 --out /tmp/claude_rl/run1

--baseline runs zero actions (the MPC alone) through the same episodes and rewards instead of
learning: the reference return. Every --checkpoint-every transitions it writes checkpoint.pt and
policy.txt (the actor for gaitAdaptation.source policy) to --out; progress.jsonl has the metrics.
"""

import argparse
import json
import signal
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from h2_gait_env import ACT_DIM, OBS_DIM, EnvConfig, H2GaitEnv  # noqa: E402


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    # Started in the background by a non-interactive shell, SIGINT arrives ignored: make it and
    # SIGTERM stop the run through the finally below, which saves the agent and closes the sims.
    signal.signal(signal.SIGINT, _interrupt)
    signal.signal(signal.SIGTERM, _interrupt)
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--num-envs", type=int, default=8)
    parser.add_argument("--steps", type=int, default=600_000, help="transitions to collect")
    parser.add_argument("--warmup", type=int, default=20_000,
                        help="transitions with exploration noise around zero (the MPC alone) first")
    parser.add_argument("--warmup-std", type=float, default=0.5)
    parser.add_argument("--updates-per-step", type=int, default=1,
                        help="gradient steps per MPC tick (EnvConfig.action_repeat ticks per decision)")
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--out", type=Path, default=Path("/tmp/claude_rl/run"))
    parser.add_argument("--workspace", type=Path, default=Path.home() / "ocs2_ros2_ws")
    parser.add_argument("--checkpoint-every", type=int, default=50_000)
    parser.add_argument("--log-every", type=float, default=60.0, help="[s] wall")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--baseline", action="store_true", help="zero actions, no learning")
    parser.add_argument("--threads", type=int, default=2)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--gamma", type=float, default=0.99, help="2 s horizon at 50 Hz decisions, for near-fall recovery")
    parser.add_argument("--critics", type=int, default=2, help="critic ensemble size (SacConfig.n_critics)")
    parser.add_argument("--actor-delay", type=int, default=1, help="critic updates per actor update")
    parser.add_argument("--prior-weight", type=float, default=0.0,
                        help="pull of the deterministic action towards zero (the MPC alone), in units of |Q|")
    parser.add_argument("--flat", action="store_true", help="flat ground only (no terrain curriculum)")
    parser.add_argument("--baseline-envs", type=int, default=0,
                        help="the last K environments run the MPC alone throughout: a live control group, "
                             "logged as baseline_*, not learned from")
    parser.add_argument("--eval-envs", type=int, default=0,
                        help="the K environments before those run the deterministic policy (what deployment "
                             "runs), logged as eval_*, learned from")
    args = parser.parse_args()

    import torch
    torch.set_num_threads(args.threads)
    from sac import Sac, SacConfig

    args.out.mkdir(parents=True, exist_ok=True)
    agent = None if args.baseline else Sac(OBS_DIM, ACT_DIM, SacConfig(batch_size=args.batch_size, gamma=args.gamma,
                                                                         n_critics=args.critics, actor_delay=args.actor_delay,
                                                                         prior_weight=args.prior_weight), seed=args.seed)
    collected = 0
    state = None
    if agent is not None and args.resume and (args.out / "checkpoint.pt").exists():
        state = torch.load(args.out / "checkpoint.pt", weights_only=False)
        agent.load_state_dict(state["agent"])
        collected = state["collected"]
        print(f"resumed at {collected} transitions (replay buffer starts empty)", flush=True)

    env = H2GaitEnv(EnvConfig(num_envs=args.num_envs, terrain=not args.flat), args.out / "sim", args.workspace,
                    seed=args.seed + collected,
                    follower_envs=args.baseline_envs + args.eval_envs if agent is not None else 0)
    if state is not None:
        # The curricula: from the checkpoint, or (older checkpoints) the last logged disturbance level.
        env.start_levels = state.get("levels")
        disturbance = state.get("disturbance")
        if disturbance is None and (args.out / "progress.jsonl").exists():
            lines = (args.out / "progress.jsonl").read_text().splitlines()
            disturbance = json.loads(lines[-1]).get("disturbance") if lines else None
        if disturbance is not None:
            env.disturbance = float(disturbance)
        print(f"curricula: disturbance {env.disturbance}, levels {env.start_levels}", flush=True)
    progress = open(args.out / ("baseline.jsonl" if args.baseline else "progress.jsonl"), "a")
    rng = np.random.default_rng(args.seed)
    try:
        obs = env.reset()
        wall_start = last_log = time.monotonic()
        collected_at_start = collected  # a resumed run's rate counts only its own transitions
        returns = np.zeros(env.n)
        # Groups: learning (stochastic policy), eval (deterministic policy), control (MPC alone). The
        # last two take the learners' terrain levels, so their fall rates compare directly.
        control = np.zeros(env.n, dtype=bool)
        evaluate = np.zeros(env.n, dtype=bool)
        if agent is not None:
            k = args.baseline_envs
            if k > 0:
                control[-k:] = True
            if args.eval_envs > 0:
                evaluate[env.n - k - args.eval_envs:env.n - k] = True
        learner = ~control & ~evaluate
        episodes, window = [], {"terms": {}, "train": []}
        groups = {"baseline": (control, []), "eval": (evaluate, [])}
        steps = 0
        print(f"{env.n} environments running; collecting {args.steps} transitions", flush=True)
        while collected < args.steps:
            if agent is None:
                action = np.zeros((env.n, ACT_DIM))
            elif collected < args.warmup:
                action = np.clip(rng.normal(0.0, args.warmup_std, (env.n, ACT_DIM)), -1.0, 1.0)
            else:
                action = agent.act(obs)
            if agent is not None and evaluate.any():
                action[evaluate] = agent.act(obs[evaluate], deterministic=True)
            action[control] = 0.0

            def train():
                # Also after a resume, whose replay buffer starts empty: never fit a few hundred samples.
                if agent is not None and collected >= args.warmup // 2 and agent.buffer.size >= args.warmup // 2:
                    for _ in range(args.updates_per_step):
                        stats = agent.update()
                        if stats:
                            window["train"].append(stats)

            next_obs, reward, terminated, truncated, active, info = env.step(action, between=train)
            returns[active] += reward[active]
            learn = active & ~control
            if learn.any():
                if agent is not None:
                    agent.buffer.add(obs[learn], action[learn], reward[learn], next_obs[learn], terminated[learn])
                    agent.norm.update(obs[learn])
                collected += int(learn.sum())
                for k, v in info["terms"].items():
                    window["terms"].setdefault(k, []).append(float(np.mean(v[learner[active]])) if learner[active].any() else 0.0)
            for i, end in zip(np.flatnonzero(terminated | truncated), info["ended"]):
                record = {"return": float(returns[i]), "length": float(info["elapsed"][i]), "fell": bool(terminated[i]), **end}
                (groups["baseline"][1] if control[i] else groups["eval"][1] if evaluate[i] else episodes).append(record)
                returns[i] = 0.0
            obs = next_obs
            steps += 1

            now = time.monotonic()
            if now - last_log > args.log_every:
                last_log = now
                record = {"collected": collected, "wall_s": round(now - wall_start, 1),
                          "transitions_per_s": round((collected - collected_at_start) / max(now - wall_start, 1e-6), 1),
                          "episodes": len(episodes), "sync_violations": info["sync_violations"]}
                if episodes:
                    record.update({"after_push_fall_rate": float(np.mean([e["after_push"] for e in episodes])),
                                   "return_mean": float(np.mean([e["return"] for e in episodes])),
                                   "length_mean": float(np.mean([e["length"] for e in episodes])),
                                   "fall_rate": float(np.mean([e["fell"] for e in episodes])),
                                   "walked_mean": float(np.mean([e["walked"] for e in episodes]))})
                    by_type = {}
                    for e in episodes:
                        by_type.setdefault(e["terrain"], []).append(e["fell"])
                    record["fall_rate_by_terrain"] = {str(k): [round(float(np.mean(v)), 3), len(v)] for k, v in by_type.items()}
                record["level_mean"] = round(float(np.mean(info["levels"][learner])), 2)
                record["disturbance"] = round(env.disturbance, 2)
                for name, (mask, group) in groups.items():
                    if not mask.any():
                        continue
                    record[f"{name}_level_mean"] = round(float(np.mean(info["levels"][mask])), 2)
                    if group:
                        record.update({f"{name}_episodes": len(group),
                                       f"{name}_return_mean": float(np.mean([e["return"] for e in group])),
                                       f"{name}_fall_rate": float(np.mean([e["fell"] for e in group])),
                                       f"{name}_after_push_fall_rate": float(np.mean([e["after_push"] for e in group]))})
                record["terms"] = {k: round(float(np.mean(v)), 4) for k, v in window["terms"].items()}
                if window["train"]:
                    record["train"] = {k: round(float(np.mean([s[k] for s in window["train"]])), 4) for k in window["train"][0]}
                if agent is not None and collected >= args.warmup:
                    record["action_mean"] = [round(float(x), 3) for x in agent.act(obs, deterministic=True).mean(0)]
                progress.write(json.dumps(record) + "\n")
                progress.flush()
                print(json.dumps(record), flush=True)
                episodes, window = [], {"terms": {}, "train": []}
                for _, group in groups.values():
                    if len(group) >= 20:
                        group.clear()

            if agent is not None and (collected // args.checkpoint_every) != ((collected - int(learn.sum())) // args.checkpoint_every):
                torch.save({"agent": agent.state_dict(), "collected": collected, "disturbance": env.disturbance,
                        "levels": env.level.tolist() if hasattr(env, "level") else None}, args.out / "checkpoint.pt")
                agent.export(args.out / "policy.txt")
                agent.export(args.out / f"policy_{collected // 1000}k.txt")
    finally:
        if agent is not None:
            torch.save({"agent": agent.state_dict(), "collected": collected, "disturbance": env.disturbance,
                        "levels": env.level.tolist() if hasattr(env, "level") else None}, args.out / "checkpoint.pt")
            agent.export(args.out / "policy.txt")
        progress.close()
        env.close()


if __name__ == "__main__":
    main()
