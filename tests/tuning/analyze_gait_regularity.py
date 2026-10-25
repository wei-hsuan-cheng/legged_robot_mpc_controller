#!/usr/bin/env python3
"""Gait regularity of a recorded wb_id_walk_test.py run: is the motion a clean, repeatable gait?

Complements analyze_running.py (flight) and analyze_leg_clearance.py (crossing). From the
measured-pose replay it detects each foot's contact (active collision geometry within 4 mm
of the floor, bouts shorter than 30 ms merged), then reports over the hold phase:
step timing and its variability, step length/width per touchdown, left/right asymmetry,
agreement between planned and replayed contact, speed steadiness, body motion, heading
drift, actuator saturation and the MPC rate from the controller log.

Replay contact is geometric, from latest-sample-paired odometry and joints: it is not a
measured contact force. Run analyze_running.py first only if its plots are wanted; this
script needs just the harness outputs (<prefix>.json/.yaml/_replay.json/_tracking.csv/.log).
"""

import argparse
import csv
import json
import re
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mujoco_scene import temporary_scene

CONTROLLER = "humanoid_whole_body_inverse_dynamics_mpc_controller"
CONTACT_Z = 0.004       # [m] collision low point within this of the floor counts as contact
MIN_BOUT = 0.03         # [s] shorter contact/no-contact bouts are merged into their neighbours
LEG_JOINTS = ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")


def lowest_point(model, data, geom):
    kind, size = model.geom_type[geom], model.geom_size[geom]
    z = data.geom_xpos[geom, 2]
    axis = data.geom_xmat[geom].reshape(3, 3)[2]
    if kind == mujoco.mjtGeom.mjGEOM_SPHERE:
        return z - size[0]
    if kind == mujoco.mjtGeom.mjGEOM_BOX:
        return z - np.abs(axis) @ size
    if kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
        return z - abs(axis[2]) * size[1] - size[0]
    if kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
        return z - abs(axis[2]) * size[1] - size[0] * np.sqrt(max(0, 1 - axis[2] ** 2))
    if kind == mujoco.mjtGeom.mjGEOM_MESH:
        mesh = model.geom_dataid[geom]
        begin, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        return z + np.min(model.mesh_vert[begin:begin + count] @ axis)
    raise ValueError(f"Unsupported foot collision geometry {kind}")


def debounce(mask, times):
    """Merge bouts shorter than MIN_BOUT into the surrounding state."""
    mask = mask.copy()
    for _ in range(2):
        edges = np.flatnonzero(np.diff(mask.astype(int))) + 1
        bounds = np.r_[0, edges, len(mask)]
        for a, b in zip(bounds[:-1], bounds[1:]):
            if 0 < a and b < len(mask) and times[b - 1] - times[a] < MIN_BOUT:
                mask[a:b] = mask[a - 1]
    return mask


def bouts(mask, times):
    edges = np.flatnonzero(np.diff(mask.astype(int))) + 1
    bounds = np.r_[0, edges, len(mask)]
    return [(bool(mask[a]), times[a], times[b - 1], a) for a, b in zip(bounds[:-1], bounds[1:])]


def cv(values):
    values = np.asarray(values, float)
    return float(values.std() / values.mean()) if len(values) > 1 and values.mean() > 0 else None


def stats(values):
    values = np.asarray(values, float)
    if not len(values):
        return None
    return dict(mean=float(values.mean()), sd=float(values.std()), min=float(values.min()),
                max=float(values.max()), n=int(len(values)))


def mpc_health(log_path):
    text = log_path.read_text(errors="replace") if log_path.exists() else ""
    rates = [float(x) for x in re.findall(r"MPC_TIMING\] target=[\d.]+ Hz, actual=([\d.]+) Hz", text)]
    sse = [float(x) for x in re.findall(r"equality SSE mean=([\d.e+-]+)", text)]
    overruns = [int(x) for x in re.findall(r"torque-window overruns=(\d+)", text)]
    exceptions = [int(x) for x in re.findall(r"solver exceptions=(\d+)", text)]
    return dict(mpc_rate=stats(rates[1:]) if len(rates) > 1 else stats(rates),
                equality_sse_mean=stats(sse),
                torque_window_overruns=max(overruns, default=None),
                solver_exceptions=max(exceptions, default=None))


def analyze(prefix, robot, plot=True, frames=None):
    root = Path(__file__).resolve().parents[2]
    with temporary_scene(robot) as scene:
        model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    result = json.loads(Path(str(prefix) + ".json").read_text())
    config = yaml.safe_load(Path(str(prefix) + ".yaml").read_text())[CONTROLLER]["ros__parameters"]
    settings = config["ocs2"]["model"]
    samples = [s for s in json.loads(Path(str(prefix) + "_replay.json").read_text()) if s["phase"] != "startup"]
    t0 = samples[0]["time"]
    times = np.array([s["time"] - t0 for s in samples])
    phase = np.array([s["phase"] for s in samples])
    hold = phase == "walk"
    if not ((phase == "ramp_in") | hold).any():
        raise SystemExit(f"{prefix}: no moving samples (verdict {result.get('verdict')})")

    joints = {model.joint(i).name: model.jnt_qposadr[i] for i in range(model.njnt)}
    feet = [model.jnt_bodyid[model.joint(name).id] for name in settings["contactParentJointNames"]]
    foot_geoms = [[g for g in range(model.ngeom) if model.geom_bodyid[g] == body
                   and (model.geom_contype[g] or model.geom_conaffinity[g])] for body in feet]
    offset = np.array(settings["contactGeometry"]["frameTranslation"], float)
    low, centre, base, rpy = [], [], [], []
    for sample in samples:
        data.qpos[:3] = sample["position"]
        data.qpos[3:7] = sample["quaternion_wxyz"]
        for name, value in zip(sample["names"], sample["q"]):
            if name in joints:
                data.qpos[joints[name]] = value
        mujoco.mj_kinematics(model, data)
        low.append([min(lowest_point(model, data, g) for g in group) for group in foot_geoms])
        centre.append([data.xpos[b] + data.xmat[b].reshape(3, 3) @ offset for b in feet])
        w, x, y, z = sample["quaternion_wxyz"]
        rpy.append([np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
                    np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)),
                    np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))])
        base.append(sample["position"])
    low, centre, base, rpy = np.array(low), np.array(centre), np.array(base), np.array(rpy)
    rpy[:, 2] = np.unwrap(rpy[:, 2])
    contact = np.column_stack([debounce(low[:, i] <= CONTACT_Z, times) for i in range(2)])

    # Samples after the fall begins (tilt > 20 deg or base below 0.7 m) are excluded everywhere below.
    upset = (np.abs(rpy[:, :2]).max(axis=1) > 0.35) | (base[:, 2] < 0.7)
    t_upset = times[np.argmax(upset)] if upset.any() else np.inf
    valid = times < t_upset
    hold &= valid
    if not hold.any():
        hold = valid & (phase == "ramp_in")
    h0, h1 = times[hold][0], times[hold][-1]
    # Stance/swing bouts of the moving window; bouts cut by the window edges are dropped.
    m0 = times[(phase == "ramp_in") | (phase == "walk")][0]
    m1 = min(times[(phase == "ramp_in") | (phase == "walk")][-1], t_upset)
    touchdowns, stance, swing = [], [[], []], [[], []]
    for foot in range(2):
        for in_contact, start, end, index in bouts(contact[:, foot], times):
            if start <= m0 or end >= m1:
                continue
            if h0 < start and end < h1:
                (stance if in_contact else swing)[foot].append(end - start)
            if in_contact:
                touchdowns.append((start, foot, index))
    touchdowns.sort()
    steps = []
    for (ta, fa, ia), (tb, fb, ib) in zip(touchdowns, touchdowns[1:]):
        heading = rpy[ib, 2]
        forward = np.array([np.cos(heading), np.sin(heading)])
        lateral = np.array([-np.sin(heading), np.cos(heading)])
        delta = centre[ib, fb, :2] - centre[ia, fa, :2]
        steps.append(dict(time=tb, from_foot=fa, to_foot=fb, period=tb - ta, alternating=fa != fb,
                          length=float(delta @ forward), width=float(abs(delta @ lateral))))
    all_alternating = [s for s in steps if s["alternating"]]
    steps = [s for s in steps if h0 < s["time"] < h1]
    alternating = [s for s in steps if s["alternating"]]
    by_side = {side: [s for s in alternating if s["to_foot"] == foot] for foot, side in enumerate(("left", "right"))}

    both, none = contact.all(axis=1), ~contact.any(axis=1)
    pose_v = np.gradient(base[:, :2], times, axis=0)
    yaw = rpy[:, 2]
    forward_v = pose_v[:, 0] * np.cos(yaw) + pose_v[:, 1] * np.sin(yaw)
    # One-second blocks of pose-derived world-x speed: the steadiness of the run, not just its mean.
    edges = np.arange(h0, h1, 1.0)
    blocks = [(base[(times >= a) & (times < a + 1), 0][-1] - base[(times >= a) & (times < a + 1), 0][0])
              for a in edges[:-1] if ((times >= a) & (times < a + 1)).sum() > 10]

    summary = dict(prefix=Path(prefix).name, verdict=result.get("verdict"), command=result.get("speed"),
                   sim_seconds=result.get("sim_seconds"), upset_time=float(t_upset) if np.isfinite(t_upset) else None)
    span = times[hold][-1] - times[hold][0]
    summary.update(
        hold_world_vx=float((base[hold, 0][-1] - base[hold, 0][0]) / span),
        hold_world_vy=float((base[hold, 1][-1] - base[hold, 1][0]) / span),
        hold_vx_1s_blocks=stats(blocks),
        hold_heading_drift=float(yaw[hold][-1] - yaw[hold][0]),
        hold_lateral_drift=float(-(base[hold][-1, 0] - base[hold][0, 0]) * np.sin(yaw[hold][0])
                                 + (base[hold][-1, 1] - base[hold][0, 1]) * np.cos(yaw[hold][0])),
        hold_roll=dict(rms=float(np.sqrt(np.mean(rpy[hold, 0] ** 2))), max=float(np.abs(rpy[hold, 0]).max())),
        hold_pitch=dict(mean=float(rpy[hold, 1].mean()), sd=float(rpy[hold, 1].std()),
                        max=float(np.abs(rpy[hold, 1]).max())),
        hold_base_z=stats(base[hold, 2]),
        contact=dict(flight_fraction=float(none[hold].mean()), double_support_fraction=float(both[hold].mean()),
                     left_stance=stats(stance[0]), right_stance=stats(stance[1]),
                     left_swing=stats(swing[0]), right_swing=stats(swing[1])),
        steps=dict(count=len(steps), non_alternating=len(steps) - len(alternating),
                   cadence=float(len(alternating) / span),
                   period=stats([s["period"] for s in alternating]),
                   period_cv=cv([s["period"] for s in alternating]),
                   length=stats([s["length"] for s in alternating]),
                   length_cv=cv([s["length"] for s in alternating]),
                   width=stats([s["width"] for s in alternating]),
                   left_length=stats([s["length"] for s in by_side["left"]]),
                   right_length=stats([s["length"] for s in by_side["right"]])),
    )
    ll, rl = summary["steps"]["left_length"], summary["steps"]["right_length"]
    if ll and rl:
        summary["steps"]["length_asymmetry"] = float(abs(ll["mean"] - rl["mean"]) / max(1e-6, 0.5 * (ll["mean"] + rl["mean"])))
    ls, rs = summary["contact"]["left_stance"], summary["contact"]["right_stance"]
    if ls and rs:
        summary["contact"]["stance_asymmetry"] = float(abs(ls["mean"] - rs["mean"]) / (0.5 * (ls["mean"] + rs["mean"])))

    # Speed bins over the whole moving window (ramp_in + hold): where along a ramp the gait degrades.
    moving = ((phase == "ramp_in") | (phase == "walk")) & valid
    smooth = np.convolve(forward_v, np.ones(101) / 101, mode="same")  # 1 s centred mean at 100 Hz
    step_times = np.array([s["time"] for s in all_alternating])
    step_speed = smooth[np.clip(np.searchsorted(times, step_times), 0, len(times) - 1)] if len(step_times) else np.array([])
    speed_bins = []
    for lo in np.arange(0.0, 3.0, 0.25):
        in_bin = moving & (smooth >= lo) & (smooth < lo + 0.25)
        if in_bin.sum() < 50:
            continue
        chosen = [s for s, v in zip(all_alternating, step_speed) if lo <= v < lo + 0.25]
        side_len = [[s["length"] for s in chosen if s["to_foot"] == f] for f in (0, 1)]
        speed_bins.append(dict(
            speed=[float(lo), float(lo + 0.25)], seconds=float(in_bin.sum() * np.median(np.diff(times))),
            steps=len(chosen), period=float(np.mean([s["period"] for s in chosen])) if chosen else None,
            period_cv=cv([s["period"] for s in chosen]), length=float(np.mean([s["length"] for s in chosen])) if chosen else None,
            length_cv=cv([s["length"] for s in chosen]),
            length_asymmetry=(float(abs(np.mean(side_len[0]) - np.mean(side_len[1])) / max(1e-6, 0.5 * (np.mean(side_len[0]) + np.mean(side_len[1]))))
                              if all(side_len) else None),
            width=float(np.mean([s["width"] for s in chosen])) if chosen else None,
            flight=float(none[in_bin].mean()), double_support=float(both[in_bin].mean()),
            roll_rms=float(np.sqrt(np.mean(rpy[in_bin, 0] ** 2))), pitch_mean=float(rpy[in_bin, 1].mean()),
            pitch_sd=float(rpy[in_bin, 1].std()), z_mean=float(base[in_bin, 2].mean()), z_sd=float(base[in_bin, 2].std()),
            yaw_rate=float(np.polyfit(times[in_bin], yaw[in_bin], 1)[0]) if in_bin.sum() > 2 else None))
    summary["speed_bins"] = speed_bins

    trace_path = Path(str(prefix) + "_tracking.csv")
    if trace_path.exists():
        with trace_path.open() as file:
            trace = list(csv.DictReader(file))
        trace_t = np.array([float(r["t"]) - t0 for r in trace])
        plan_mode = np.nan_to_num(np.array([float(r["plan_mode"]) for r in trace]), nan=-1.0)
        # Mode code: bit 0 = right in contact, bit 1 = left in contact (0 FLY, 1 RF, 2 LF, 3 STANCE).
        index = np.clip(np.searchsorted(trace_t, times), 0, len(trace_t) - 1)
        planned = plan_mode[index].astype(int)
        actual = 2 * contact[:, 0] + contact[:, 1]
        summary["planned_vs_replayed_mode_agreement"] = float(np.mean(planned[hold] == actual[hold]))
        summary["planned_flight_fraction"] = float(np.mean(planned[hold] == 0))
        est_vx = np.array([float(r["obs_base/vx"]) for r in trace])
        plan_vx = np.array([float(r["plan_base/vx"]) for r in trace])
        moving = np.array([r["test_phase"] == "walk" for r in trace])
        summary["hold_estimated_vx_mean"] = float(est_vx[moving].mean()) if moving.any() else None
        summary["hold_planned_vx_mean"] = float(plan_vx[moving].mean()) if moving.any() else None

    names = samples[0]["names"]
    if all(len(s["effort"]) == len(names) for s in samples):
        effort = np.array([s["effort"] for s in samples])
        saturation = {}
        for i, name in enumerate(names):
            if not any(k in name for k in LEG_JOINTS) or name not in joints:
                continue
            jid = model.joint(name).id
            act = np.flatnonzero(model.actuator_trnid[:, 0] == jid)
            limit = (float(np.abs(model.jnt_actfrcrange[jid]).max()) if model.jnt_actfrclimited[jid]
                     else float(np.abs(model.actuator_ctrlrange[act[0]]).max()) if len(act) else None)
            if limit:
                saturation[name] = float(np.mean(np.abs(effort[hold, i]) >= 0.95 * limit))
        worst = max(saturation, key=saturation.get) if saturation else None
        summary["leg_torque_saturation"] = dict(worst_joint=worst, worst_fraction=saturation.get(worst),
                                                per_joint={k: v for k, v in saturation.items() if v > 0})
    summary.update(mpc_health(Path(str(prefix) + ".log")))

    if plot:
        fig, axes = plt.subplots(5, 1, figsize=(12, 13), sharex=True)
        for foot, (label, colour) in enumerate((("left", "tab:blue"), ("right", "tab:orange"))):
            for in_contact, start, end, _ in bouts(contact[:, foot], times):
                if in_contact:
                    axes[0].broken_barh([(start, end - start)], (1 - foot + 0.1, 0.8), color=colour)
        if trace_path.exists():
            for foot, bit in enumerate((2, 1)):
                on = (plan_mode.astype(int) & bit) > 0
                for in_contact, start, end, _ in bouts(on, trace_t):
                    if in_contact:
                        axes[0].broken_barh([(start, end - start)], (3 - foot + 0.1, 0.8), color="gray")
        axes[0].set_yticks([0.5, 1.5, 2.5, 3.5], ["right (replay)", "left (replay)", "right (plan)", "left (plan)"])
        axes[0].set_title(f"{Path(prefix).name}: contact, planned vs replayed")
        axes[1].plot(times, forward_v, lw=0.6, label="pose-derived forward speed")
        if trace_path.exists():
            axes[1].plot(trace_t, est_vx, lw=0.6, label="estimated vx (MPC observation)")
            axes[1].plot(trace_t, plan_vx, lw=0.8, label="planned vx")
        axes[1].axhline(result.get("speed", 0), color="k", ls="--", lw=0.7, label="command")
        axes[1].set_ylabel("Speed [m/s]")
        axes[1].legend(loc="upper left", fontsize=8)
        axes[2].plot(times, np.degrees(rpy[:, 0]), label="roll")
        axes[2].plot(times, np.degrees(rpy[:, 1]), label="pitch")
        axes[2].plot(times, np.degrees(yaw - yaw[0]), label="yaw change")
        axes[2].set_ylabel("Base angle [deg]")
        axes[2].legend(loc="upper left", fontsize=8)
        axes[3].plot(times, base[:, 2], label="base z")
        axes[3].plot(times, low[:, 0], lw=0.6, label="left foot low point")
        axes[3].plot(times, low[:, 1], lw=0.6, label="right foot low point")
        axes[3].set_ylabel("Height [m]")
        axes[3].legend(loc="upper left", fontsize=8)
        for side, marker in (("left", "o"), ("right", "s")):
            chosen = by_side[side]
            axes[4].plot([s["time"] for s in chosen], [s["length"] for s in chosen], marker, label=f"step to {side}: length")
            axes[4].plot([s["time"] for s in chosen], [s["period"] for s in chosen], marker, mfc="none",
                         label=f"step to {side}: period [s]")
        axes[4].set_ylabel("Step length [m] / period [s]")
        axes[4].legend(loc="upper left", fontsize=8)
        axes[-1].set_xlabel("Time after takeover [s]")
        for axis in axes:
            axis.axvspan(h0, h1, color="green", alpha=0.05)
        fig.tight_layout()
        fig.savefig(str(prefix) + "_gait.png", dpi=120)
        plt.close(fig)
    if frames is not None:
        from PIL import Image, ImageDraw
        # One gait cycle (two steps) in 8 frames: side view on top, front view below.
        cycle = 2 * (summary["steps"]["period"]["mean"] if summary["steps"]["period"] else 0.34)
        start = (h0 + h1) / 2 if frames == "auto" else float(frames)
        chosen = [int(np.abs(times - start - k * cycle / 8).argmin()) for k in range(8)]
        size = 450
        model.vis.global_.offwidth = max(model.vis.global_.offwidth, size)
        model.vis.global_.offheight = max(model.vis.global_.offheight, size)
        sheet = Image.new("RGB", (8 * size, 2 * size))
        with mujoco.Renderer(model, height=size, width=size) as renderer:
            for column, index in enumerate(chosen):
                sample = samples[index]
                data.qpos[:3] = sample["position"]
                data.qpos[3:7] = sample["quaternion_wxyz"]
                for name, value in zip(sample["names"], sample["q"]):
                    if name in joints:
                        data.qpos[joints[name]] = value
                mujoco.mj_forward(model, data)
                heading = np.degrees(rpy[index, 2])
                for row, azimuth in enumerate((heading + 90, heading + 180)):
                    camera = mujoco.MjvCamera()
                    camera.lookat[:] = data.qpos[:3] + [0, 0, -.35]
                    camera.distance = 2.7 if robot == "g1" else 3.3
                    camera.azimuth, camera.elevation = azimuth, -8
                    renderer.update_scene(data, camera=camera)
                    frame = Image.fromarray(renderer.render())
                    contact_text = "".join(("L" if contact[index, 0] else "-", "R" if contact[index, 1] else "-"))
                    ImageDraw.Draw(frame).text((10, 10), f"t={times[index]:.2f}s v={smooth[index]:.2f} {contact_text}",
                                               fill="white")
                    sheet.paste(frame, (size * column, size * row))
        sheet.save(f"{prefix}_stride_{start:.1f}.png")
    Path(str(prefix) + "_gait.json").write_text(json.dumps(summary, indent=2) + "\n")
    return summary


def brief(s):
    st, c = s["steps"], s["contact"]
    def f(x, k="mean", fmt="{:.3f}"):
        return fmt.format(x[k]) if x else "-"
    rate = s.get("mpc_rate")
    return (f"{s['prefix']}: {s['verdict']} cmd {s['command']} | vx {s['hold_world_vx']:.3f} "
            f"(1s {f(s['hold_vx_1s_blocks'], 'min', '{:.2f}')}-{f(s['hold_vx_1s_blocks'], 'max', '{:.2f}')}) "
            f"yaw {s['hold_heading_drift']:+.3f} lat {s['hold_lateral_drift']:+.2f} | "
            f"roll rms {s['hold_roll']['rms']:.3f} max {s['hold_roll']['max']:.3f} pitch {s['hold_pitch']['mean']:+.3f}"
            f"+-{s['hold_pitch']['sd']:.3f} z {f(s['hold_base_z'])}+-{f(s['hold_base_z'], 'sd')} | "
            f"cad {st['cadence']:.2f}/s period {f(st['period'])} cv {st['period_cv'] or 0:.2f} "
            f"len {f(st['length'])} cv {st['length_cv'] or 0:.2f} L/R {f(st['left_length'])}/{f(st['right_length'])} "
            f"width {f(st['width'])} nonalt {st['non_alternating']} | flight {c['flight_fraction']:.3f} "
            f"dsupp {c['double_support_fraction']:.3f} agree {s.get('planned_vs_replayed_mode_agreement', float('nan')):.2f} | "
            f"mpc {f(rate, 'mean', '{:.1f}')}Hz min {f(rate, 'min', '{:.1f}')} sat "
            f"{(s.get('leg_torque_saturation') or {}).get('worst_joint')}:"
            f"{(s.get('leg_torque_saturation') or {}).get('worst_fraction') or 0:.3f}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("prefixes", nargs="+", type=Path)
    parser.add_argument("--robot-name", choices=["g1", "h2"], required=True)
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--frames", action="append", default=[], metavar="T|auto",
                        help="render one gait cycle from time T after takeover (repeatable; auto = mid-hold)")
    parser.add_argument("--bins", action="store_true", help="print the per-speed-bin table")
    args = parser.parse_args()
    for prefix in args.prefixes:
        summary = None
        for frame in args.frames or [None]:
            summary = analyze(prefix.resolve(), args.robot_name, not args.no_plot, frame)
        print(brief(summary))
        if args.bins:
            print("  speed     sec steps period  cv   len   cv  asym width flight dsup roll  pitch+-sd     z+-sd       yaw/s")
            for b in summary["speed_bins"]:
                fmt = lambda v, f="{:.3f}": f.format(v) if v is not None else "  -  "
                print(f"  {b['speed'][0]:.2f}-{b['speed'][1]:.2f} {b['seconds']:4.1f} {b['steps']:4d} {fmt(b['period'])} "
                      f"{fmt(b['period_cv'], '{:.2f}')} {fmt(b['length'], '{:.2f}')} {fmt(b['length_cv'], '{:.2f}')} "
                      f"{fmt(b['length_asymmetry'], '{:.2f}')} {fmt(b['width'], '{:.2f}')} {b['flight']:.3f} "
                      f"{b['double_support']:.2f} {b['roll_rms']:.3f} {b['pitch_mean']:+.3f}+-{b['pitch_sd']:.3f} "
                      f"{b['z_mean']:.3f}+-{b['z_sd']:.3f} {fmt(b['yaw_rate'], '{:+.3f}')}")
