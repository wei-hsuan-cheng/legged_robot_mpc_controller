#!/usr/bin/env python3
"""Contact detection and hidden filter height from estimator_durability_test.py internals.

Reads <prefix>_internals.csv (estimator diagnostics plus the simulator's per-foot contact ground
truth) and reports, over the hold phase, per foot:
  - how the stance flags the filter used, and the torque detector's own decision (which runs every
    tick, also when the filter uses another contact source), agree with the true contact:
    agreement, false stance, missed stance, and touchdown / liftoff latency;
  - the torque-estimated normal force against the true one, in stance and in swing;
and the filter's hidden height drift. With the simulator's pelvis height in the log it is measured
against that truth, with the vertical velocity error and the split of the height drift into
stance, flight and the transitions between them; otherwise against the stance-foot kinematic
height. Writes <prefix>_contact.png and <prefix>_contact.json.
"""

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

FEET = ("left", "right")


def load(prefix):
    with open(str(prefix) + "_internals.csv") as file:
        rows = list(csv.reader(file))
    header, rows = rows[0], rows[1:]
    phase = np.array([r[0] for r in rows])
    data = np.array([[float(x) for x in r[1:]] for r in rows])
    columns = {}
    for i, name in enumerate(header[1:]):
        # Drop the controller prefix of estimator interfaces: humanoid_inekf/diagnostics.x -> diagnostics.x
        key = name.split("/", 1)[1] if name.split("/", 1)[0].startswith("humanoid_") else name
        columns[key] = data[:, i]
    return phase, columns


def edges(flag, t, rising):
    change = np.diff(flag.astype(int))
    return t[1:][change == (1 if rising else -1)]


def latency(truth_events, detected_events, window=0.15):
    """Signed delay of the nearest detected event to each true event (s), within the window."""
    if len(detected_events) == 0:
        return np.array([])
    out = []
    for event in truth_events:
        nearest = detected_events[np.argmin(np.abs(detected_events - event))]
        if abs(nearest - event) <= window:
            out.append(nearest - event)
    return np.array(out)


def flag_stats(truth, flag, t):
    lat_td = latency(edges(truth, t, True), edges(flag, t, True))
    lat_lo = latency(edges(truth, t, False), edges(flag, t, False))
    stat = lambda x: dict(median_ms=float(1e3 * np.median(x)), p95_abs_ms=float(1e3 * np.percentile(np.abs(x), 95)),
                          matched=int(len(x))) if len(x) else dict(matched=0)
    return dict(agreement=float(np.mean(truth == flag)),
                false_stance=float(np.mean(flag & ~truth)), missed_stance=float(np.mean(~flag & truth)),
                touchdowns=int(len(edges(truth, t, True))), detected_touchdowns=int(len(edges(flag, t, True))),
                touchdown_latency=stat(lat_td), liftoff_latency=stat(lat_lo))


def analyze(prefix, plot=True, window=4.0):
    phase, d = load(prefix)
    hold = phase == "hold"
    if hold.sum() < 50:
        hold = np.isin(phase, ("hold", "ramp_in", "ramp_out", "settle", "rest"))
    t = d["t"]
    summary = dict(prefix=Path(prefix).name, samples=int(hold.sum()), rate_hz=float(1 / np.median(np.diff(t))))
    for foot in FEET:
        truth_key = f"{foot}_foot_contact/contact"
        if truth_key not in d:
            continue
        truth = d[truth_key][hold] > 0.5
        used = d[f"diagnostics.foot_stance.{foot}"][hold] > 0.5
        torque = d[f"diagnostics.foot_torque_stance.{foot}"][hold] > 0.5
        f_true = d[f"{foot}_foot_contact/normal_force"][hold]
        f_est = d[f"diagnostics.foot_normal_force.{foot}"][hold]
        rms = lambda m: float(np.sqrt(np.mean((f_est[m] - f_true[m]) ** 2))) if m.any() else None
        summary[foot] = dict(used=flag_stats(truth, used, t[hold]), torque_detector=flag_stats(truth, torque, t[hold]),
                             force_rms_stance=rms(truth), force_rms_swing=rms(~truth),
                             force_bias_stance=float(np.mean(f_est[truth] - f_true[truth])) if truth.any() else None,
                             swing_force_p95=float(np.percentile(f_est[~truth], 95)) if (~truth).any() else None,
                             stance_force_p05=float(np.percentile(f_est[truth], 5)) if truth.any() else None,
                             true_stance_fraction=float(truth.mean()))
    truth_z = "pelvis/position.z" in d
    if truth_z:
        # Against the truth: the kinematic height is itself off at heel strike and toe-off.
        valid = hold.copy()
        drift = d["diagnostics.inekf_height"] - d["pelvis/position.z"]
        true_vz = np.gradient(d["pelvis/position.z"], t)
        vz_err = d["linear_velocity_world.z"] - true_vz
        summary["vz_error_rms"] = float(np.sqrt(np.mean(vz_err[hold] ** 2)))
        summary["vz_error_mean"] = float(np.mean(vz_err[hold]))
        summary["reported_height_error_p95"] = float(np.percentile(
            np.abs(d["diagnostics.reported_height"] - d["pelvis/position.z"])[hold], 95))
        used = (d["diagnostics.foot_stance.left"] > 0.5) | (d["diagnostics.foot_stance.right"] > 0.5)
        idx = np.flatnonzero(hold)
        segments = np.split(idx, np.flatnonzero(np.diff(used[idx].astype(int))) + 1)
        split = dict(stance=0.0, flight=0.0, transitions=0.0)
        for k, seg in enumerate(segments):
            split["stance" if used[seg[0]] else "flight"] += float(drift[seg[-1]] - drift[seg[0]])
            if k:
                split["transitions"] += float(drift[seg[0]] - drift[segments[k - 1][-1]])
        summary["hidden_height_split_m"] = split
    else:
        valid = hold & (d["diagnostics.kinematic_height_valid"] > 0.5)
        drift = d["diagnostics.inekf_height"] - d["diagnostics.kinematic_height"]
    if valid.any():
        tv, dv = t[valid], drift[valid]
        minutes = np.floor((tv - tv[0]) / 60).astype(int)
        per_minute = [float(np.median(dv[minutes == m])) for m in range(minutes.max() + 1) if (minutes == m).sum() > 10]
        summary["hidden_height_drift_m"] = dict(first=per_minute[0], last=per_minute[-1], per_minute=per_minute,
                                                rate_m_per_min=float(np.polyfit(tv - tv[0], dv, 1)[0] * 60))
    Path(str(prefix) + "_contact.json").write_text(json.dumps(summary, indent=2) + "\n")

    if plot and "left_foot_contact/contact" in d:
        h = np.flatnonzero(hold)
        mid = t[h[len(h) // 2]]
        m = hold & (t >= mid) & (t < mid + window)
        fig, axes = plt.subplots(4, 1, figsize=(12, 11))
        for k, foot in enumerate(FEET):
            a = axes[k]
            a.plot(t[m] - mid, d[f"{foot}_foot_contact/normal_force"][m], "k", lw=1.5, label="true normal force")
            a.plot(t[m] - mid, d[f"diagnostics.foot_normal_force.{foot}"][m], "C0", label="torque-estimated normal force")
            top = max(1.0, np.nanmax(d[f"{foot}_foot_contact/normal_force"][m]))
            a.fill_between(t[m] - mid, 0, top * d[f"{foot}_foot_contact/contact"][m], color="0.85", label="true contact")
            a.plot(t[m] - mid, 1.05 * top * d[f"diagnostics.foot_torque_stance.{foot}"][m], "C3", lw=1, label="torque detector stance")
            a.plot(t[m] - mid, 1.10 * top * d[f"diagnostics.foot_stance.{foot}"][m], "C2", lw=1, label="stance used by the filter")
            a.set_ylabel(f"{foot} foot [N]")
            a.legend(fontsize=7, loc="upper right", ncol=2)
        if valid.any():
            axes[2].plot((t[valid] - t[valid][0]) / 60, 100 * drift[valid], ",", alpha=0.3)
            axes[2].set_ylabel(("filter - true" if truth_z else "filter - kinematic") + " height [cm]")
            axes[2].set_xlabel("hold time [min]")
        a = axes[3]
        for k, foot in enumerate(FEET):
            truth = d[f"{foot}_foot_contact/contact"][hold] > 0.5
            f_est = d[f"diagnostics.foot_normal_force.{foot}"][hold]
            a.hist(f_est[truth], bins=80, alpha=0.5, label=f"{foot}: true stance")
            a.hist(f_est[~truth], bins=80, alpha=0.5, label=f"{foot}: true swing")
        a.set_xlabel("torque-estimated normal force [N]")
        a.set_yscale("log")
        a.legend(fontsize=7)
        for a in axes:
            a.grid(alpha=0.3)
        fig.suptitle(f"{Path(prefix).name}: contact detection and hidden height")
        fig.tight_layout()
        fig.savefig(str(prefix) + "_contact.png", dpi=105)
        plt.close(fig)
    return summary


def line(s):
    parts = [s["prefix"]]
    for foot in FEET:
        if foot in s:
            f = s[foot]
            td = f["torque_detector"]
            parts.append(f"{foot}: used agree {f['used']['agreement']:.3f} | torque agree {td['agreement']:.3f} "
                         f"false {td['false_stance']:.3f} missed {td['missed_stance']:.3f} "
                         f"td {td['touchdown_latency'].get('median_ms', float('nan')):+.0f}ms "
                         f"lo {td['liftoff_latency'].get('median_ms', float('nan')):+.0f}ms "
                         f"F rms st {f['force_rms_stance'] or float('nan'):.0f} sw {f['force_rms_swing'] or float('nan'):.0f} N "
                         f"(swing p95 {f['swing_force_p95'] or float('nan'):.0f}, stance p05 {f['stance_force_p05'] or float('nan'):.0f})")
    if "hidden_height_drift_m" in s:
        h = s["hidden_height_drift_m"]
        parts.append(f"hidden height {100*h['first']:+.1f} -> {100*h['last']:+.1f} cm ({100*h['rate_m_per_min']:+.1f} cm/min)")
    if "hidden_height_split_m" in s:
        sp = s["hidden_height_split_m"]
        parts.append(f"split stance {100*sp['stance']:+.1f} flight {100*sp['flight']:+.1f} transitions {100*sp['transitions']:+.1f} cm"
                     f" | vz err rms {100*s['vz_error_rms']:.1f} mean {100*s['vz_error_mean']:+.2f} cm/s"
                     f" | reported height p95 {1000*s['reported_height_error_p95']:.1f} mm")
    return " | ".join(parts)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("prefixes", nargs="+", type=Path)
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()
    for prefix in args.prefixes:
        print(line(analyze(prefix, not args.no_plot)))
