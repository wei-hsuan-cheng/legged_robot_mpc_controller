#!/usr/bin/env python3
"""Plot and score MPC plan vs joint command vs measurement from a claude_tune_probe *_data.npz.

usage: plot_tracking.py <prefix> [title]
Writes <prefix>_{joints,base,spectra,ff_fb,solver}.png and <prefix>_diag.json and prints
a DIAG line. The question each number answers: is the problem already in what the MPC plans and
commands (MPC level), or does the measurement depart from a good command (joint servo level)?
"""
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

P = sys.argv[1]
TITLE = sys.argv[2] if len(sys.argv) > 2 else P.rstrip("/").split("/")[-1]
# A case folder from run_case.sh (data.npz -> joints.png ...) or a file prefix (<p>_data.npz -> <p>_joints.png ...).
OUT = (lambda name: os.path.join(P, name)) if os.path.isdir(P) else (lambda name: P + "_" + name)
d = np.load(OUT("data.npz"), allow_pickle=False)
# ---------------- SUMMARY: ground truth + joint states at full rate ----------------
def band_t(tt, x, lo=4.0, hi=40.0, fs=500.0):
    """RMS of x restricted to [lo, hi] Hz after resampling (tt must be sampled well above 2*hi)."""
    if len(tt) < 100:
        return float("nan")
    tu = np.arange(tt[0], tt[-1], 1.0 / fs)
    xu = np.interp(tu, tt, x)
    xu = xu - np.polyval(np.polyfit(tu - tu[0], xu, 1), tu - tu[0])
    f = np.fft.rfftfreq(len(xu), 1.0 / fs)
    X = np.fft.rfft(xu)
    X[(f < lo) | (f > hi)] = 0
    return float(np.sqrt(np.mean(np.fft.irfft(X, len(xu)) ** 2)))


SETTLE, RAMP, HOLD, STOP, VX = d["phases"]
gt = d["gt"]
fell_wall = float(d["fell"]) if "fell" in d.files else float("nan")
w2s = lambda w: float(np.interp(w, gt[:, 1], gt[:, 0]))
w0, w1 = w2s(SETTLE + RAMP + 3.0), w2s(SETTLE + RAMP + HOLD)
# Joint signals: /joint_states header stamps only advance with the 100 Hz sim clock, so use the
# measured joints of ~/tracking (exact control-tick times, trackingRate) when it is there.
TN = list(d["track_names"]) if "track_names" in d.files else []
if TN and d["track"].size:
    TR = d["track"]
    jn = [n.split("/", 1)[1] for n in TN if n.startswith("meas_q/")]
    jt = TR[:, TN.index("t")]
    JQ = np.column_stack([TR[:, TN.index("meas_q/" + n)] for n in jn])
    JV = np.column_stack([TR[:, TN.index("meas_v/" + n)] for n in jn])
    JE = np.column_stack([TR[:, TN.index("meas_tau/" + n)] for n in jn])
else:
    jt, jn = d["js_t"], list(d["js_names"])
    JQ, JV, JE = d["js_q"], d["js_v"], d["js_e"]
o = np.argsort(jt); jt, JQ, JV, JE = jt[o], JQ[o], JV[o], JE[o]
jt, u = np.unique(jt, return_index=True); JQ, JV, JE = JQ[u], JV[u], JE[u]
G_ARMS = ["left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_elbow_joint",
          "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_elbow_joint"]
G_LEGS = [n for n in jn if any(k in n for k in ("hip", "knee", "ankle"))]
metrics = {"survived": bool(np.isnan(fell_wall)), "fell_at": None if np.isnan(fell_wall) else round(fell_wall - SETTLE, 1),
           "rates_hz": {"joints": round(len(jt) / (jt[-1] - jt[0]), 1), "ground_truth": round(len(np.unique(gt[:, 0])) / (gt[-1, 0] - gt[0, 0]), 1)}}
if np.isnan(fell_wall) or fell_wall > SETTLE + RAMP + HOLD:
    g = gt[(gt[:, 0] >= w0) & (gt[:, 0] <= w1)]
    m = (jt >= w0) & (jt <= w1)
    tt = g[:, 0]
    W = {"vx_mean": float((g[-1, 2] - g[0, 2]) / (tt[-1] - tt[0])),
         "vy_mean": float((g[-1, 3] - g[0, 3]) / (tt[-1] - tt[0])),
         "yaw_rate_mean": float(np.unwrap(g[:, 10])[-1] - np.unwrap(g[:, 10])[0]) / float(tt[-1] - tt[0]),
         "z_sd_mm": 1000 * float(np.std(g[:, 4] - np.polyval(np.polyfit(tt - tt[0], g[:, 4], 1), tt - tt[0]))),
         "vz_rms": float(np.sqrt(np.mean(g[:, 7] ** 2))),
         "roll_rms_deg": math.degrees(float(np.std(g[:, 8]))), "pitch_rms_deg": math.degrees(float(np.std(g[:, 9]))),
         "max_tilt_deg": math.degrees(float(np.max(np.hypot(g[:, 8], g[:, 9])))),
         "yaw_change_deg": math.degrees(float(g[-1, 10] - g[0, 10])),
         "pelvis_acc_hf": band_t(tt[1:], np.diff(g[:, 7]) / np.maximum(np.diff(tt), 1e-6)),
         "arm_vel_hf": float(np.mean([band_t(jt[m], JV[m, jn.index(n)]) for n in G_ARMS])),
         "arm_tau_hf": float(np.mean([band_t(jt[m], JE[m, jn.index(n)]) for n in G_ARMS])),
         "leg_vel_hf": float(np.mean([band_t(jt[m], JV[m, jn.index(n)]) for n in G_LEGS])),
         "leg_tau_hf": float(np.mean([band_t(jt[m], JE[m, jn.index(n)]) for n in G_LEGS])),
         "arm_swing_pp_deg": math.degrees(float(np.mean([np.ptp(JQ[m, jn.index(n)]) for n in ("left_shoulder_pitch_joint", "right_shoulder_pitch_joint")])))}
    metrics["walk"] = {k: round(v, 4) for k, v in W.items()}
json.dump(metrics, open(OUT("metrics.json"), "w"), indent=1)
Wm = metrics.get("walk", {})
nan = float("nan")
PROF = str(d["profile"]) if "profile" in d.files else "vx"
metrics["profile"] = PROF
print(f"SUMMARY {TITLE} [{PROF}] vy {Wm.get('vy_mean', nan):.3f} yaw_rate {Wm.get('yaw_rate_mean', nan):.3f} | survived={metrics['survived']} fell_at={metrics['fell_at']} | rates joints {metrics['rates_hz']['joints']} gt {metrics['rates_hz']['ground_truth']} Hz"
      f" | vx {Wm.get('vx_mean', nan):.3f} z_sd {Wm.get('z_sd_mm', nan):.1f}mm vz_rms {Wm.get('vz_rms', nan):.3f} roll {Wm.get('roll_rms_deg', nan):.2f} pitch {Wm.get('pitch_rms_deg', nan):.2f}"
      f" tilt_max {Wm.get('max_tilt_deg', nan):.1f} acc_hf {Wm.get('pelvis_acc_hf', nan):.2f} | arm vel_hf {Wm.get('arm_vel_hf', nan):.3f} tau_hf {Wm.get('arm_tau_hf', nan):.2f}"
      f" swing {Wm.get('arm_swing_pp_deg', nan):.1f} | leg vel_hf {Wm.get('leg_vel_hf', nan):.3f} tau_hf {Wm.get('leg_tau_hf', nan):.2f} | yaw {Wm.get('yaw_change_deg', nan):.1f}")

names = list(d["track_names"])
T = d["track"]
if T.size == 0 or not names:
    print("DIAG no tracking data")
    sys.exit(0)
col = {n: i for i, n in enumerate(names)}
c = lambda n: T[:, col[n]]
t = c("t")
loco = c("phase") == np.max(c("phase")[c("policy_valid") > 0]) if np.any(c("policy_valid") > 0) else np.zeros(len(t), bool)
# Walking window in sim time (w0, w1 above), restricted to ticks with a valid MPC policy.
walk = (t >= w0) & (t <= w1) & (c("policy_valid") > 0)
if walk.sum() < 200:
    walk = c("policy_valid") > 0
J = [n.split("/", 1)[1] for n in names if n.startswith("cmd_q/")]
LEGS = [j for j in J if any(k in j for k in ("hip", "knee", "ankle"))]
ARMS = [j for j in J if any(k in j for k in ("shoulder", "elbow"))]
WAIST = [j for j in J if "waist" in j]
FEET = sorted({n.split("/")[1] for n in names if n.startswith("plan_foot/")})


def resample(x, mask, fs=500.0):
    tt = t[mask]
    tu = np.arange(tt[0], tt[-1], 1.0 / fs)
    return tu, np.interp(tu, tt, x[mask])


def band(x, mask, lo=4.0, hi=40.0, fs=500.0):
    tu, xu = resample(x, mask, fs)
    xu = xu - np.polyval(np.polyfit(tu - tu[0], xu, 1), tu - tu[0])
    f = np.fft.rfftfreq(len(xu), 1 / fs)
    X = np.fft.rfft(xu)
    X[(f < lo) | (f > hi)] = 0
    return float(np.sqrt(np.mean(np.fft.irfft(X, len(xu)) ** 2)))


def psd(x, mask, fs=500.0, seg=1024):
    tu, xu = resample(x, mask, fs)
    xu = xu - xu.mean()
    n = min(seg, len(xu))
    win = np.hanning(n)
    segs = [xu[i:i + n] * win for i in range(0, len(xu) - n + 1, n // 2)]
    S = np.mean([np.abs(np.fft.rfft(s)) ** 2 for s in segs], axis=0) / (fs * (win ** 2).sum())
    return np.fft.rfftfreq(n, 1 / fs), S


diag = {"title": TITLE, "samples_walk": int(walk.sum())}
for gname, group in (("legs", LEGS), ("waist", WAIST), ("arms", ARMS)):
    e_q = np.array([c("meas_q/" + j)[walk] - c("cmd_q/" + j)[walk] for j in group])
    e_v = np.array([c("meas_v/" + j)[walk] - c("cmd_qd/" + j)[walk] for j in group])
    pd = {j: c("meas_tau/" + j) - c("cmd_tau/" + j) for j in group}
    diag[gname] = {
        "track_err_q_deg": round(math.degrees(float(np.sqrt(np.mean(e_q ** 2)))), 3),
        "track_err_v": round(float(np.sqrt(np.mean(e_v ** 2))), 4),
        # HF (4-40 Hz) content: in the MPC's own command vs in what the servo adds.
        "cmd_q_hf_deg": round(math.degrees(np.mean([band(c("cmd_q/" + j), walk) for j in group])), 4),
        "meas_q_hf_deg": round(math.degrees(np.mean([band(c("meas_q/" + j), walk) for j in group])), 4),
        "cmd_tau_hf": round(float(np.mean([band(c("cmd_tau/" + j), walk) for j in group])), 3),
        "pd_tau_hf": round(float(np.mean([band(pd[j], walk) for j in group])), 3),
        "pd_tau_rms": round(float(np.mean([np.sqrt(np.mean(pd[j][walk] ** 2)) for j in group])), 3),
        "cmd_tau_rms": round(float(np.mean([np.sqrt(np.mean(c("cmd_tau/" + j)[walk] ** 2)) for j in group])), 3),
        # Peaks the averages hide (touchdown impacts): 99.9th percentile and max of |PD torque|.
        "pd_tau_p999": round(float(np.max([np.percentile(np.abs(pd[j][walk]), 99.9) for j in group])), 2),
        "pd_tau_max": {j.replace("_joint", ""): round(float(np.max(np.abs(pd[j][walk]))), 1)
                       for j in sorted(group, key=lambda j: -np.max(np.abs(pd[j][walk])))[:3]},
    }
detr = lambda x, m: x[m] - np.polyval(np.polyfit(t[m] - t[m][0], x[m], 1), t[m] - t[m][0])
diag["base"] = {
    "plan_z_sd_mm": round(1000 * float(np.std(detr(c("plan_base/z"), walk))), 2),
    "obs_z_sd_mm": round(1000 * float(np.std(detr(c("obs_base/z"), walk))), 2),
    "plan_vz_rms": round(float(np.sqrt(np.mean(c("plan_base/vz")[walk] ** 2))), 4),
    "obs_vz_rms": round(float(np.sqrt(np.mean(c("obs_base/vz")[walk] ** 2))), 4),
    "z_track_err_mm": round(1000 * float(np.sqrt(np.mean((c("obs_base/z") - c("plan_base/z"))[walk] ** 2))), 2),
    "roll_track_err_deg": round(math.degrees(float(np.sqrt(np.mean((c("obs_base/roll") - c("plan_base/roll"))[walk] ** 2)))), 3),
    "pitch_track_err_deg": round(math.degrees(float(np.sqrt(np.mean((c("obs_base/pitch") - c("plan_base/pitch"))[walk] ** 2)))), 3),
}
for f in FEET:
    diag["foot_" + f] = {"z_track_err_mm": round(1000 * float(np.sqrt(np.mean((c(f"obs_foot/{f}/z") - c(f"plan_foot/{f}/z"))[walk] ** 2))), 2),
                         "xy_track_err_mm": round(1000 * float(np.sqrt(np.mean((c(f"obs_foot/{f}/x") - c(f"plan_foot/{f}/x"))[walk] ** 2
                                                                                + (c(f"obs_foot/{f}/y") - c(f"plan_foot/{f}/y"))[walk] ** 2))), 2)}
# Command jumps at policy switches: the MPC replans each update; a jump there is plan inconsistency.
t0 = c("policy_t0")
switch = np.zeros(len(t), bool)
switch[1:] = np.diff(t0) != 0
switch &= walk
dtau = {j: np.abs(np.diff(c("cmd_tau/" + j), prepend=np.nan)) for j in J}
for gname, group in (("legs", LEGS), ("arms", ARMS)):
    at = np.nanmean([dtau[j][switch] for j in group])
    off = np.nanmean([dtau[j][walk & ~switch] for j in group])
    diag[gname]["cmd_tau_jump_at_switch"] = round(float(at), 3)
    diag[gname]["cmd_tau_step_otherwise"] = round(float(off), 3)
diag["policy_switch_hz"] = round(float(switch.sum() / max(1e-6, t[walk][-1] - t[walk][0])), 1)
json.dump(diag, open(OUT("diag.json"), "w"), indent=1)

# ---------------- figures ----------------
mid = t[walk][0] + 0.5 * (t[walk][-1] - t[walk][0])
zoom = walk & (t >= mid) & (t <= mid + 2.5)
show = ["left_hip_pitch_joint", "left_hip_roll_joint", "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_elbow_joint"]
fig, ax = plt.subplots(len(show), 2, figsize=(15, 2.0 * len(show)), sharex=True)
for i, j in enumerate(show):
    ax[i, 0].plot(t[zoom], np.degrees(c("plan_q/" + j)[zoom]), lw=1.2, label="MPC plan q")
    ax[i, 0].plot(t[zoom], np.degrees(c("meas_q/" + j)[zoom]), lw=1.0, label="measured q")
    ax[i, 0].set_ylabel(j.replace("_joint", "").replace("left_", "L ") + " [deg]", fontsize=8)
    ax[i, 1].plot(t[zoom], c("plan_tau/" + j)[zoom], lw=1.0, label="MPC tau")
    ax[i, 1].plot(t[zoom], c("cmd_tau/" + j)[zoom], lw=1.0, ls="--", label="tau_ff sent")
    ax[i, 1].plot(t[zoom], c("meas_tau/" + j)[zoom], lw=0.8, label="applied (ff + PD)")
    ax[i, 1].set_ylabel("[Nm]", fontsize=8)
ax[0, 0].legend(fontsize=7, loc="upper right"); ax[0, 1].legend(fontsize=7, loc="upper right")
ax[-1, 0].set_xlabel("sim time [s]"); ax[-1, 1].set_xlabel("sim time [s]")
fig.suptitle(f"{TITLE}: joint position (left) and torque (right), 2.5 s of walking")
fig.tight_layout(); fig.savefig(OUT("joints.png"), dpi=80); plt.close(fig)

fig, ax = plt.subplots(5, 1, figsize=(14, 12), sharex=True)
g = gt[(gt[:, 0] >= t[zoom][0]) & (gt[:, 0] <= t[zoom][-1])]
ax[0].plot(t[zoom], c("plan_base/z")[zoom], label="MPC plan"); ax[0].plot(t[zoom], c("obs_base/z")[zoom], label="estimate"); ax[0].plot(g[:, 0], g[:, 4], lw=0.8, label="ground truth")
ax[0].set_ylabel("pelvis z [m]"); ax[0].legend(fontsize=8)
ax[1].plot(t[zoom], c("plan_base/vz")[zoom], label="MPC plan"); ax[1].plot(t[zoom], c("obs_base/vz")[zoom], label="estimate"); ax[1].set_ylabel("pelvis vz [m/s]"); ax[1].legend(fontsize=8)
for a in ("roll", "pitch"):
    ax[2].plot(t[zoom], np.degrees(c(f"plan_base/{a}")[zoom]), label=f"plan {a}"); ax[2].plot(t[zoom], np.degrees(c(f"obs_base/{a}")[zoom]), ls="--", label=f"est {a}")
ax[2].set_ylabel("[deg]"); ax[2].legend(fontsize=8, ncol=4)
for f in FEET:
    ax[3].plot(t[zoom], c(f"plan_foot/{f}/z")[zoom], label=f"plan {f}"); ax[3].plot(t[zoom], c(f"obs_foot/{f}/z")[zoom], ls="--", label=f"est {f}")
    ax[4].plot(t[zoom], c(f"plan_wrench/{f}/fz")[zoom], label=f"plan fz {f}")
ax[3].set_ylabel("foot z [m]"); ax[3].legend(fontsize=8, ncol=4); ax[4].set_ylabel("planned fz [N]"); ax[4].legend(fontsize=8)
ax[4].set_xlabel("sim time [s]")
fig.suptitle(f"{TITLE}: base and feet, MPC plan vs estimate"); fig.tight_layout(); fig.savefig(OUT("base.png"), dpi=80); plt.close(fig)

fig, ax = plt.subplots(2, 3, figsize=(16, 8))
for k, (gname, group) in enumerate((("legs", LEGS), ("arms", ARMS), ("waist", WAIST))):
    S = lambda key: np.mean([psd(c(key + "/" + j), walk)[1] for j in group], axis=0)
    f = psd(c("cmd_tau/" + group[0]), walk)[0]
    pdS = np.mean([psd(c("meas_tau/" + j) - c("cmd_tau/" + j), walk)[1] for j in group], axis=0)
    ax[0, k].semilogy(f, S("cmd_tau"), label="tau_ff (MPC)"); ax[0, k].semilogy(f, pdS, label="PD torque (applied - ff)")
    ax[0, k].set_title(f"{gname}: torque PSD"); ax[0, k].set_xlim(0, 60); ax[0, k].legend(fontsize=8)
    ax[1, k].semilogy(f, S("cmd_q"), label="q command (MPC)"); ax[1, k].semilogy(f, S("meas_q"), label="q measured")
    ax[1, k].set_title(f"{gname}: position PSD"); ax[1, k].set_xlim(0, 60); ax[1, k].set_xlabel("Hz"); ax[1, k].legend(fontsize=8)
fig.suptitle(f"{TITLE}: where the high-frequency content is (walking window)"); fig.tight_layout(); fig.savefig(OUT("spectra.png"), dpi=80); plt.close(fig)

# ---- feedforward (MPC tau_ff) vs feedback (servo PD = applied - tau_ff) ----
fig, ax = plt.subplots(5, 1, figsize=(14, 13), sharex=False)
groups = (("legs", LEGS), ("waist", WAIST), ("arms", ARMS))
labels = [g for g, _ in groups]
ff_rms = [diag[g]["cmd_tau_rms"] for g in labels]; fb_rms = [diag[g]["pd_tau_rms"] for g in labels]
ff_hf = [diag[g]["cmd_tau_hf"] for g in labels]; fb_hf = [diag[g]["pd_tau_hf"] for g in labels]
xx = np.arange(len(labels))
ax[0].bar(xx - 0.3, ff_rms, 0.2, label="FF rms"); ax[0].bar(xx - 0.1, fb_rms, 0.2, label="FB rms")
ax[0].bar(xx + 0.1, ff_hf, 0.2, label="FF 4-40 Hz"); ax[0].bar(xx + 0.3, fb_hf, 0.2, label="FB 4-40 Hz")
ax[0].set_xticks(xx); ax[0].set_xticklabels(labels); ax[0].set_ylabel("[Nm], mean over joints"); ax[0].legend(fontsize=8, ncol=4)
for i, j in enumerate(["left_hip_roll_joint", "left_knee_joint", "left_ankle_roll_joint", "left_shoulder_pitch_joint"], start=1):
    ax[i].plot(t[zoom], c("cmd_tau/" + j)[zoom], label="FF (MPC tau_ff)")
    ax[i].plot(t[zoom], (c("meas_tau/" + j) - c("cmd_tau/" + j))[zoom], label="FB (servo PD)")
    ax[i].set_ylabel(j.replace("_joint", "").replace("left_", "L ") + " [Nm]", fontsize=8); ax[i].legend(fontsize=7, loc="upper right")
ax[-1].set_xlabel("sim time [s]")
fig.suptitle(f"{TITLE}: feedforward vs feedback torque"); fig.tight_layout(); fig.savefig(OUT("ff_fb.png"), dpi=80); plt.close(fig)

# ---- solver metrics (per solve) ----
if "solver" in d.files and d["solver"].size and len(d["solver_names"]):
    sn = list(d["solver_names"]); S = d["solver"]; sc = {n: i for i, n in enumerate(sn)}
    st = S[:, sc["t"]]
    sw = (st >= w0) & (st <= w1)
    if sw.sum() > 20:
        adv = S[sw, sc["advance_ms"]]
        diag["solver"] = {"solves": int(sw.sum()), "solve_hz": round(float(sw.sum() / (st[sw][-1] - st[sw][0])), 1),
                          "advance_ms_mean": round(float(adv.mean()), 2), "advance_ms_p95": round(float(np.percentile(adv, 95)), 2),
                          "advance_ms_max": round(float(adv.max()), 2), "eq_sse_mean": float(np.nanmean(S[sw, sc["eq_sse"]])),
                          "dyn_sse_max": float(np.nanmax(S[sw, sc["dyn_sse"]])), "cost_mean": round(float(np.nanmean(S[sw, sc["cost"]])), 3)}
        cost_cols = [n for n in sn if "/" in n and n.split("/")[0] in ("cost", "state_cost", "soft", "state_soft")]
        # Raw means, largest magnitude first. Soft constraints use relaxed log barriers, which are negative
        # well inside the feasible set, so a "share of the total" would be meaningless.
        means = {n: float(np.nanmean(S[sw, sc[n]])) for n in cost_cols}
        diag["solver"]["cost_terms_mean"] = {n: round(v, 3) for n, v in sorted(means.items(), key=lambda kv: -abs(kv[1]))[:12]}
        if "eq_sse_near" in sc:
            diag["solver"]["eq_sse_near_mean"] = float(np.nanmean(S[sw, sc["eq_sse_near"]]))
            near = [n for n in sn if n.startswith("eq_sse_near/")]
            diag["solver"]["eq_sse_near_by_term_mean"] = {n[12:]: float(np.nanmean(S[sw, sc[n]])) for n in sorted(near, key=lambda n: -np.nanmean(S[sw, sc[n]]))[:4]}
        eqc = [n for n in sn if n.startswith("eq_sse/")]
        diag["solver"]["eq_sse_by_term_mean"] = {n[7:]: float(np.nanmean(S[sw, sc[n]])) for n in sorted(eqc, key=lambda n: -np.nanmean(S[sw, sc[n]]))[:6]}
        fig, ax = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
        ax[0].plot(st[sw], adv, lw=0.8); ax[0].axhline(10.0, color="r", ls="--", lw=0.8, label="10 ms (100 Hz)")
        ax[0].set_ylabel("solve time [ms]"); ax[0].legend(fontsize=8)
        ax[1].semilogy(st[sw], S[sw, sc["eq_sse"]], label="equality SSE"); ax[1].semilogy(st[sw], S[sw, sc["dyn_sse"]] + 1e-30, label="dynamics SSE")
        for n in sorted(eqc, key=lambda n: -np.nanmean(S[sw, sc[n]]))[:4]:
            ax[1].semilogy(st[sw], S[sw, sc[n]] + 1e-30, lw=0.6, label=n[7:])
        if "eq_sse_near" in sc:
            ax[1].semilogy(st[sw], S[sw, sc["eq_sse_near"]] + 1e-30, color="k", lw=1.0, label="equality SSE, first 3 nodes")
        ax[1].set_ylabel("constraint violation"); ax[1].legend(fontsize=7, ncol=3)
        ax[2].plot(st[sw], S[sw, sc["cost"]], label="total cost (horizon)"); ax[2].set_ylabel("cost"); ax[2].legend(fontsize=8)
        for n in list(diag["solver"]["cost_terms_mean"])[:8]:
            ax[3].plot(st[sw], S[sw, sc[n]], lw=0.8, label=n.split("/", 1)[1][:40])
        ax[3].set_ylabel("cost terms at node 0"); ax[3].set_xlabel("sim time [s]"); ax[3].legend(fontsize=7, ncol=2)
        fig.suptitle(f"{TITLE}: solver"); fig.tight_layout(); fig.savefig(OUT("solver.png"), dpi=80); plt.close(fig)
json.dump(diag, open(OUT("diag.json"), "w"), indent=1)

L, A, B = diag["legs"], diag["arms"], diag["base"]
print(f"DIAG {TITLE} | base z sd plan {B['plan_z_sd_mm']} / obs {B['obs_z_sd_mm']} mm, vz rms plan {B['plan_vz_rms']} / obs {B['obs_vz_rms']}"
      f" | legs PD peak p99.9 {L['pd_tau_p999']} Nm max {list(L['pd_tau_max'].items())[0]}"
      f" | legs q err {L['track_err_q_deg']}deg, q_hf cmd {L['cmd_q_hf_deg']} meas {L['meas_q_hf_deg']}, tau_hf ff {L['cmd_tau_hf']} pd {L['pd_tau_hf']}, jump@switch {L['cmd_tau_jump_at_switch']} vs {L['cmd_tau_step_otherwise']}"
      f" | arms q err {A['track_err_q_deg']}deg, q_hf cmd {A['cmd_q_hf_deg']} meas {A['meas_q_hf_deg']}, tau_hf ff {A['cmd_tau_hf']} pd {A['pd_tau_hf']}, jump@switch {A['cmd_tau_jump_at_switch']} vs {A['cmd_tau_step_otherwise']}"
      f" | switches {diag['policy_switch_hz']} Hz"
      + (f" | solve {diag['solver']['advance_ms_mean']} ms mean, {diag['solver']['advance_ms_p95']} p95, {diag['solver']['solve_hz']} Hz, eq_sse {diag['solver']['eq_sse_mean']:.1e}"
         f" (first 3 nodes {diag['solver'].get('eq_sse_near_mean', float('nan')):.1e})" if "solver" in diag else ""))
