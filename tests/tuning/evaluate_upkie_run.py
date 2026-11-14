#!/usr/bin/env python3
"""Evaluate one wb_id_walk_test run of Upkie: statistics, per-dimension cost shares and a cmd/plan/GT plot.

usage: evaluate_upkie_run.py PREFIX [--plot] [--quiet]     (pip install numpy pyyaml matplotlib)
Writes PREFIX_eval.json, and PREFIX_eval.png with --plot.
Needs PREFIX.json (harness args), PREFIX.yaml (evaluated parameters), PREFIX_tracking.csv, PREFIX_solver.csv.
The base command is rebuilt from the harness arguments (the controller then rate-limits it).
Joint pairs are mirrored (right = -left in a symmetric posture), so left + right is the asymmetry; the wheel
fore/aft gap (left minus right along the heading) is the leg-crossing metric.
Cost shares are 0.5 w e^2 of each state dimension at the measured state (and the planned wrench/torque for the
input terms), so they show which term dominates; joint accelerations are not logged and are left out.
"""
import csv, json, sys
import numpy as np
import yaml

prefix = sys.argv[1]
args = json.load(open(prefix + ".json"))
params = yaml.safe_load(open(prefix + ".yaml"))["humanoid_whole_body_inverse_dynamics_mpc_controller"]["ros__parameters"]
costs, ref = params["ocs2"]["costs"], params["ocs2"]["reference"]
qs = costs.get("qScaling", 1.0)
W = {k: np.array(v, float) * qs for k, v in costs["q"].items()}
R = {k: np.array(v, float) * costs.get("rScaling", 1.0) for k, v in costs["r"].items()}
q_default = np.array(ref["defaultJointState"], float)
joints = ["left_hip", "left_knee", "left_wheel", "right_hip", "right_knee", "right_wheel"]

rows = list(csv.DictReader(open(prefix + "_tracking.csv")))
col = lambda k: np.array([float(r[k]) if r[k] not in ("", "nan") else np.nan for r in rows])
phase = np.array([r["test_phase"] for r in rows])
t_abs = col("t")
t0 = t_abs[np.argmax(phase == "settle")] if np.any(phase == "settle") else t_abs[0]
t = t_abs - t0

# Command rebuilt from the harness profile.
S, Rp, Hd = args["settle"], args["ramp"], args["hold"]
def profile(e):
    if e < S: return 0.0
    if e < S + Rp: return (e - S) / Rp if Rp > 0 else 1.0
    if e < S + Rp + Hd: return 1.0
    if e < S + 2 * Rp + Hd: return 1 - (e - S - Rp - Hd) / Rp if Rp > 0 else 0.0
    return 0.0
frac = np.array([profile(e) for e in t])
cmd_vx = args["speed"] * frac if args["axis"] == "vx" else np.zeros_like(t)
cmd_wz = args["speed"] * frac if args["axis"] == "yaw" else np.zeros_like(t)
cmd_h = np.full_like(t, args["height"] if args.get("height") else ref["defaultBaseHeight"])
for when, value in args.get("height_sequence") or []:
    cmd_h[t >= when] = value
# Posture reference as the WB-ID target calculator builds it: with a posture table the height command is clamped to
# [minBaseHeight, maxBaseHeight] (runs before those keys existed: the table range) and the posture interpolated there.
table_h = np.array(ref.get("postureHeights") or [], float)
if table_h.size:
    cmd_h = np.clip(cmd_h, ref.get("minBaseHeight", table_h[0]), ref.get("maxBaseHeight", table_h[-1]))
    table_q = np.array(ref["postureJointStates"], float).reshape(table_h.size, -1)
    q_ref = np.stack([np.interp(cmd_h, table_h, table_q[:, i]) for i in range(table_q.shape[1])], 1)
else:
    q_ref = np.tile(q_default, (t.size, 1))

yaw = col("obs_base/yaw"); h = np.stack([np.cos(yaw), np.sin(yaw)], 1)
v_fwd = h[:, 0] * col("obs_base/vx") + h[:, 1] * col("obs_base/vy")
v_lat = -h[:, 1] * col("obs_base/vx") + h[:, 0] * col("obs_base/vy")
p_v_fwd = h[:, 0] * col("plan_base/vx") + h[:, 1] * col("plan_base/vy")
pL = np.stack([col("obs_foot/left_tire_center/x"), col("obs_foot/left_tire_center/y")], 1)
pR = np.stack([col("obs_foot/right_tire_center/x"), col("obs_foot/right_tire_center/y")], 1)
pB = np.stack([col("obs_base/x"), col("obs_base/y")], 1)
dx_wheels = np.sum((pL - pR) * h, 1)                 # left minus right wheel, along the heading
off_L, off_R = np.sum((pL - pB) * h, 1), np.sum((pR - pB) * h, 1)
mq = {j: col("meas_q/" + j) for j in joints}
pq = {j: col("plan_q/" + j) for j in joints}
hip_asym, knee_asym = mq["left_hip"] + mq["right_hip"], mq["left_knee"] + mq["right_knee"]

def band_rms(x, tt, lo=8.0, hi=45.0):
    x, tt = x[np.isfinite(x)], tt[np.isfinite(x)]
    if x.size < 64: return np.nan
    dt = np.median(np.diff(tt)); x = x - np.polyval(np.polyfit(tt, x, 2), tt)
    f = np.fft.rfftfreq(x.size, dt); X = np.fft.rfft(x); X[(f < lo) | (f > hi)] = 0
    return float(np.sqrt(np.mean(np.fft.irfft(X, x.size) ** 2)))

# Per-dimension cost at the measured state, 0.5 w e^2 as in OCS2's quadratic cost.
err = {
    "base z": col("obs_base/z") - cmd_h, "base pitch": col("obs_base/pitch"), "base roll": col("obs_base/roll"),
    "v fwd": v_fwd - cmd_vx, "v lat": v_lat, "v z": col("obs_base/vz"), "yaw rate": col("obs_base/dyaw") - cmd_wz,
    "pitch rate": col("obs_base/dpitch"), "roll rate": col("obs_base/droll"),
}
w_of = {"base z": W["basePose"][2], "base pitch": W["basePose"][4], "base roll": W["basePose"][5],
        "v fwd": W["baseVelocity"][0], "v lat": W["baseVelocity"][1], "v z": W["baseVelocity"][2],
        "yaw rate": W["baseVelocity"][3], "pitch rate": W["baseVelocity"][4], "roll rate": W["baseVelocity"][5]}
for i, j in enumerate(joints):
    err["q " + j] = mq[j] - q_ref[:, i]; w_of["q " + j] = W["jointPositions"][i]
    err["v " + j] = col("meas_v/" + j); w_of["v " + j] = W["jointVelocities"][i]
wr = R["contactWrenches"]
for i, (side, comp) in enumerate([(s, c) for s in ("left", "right") for c in ("fx", "fy", "fz", "mx", "my", "mz")]):
    err[f"R {side} {comp}"] = col(f"plan_wrench/{side}_tire_center/{comp}"); w_of[f"R {side} {comp}"] = wr[i]
for i, j in enumerate(joints):
    err["R tau " + j] = col("plan_tau/" + j); w_of["R tau " + j] = R["jointTorques"][i]

def summarize(mask, label):
    out = {"label": label, "seconds": float(np.ptp(t[mask])) if mask.any() else 0.0}
    out["max|pitch|"], out["max|roll|"] = float(np.nanmax(np.abs(col("obs_base/pitch")[mask]))), float(np.nanmax(np.abs(col("obs_base/roll")[mask])))
    out["height err mean [mm]"] = float(1000 * np.nanmean((col("obs_base/z") - cmd_h)[mask]))
    out["wheel dx L-R mean/max|.| [mm]"] = (float(1000 * np.nanmean(dx_wheels[mask])), float(1000 * np.nanmax(np.abs(dx_wheels[mask]))))
    out["hip asym mean/max|.|"] = (float(np.nanmean(hip_asym[mask])), float(np.nanmax(np.abs(hip_asym[mask]))))
    out["knee asym mean/max|.|"] = (float(np.nanmean(knee_asym[mask])), float(np.nanmax(np.abs(knee_asym[mask]))))
    out["knee L/R mean"] = (float(np.nanmean(mq["left_knee"][mask])), float(np.nanmean(mq["right_knee"][mask])))
    out["knee 8-45 Hz q rms L/R [mrad]"] = tuple(1000 * band_rms(mq[j][mask], t[mask]) for j in ("left_knee", "right_knee"))
    out["knee 8-45 Hz tau rms L/R [Nm]"] = tuple(band_rms(col("meas_tau/" + j)[mask], t[mask]) for j in ("left_knee", "right_knee"))
    c = {k: float(np.nanmean(0.5 * w_of[k] * e[mask] ** 2)) for k, e in err.items()}
    total = sum(c.values()) or 1.0
    out["cost share"] = {k: round(v / total, 3) for k, v in sorted(c.items(), key=lambda kv: -kv[1])[:8]}
    return out

walk = phase == "walk"
late = walk & (t > (t[walk][0] + 0.4 * np.ptp(t[walk]))) if walk.any() else walk
res = {"prefix": prefix.split("/")[-1], "verdict": args.get("verdict"), "sim_seconds": args.get("sim_seconds")}
if late.any():
    if args["axis"] == "vx":
        res["vx tracking"] = float(np.nanmean(v_fwd[late]) / args["speed"])
    if args["axis"] == "yaw":
        res["yaw tracking"] = float(np.nanmean(col("obs_base/dyaw")[late]) / args["speed"])
        res["yaw plan/cmd"] = float(np.nanmean(col("plan_base/dyaw")[late]) / args["speed"])
res["phases"] = [summarize(phase == p, p) for p in ("settle", "walk", "rest") if np.any(phase == p)]
solver = list(csv.DictReader(open(prefix + "_solver.csv")))
terms = [k for k in solver[0] if k.startswith(("cost/", "state_cost/", "soft/", "state_soft/"))]
tot = {k: np.nanmean([float(r[k]) for r in solver if r["test_phase"] == "walk" and r[k] not in ("", "nan")] or [np.nan]) for k in terms}
s = sum(v for v in tot.values() if np.isfinite(v)) or 1.0
res["solver cost share (walk, whole horizon)"] = {k: round(v / s, 3) for k, v in sorted(tot.items(), key=lambda kv: -kv[1]) if np.isfinite(v)}

if "--quiet" not in sys.argv:
    print(json.dumps(res, indent=1, default=float))
json.dump(res, open(prefix + "_eval.json", "w"), indent=1, default=float)

if "--plot" in sys.argv:
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(8, 1, figsize=(13, 22), sharex=True)
    ax[0].plot(t, col("obs_base/z"), label="GT"); ax[0].plot(t, col("plan_base/z"), "--", label="plan"); ax[0].plot(t, cmd_h, ":", label="cmd"); ax[0].set_ylabel("base z [m]")
    for k, ls in (("pitch", "-"), ("roll", "-")):
        ax[1].plot(t, col("obs_base/" + k), ls, label=k + " GT"); ax[1].plot(t, col("plan_base/" + k), "--", label=k + " plan")
    ax[1].set_ylabel("rad")
    ax[2].plot(t, v_fwd, label="GT"); ax[2].plot(t, p_v_fwd, "--", label="plan"); ax[2].plot(t, cmd_vx, ":", label="cmd"); ax[2].set_ylabel("v fwd [m/s]")
    ax[3].plot(t, col("obs_base/dyaw"), label="GT"); ax[3].plot(t, col("plan_base/dyaw"), "--", label="plan"); ax[3].plot(t, cmd_wz, ":", label="cmd"); ax[3].set_ylabel("yaw rate [rad/s]")
    for a, jl, jr, i in ((ax[4], "left_hip", "right_hip", 0), (ax[5], "left_knee", "right_knee", 1)):
        a.plot(t, mq[jl], label="L GT"); a.plot(t, -mq[jr], label="-R GT"); a.plot(t, pq[jl], "--", label="L plan"); a.plot(t, -pq[jr], "--", label="-R plan")
        a.plot(t, q_ref[:, i], color="k", lw=0.8, ls=":", label="posture ref"); a.set_ylabel(jl.split("_")[1] + " [rad]")
    ax[6].plot(t, 1000 * dx_wheels, label="wheel dx L-R [mm]"); ax[6].plot(t, 1000 * off_L, label="left wheel ahead of base [mm]"); ax[6].plot(t, 1000 * off_R, label="right wheel ahead of base [mm]"); ax[6].set_ylabel("mm")
    ax[7].plot(t, col("meas_tau/left_knee"), label="L GT"); ax[7].plot(t, -col("meas_tau/right_knee"), label="-R GT")
    ax[7].plot(t, col("cmd_tau/left_knee"), "--", label="L ff"); ax[7].plot(t, -col("cmd_tau/right_knee"), "--", label="-R ff"); ax[7].set_ylabel("knee tau [N m]")
    for a in ax:
        a.legend(ncol=5, fontsize=7, loc="upper left"); a.grid(alpha=0.3)
        for i in range(1, len(phase)):
            if phase[i] != phase[i - 1]: a.axvline(t[i], color="grey", lw=0.6, ls=":")
    ax[-1].set_xlabel("time since settle [s]"); ax[0].set_title(res["prefix"] + f"  verdict {res['verdict']}")
    fig.tight_layout(); fig.savefig(prefix + "_eval.png", dpi=70); print("wrote", prefix + "_eval.png")
