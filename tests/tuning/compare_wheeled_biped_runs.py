"""Side-by-side drift, pitch and push metrics for Upkie runs (PREFIX_tracking.csv from wb_id_walk_test.py --record-tracking).

    python3 compare_wheeled_biped_runs.py /tmp/run_a /tmp/run_b ...
"""
import csv, json, sys, numpy as np
from pathlib import Path
runs = sys.argv[1:]
def load(prefix):
    rows = list(csv.DictReader(open(prefix + "_tracking.csv")))
    col = lambda name: np.array([float(r[name]) if r[name] not in ("", "nan") else np.nan for r in rows])
    return rows, col
print(f"{'run':28s} {'verdict':12s} {'settle v':>9s} {'settle dx':>9s} {'rest v':>8s} {'rest dx':>8s} {'walk v':>7s} {'|pitch|max':>10s} {'wheelff':>8s} {'solve ms':>8s}")
for prefix in runs:
    verdict = json.loads(Path(prefix + ".json").read_text()).get("verdict") if Path(prefix + ".json").exists() else "?"
    if not Path(prefix + "_tracking.csv").exists():
        print(f"{Path(prefix).name:28s} {verdict:12s} (no tracking)"); continue
    rows, col = load(prefix)
    phase = np.array([r["test_phase"] for r in rows]); t = col("t"); yaw = col("obs_base/yaw")
    vf = np.cos(yaw) * col("obs_base/vx") + np.sin(yaw) * col("obs_base/vy")
    x = col("obs_base/x"); y = col("obs_base/y")
    def stats(ph):
        m = phase == ph
        if m.sum() < 10: return np.nan, np.nan
        sel = m & (t > t[m][0] + 2.0)  # skip transients
        if sel.sum() < 10: sel = m
        dx = np.hypot(x[sel][-1] - x[sel][0], y[sel][-1] - y[sel][0])
        return np.nanmean(vf[sel]), dx
    sv, sdx = stats("settle"); rv, rdx = stats("rest")
    wv = np.nanmean(vf[phase == "walk"]) if (phase == "walk").any() else np.nan
    pmax = np.nanmax(np.abs(col("obs_base/pitch")[phase != "startup"]))
    wff = np.nanmean(np.abs(col("cmd_tau/left_wheel")[phase == "settle"]))
    sp = Path(prefix + "_solver.csv")
    solve = np.nan
    if sp.exists():
        srows = list(csv.DictReader(open(sp)))
        solve = np.mean([float(r["advance_ms"]) for r in srows if r["advance_ms"] not in ("", "nan")])
    print(f"{Path(prefix).name:28s} {verdict:12s} {sv:+9.4f} {sdx:9.3f} {rv:+8.4f} {rdx:8.3f} {wv:7.3f} {pmax:10.3f} {wff:8.3f} {solve:8.2f}")
