#!/usr/bin/env python3
"""Analyse the Phase-0 diagnostics CSVs written by the centroidal MPC controller.

Usage:
    python3 analyze_diagnostics.py /tmp/centroidal_mpc_diag_20260822_143000
    python3 analyze_diagnostics.py <prefix> --plot          # also write PNGs
    python3 analyze_diagnostics.py <prefix> --phase 2       # steady state only

The prefix is the path WITHOUT the '_state.csv' / '_cost.csv' suffix, i.e. exactly
what was passed as diagnosticsLogPrefix (with %t already expanded - the resolved
path is printed in the controller's activation log line).

Depends only on numpy (and matplotlib for --plot), both of which are already in
the ROS container; deliberately no pandas, which is not.

What this reports, and why each number is the one to look at:

  Filter consistency (NIS).  Closeness to ground truth does not tell you whether
  a filter is trustworthy; a filter can track well while believing its own error
  is far smaller than it really is. NIS = r' S^-1 r compares the innovation
  actually seen against the covariance the filter predicted for it, and for a
  consistent filter it is chi-squared with `dim` degrees of freedom. NIS/dof well
  below 1 means the measurement noise is overstated (good data being ignored);
  well above 1 means the filter is over-confident. Four heel/toe landmarks
  treated as independent when they are rigidly linked is precisely a mechanism
  for the second.

  Height architecture.  The kinematic blend is applied OUTSIDE the filter, so
  height_inekf keeps drifting uncorrected while height_reported looks fine - and
  the touchdown anchors are computed from height_inekf, so its drift compounds
  into them. The drift rate of (height_inekf - height_reported) over a long run
  is the number that decides whether this has to become an in-filter pseudo-
  measurement.

  Momentum injection.  The centroidal state is normalized momentum, so estimator
  error enters the dominant state directly. hbar_err is the momentum difference
  produced by feeding the estimate instead of ground truth through the SAME
  A_G(q) with the same joint state, which isolates the estimator's own
  contribution from any model difference.

  Cost balance.  A summed cost says nothing about which term decides the posture
  the solver picks. The ranked per-term table does, and the leg-torque vs base-z
  comparison is the specific question behind the untracked height command.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

try:
    import numpy as np
except ImportError:  # pragma: no cover
    sys.exit("needs numpy: pip3 install numpy")


# --------------------------------------------------------------------------
# loading
# --------------------------------------------------------------------------

class Table:
    """Column-oriented float view of a CSV. Empty fields become NaN."""

    def __init__(self, columns: list[str], data: "np.ndarray"):
        self.columns = columns
        self._index = {name: i for i, name in enumerate(columns)}
        self.data = data  # shape (rows, len(columns))

    @classmethod
    def read(cls, path: Path) -> "Table":
        with path.open(newline="") as fh:
            reader = csv.reader(fh)
            try:
                header = next(reader)
            except StopIteration:
                return cls([], np.zeros((0, 0)))
            rows = []
            width = len(header)
            for line in reader:
                if not line:
                    continue
                # A short final line is a partially flushed row from a run that
                # was killed rather than deactivated; pad it instead of failing.
                if len(line) < width:
                    line = line + [""] * (width - len(line))
                elif len(line) > width:
                    line = line[:width]
                rows.append([float(v) if v not in ("", "nan") else np.nan for v in line])
        data = np.asarray(rows, dtype=float) if rows else np.zeros((0, width))
        return cls(header, data)

    def __len__(self) -> int:
        return self.data.shape[0]

    def __contains__(self, name: str) -> bool:
        return name in self._index

    def has(self, *names: str) -> bool:
        return all(n in self._index for n in names)

    def col(self, name: str) -> "np.ndarray":
        return self.data[:, self._index[name]]

    def select(self, mask: "np.ndarray") -> "Table":
        return Table(self.columns, self.data[mask])

    @property
    def empty(self) -> bool:
        return self.data.shape[0] == 0


# --------------------------------------------------------------------------
# statistics
# --------------------------------------------------------------------------

def chi2_bounds(dof: int) -> tuple[float, float]:
    """Two-sided 95% interval for chi2(dof), Wilson-Hilferty approximation.

    Avoids a scipy dependency; accurate to well under a percent for dof >= 3,
    which is the only regime here (3 per corrected contact).
    """
    if dof <= 0:
        return (float("nan"), float("nan"))
    z = 1.959963984540054
    a = 2.0 / (9.0 * dof)
    lo = dof * (1.0 - a - z * np.sqrt(a)) ** 3
    hi = dof * (1.0 - a + z * np.sqrt(a)) ** 3
    return (float(max(lo, 0.0)), float(hi))


def finite(values: "np.ndarray") -> "np.ndarray":
    return values[np.isfinite(values)]


def rms(values: "np.ndarray") -> float:
    v = finite(values)
    return float(np.sqrt(np.mean(v**2))) if v.size else float("nan")


def peak_abs(values: "np.ndarray") -> float:
    v = finite(values)
    return float(np.max(np.abs(v))) if v.size else float("nan")


def nanmean(values: "np.ndarray") -> float:
    v = finite(values)
    return float(np.mean(v)) if v.size else float("nan")


def nanstd(values: "np.ndarray") -> float:
    v = finite(values)
    return float(np.std(v)) if v.size else float("nan")


def drift_rate(time: "np.ndarray", values: "np.ndarray") -> float:
    """Least-squares slope [unit/s] - the honest way to state a drift."""
    mask = np.isfinite(time) & np.isfinite(values)
    if mask.sum() < 10:
        return float("nan")
    return float(np.polyfit(time[mask], values[mask], 1)[0])


def last_finite(values: "np.ndarray") -> float:
    v = finite(values)
    return float(v[-1]) if v.size else float("nan")


def section(title: str) -> None:
    print()
    print("=" * 78)
    print(title)
    print("=" * 78)


# --------------------------------------------------------------------------
# analyses
# --------------------------------------------------------------------------

PHASE_NAMES = {
    0: "0 pre-init (estimator not seeded)",
    1: "1 warm-up  (ground truth drives control)",
    2: "2 closed   (the estimate drives control)",
}


def describe_phases(state: Table) -> None:
    section("RUN SEGMENTS")
    t = state.col("t")
    phase = state.col("phase")
    present = sorted({int(p) for p in finite(phase)})
    for p in present:
        mask = phase == p
        label = PHASE_NAMES.get(p, f"{p} unknown")
        print(f"  {label:<42} {t[mask].min():8.2f} -> {t[mask].max():8.2f} s "
              f"({int(mask.sum())} samples)")
    if 2 not in present:
        print("  NOTE: the estimate never drove control in this run (no phase-2")
        print("  samples). Either floatingBase.source is not state_estimator, or the")
        print("  run ended inside the warm-up window.")


def analyse_consistency(state: Table) -> None:
    section("FILTER CONSISTENCY (the question closeness-to-GT cannot answer)")
    if not state.has("corr_applied", "corr_dim", "nis"):
        print("  correction columns absent")
        return

    applied_mask = state.col("corr_applied") > 0.5
    if not applied_mask.any():
        print("  No corrections fired in this segment. Either no contact was ever")
        print("  indicated, or the contact bookkeeping never augmented a landmark.")
        return

    sub = state.select(applied_mask)
    print(f"  corrections applied : {len(sub)} / {len(state)} samples "
          f"({100.0 * len(sub) / len(state):.1f}%)")

    dims = sub.col("corr_dim")
    for dof in sorted({int(d) for d in finite(dims)}):
        nis = finite(sub.col("nis")[dims == dof])
        if nis.size == 0:
            continue
        lo, hi = chi2_bounds(dof)
        above = float(np.mean(nis > hi))
        below = float(np.mean(nis < lo))
        print(f"\n  dim={dof:2d} ({dof // 3} contacts corrected), {nis.size} samples")
        print(f"    chi2 95% band      : [{lo:8.2f}, {hi:8.2f}]")
        print(f"    NIS mean / median  : {nis.mean():8.2f} / {np.median(nis):8.2f}"
              f"   (consistent filter: ~{dof})")
        print(f"    NIS/dof mean       : {nis.mean() / dof:8.2f}"
              f"   (consistent filter: ~1.0)")
        print(f"    above band         : {100 * above:6.1f}%   "
              f"(>5% => OVER-CONFIDENT: P too small / measurements double-counted)")
        print(f"    below band         : {100 * below:6.1f}%   "
              f"(>5% => noise OVERSTATED: good data being ignored)")

    print("\n  Reading it: heel and toe of one foot are rigidly linked but are fed to")
    print("  the filter as two independent stationary landmarks. That over-counts the")
    print("  information per foot and shows up here as NIS above the band. If it does,")
    print("  the fix is the contact model (one landmark per foot, or an inflated N),")
    print("  not a finer sweep of contactPosition.")


def analyse_covariance(state: Table) -> None:
    section("COVARIANCE AND BIAS")
    blocks = [("attitude  [rad]", "P_att_std"),
              ("velocity  [m/s]", "P_vel_std"),
              ("position  [m]", "P_pos_std"),
              ("gyro bias [rad/s]", "P_bg_std"),
              ("acc bias  [m/s^2]", "P_ba_std")]
    print(f"  {'block':<18} {'mean sigma (x, y, z)':<40} final z")
    for label, prefix in blocks:
        cols = [f"{prefix}_{a}" for a in "xyz"]
        if not state.has(*cols):
            continue
        means = [nanmean(state.col(c)) for c in cols]
        print(f"  {label:<18} ({means[0]:.3e}, {means[1]:.3e}, {means[2]:.3e})"
              f"    {last_finite(state.col(cols[2])):.3e}")

    if "estimating_bias" in state:
        enabled = last_finite(state.col("estimating_bias")) > 0.5
        print(f"\n  bias estimation enabled : {enabled}")
    for label, prefix in (("gyro bias  [rad/s]", "bias_gyro"),
                          ("accel bias [m/s^2]", "bias_accel")):
        cols = [f"{prefix}_{a}" for a in "xyz"]
        if not state.has(*cols):
            continue
        final = [last_finite(state.col(c)) for c in cols]
        spread = [float(np.ptp(finite(state.col(c))) if finite(state.col(c)).size else np.nan)
                  for c in cols]
        print(f"  {label} final=({final[0]:+.3e}, {final[1]:+.3e}, {final[2]:+.3e})"
              f"  range=({spread[0]:.2e}, {spread[1]:.2e}, {spread[2]:.2e})")
        if max((s for s in spread if np.isfinite(s)), default=0.0) == 0.0:
            print("    ^ range is exactly zero: this bias state is NOT being estimated.")
            print("      The measurement Jacobian has no bias columns, so the bias can")
            print("      only be observed through its cross-covariance with the")
            print("      attitude/velocity/position block.")


def analyse_accuracy(state: Table) -> None:
    section("ESTIMATE vs GROUND TRUTH")
    groups = [("position   [m]", "err_p"),
              ("orientation[rad]", "err_rpy"),
              ("lin vel W  [m/s]", "err_v_world"),
              ("ang vel B  [rad/s]", "err_w_local")]
    print(f"  {'signal':<20} {'RMS (x, y, z)':<40} peak |.|")
    for label, prefix in groups:
        cols = [f"{prefix}_{a}" for a in "xyz"]
        if not state.has(*cols):
            continue
        r = [rms(state.col(c)) for c in cols]
        p = max(peak_abs(state.col(c)) for c in cols)
        print(f"  {label:<20} ({r[0]:.4f}, {r[1]:.4f}, {r[2]:.4f})"
              f"{'':<13} {p:.4f}")

    axes = ("lin_x", "lin_y", "lin_z", "ang_x", "ang_y", "ang_z")
    mom_cols = [f"hbar_err_{a}" for a in axes]
    if state.has(*mom_cols) and finite(state.col(mom_cols[0])).size:
        print("\n  Normalized momentum injected by using the estimate instead of GT")
        print("  (same A_G(q), same joint state - the estimator's own contribution to")
        print("  the dominant MPC state):")
        for col in mom_cols:
            print(f"    {col:<18} RMS={rms(state.col(col)):.5f}   "
                  f"peak={peak_abs(state.col(col)):.5f}")
    else:
        print("\n  (momentum columns are empty: the joint state or the GT body was")
        print("   unavailable on the logged ticks)")


def analyse_height(state: Table) -> None:
    section("HEIGHT CONDITIONING (the blend happens OUTSIDE the filter)")
    if not state.has("height_inekf", "height_reported", "t"):
        print("  height columns absent")
        return

    t = state.col("t")
    inekf = state.col("height_inekf")
    reported = state.col("height_reported")
    gap = inekf - reported
    rate = drift_rate(t, gap)
    duration = float(np.nanmax(t) - np.nanmin(t))

    print(f"  run duration                : {duration:.1f} s")
    print(f"  reported height  mean/std   : {nanmean(reported):.4f} / {nanstd(reported):.4f} m")
    print(f"  filter height    mean/std   : {nanmean(inekf):.4f} / {nanstd(inekf):.4f} m")
    print(f"  (filter - reported) drift   : {rate:+.5f} m/s  => {rate * 60:+.3f} m/min")
    print(f"  (filter - reported) final   : {last_finite(gap):+.4f} m")

    anchors = [c for c in state.columns if c.endswith("_ground_anchor")]
    if anchors:
        print("\n  Touchdown anchors (computed FROM the filter height, so filter drift")
        print("  compounds into them):")
        for col in anchors:
            print(f"    {col:<36} final={last_finite(state.col(col)):+.4f} m  "
                  f"drift={drift_rate(t, state.col(col)):+.5f} m/s")

    if duration < 40 and np.isfinite(rate) and abs(rate) > 1e-4:
        print(f"\n  NOTE: this run is only {duration:.0f} s long. At {rate:+.5f} m/s the")
        print(f"  gap would reach {abs(rate) * 60:.3f} m after a minute. Run 60 s+ before")
        print("  drawing a conclusion about drift.")

    if state.has("gt_p_z"):
        err = reported - state.col("gt_p_z")
        print(f"\n  reported height error vs GT : RMS={rms(err):.4f} m  "
              f"peak={peak_abs(err):.4f} m")


def analyse_contacts(state: Table, contacts: list[str]) -> None:
    section("CONTACT BOOKKEEPING")
    if "num_augmented_contacts" in state:
        values = finite(state.col("num_augmented_contacts"))
        print("  landmarks augmented in the filter state:")
        for n in sorted({int(v) for v in values}):
            share = 100.0 * float(np.mean(values == n))
            print(f"    {n} landmarks : {share:5.1f}% of samples")

    print(f"\n  {'contact':<20} {'stance %':>9} {'added':>7} {'removed':>8} "
          f"{'innov RMS [m]':>14} {'|innov|/sigma':>14}")
    for name in contacts:
        stance_col = f"{name}_in_stance"
        if stance_col not in state:
            continue
        stance = state.col(stance_col)
        added = int(np.nansum(state.col(f"{name}_added"))) if f"{name}_added" in state else 0
        removed = int(np.nansum(state.col(f"{name}_removed"))) if f"{name}_removed" in state else 0

        innov_cols = [f"{name}_innov_{a}" for a in "xyz"]
        std_cols = [f"{name}_innov_std_{a}" for a in "xyz"]
        innov_rms = ratio_mean = float("nan")
        if state.has(f"{name}_corrected", *innov_cols, *std_cols):
            mask = state.col(f"{name}_corrected") > 0.5
            if mask.any():
                sub = state.select(mask)
                innov = np.column_stack([sub.col(c) for c in innov_cols])
                sigma = np.column_stack([sub.col(c) for c in std_cols])
                innov_rms = rms(innov.ravel())
                with np.errstate(divide="ignore", invalid="ignore"):
                    ratio = np.abs(innov) / sigma
                ratio_mean = nanmean(ratio.ravel())
        print(f"  {name:<20} {100 * nanmean(stance):8.1f}% {added:7d} {removed:8d} "
              f"{innov_rms:14.5f} {ratio_mean:14.2f}")

    print("\n  |innov|/sigma is the per-axis normalized innovation. Persistently > 1")
    print("  means the filter's predicted innovation covariance is too small for the")
    print("  motion the foot actually undergoes - e.g. a flat foot rolling through")
    print("  heel-strike and toe-off, which a position-only correction can only")
    print("  absorb into the base pose and velocity.")

    landmark_cols = [c for c in state.columns if c.endswith("_landmark_z")]
    if landmark_cols:
        print("\n  Landmark height spread within the run (the 'foot stretch' the filter")
        print("  has to absorb somewhere):")
        for col in landmark_cols:
            values = finite(state.col(col))
            values = values[values != 0.0]
            if values.size == 0:
                continue
            print(f"    {col:<36} std={values.std():.5f} m  "
                  f"range={np.ptp(values):.5f} m")


def analyse_signal_quality(state: Table) -> None:
    """Is the data itself trustworthy before any conclusion is drawn from it?

    Three separate failure modes, which look alike in a summary statistic but
    have completely different causes: non-finite values (a divergence or an
    uninitialised read), step discontinuities (a torn cross-thread read, or a
    filter reset), and high-frequency content (genuine measurement noise).
    """
    section("SIGNAL QUALITY")

    groups = [
        ("est position", [f"est_p_{a}" for a in "xyz"], 0.05),
        ("est orientation", [f"est_rpy_{a}" for a in "xyz"], 0.10),
        ("est lin vel", [f"est_v_world_{a}" for a in "xyz"], 0.50),
        ("est ang vel", [f"est_w_local_{a}" for a in "xyz"], 0.50),
        ("gt position", [f"gt_p_{a}" for a in "xyz"], 0.05),
        ("gt lin vel", [f"gt_v_world_{a}" for a in "xyz"], 0.50),
        ("momentum est", [f"hbar_est_{a}" for a in
                          ("lin_x", "lin_y", "lin_z", "ang_x", "ang_y", "ang_z")], 0.50),
    ]

    print(f"  {'signal':<16} {'non-finite':>11} {'max |step|':>11} {'jumps':>7} "
          f"{'noise (HF rms)':>15}")
    for label, cols, jump_threshold in groups:
        cols = [c for c in cols if c in state]
        if not cols:
            continue
        non_finite = 0
        max_step = 0.0
        jumps = 0
        noise = 0.0
        for col in cols:
            values = state.col(col)
            non_finite += int(np.sum(~np.isfinite(values)))
            good = values[np.isfinite(values)]
            if good.size < 3:
                continue
            steps = np.abs(np.diff(good))
            max_step = max(max_step, float(steps.max()))
            jumps += int(np.sum(steps > jump_threshold))
            # Second difference isolates content at the sampling frequency from
            # the smooth motion underneath it.
            noise = max(noise, float(np.sqrt(np.mean(np.diff(good, 2) ** 2)) / np.sqrt(6.0)))
        flag = ""
        if non_finite:
            flag = "  <-- NON-FINITE VALUES"
        elif jumps:
            flag = "  <-- discontinuities"
        print(f"  {label:<16} {non_finite:>11d} {max_step:>11.4f} {jumps:>7d} "
              f"{noise:>15.5f}{flag}")

    print("\n  'jumps' counts consecutive-sample steps above a per-signal threshold.")
    print("  A handful during a fall is expected. A steady trickle while the robot is")
    print("  upright is not, and would point at a torn read rather than at noise.")


def find_fall(state: Table) -> float | None:
    """Time of the first sign of loss of balance, or None.

    Uses ground truth, not the estimate: the estimate is exactly what is in
    question once things go wrong. A fall is called on the pelvis dropping well
    below its commanded range or the trunk tipping past a recoverable angle.
    """
    if not state.has("t", "gt_p_z"):
        return None
    t = state.col("t")
    height = state.col("gt_p_z")
    bad = np.isfinite(height) & (height < 0.55)
    if state.has("gt_rpy_y", "gt_rpy_z"):
        pitch = np.abs(state.col("gt_rpy_y"))
        roll = np.abs(state.col("gt_rpy_x")) if "gt_rpy_x" in state else np.zeros_like(pitch)
        bad = bad | (np.isfinite(pitch) & (pitch > 0.5)) | (np.isfinite(roll) & (roll > 0.5))
    if not bad.any():
        return None
    return float(t[np.argmax(bad)])


def analyse_timeline(state: Table, window: float) -> None:
    section(f"TIMELINE ({window:.0f}s windows)")
    t = state.col("t")
    t0 = float(np.nanmin(t))
    rel = t - t0

    def col_or_nan(name: str) -> "np.ndarray":
        return state.col(name) if name in state else np.full_like(t, np.nan)

    err_p = np.sqrt(sum(col_or_nan(f"err_p_{a}") ** 2 for a in "xyz"))
    err_v = np.sqrt(sum(col_or_nan(f"err_v_world_{a}") ** 2 for a in "xyz"))

    print(f"  {'t [s]':>7} {'gt_z':>7} {'rep_z':>7} {'|e_p|max':>9} {'|e_v|max':>9} "
          f"{'pitch':>7} {'roll':>7} {'NIS/dof':>8} {'#lm':>5} {'phase':>6}")
    edges = np.arange(0.0, float(np.nanmax(rel)) + window, window)
    for lo in edges:
        mask = (rel >= lo) & (rel < lo + window)
        if not mask.any():
            continue
        def mx(values):
            v = values[mask]
            v = v[np.isfinite(v)]
            return float(np.max(np.abs(v))) if v.size else float("nan")
        def mn(values):
            v = values[mask]
            v = v[np.isfinite(v)]
            return float(np.mean(v)) if v.size else float("nan")
        print(f"  {lo:>7.0f} {mn(col_or_nan('gt_p_z')):7.3f} "
              f"{mn(col_or_nan('height_reported')):7.3f} "
              f"{mx(err_p):9.3f} {mx(err_v):9.3f} "
              f"{mx(col_or_nan('gt_rpy_y')):7.3f} {mx(col_or_nan('gt_rpy_x')):7.3f} "
              f"{mn(col_or_nan('nis_per_dof')):8.2f} "
              f"{mn(col_or_nan('num_augmented_contacts')):5.1f} "
              f"{mn(col_or_nan('phase')):6.1f}")


def analyse_fall(state: Table, fall_time: float, lead: float) -> None:
    section(f"FALL ONSET at t = {fall_time:.2f} s  (showing the {lead:.0f}s before)")
    t = state.col("t")
    mask = (t >= fall_time - lead) & (t <= fall_time + 2.0)
    if not mask.any():
        print("  no samples around the onset")
        return
    sub = state.select(mask)
    st = sub.col("t")

    # Sample sparsely enough to read, densely enough to see a single touchdown.
    step = max(1, len(sub) // 60)
    def col(name):
        return sub.col(name) if name in sub else np.full(len(sub), np.nan)

    print(f"  {'t':>8} {'gt_z':>7} {'est_z':>7} {'gt_pitch':>9} {'est_pitch':>10} "
          f"{'|e_v|':>7} {'NIS/dof':>8} {'#lm':>4} {'cL':>3} {'cR':>3}")
    err_v = np.sqrt(sum(col(f"err_v_world_{a}") ** 2 for a in "xyz"))
    for i in range(0, len(sub), step):
        print(f"  {st[i] - fall_time:>8.3f} {col('gt_p_z')[i]:7.3f} "
              f"{col('est_p_z')[i]:7.3f} {col('gt_rpy_y')[i]:9.4f} "
              f"{col('est_rpy_y')[i]:10.4f} {err_v[i]:7.3f} "
              f"{col('nis_per_dof')[i]:8.2f} {col('num_augmented_contacts')[i]:4.0f} "
              f"{col('contact_left')[i]:3.0f} {col('contact_right')[i]:3.0f}")

    print("\n  Times are relative to the onset. What to look for: does the estimate")
    print("  diverge from ground truth BEFORE the pelvis drops (estimator drove the")
    print("  fall), or only after (the controller lost balance and the estimator")
    print("  merely followed)? That distinction decides whether the next change")
    print("  belongs in the filter or in the controller.")


def analyse_joints(state: Table, top: int = 12) -> None:
    """Per-joint jitter, and whether the command or the tracking is responsible.

    "The arm jitters" has two causes that are indistinguishable in the measured
    angle alone: the MPC is commanding a jittery reference and the joint is
    following it faithfully, or the reference is smooth and the low-level PD is
    ringing around it. Logging both lets them be separated:

      cmd jitter  high-frequency content of the commanded angle
      meas jitter high-frequency content of the measured angle
      ratio       meas / cmd. Near 1 means the joint is tracking a jittery
                  command (fix the reference). Much greater than 1 means the
                  command is smooth and the joint is ringing (fix the PD gains,
                  or the model the feedforward torque comes from).
    """
    joints = [c[2:] for c in state.columns if c.startswith("q_")]
    if not joints:
        print()
        print("  (no joint columns in this log - it predates joint-level logging)")
        return

    section("JOINT JITTER")

    def hf(name: str) -> float:
        """RMS of the second difference: content at the sampling rate, with the
        smooth motion underneath differenced away."""
        if name not in state:
            return float("nan")
        values = finite(state.col(name))
        if values.size < 8:
            return float("nan")
        return float(np.sqrt(np.mean(np.diff(values, 2) ** 2)) / np.sqrt(6.0))

    def amplitude(name: str) -> tuple[float, float]:
        """Peak-to-peak and std of the joint angle about its own mean.

        Per-sample jitter and visible motion are different things: 1e-4 rad of
        sample-to-sample content is invisible, while a 5 degree oscillation at a
        few Hz is exactly what "the arm is jittering" usually means. The
        high-frequency metric alone cannot see the second, so report both.
        """
        if name not in state:
            return (float("nan"), float("nan"))
        values = finite(state.col(name))
        if values.size < 8:
            return (float("nan"), float("nan"))
        return (float(np.ptp(values)), float(np.std(values)))

    rows = []
    for joint in joints:
        meas = hf(f"q_{joint}")
        cmd = hf(f"qcmd_{joint}")
        tau = f"tau_{joint}"
        tau_rms = rms(state.col(tau)) if tau in state else float("nan")
        span, sd = amplitude(f"q_{joint}")
        cmd_span, _ = amplitude(f"qcmd_{joint}")
        ratio = meas / cmd if (np.isfinite(cmd) and cmd > 1e-12) else float("nan")
        rows.append((joint, meas, cmd, ratio, tau_rms, span, sd, cmd_span))

    # Rank by visible motion, not by per-sample content: that is what the
    # complaint is about.
    rows.sort(key=lambda r: (r[5] if np.isfinite(r[5]) else -1.0), reverse=True)

    print(f"  {'joint':<26} {'p-p [deg]':>10} {'cmd p-p':>9} {'std [deg]':>10} "
          f"{'meas jit':>10} {'cmd jit':>10} {'ratio':>6} {'tau RMS':>8}")
    for joint, meas, cmd, ratio, tau_rms, span, sd, cmd_span in rows[:top]:
        print(f"  {joint[:26]:<26} {np.degrees(span):10.3f} "
              f"{np.degrees(cmd_span):9.3f} {np.degrees(sd):10.4f} "
              f"{meas:10.2e} {cmd:10.2e} {ratio:6.2f} {tau_rms:8.3f}")

    arms = [r for r in rows if any(k in r[0] for k in
                                   ("shoulder", "elbow", "wrist"))]
    legs = [r for r in rows if any(k in r[0] for k in
                                   ("hip", "knee", "ankle"))]
    for label, group in (("arm", arms), ("leg", legs)):
        values = [r[1] for r in group if np.isfinite(r[1])]
        commands = [r[2] for r in group if np.isfinite(r[2])]
        if values:
            print(f"\n  {label} joints: mean measured jitter {np.mean(values):.3e}, "
                  f"mean commanded jitter "
                  f"{np.mean(commands) if commands else float('nan'):.3e}")

    print("\n  Jitter units are rad (or Nm for tau) of per-sample high-frequency")
    print("  content. Compare arms against legs in the same run: the legs carry the")
    print("  load and see contact impacts, so some content there is physical. Arms")
    print("  in free space have no such excuse.")


def analyse_lateral(state: Table, contacts: list[str], fall_time: float | None) -> None:
    """Step-by-step lateral balance, which is what a roll divergence is about.

    The single most useful discriminator is whether roll grows smoothly or in
    once-per-step jumps. Smooth growth points at a continuously acting feedback
    deficiency; step-locked growth points at foot placement, because that is the
    only thing that changes discretely at touchdown. They call for different
    fixes, and the aggregate roll trace cannot tell them apart.

    Step positions are taken from the InEKF's contact landmarks: a landmark is
    created where the foot touched down and held through stance, so its value at
    each touchdown is that step's foothold.
    """
    if not state.has("t", "gt_rpy_x"):
        return
    section("LATERAL BALANCE, STEP BY STEP")

    t = state.col("t")
    roll = state.col("gt_rpy_x")
    base_y = state.col("gt_p_y") if "gt_p_y" in state else np.full_like(t, np.nan)

    # Touchdown events: a rising edge of the per-contact stance flag.
    events = []
    for name in contacts:
        flag_col = f"{name}_in_stance"
        if flag_col not in state:
            continue
        flag = state.col(flag_col) > 0.5
        rising = np.flatnonzero((~flag[:-1]) & flag[1:]) + 1
        for i in rising:
            events.append((float(t[i]), name, int(i)))
    if not events:
        print("  no touchdown transitions in this log (the robot never broke contact)")
        return
    events.sort()

    print(f"  {'step':>5} {'t [s]':>8} {'contact':<20} {'roll [deg]':>11} "
          f"{'d roll':>8} {'foot y':>9} {'base y':>9} {'y offset':>9}")
    previous_roll = None
    shown = 0
    for index, (event_time, name, row) in enumerate(events):
        r = float(roll[row])
        delta = (r - previous_roll) if previous_roll is not None else float("nan")
        previous_roll = r
        foot_y_col = f"{name}_landmark_y"
        foot_y = float(state.col(foot_y_col)[row]) if foot_y_col in state else float("nan")
        by = float(base_y[row])
        # How far the base sits laterally from the foot that just landed. A
        # healthy gait keeps this bounded and alternating in sign.
        offset = by - foot_y
        if fall_time is not None and event_time > fall_time + 0.5:
            break
        print(f"  {index:>5} {event_time:8.2f} {name:<20} {np.degrees(r):11.3f} "
              f"{np.degrees(delta):8.3f} {foot_y:9.4f} {by:9.4f} {offset:9.4f}")
        shown += 1
        if shown > 40:
            print("  ... (truncated)")
            break

    # Smooth vs step-locked: compare the roll change across touchdowns with the
    # roll change between them.
    pre_fall = [e for e in events
                if fall_time is None or e[0] < fall_time]
    if len(pre_fall) >= 4:
        rows = [e[2] for e in pre_fall]
        at_events = np.array([roll[i] for i in rows])
        jumps = np.abs(np.diff(at_events))
        total = abs(at_events[-1] - at_events[0])
        print(f"\n  roll at first touchdown {np.degrees(at_events[0]):+.2f} deg, "
              f"at last before the fall {np.degrees(at_events[-1]):+.2f} deg")
        print(f"  total growth {np.degrees(total):.2f} deg over {len(pre_fall)} steps, "
              f"mean per step {np.degrees(np.mean(jumps)):.3f} deg")
        signs = np.sign(np.diff(at_events))
        same = int(np.sum(signs == signs[0])) if signs.size else 0
        print(f"  step-to-step roll change keeps the same sign in {same}/{signs.size} "
              f"steps - a divergence walks one way, an oscillation alternates")


def analyse_solver(cost: Table, fall_time: float | None) -> None:
    """Did the optimizer solve the problem, or fail to?

    This is the fork the whole diagnosis turns on. If the defects and constraint
    violations stay small while the robot falls, the solver is doing its job and
    the problem being posed is the wrong problem - a formulation error, and no
    amount of solver tuning or extra iterations will help. If they blow up first,
    it is the opposite.
    """
    if "solver_dynamics_violation_sse" not in cost:
        print()
        print("  (no solver-health columns - log predates them)")
        return

    section("SOLVER HEALTH (formulation error vs optimizer failure)")
    fields = [
        ("merit", "solver_merit"),
        ("cost", "solver_cost"),
        ("dynamics violation SSE", "solver_dynamics_violation_sse"),
        ("equality constr. SSE", "solver_equality_sse"),
        ("dual feasibility SSE", "solver_dual_feasibility_sse"),
    ]
    print(f"  {'quantity':<26} {'median':>12} {'p95':>12} {'max':>12}")
    for label, col in fields:
        values = finite(cost.col(col))
        if values.size == 0:
            continue
        print(f"  {label:<26} {np.median(values):12.4e} "
              f"{np.percentile(values, 95):12.4e} {values.max():12.4e}")

    if "policy_updated" in cost:
        updated = finite(cost.col("policy_updated"))
        if updated.size:
            failures = int(np.sum(updated < 0.5))
            print(f"\n  policy updates: {updated.size - failures} ok, {failures} failed")

    if fall_time is None:
        return

    # The decisive comparison: solver health well before the fall against the
    # window immediately preceding it.
    t = cost.col("t")
    before = (t < fall_time - 5.0)
    approach = (t >= fall_time - 3.0) & (t < fall_time)
    after = (t >= fall_time)
    if not (before.any() and approach.any()):
        return
    print(f"\n  Around the fall at t = {fall_time:.2f} s:")
    print(f"  {'quantity':<26} {'>5s before':>13} {'last 3s':>13} {'after':>13}")
    for label, col in fields:
        values = cost.col(col)
        def med(mask):
            v = finite(values[mask])
            return float(np.median(v)) if v.size else float("nan")
        print(f"  {label:<26} {med(before):13.4e} {med(approach):13.4e} "
              f"{med(after):13.4e}")
    print("\n  If the defect and constraint columns are flat from 'before' to")
    print("  'last 3s', the optimizer was still solving the problem correctly while")
    print("  balance was already being lost - which puts the fault in the cost,")
    print("  constraints or reference, not in the solver.")


def analyse_cost(cost: Table) -> None:
    section("COST BREAKDOWN (which term actually decides the posture)")
    run_cols = [c for c in cost.columns if c.startswith("run_")]
    term_cols = [c for c in cost.columns if c.startswith("term_")]

    for label, cols, tag in (("RUNNING", run_cols, "run_"),
                             ("TERMINAL", term_cols, "term_")):
        if not cols:
            continue
        means = [(c, nanmean(cost.col(c))) for c in cols]
        means = [(c, m) for c, m in means if np.isfinite(m)]
        means.sort(key=lambda kv: kv[1], reverse=True)
        total = sum(m for _, m in means)
        print(f"\n  {label} terms, ranked by mean value ({len(cost)} samples):")
        print(f"    {'term':<52} {'mean':>12} {'share':>8}")
        for name, value in means:
            share = 100.0 * value / total if total else float("nan")
            print(f"    {name[len(tag):][:52]:<52} {value:12.4f} {share:7.1f}%")

    # The specific question behind the untracked height command: reaching a lower
    # base-z costs stance knee torque, and if that term is an order of magnitude
    # heavier than the base-z tracking error then raising the base-z weight cannot
    # move the posture - which is what was observed.
    torque = [c for c in run_cols if "torque" in c.lower()]
    base = [c for c in run_cols + term_cols
            if "base" in c.lower() or "motiontracking" in c.lower()]
    if torque and base:
        print("\n  Leg-torque vs base-tracking (the height-command question):")
        for col in torque + base:
            print(f"    {col:<52} mean={nanmean(cost.col(col)):10.4f}")
        print("    If the torque term dominates, raising the base-z weight will not")
        print("    move the posture. Decisive test: set legTorqueCost.scaling to 0.0")
        print("    and rerun the same stance case.")

    if cost.has("x_base_z", "ref_base_z"):
        err = cost.col("x_base_z") - cost.col("ref_base_z")
        print(f"\n  base z: actual mean={nanmean(cost.col('x_base_z')):.4f} m  "
              f"reference mean={nanmean(cost.col('ref_base_z')):.4f} m  "
              f"error RMS={rms(err):.4f} m")

    ang = [f"x_hbar_ang_{a}" for a in "xyz"]
    ref_ang = [f"ref_hbar_ang_{a}" for a in "xyz"]
    if cost.has(*ang, *ref_ang):
        print("\n  Angular momentum, actual vs reference. The arm-swing reference")
        print("  commands the very momentum the momentum reference asks to be zero;")
        print("  a large actual against a ~0 reference is that conflict showing up:")
        for a, r in zip(ang, ref_ang):
            print(f"    {a:<18} RMS={rms(cost.col(a)):.5f}     "
                  f"{r:<18} RMS={rms(cost.col(r)):.5f}")

    if "advance_ms" in cost:
        values = finite(cost.col("advance_ms"))
        if values.size:
            print(f"\n  MPC advance time: mean={values.mean():.2f} ms  "
                  f"p95={np.percentile(values, 95):.2f} ms  max={values.max():.2f} ms")


# --------------------------------------------------------------------------
# plots
# --------------------------------------------------------------------------

def make_plots(state: Table, cost: Table | None, prefix: Path) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("  (matplotlib not installed; skipping plots)")
        return

    t = state.col("t")
    fig, axes = plt.subplots(4, 1, figsize=(13, 14), sharex=True)

    ax = axes[0]
    for a in "xyz":
        col = f"err_v_world_{a}"
        if col in state:
            ax.plot(t, state.col(col), lw=0.8, label=f"e_v {a}")
    ax.set_ylabel("lin vel error [m/s]")
    ax.legend(ncol=3, fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[1]
    if state.has("corr_applied", "nis_per_dof"):
        mask = state.col("corr_applied") > 0.5
        ax.plot(t[mask], state.col("nis_per_dof")[mask], lw=0.6, label="NIS / dof")
        ax.axhline(1.0, color="k", ls="--", lw=0.8, label="consistent (1.0)")
        ax.set_yscale("log")
    ax.set_ylabel("NIS / dof")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[2]
    for col, label in (("height_inekf", "filter height"),
                       ("height_reported", "reported (blended)"),
                       ("gt_p_z", "ground truth")):
        if col in state:
            ax.plot(t, state.col(col), lw=0.8, label=label)
    ax.set_ylabel("base height [m]")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[3]
    if "contact_left" in state:
        ax.plot(t, state.col("contact_left"), lw=0.8, label="left")
    if "contact_right" in state:
        ax.plot(t, state.col("contact_right") * 0.9, lw=0.8, label="right")
    if "phase" in state:
        ax.plot(t, state.col("phase") / 2.0, lw=0.8, ls="--", label="phase/2")
    ax.set_ylabel("contact / phase")
    ax.set_xlabel("t [s]")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    out = prefix.with_name(prefix.name + "_state.png")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"  wrote {out}")

    if cost is not None and not cost.empty:
        run_cols = [c for c in cost.columns if c.startswith("run_")]
        if run_cols:
            ranked = sorted(((c, nanmean(cost.col(c))) for c in run_cols),
                            key=lambda kv: (kv[1] if np.isfinite(kv[1]) else -np.inf),
                            reverse=True)[:8]
            fig2, ax2 = plt.subplots(figsize=(13, 6))
            ct = cost.col("t")
            for name, _ in ranked:
                ax2.plot(ct, cost.col(name), lw=0.8, label=name[4:][:40])
            ax2.set_yscale("symlog", linthresh=1e-4)
            ax2.set_xlabel("t [s]")
            ax2.set_ylabel("cost term value")
            ax2.legend(fontsize=7, ncol=2)
            ax2.grid(alpha=0.3)
            out2 = prefix.with_name(prefix.name + "_cost.png")
            fig2.tight_layout()
            fig2.savefig(out2, dpi=110)
            print(f"  wrote {out2}")


# --------------------------------------------------------------------------

def main() -> int | str:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("prefix", type=Path,
                        help="log path prefix, without _state.csv / _cost.csv")
    parser.add_argument("--phase", type=int, default=None,
                        help="restrict the state analysis to one phase "
                             "(0 pre-init, 1 warm-up, 2 estimate driving control)")
    parser.add_argument("--from-time", type=float, default=None)
    parser.add_argument("--to-time", type=float, default=None)
    parser.add_argument("--plot", action="store_true", help="also write PNG summaries")
    parser.add_argument("--timeline", type=float, nargs="?", const=5.0, default=None,
                        metavar="WINDOW",
                        help="print a per-window timeline (default window 5 s)")
    parser.add_argument("--fall-lead", type=float, default=6.0,
                        help="seconds before the fall onset to dump in detail")
    args = parser.parse_args()

    state_path = args.prefix.with_name(args.prefix.name + "_state.csv")
    cost_path = args.prefix.with_name(args.prefix.name + "_cost.csv")

    if not state_path.exists():
        return f"no such file: {state_path}"
    state = Table.read(state_path)
    if state.empty:
        return f"{state_path} has a header but no rows"

    cost = Table.read(cost_path) if cost_path.exists() else None

    t = state.col("t")
    print(f"state log : {state_path}  ({len(state)} rows, "
          f"{np.nanmax(t) - np.nanmin(t):.1f} s)")
    if cost is not None:
        print(f"cost  log : {cost_path}  ({len(cost)} rows)")

    describe_phases(state)

    if args.phase is not None:
        state = state.select(state.col("phase").astype(int) == args.phase)
        print(f"\n  [restricted to phase {args.phase}: {len(state)} samples]")
    if args.from_time is not None:
        state = state.select(state.col("t") >= args.from_time)
        if cost is not None and not cost.empty:
            cost = cost.select(cost.col("t") >= args.from_time)
    if args.to_time is not None:
        state = state.select(state.col("t") <= args.to_time)
        if cost is not None and not cost.empty:
            cost = cost.select(cost.col("t") <= args.to_time)
    if state.empty:
        return "no samples left after filtering"

    contacts = sorted({c[: -len("_in_stance")]
                       for c in state.columns if c.endswith("_in_stance")})

    # Whether the robot stayed up governs how every number below should be read,
    # so it is established first.
    fall_time = find_fall(state)
    if fall_time is None:
        print("\n  No loss of balance detected (ground-truth pelvis stayed above 0.55 m "
              "and trunk within 0.5 rad).")
    else:
        print(f"\n  *** LOSS OF BALANCE at t = {fall_time:.2f} s "
              f"({fall_time - float(np.nanmin(state.col('t'))):.2f} s into the log). "
              "Aggregate statistics below mix pre- and post-fall data; use "
              "--to-time to look at the healthy segment alone. ***")

    if args.timeline is not None:
        analyse_timeline(state, args.timeline)
    if fall_time is not None:
        analyse_fall(state, fall_time, args.fall_lead)

    analyse_signal_quality(state)
    analyse_consistency(state)
    analyse_covariance(state)
    analyse_accuracy(state)
    analyse_height(state)
    analyse_contacts(state, contacts)
    analyse_lateral(state, contacts, fall_time)
    analyse_joints(state)
    if cost is not None and not cost.empty:
        analyse_solver(cost, fall_time)
        analyse_cost(cost)

    if args.plot:
        section("PLOTS")
        make_plots(state, cost, args.prefix)

    return 0


if __name__ == "__main__":
    sys.exit(main())
