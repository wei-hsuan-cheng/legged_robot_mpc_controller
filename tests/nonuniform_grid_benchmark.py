#!/usr/bin/env python3
"""Serial H2 uniform/geometric-grid comparisons with isolated terrain and InEKF feedback."""

import argparse
import json
from pathlib import Path
import subprocess
import sys

from mujoco_scene import write_scene


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--profiles", nargs="+", choices=["uniform", "geometric", "fast", "conservative", "balanced"],
                        default=["uniform", "geometric"])
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument("--mpc-rate", type=int, default=100)
    parser.add_argument("--rough-hold", type=float, default=10.0,
                        help="Low-speed uneven-field hold [s]; keep the traversal inside the field")
    parser.add_argument("--estimator", choices=["humanoid_inekf", "humanoid_linear_kf"], default="humanoid_inekf")
    parser.add_argument("--no-tracking", action="store_true", help="Measure normal runtime without optional per-solve diagnostics")
    parser.add_argument("--set", action="append", default=[], metavar="KEY=JSON",
                        help="Apply the same controller override to every compared profile")
    parser.add_argument("--cases", nargs="+", choices=["flat", "rough", "vy", "yaw", "run", "run2", "rough_run", "rough_run2"], default=["flat", "rough"])
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    rough = write_scene(args.output / "rough_scene.xml", "h2", "rough")
    cases = {"flat": ("vx", 0.3, 20, 3), "rough": ("vx", 0.2, args.rough_hold, 3),
             "vy": ("vy", 0.25, 20, 3), "yaw": ("yaw", 0.6, 20, 3),
             "run": ("vx", 1.0, 20, 3), "run2": ("vx", 2.0, 15, 3),
             "rough_run": ("vx", 1.0, 1, 1), "rough_run2": ("vx", 2.0, 1, 1)}
    results = []
    # Alternate order to reduce systematic warm-cache/VM-load bias.
    for repeat in range(args.repeats):
        profiles = args.profiles if repeat % 2 == 0 else list(reversed(args.profiles))
        for case in args.cases:
            axis, speed, hold, ramp = cases[case]
            for profile in profiles:
                prefix = args.output / f"{profile}_{case}_{repeat + 1}"
                cmd = [sys.executable, str(root / "wb_id_walk_test.py"), "--robot-name", "h2",
                       "--axis", axis, "--speed", str(speed), "--hold", str(hold), "--settle", "4",
                       "--ramp", str(ramp), "--rest", "5", "--mpc-rate", str(args.mpc_rate),
                       "--source", "state_estimator", "--estimator", args.estimator,
                       "--record-joints", "--prefix", str(prefix),
                       "--set", "ocs2.solver.sqp.timeGrid.activate=" + ("false" if profile == "uniform" else "true")]
                if not args.no_tracking:
                    cmd += ["--record-tracking"]
                if profile == "geometric":
                    cmd += ["--set", "ocs2.solver.sqp.timeGrid.maxStep=0.05",
                            "--set", "ocs2.solver.sqp.timeGrid.growthFactor=1.2"]
                if profile == "fast":
                    cmd += ["--set", "ocs2.solver.sqp.timeGrid.maxStep=0.06",
                            "--set", "ocs2.solver.sqp.timeGrid.growthFactor=1.3"]
                if profile == "conservative":
                    cmd += ["--set", "ocs2.solver.sqp.timeGrid.maxStep=0.04",
                            "--set", "ocs2.solver.sqp.timeGrid.growthFactor=1.4"]
                if profile == "balanced":
                    cmd += ["--set", "ocs2.solver.sqp.timeGrid.maxStep=0.035",
                            "--set", "ocs2.solver.sqp.timeGrid.growthFactor=1.4"]
                if speed > 1.0:
                    cmd += ["--set", "ocs2.reference.maxDisplacementVelocityX=2.0"]
                if case.startswith("rough"):
                    cmd += ["--scene", str(rough)]
                for override in args.set:
                    cmd += ["--set", override]
                print("RUN", prefix.name, flush=True)
                with prefix.with_suffix(".runner.log").open("w") as log:
                    completed = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT)
                result_file = prefix.with_suffix(".json")
                result = json.loads(result_file.read_text()) if result_file.exists() else {}
                results.append(dict(profile=profile, case=case, repeat=repeat + 1,
                                    exit_code=completed.returncode, prefix=str(prefix),
                                    balance_survived=result.get("balance_survived", result.get("verdict") in ("SUCCESS", "NO_TRACKING")),
                                    verdict=result.get("verdict", "NO_RESULT")))
                (args.output / "progress.json").write_text(json.dumps(results, indent=2) + "\n")
                print(json.dumps(results[-1]), flush=True)
                if result.get("verdict") in ("STARTUP_ERROR", "LAUNCH_EXITED", "TIMEOUT") or not result:
                    return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
