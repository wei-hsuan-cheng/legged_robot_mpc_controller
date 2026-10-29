#!/usr/bin/env python3
"""Compile and run a native tangent regression using a sourced workspace's build flags."""

import argparse
import json
from pathlib import Path
import shlex
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path.cwd())
    parser.add_argument("--test", default="pinocchio_tangent_codegen_test")
    parser.add_argument("--compile-base-cost", action="store_true", help="Compile the base cost source before a full rebuild")
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    build = workspace / "build/legged_robot_mpc_controller"
    entries = json.loads((build / "compile_commands.json").read_text())
    entry = next(row for row in entries if row["file"].endswith("WbIdDynamics.cpp"))
    command = shlex.split(entry["command"])
    index = command.index("-o")
    del command[index:index + 2]
    command.remove("-c")
    command.remove(entry["file"])
    link = shlex.split((build / "CMakeFiles/legged_robot_humanoid_wb_id_core.dir/link.txt").read_text())
    link = link[next(i for i, value in enumerate(link) if value.startswith("-Wl,-rpath")):]
    link = [str(build / value) if value.startswith("liblegged_robot_") else value for value in link]
    source = Path(__file__).resolve().parent / (args.test + ".cpp")
    extra_sources = [str(source.parents[1] / "src/core/humanoid_wb_id_mpc/src/cost/WbIdBaseMotionCost.cpp")] if args.compile_base_cost else []
    with tempfile.TemporaryDirectory(prefix="tangent_native_") as temp:
        executable = str(Path(temp) / args.test)
        subprocess.run(command + ["-O0", str(source)] + extra_sources + ["-o", executable,
                                  str(build / "liblegged_robot_humanoid_wb_id_core.so")] + link, cwd=workspace, check=True)
        subprocess.run([executable], cwd=workspace, check=True)


if __name__ == "__main__":
    main()
