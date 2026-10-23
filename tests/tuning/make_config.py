#!/usr/bin/env python3
"""Write a variant of config/g1/ros2_controllers.yaml for one WB-ID tuning experiment.

usage:
  make_config.py <out.yaml> [--in <in.yaml>] [--tracking HZ] [--kp SPEC] [--kd SPEC] [path=value ...]

  path=value   key path below the WB controller's ros__parameters, e.g.
               ocs2.costs.q.baseVelocity=[30.0,15.0,10.0,30.0,30.0,30.0]
               (inline scalars and flow sequences, also multi-line ones, are replaced in place)
  --kp/--kd    control.jointEffortPd gains by joint group, unset groups keep the input values:
               hp hr hy kn ap ar (hip pitch/roll/yaw, knee, ankle pitch/roll, both legs),
               wy wr wp (waist), sp sr sy el (shoulder pitch/roll/yaw, elbow, both arms);
               "all:200" sets every joint, e.g. --kp all:200 --kd ap:20,ar:20
  --tracking   diagnostics.trackingRate [Hz] (adds the key if missing); 0 leaves it off

The text is edited in place, so comments and $(var ...) substitutions survive.
"""
import argparse
import pathlib
import re

WB = "humanoid_whole_body_inverse_dynamics_mpc_controller"
LEG = ["hp", "hr", "hy", "kn", "ap", "ar"]
ORDER = LEG + LEG + ["wy", "wr", "wp"] + ["sp", "sr", "sy", "el"] * 2
DEFAULT_IN = pathlib.Path(__file__).resolve().parents[2] / "config/g1/ros2_controllers.yaml"


def locate(lines, keys):
    """Index range [i, j] of the line(s) holding the value of the key path."""
    stack = []
    for i, line in enumerate(lines):
        m = re.match(r"^(\s*)([A-Za-z0-9_]+):(.*)$", line)
        if not m or line.lstrip().startswith("#"):
            continue
        indent = len(m.group(1))
        while stack and stack[-1][0] >= indent:
            stack.pop()
        stack.append((indent, m.group(2)))
        if [k for _, k in stack] == keys:
            rest = m.group(3).split("#")[0].strip()
            j = i
            if not rest:
                j = i + 1
                while not lines[j].strip():
                    j += 1
                if not lines[j].lstrip().startswith("["):
                    raise ValueError(f"{'.'.join(keys)}: not a scalar or flow sequence")
                depth = lines[j].count("[") - lines[j].count("]")
            else:
                depth = rest.count("[") - rest.count("]")
            while depth > 0:
                j += 1
                depth += lines[j].count("[") - lines[j].count("]")
            return i, j, m.group(1), m.group(2)
    raise KeyError(".".join(keys))


def read_list(lines, keys):
    i, j, _, _ = locate(lines, keys)
    text = " ".join(lines[i:j + 1]).split(":", 1)[1]
    return [float(x) for x in re.findall(r"-?\d+(?:\.\d+)?(?:[eE]-?\d+)?", text)]


def set_value(lines, keys, value):
    i, j, indent, key = locate(lines, keys)
    lines[i:j + 1] = [f"{indent}{key}: {value}"]


def gains(current, spec):
    values = dict(zip(range(len(ORDER)), current))
    for item in filter(None, spec.split(",")):
        group, x = item.split(":")
        for k, g in enumerate(ORDER):
            if group in ("all", g):
                values[k] = float(x)
    return "[" + ", ".join(f"{values[k]:.1f}" for k in range(len(ORDER))) + "]"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out")
    ap.add_argument("--in", dest="inp", default=str(DEFAULT_IN))
    ap.add_argument("--tracking", type=float, default=500.0)
    ap.add_argument("--kp", default="")
    ap.add_argument("--kd", default="")
    ap.add_argument("edits", nargs="*")
    a = ap.parse_intermixed_args()
    lines = open(a.inp).read().split("\n")
    root = [WB, "ros__parameters"]
    for name, spec in (("kp", a.kp), ("kd", a.kd)):
        if spec:
            keys = root + ["control", "jointEffortPd", name]
            set_value(lines, keys, gains(read_list(lines, keys), spec))
    for edit in a.edits:
        path, value = edit.split("=", 1)
        set_value(lines, root + path.split("."), value)
    if a.tracking > 0:
        try:
            set_value(lines, root + ["diagnostics", "trackingRate"], f"{a.tracking}")
        except KeyError:
            i, _, indent, key = locate(lines, root + ["diagnostics", "statusLogPeriod"])
            lines.insert(i + 1, f"{indent}trackingRate: {a.tracking}")
    open(a.out, "w").write("\n".join(lines))
    print(a.out)


if __name__ == "__main__":
    main()
