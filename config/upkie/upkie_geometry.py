"""Upkie contact geometry for the offline tools: config/upkie/geometry.yaml, the file the MPC (paths.geometryFile) and
the wheel-aware KF (geometryFile) read, checked against the simulated robot's collision tire."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

HERE = Path(__file__).resolve().parent
GEOMETRY = HERE / "geometry.yaml"
MJCF = HERE.parents[1] / "description" / "mujoco" / "robots" / "upkie" / "upkie.xml"


def load(path=GEOMETRY):
    """The geometry map: wheel {radius, half_width, center_offset, axle_axis} and knee_housings [{frame, center,
    radius, half_width}]."""
    return yaml.safe_load(Path(path).read_text())["geometry"]


def mjcf_tire(mjcf=MJCF):
    """Radius (circumradius of the prism) and width of upkie.xml's tire collision mesh [m]."""
    mesh = next(m for m in ET.parse(mjcf).getroot().iter("mesh") if m.get("name") == "tire_collision")
    v = [float(x) for x in mesh.get("vertex").split()]
    radius = max(math.hypot(v[i], v[i + 1]) for i in range(0, len(v), 3))
    return radius, max(v[2::3]) - min(v[2::3])


def check_mjcf(geometry, mjcf=MJCF):
    """The tools plan with the geometry file and simulate with the MJCF: they must describe the same tire."""
    radius, width = mjcf_tire(mjcf)
    wheel = geometry["wheel"]
    if abs(radius - wheel["radius"]) > 1e-6 or abs(width - 2 * wheel["half_width"]) > 1e-6:
        raise ValueError(f"{mjcf} has a {radius:.4f} m radius, {width:.4f} m wide collision tire, but the geometry file "
                         f"says {wheel['radius']} m and {2 * wheel['half_width']} m: update one of them")
