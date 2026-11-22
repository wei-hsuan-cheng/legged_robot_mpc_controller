"""Upkie wheel (hub + tire) as parametric CadQuery solids.

A reconstruction of this folder's wheel_hub.stl and tire.stl (the mjlab_upkie meshes; see
description/urdf/robots/upkie/MODEL_SOURCE.md): every dimension below was measured from those meshes (vertex
circles and ray-cast cross-sections), so the solids overlay them closely, but small features (fillets, counterbores
of the screw holes) are simplified. It is not the upstream design source, and nothing here is load- or
fabrication-checked.

Units: mm. Each part is built in its STL's own frame (wheel axis = +Y): the hub spans y = 0 (motor side) to -42, the
tire y = -22 to +22. The assembly places both in the wheel link frame (axis = +Z, origin at the wheel joint) exactly as
upkie.xml places the meshes, so the tire centre sits 31.2 mm along the axle (the MPC's wheel contact frame).

Render in VS Code with the OCP CAD Viewer extension: select a Python environment with cadquery and ocp_vscode, open
the viewer, then run this file (Run > Run Without Debugging). Headless:

    python upkie_wheel.py --export /tmp/upkie_wheel      # STEP + STL, plus the validity checks
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent


@dataclass
class TireParams:
    crown_radius: float = 55.5        # tread radius at the centre plane
    tread_edge_radius: float = 54.0   # tread radius where the shoulder starts
    tread_half_width: float = 18.2    # axial half-width of the crowned tread
    shoulder_mid: tuple[float, float] = (52.3, 20.9)  # (r, |y|) point on the rounded shoulder
    face_outer_radius: float = 49.4   # where the shoulder meets the side faces
    half_width: float = 22.0          # tire width 44
    bore_motor_side: float = 41.2     # bore at the motor side (+y), over the hub flange
    bore_motor_depth: float = 2.8     # its depth from the +y face
    bore: float = 38.7                # main bore, on the hub rim
    lip_start: float = -15.0          # y where the outboard inner lip starts
    lip_radius: float = 28.2          # lip bore, clamped by the hub's outboard ring
    lip_end: float = -18.7            # y where the lip steps out again
    outboard_bore: float = 30.7       # bore from the lip to the outboard (-y) face


@dataclass
class HubParams:
    flange_radius: float = 41.5       # motor-side flange
    flange_thickness: float = 1.6
    recess_radius: float = 31.6       # motor-side recess (rotor bell clearance)
    recess_depth: float = 3.1
    rim_radius: float = 38.0          # tire seat
    bead_radius: float = 39.9         # tire-retaining bead on the seat
    bead_y: tuple[float, float] = (-6.1, -7.1)
    rim_end_y: float = -33.4          # end of the tire seat (outboard)
    rim_inner_radius: float = 21.5    # inner radius of the rim ring, between the spokes
    chamfer_radius: float = 23.4      # recess-to-rim chamfer
    ring_outer: float = 28.0          # outboard ring (holds the tire lip)
    ring_ridge: float = 30.5
    ring_ridge_y: float = -39.5
    ring_inner_end: float = 26.4      # inner radius of the ring at the outboard face
    ring_cone_start_y: float = -37.0
    depth: float = 42.0               # overall axial length
    boss_radius: float = 13.0         # central boss, bolted to the MJ5208 rotor
    boss_y: tuple[float, float] = (-7.1, -20.3)
    spokes: int = 6
    spoke_width: float = 8.5          # best match to the STL (8-9 mm); its spokes are filleted and taper
    spoke_root_radius: float = 12.0   # spokes start inside the boss
    spoke_taper: tuple[tuple[float, float], tuple[float, float]] = ((14.0, -20.3), (20.0, -33.4))  # inner edge
    bolt_count: int = 3
    bolt_circle_radius: float = 8.5   # measured from the hole edges (r 6.7 to ~10.3)
    bolt_diameter: float = 3.6        # about M3 clearance
    bolt_angles_deg: tuple[float, ...] = (60.0, 180.0, 300.0)  # atan2(z, x) in the STL frame


def revolve_profile(points: list[tuple[float, float]]) -> cq.Workplane:
    """Revolve a closed (r, y) polyline about the Y axis."""
    return cq.Workplane("XY").polyline(points).close().revolve(360, (0, 0, 0), (0, 1, 0))


def make_tire(p: TireParams = TireParams()) -> cq.Workplane:
    w, sm = p.half_width, p.shoulder_mid
    sketch = (
        cq.Workplane("XY")
        .moveTo(p.tread_edge_radius, p.tread_half_width)
        .threePointArc(sm, (p.face_outer_radius, w))                                  # +y shoulder
        .lineTo(p.bore_motor_side, w)                                                  # +y (motor-side) face
        .lineTo(p.bore_motor_side, w - p.bore_motor_depth)
        .lineTo(p.bore, w - p.bore_motor_depth)
        .lineTo(p.bore, p.lip_start)                                                   # main bore
        .lineTo(p.lip_radius, p.lip_start)
        .lineTo(p.lip_radius, p.lip_end)                                               # inner lip
        .lineTo(p.outboard_bore, p.lip_end)
        .lineTo(p.outboard_bore, -w)
        .lineTo(p.face_outer_radius, -w)                                               # -y (outboard) face
        .threePointArc((sm[0], -sm[1]), (p.tread_edge_radius, -p.tread_half_width))    # -y shoulder
        .threePointArc((p.crown_radius, 0.0), (p.tread_edge_radius, p.tread_half_width))  # crowned tread
        .close()
    )
    return sketch.revolve(360, (0, 0, 0), (0, 1, 0))


def make_hub(p: HubParams = HubParams()) -> cq.Workplane:
    # Rim, motor-side flange, bead and outboard ring: one revolved profile.
    ring = revolve_profile([
        (p.recess_radius, 0.0), (p.flange_radius, 0.0), (p.flange_radius, -p.flange_thickness),
        (p.rim_radius, -p.flange_thickness), (p.rim_radius, p.bead_y[0]), (p.bead_radius, p.bead_y[0]),
        (p.bead_radius, p.bead_y[1]), (p.rim_radius, p.bead_y[1]), (p.rim_radius, p.rim_end_y),
        (p.ring_outer, p.rim_end_y), (p.ring_outer, p.ring_ridge_y), (p.ring_ridge, p.ring_ridge_y),
        (p.ring_ridge, -(p.depth - 1.0)), (p.ring_ridge - 1.0, -p.depth), (p.ring_inner_end, -p.depth),
        (p.rim_inner_radius, p.ring_cone_start_y), (p.rim_inner_radius, -5.0),
        (p.chamfer_radius, -p.recess_depth), (p.recess_radius, -p.recess_depth),
    ])
    y0, y1 = p.boss_y
    boss = cq.Solid.makeCylinder(p.boss_radius, y0 - y1, pnt=cq.Vector(0, y0, 0), dir=cq.Vector(0, -1, 0))
    hub = ring.union(boss)

    # Spokes: a tapered gusset in the (r, y) plane, extruded tangentially and repeated around the axis.
    (ti_r, ti_y), (to_r, to_y) = p.spoke_taper
    spoke = (cq.Workplane("XY")
             .polyline([(p.spoke_root_radius, y0), (p.rim_inner_radius + 0.5, y0), (p.rim_inner_radius + 0.5, to_y),
                        (to_r, to_y), (ti_r, ti_y), (p.spoke_root_radius, y1)])
             .close()
             .extrude(p.spoke_width / 2, both=True))
    for k in range(p.spokes):
        hub = hub.union(spoke.rotate((0, 0, 0), (0, 1, 0), k * 360.0 / p.spokes))

    # Bolt holes through the boss, along the axis.
    for a in p.bolt_angles_deg[: p.bolt_count]:
        x = p.bolt_circle_radius * math.cos(math.radians(a))
        z = p.bolt_circle_radius * math.sin(math.radians(a))
        hole = cq.Solid.makeCylinder(p.bolt_diameter / 2, (y0 - y1) + 2.0,
                                     pnt=cq.Vector(x, y0 + 1.0, z), dir=cq.Vector(0, -1, 0))
        hub = hub.cut(hole)
    return hub


# upkie.xml places both meshes with quat (0.7071, 0.7071, 0, 0): +90 deg about x, mesh +Y -> link +Z.
HUB_IN_LINK = cq.Location(cq.Vector(0, 0, -10.4), cq.Vector(1, 0, 0), 90)
TIRE_IN_LINK = cq.Location(cq.Vector(0, 0, -31.2), cq.Vector(1, 0, 0), 90)


def make_wheel_assembly(hub: cq.Workplane, tire: cq.Workplane) -> cq.Assembly:
    assembly = cq.Assembly(name="upkie_wheel")
    assembly.add(hub, name="wheel_hub", loc=HUB_IN_LINK, color=cq.Color(0.90, 0.90, 0.90))
    assembly.add(tire, name="tire", loc=TIRE_IN_LINK, color=cq.Color(0.30, 0.30, 0.30))
    return assembly


def load_reference_mesh(name: str) -> cq.Shape | None:
    """This folder's STL (meters) as a mm shape, for an overlay; None if it cannot be read."""
    from OCP.StlAPI import StlAPI_Reader
    from OCP.TopoDS import TopoDS_Shape

    path = HERE / f"{name}.stl"
    shape = TopoDS_Shape()
    if not path.exists() or not StlAPI_Reader().Read(shape, str(path)):
        return None
    return cq.Shape.cast(shape).scale(1000.0)


def report(name: str, part: cq.Workplane, reference: cq.Shape | None) -> None:
    solid = part.val()
    bb = solid.BoundingBox()
    line = (f"{name:9s} valid {solid.isValid()}, solids {len(part.solids().vals())}, volume {solid.Volume() / 1e3:7.2f} cm^3, "
            f"size {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm (y {bb.ymin:.1f}..{bb.ymax:.1f})")
    if reference is not None:
        rb = reference.BoundingBox()
        line += f" | STL size {rb.xlen:.1f} x {rb.ylen:.1f} x {rb.zlen:.1f} mm (y {rb.ymin:.1f}..{rb.ymax:.1f})"
    print(line)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--export", type=Path, help="write STEP (assembly and parts) and STL (parts) into this folder")
    parser.add_argument("--no-reference", action="store_true", help="do not overlay the original STL meshes")
    parser.add_argument("--no-show", action="store_true", help="do not send anything to OCP CAD Viewer")
    args = parser.parse_args()

    hub, tire = make_hub(), make_tire()
    ref_hub = None if args.no_reference else load_reference_mesh("wheel_hub")
    ref_tire = None if args.no_reference else load_reference_mesh("tire")
    report("hub", hub, ref_hub)
    report("tire", tire, ref_tire)
    assembly = make_wheel_assembly(hub, tire)

    if args.export:
        args.export.mkdir(parents=True, exist_ok=True)
        assembly.export(str(args.export / "upkie_wheel.step"))
        for name, part in (("upkie_wheel_hub", hub), ("upkie_tire", tire)):
            cq.exporters.export(part, str(args.export / f"{name}.step"))
            cq.exporters.export(part, str(args.export / f"{name}.stl"), tolerance=0.05, angularTolerance=0.1)
        print(f"exported to {args.export}")

    if not args.no_show:
        try:
            from ocp_vscode import show
        except ImportError:
            print("ocp_vscode is not installed: nothing shown (pip install ocp_vscode, or use --no-show)")
            return
        objects, names, colors, alphas = [assembly], ["upkie_wheel"], [None], [1.0]
        for ref, label, loc in ((ref_hub, "wheel_hub.stl", HUB_IN_LINK), (ref_tire, "tire.stl", TIRE_IN_LINK)):
            if ref is not None:
                objects.append(ref.moved(loc))
                names.append(f"reference {label}")
                colors.append("orange")
                alphas.append(0.25)
        # ocp_vscode reports a missing viewer itself ("Unexpected error talking to the viewer").
        print("sending to OCP CAD Viewer (open its panel in VS Code first if nothing appears)")
        show(*objects, names=names, colors=colors, alphas=alphas)


if __name__ == "__main__":
    main()
