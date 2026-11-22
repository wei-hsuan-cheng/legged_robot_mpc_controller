"""Upkie wheel (hub + tire) as parametric CadQuery solids.

A reconstruction of this folder's wheel_hub.stl and tire.stl (the mjlab_upkie meshes; see
description/urdf/robots/upkie/MODEL_SOURCE.md): every dimension below was measured from those meshes (vertex
circles and ray-cast cross-sections), so the solids overlay them closely, but small features (fillets, counterbores
of the screw holes) are simplified. It is not the upstream design source, and nothing here is load- or
fabrication-checked.

Units: mm. Each part is built in its STL's own frame (wheel axis = +Y): the hub spans y = 0 (motor side) to -42, the
tire y = -22 to +22. The assembly places both in the wheel link frame (axis = +Z, origin at the wheel joint) exactly as
upkie.xml places the meshes, so the tire centre sits 31.2 mm along the axle (the MPC's wheel contact frame).

Larger wheels (--nominal-radius, mm; 55 is the original): the tire and the hub's rim move radially outward by the
difference, keeping the tread profile, the 44 mm width, the tire's radial thickness and the motor interface (boss and
bolt circle); the spokes lengthen. --meshes writes the parts as STL in meters in the original meshes' frames, for
upkie.xml, and --inertial prints the wheel link's MJCF <inertial>: hub + tire from these solids plus the unchanged
MJ5208 rotor, with densities calibrated so the 55 mm wheel reproduces the original link's mass and axle inertia.

Render in VS Code with the OCP CAD Viewer extension: select a Python environment with cadquery and ocp_vscode, open
the viewer, then run this file (Run > Run Without Debugging). Headless:

    python upkie_wheel.py --export /tmp/upkie_wheel      # STEP + STL, plus the validity checks
    python upkie_wheel.py --nominal-radius 110 --meshes . --inertial --no-show   # the 2x wheel for upkie.xml
"""

from __future__ import annotations

import argparse
import math
import struct
from dataclasses import dataclass, replace
from pathlib import Path

import cadquery as cq
import numpy as np

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


NOMINAL_RADIUS = 55.0  # mm: the original tire's nominal radius (upkie.xml's collision prism); its tread crown is 55.5


def wheel_params(nominal_radius: float = NOMINAL_RADIUS) -> tuple[TireParams, HubParams]:
    """Tire and hub parameters for a wheel of this nominal radius [mm]: the tire and the hub's rim (seat, bead,
    flange, outboard ring, the spokes' outer end) move outward by nominal_radius - 55; the boss and bolt circle stay."""
    dr = nominal_radius - NOMINAL_RADIUS
    t, h = TireParams(), HubParams()
    tire = replace(t, crown_radius=t.crown_radius + dr, tread_edge_radius=t.tread_edge_radius + dr,
                   shoulder_mid=(t.shoulder_mid[0] + dr, t.shoulder_mid[1]), face_outer_radius=t.face_outer_radius + dr,
                   bore_motor_side=t.bore_motor_side + dr, bore=t.bore + dr, lip_radius=t.lip_radius + dr,
                   outboard_bore=t.outboard_bore + dr)
    inner, outer = h.spoke_taper
    hub = replace(h, flange_radius=h.flange_radius + dr, recess_radius=h.recess_radius + dr,
                  rim_radius=h.rim_radius + dr, bead_radius=h.bead_radius + dr,
                  rim_inner_radius=h.rim_inner_radius + dr, chamfer_radius=h.chamfer_radius + dr,
                  ring_outer=h.ring_outer + dr, ring_ridge=h.ring_ridge + dr, ring_inner_end=h.ring_inner_end + dr,
                  spoke_taper=(inner, (outer[0] + dr, outer[1])))
    return tire, hub


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


# The original wheel link's inertial in upkie.xml (both wheels): hub + tire + the MJ5208 rotor.
LINK_MASS = 0.2585
LINK_COM = np.array([-7.85066e-06, 4.53426e-06, -0.0252368])
LINK_INERTIA = np.array([[0.000202065, -7.7265e-09, 1.97562e-08],
                         [-7.7265e-09, 0.000202069, -1.14162e-08],
                         [1.97562e-08, -1.14162e-08, 0.000313606]])
ROTOR_POSE = (np.array([0.0, 0.0, 0.0075]), np.diag([1.0, -1.0, -1.0]))  # upkie.xml: pos, quat (0 1 0 0)


def mesh_moments(vertices: np.ndarray, triangles: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    """Volume, first moment (int x dV) and second moment (int x x^T dV) of a closed, outward-oriented mesh."""
    a, b, c = (vertices[triangles[:, k]] for k in range(3))
    det = np.einsum("ij,ij->i", a, np.cross(b, c))  # 6 x signed volume of each tetrahedron with the origin
    s = a + b + c
    outer = lambda u: np.einsum("ni,nj->nij", u, u)  # noqa: E731
    second = (det[:, None, None] * (outer(a) + outer(b) + outer(c) + outer(s))).sum(0) / 120.0
    return det.sum() / 6.0, (det[:, None] * s).sum(0) / 24.0, second


def solid_moments(part: cq.Workplane, location: cq.Location) -> tuple[float, np.ndarray, np.ndarray]:
    """Moments of a part placed in the wheel link frame, in m."""
    vertices, triangles = part.val().moved(location).tessellate(0.02, 0.1)
    v = np.array([[q.x, q.y, q.z] for q in vertices]) * 1e-3
    return mesh_moments(v, np.array(triangles))


def stl_moments(path: Path, translation: np.ndarray, rotation: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    """Moments of a binary STL (m) placed in the wheel link frame."""
    data = path.read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    record = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    v = np.frombuffer(data[84:84 + 50 * n], dtype=record, count=n)["v"].astype(float).reshape(-1, 3)
    return mesh_moments(v @ rotation.T + translation, np.arange(3 * n).reshape(n, 3))


def link_inertial(tire: cq.Workplane, hub: cq.Workplane) -> dict:
    """Wheel link mass, COM and inertia about the COM (link frame, SI) with this hub and tire and the original
    rotor. One density for hub + tire and one for the rotor, fitted (least squares) to the original link's mass, COM
    and inertia with the 55 mm parts; the original's predicted values are returned as a check."""
    def total(*moments):
        return tuple(sum(x) for x in zip(*moments))

    def combine(rho_w, wheel, rho_r, rotor):
        mass = rho_w * wheel[0] + rho_r * rotor[0]
        com = (rho_w * wheel[1] + rho_r * rotor[1]) / mass
        second = rho_w * wheel[2] + rho_r * rotor[2] - mass * np.outer(com, com)
        return mass, com, np.trace(second) * np.eye(3) - second

    rotor = stl_moments(HERE / "mj5208_rotor.stl", *ROTOR_POSE)
    original = total(solid_moments(make_hub(), HUB_IN_LINK), solid_moments(make_tire(), TIRE_IN_LINK))
    # Linear in the densities: mass, first moment along the axle, and the second moments about the joint
    # (axle and transverse), each relative to the original link's value: a least-squares fit of the two densities.
    def features(m):
        return np.array([m[0], m[1][2], m[2][0, 0] + m[2][1, 1], m[2][1, 1] + m[2][2, 2]])
    com_term = LINK_MASS * np.outer(LINK_COM, LINK_COM)
    origin = LINK_INERTIA + np.trace(com_term) * np.eye(3) - com_term  # the original inertia about the joint
    target = np.array([LINK_MASS, LINK_MASS * LINK_COM[2], origin[2, 2], origin[0, 0]])
    A = np.column_stack([features(original), features(rotor)]) / target[:, None]
    rho_w, rho_r = np.linalg.lstsq(A, np.ones(4), rcond=None)[0]
    check = combine(rho_w, original, rho_r, rotor)
    mass, com, inertia = combine(rho_w, total(solid_moments(hub, HUB_IN_LINK), solid_moments(tire, TIRE_IN_LINK)),
                                 rho_r, rotor)
    return dict(mass=mass, com=com, inertia=inertia, rho_wheel=rho_w, rho_rotor=rho_r, check=check)


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
    parser.add_argument("--nominal-radius", type=float, default=NOMINAL_RADIUS,
                        help="tire nominal radius [mm]; 55 is the original Upkie wheel")
    parser.add_argument("--export", type=Path, help="write STEP (assembly and parts) and STL (parts) into this folder")
    parser.add_argument("--meshes", type=Path,
                        help="write wheel_hub<suffix>.stl and tire<suffix>.stl in meters (the upkie.xml mesh frames)")
    parser.add_argument("--suffix", help="mesh file suffix (default _r<nominal radius>; none for the 55 mm wheel)")
    parser.add_argument("--inertial", action="store_true", help="print the wheel link's MJCF <inertial>")
    parser.add_argument("--no-reference", action="store_true", help="do not overlay the original STL meshes")
    parser.add_argument("--no-show", action="store_true", help="do not send anything to OCP CAD Viewer")
    args = parser.parse_args()

    tire_params, hub_params = wheel_params(args.nominal_radius)
    hub, tire = make_hub(hub_params), make_tire(tire_params)
    ref_hub = None if args.no_reference else load_reference_mesh("wheel_hub")
    ref_tire = None if args.no_reference else load_reference_mesh("tire")
    report("hub", hub, ref_hub)
    report("tire", tire, ref_tire)
    assembly = make_wheel_assembly(hub, tire)

    if args.meshes:
        resized = abs(args.nominal_radius - NOMINAL_RADIUS) > 1e-9
        suffix = args.suffix if args.suffix is not None else (f"_r{args.nominal_radius:g}" if resized else "")
        args.meshes.mkdir(parents=True, exist_ok=True)
        for name, part in ((f"wheel_hub{suffix}", hub), (f"tire{suffix}", tire)):
            part.val().scale(1e-3).exportStl(str(args.meshes / f"{name}.stl"), tolerance=2e-4, angularTolerance=0.2,
                                              relative=False)  # 0.2 mm chord error
        print(f"meshes: {args.meshes}/wheel_hub{suffix}.stl, tire{suffix}.stl (m)")

    if args.inertial:
        link = link_inertial(tire, hub)
        m, c, inertia = link["check"]
        print(f"densities: hub + tire {link['rho_wheel']:.0f} kg/m^3, rotor {link['rho_rotor']:.0f} kg/m^3; "
              f"55 mm check: mass {m:.4f} kg (upkie.xml {LINK_MASS}), COM z {c[2] * 1e3:.2f} mm "
              f"({LINK_COM[2] * 1e3:.2f}), Ixx {inertia[0, 0]:.4g} ({LINK_INERTIA[0, 0]:.4g}), "
              f"Izz {inertia[2, 2]:.4g} ({LINK_INERTIA[2, 2]:.4g}) kg m^2")
        c, i = link["com"], link["inertia"]
        print(f'<inertial pos="{c[0]:.6g} {c[1]:.6g} {c[2]:.6g}" mass="{link["mass"]:.6g}" fullinertia="{i[0, 0]:.6g} '
              f'{i[1, 1]:.6g} {i[2, 2]:.6g} {i[0, 1]:.6g} {i[0, 2]:.6g} {i[1, 2]:.6g}"/>')

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
