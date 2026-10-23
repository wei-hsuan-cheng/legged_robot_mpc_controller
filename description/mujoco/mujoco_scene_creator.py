#!/usr/bin/env python3
"""Generate modular G1 MuJoCo scenes and reusable studio backgrounds.

Scene entry points include a robot, one background and optional terrain modules.
common/backgrounds/background_day.xml keeps the original daytime appearance; background_night.xml
owns studio lighting, camera settings and the satin floor. Both backgrounds load
common/materials/terrain_materials.xml once, providing textures/studio_terrain.png to stairs, ramps,
slopes and boxes. Geometry/contact definitions stay in their own modules, so terrain
edits immediately reach every scene without copying or regenerating geometry.

Usage (paths are relative to description/mujoco unless absolute):
    python3 description/mujoco/mujoco_scene_creator.py
    python3 description/mujoco/mujoco_scene_creator.py --terrain none -o scene_flat.xml
    python3 description/mujoco/mujoco_scene_creator.py --reference scene_ramp.xml
    python3 description/mujoco/mujoco_scene_creator.py --background day --terrain ramp -o scene_day_ramp.xml
    python3 description/mujoco/mujoco_scene_creator.py --style studio_blue -o scene_blue.xml
    python3 description/mujoco/mujoco_scene_creator.py --preview /tmp/studio.png

The default run updates the shared common/backgrounds/background_night.xml. Appearance overrides write
an output-specific background module. Preview rendering needs the mujoco Python
package; XML/texture generation uses only the standard library. Select the scene
with g1.launch.py mujocoModelFile:=scene.xml.
"""

import argparse
import copy
import dataclasses
import math
import os
import random
import struct
import xml.etree.ElementTree as ET
import zlib

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_SCENE_DIR = SCRIPT_DIR

# Environment includes known in description/mujoco. "none" drops them.
TERRAIN_FILES = {
    "stairs": "terrains/stairs.xml",
    "boxes": "objects/boxes.xml",
    "scattered_boxes": "objects/boxes_scattered_50.xml",
    "slope": "terrains/slope.xml",
    "ramp": "terrains/ramp.xml",
    "rough": "terrains/terrain_10x10.xml",
}

RGB = tuple  # (r, g, b) in [0, 1]


@dataclasses.dataclass(frozen=True)
class Light:
  """A light placed relative to the viewer, aimed at the target body's CoM.

  bearing 0 is the camera's side of the robot, 180 is straight behind the robot as seen
  from the camera, positive turns counter-clockwise seen from above. Placing lights
  this way keeps the look when the camera azimuth changes.
  """
  bearing: float             # [deg]
  elevation: float           # above the horizon [deg]
  distance: float            # from the robot spawn point [m]
  diffuse: float
  specular: float
  cutoff: float = 60.0       # spot half-angle [deg]
  exponent: float = 6.0      # intensity ~ cos(angle)^exponent: sets the size of the pool on the floor
  castshadow: bool = False
  directional: bool = False  # parallel rays, with no spotlight pool or distance falloff

  def position(self, camera_azimuth):
    # The free camera looks along azimuth, so it sits on the opposite side of its lookat.
    heading = math.radians(camera_azimuth + 180.0 + self.bearing)
    el = math.radians(self.elevation)
    return (self.distance * math.cos(el) * math.cos(heading),
            self.distance * math.cos(el) * math.sin(heading),
            self.distance * math.sin(el))


@dataclasses.dataclass(frozen=True)
class Style:
  description: str
  background: RGB
  # Floor texture: one tile is one grid cell, with a "+" mark at every grid point.
  floor_base: RGB
  floor_line: RGB
  floor_mark: RGB
  cell: float = 0.5                  # grid spacing [m]
  line_width: float = 0.004          # [m]
  mark_half_length: float = 0.05     # arm length of the "+" [m]
  mark_thickness: float = 0.008      # [m]
  texture_px: int = 512              # tile resolution
  floor_reflectance: float = 0.08    # mirror strength; MuJoCo renders it for planes only
  floor_specular: float = 0.15
  floor_shininess: float = 0.2
  terrain_base: RGB = (0.42, 0.44, 0.46)
  # Robot finish, via the default geom class. None keeps the plain look.
  robot_specular: float = 1.0
  robot_shininess: float = 0.8
  headlight_ambient: float = 0.22
  headlight_diffuse: float = 0.28    # fill shadowed faces without a concentrated light pool
  headlight_specular: float = 0.1
  lights: tuple = ()
  # Viewer start pose. mujoco_ros2_control looks at `center` from 2 * extent.
  azimuth: float = -90.0            # side view from the robot's left: terrain ahead (+x) sits beside it
  elevation: float = -10.0
  center: tuple = (0.5, 0.0, 0.5)
  extent: float = 1.6


STUDIO_LIGHTS = (
    # Parallel rays illuminate the whole terrain instead of a bright spotlight pool.
    Light(bearing=30.0, elevation=70.0, distance=5.5, diffuse=0.45, specular=0.18,
          castshadow=True, directional=True),
    # Broad non-shadowing fill reduces contrast on risers and the robot's far side.
    Light(bearing=165.0, elevation=35.0, distance=4.0, diffuse=0.45, specular=0.12,
          directional=True),
)

STYLES = {
    "studio": Style(
        description="Dark studio: black void, satin floor, white + marks, matte terrain, glossy robot",
        background=(0.0, 0.0, 0.0),
        floor_base=(0.050, 0.052, 0.058),
        floor_line=(0.090, 0.093, 0.100),
        floor_mark=(0.92, 0.92, 0.92),
        lights=STUDIO_LIGHTS,
    ),
    "studio_blue": Style(
        description="the same studio with a deep navy floor and pale blue marks",
        background=(0.0, 0.0, 0.0),
        floor_base=(0.020, 0.040, 0.085),
        floor_line=(0.045, 0.080, 0.150),
        floor_mark=(0.70, 0.82, 1.00),
        lights=STUDIO_LIGHTS,
    ),
}


# ----------------------------------------------------------------------------- PNG

def write_png(path, width, height, rows):
  """Write 8-bit RGB rows (bytes of length 3 * width each) as a PNG, stdlib only."""
  def chunk(tag, data):
    return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

  raw = b"".join(b"\x00" + row for row in rows)
  with open(path, "wb") as f:
    f.write(b"\x89PNG\r\n\x1a\n")
    f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)))
    f.write(chunk(b"IDAT", zlib.compress(raw, 9)))
    f.write(chunk(b"IEND", b""))


def floor_texture_rows(style):
  """One grid cell: faint lines on the cell border, a "+" centred on the corner.

  The mark sits on the tile corner, so with texuniform one lands on the world origin
  where the robot spawns. Edges are anti-aliased by pixel coverage.
  """
  n = style.texture_px
  px_per_m = n / style.cell

  def coverage(half_width_px, dist_px):
    return min(1.0, max(0.0, half_width_px - dist_px + 0.5))

  # Distance from each pixel centre to the nearest tile edge (wraps, so the corner is one point).
  dist = [min(i + 0.5, n - (i + 0.5)) for i in range(n)]
  line = [coverage(0.5 * style.line_width * px_per_m, d) for d in dist]
  thick = [coverage(0.5 * style.mark_thickness * px_per_m, d) for d in dist]
  length = [coverage(style.mark_half_length * px_per_m, d) for d in dist]

  base, grid, mark = style.floor_base, style.floor_line, style.floor_mark
  rows = []
  for y in range(n):
    row = bytearray(3 * n)
    for x in range(n):
      a_line = max(line[x], line[y])
      a_mark = max(thick[y] * length[x], thick[x] * length[y])
      for c in range(3):
        v = base[c] + (grid[c] - base[c]) * a_line
        v += (mark[c] - v) * a_mark
        row[3 * x + c] = int(round(255.0 * v))
    rows.append(bytes(row))
  return rows


def terrain_texture_rows(style):
  """Seamless fine aggregate with restrained variation, without collision geometry."""
  rng = random.Random(7)
  n = style.texture_px
  rows = []
  for y in range(n):
    row = bytearray(3 * n)
    for x in range(n):
      grain = rng.uniform(-0.035, 0.035)
      grain += 0.012 * math.sin(2 * math.pi * x / n) * math.cos(4 * math.pi * y / n)
      for c, base in enumerate(style.terrain_base):
        row[3 * x + c] = round(255 * min(1.0, max(0.0, base + grain)))
    rows.append(bytes(row))
  return rows


# ----------------------------------------------------------------------------- scene

def fmt(values):
  return " ".join(f"{round(v, 3) + 0.0:g}" for v in values)


def resolve(path, base_dir):
  return path if os.path.isabs(path) else os.path.join(base_dir, path)


def build_background(style, style_name, texture_rel, target_body, scene_dir):
  root = ET.Element("mujoco", model="background_night" if style_name == "studio" else f"background_{style_name}")
  root.append(ET.Comment(" Generated by description/mujoco/mujoco_scene_creator.py; shared by scene entry points. "))
  ET.SubElement(root, "include", file="../materials/terrain_materials.xml")
  if style.robot_specular is not None:
    ET.SubElement(ET.SubElement(root, "default"), "geom", material="studio_finish")
  visual = ET.SubElement(root, "visual")
  ET.SubElement(visual, "headlight", ambient=fmt([style.headlight_ambient] * 3),
                diffuse=fmt([style.headlight_diffuse] * 3), specular=fmt([style.headlight_specular] * 3))
  ET.SubElement(visual, "rgba", haze=fmt(style.background + (1.0,)), fog=fmt(style.background + (1.0,)))
  ET.SubElement(visual, "quality", shadowsize="4096", offsamples="8")
  ET.SubElement(visual, "global", azimuth=f"{style.azimuth:g}", elevation=f"{style.elevation:g}",
                offwidth="1920", offheight="1080")
  ET.SubElement(root, "statistic", center=fmt(style.center), extent=f"{style.extent:g}")
  asset = ET.SubElement(root, "asset")
  ET.SubElement(asset, "texture", type="skybox", builtin="flat", rgb1=fmt(style.background),
                rgb2=fmt(style.background), width="32", height="32")
  ET.SubElement(asset, "texture", type="2d", name="studio_floor", file=texture_rel)
  ET.SubElement(asset, "material", name="studio_floor", texture="studio_floor", texuniform="true",
                texrepeat=fmt([1.0 / style.cell] * 2), rgba="1 1 1 1",
                reflectance=f"{style.floor_reflectance:g}", specular=f"{style.floor_specular:g}",
                shininess=f"{style.floor_shininess:g}")
  if style.robot_specular is not None:
    ET.SubElement(asset, "material", name="studio_finish", specular=f"{style.robot_specular:g}",
                  shininess=f"{style.robot_shininess:g}")
  world = ET.SubElement(root, "worldbody")
  for i, light in enumerate(style.lights):
    ET.SubElement(world, "light", name=f"studio_light_{i}", mode="targetbodycom", target=target_body,
                  pos=fmt(light.position(style.azimuth)), diffuse=fmt([light.diffuse] * 3),
                  specular=fmt([light.specular] * 3), cutoff=f"{light.cutoff:g}", exponent=f"{light.exponent:g}",
                  directional="true" if light.directional else "false",
                  castshadow="true" if light.castshadow else "false")
  # Both backgrounds use exactly the same physical floor; only its material differs.
  floor = copy.deepcopy(ET.parse(os.path.join(scene_dir, "common", "backgrounds", "background_day.xml")).find("./worldbody/geom"))
  floor.set("material", "studio_floor")
  world.append(floor)
  ET.indent(root, space="  ")
  return ET.ElementTree(root)


def build_scene(reference_path, background, robot, terrain, command):
  ref = ET.parse(reference_path).getroot()
  root = ET.Element("mujoco", model=ref.get("model", "scene"))
  root.append(ET.Comment(f" Generated scene composition; options: {command} "))
  found_background = False
  found_robot = False
  terrain_files = {os.path.basename(path) for path in TERRAIN_FILES.values()}
  for elem in ref:
    elem = copy.deepcopy(elem)
    if elem.tag == "include":
      name = os.path.basename(elem.get("file", ""))
      if name.startswith("background_"):
        elem.set("file", background)
        found_background = True
      elif name in terrain_files:
        if terrain is not None:
          continue
      elif not found_robot:
        if robot is not None:
          elem.set("file", robot)
        found_robot = True
    root.append(elem)
  if not found_background or not found_robot:
    raise SystemExit("reference must be a modular scene including a robot and background_*.xml")
  if terrain is not None:
    for t in terrain:
      if t != "none":
        ET.SubElement(root, "include", file=TERRAIN_FILES[t])
  ET.indent(root, space="  ")
  return ET.ElementTree(root)


# ----------------------------------------------------------------------------- preview

def render_preview(scene_path, out_path, width, height):
  try:
    import mujoco
  except ImportError:
    raise SystemExit("--preview needs the mujoco Python package (pip install mujoco)")

  model = mujoco.MjModel.from_xml_path(scene_path)
  model.vis.global_.offwidth = max(model.vis.global_.offwidth, width)
  model.vis.global_.offheight = max(model.vis.global_.offheight, height)
  data = mujoco.MjData(model)
  mujoco.mj_forward(model, data)

  # Same start pose as mujoco_ros2_control's viewer (mujoco_rendering.cpp).
  cam = mujoco.MjvCamera()
  cam.type = mujoco.mjtCamera.mjCAMERA_FREE
  cam.azimuth = model.vis.global_.azimuth
  cam.elevation = model.vis.global_.elevation
  cam.lookat[:] = model.stat.center
  cam.distance = 2.0 * model.stat.extent

  with mujoco.Renderer(model, height, width) as renderer:
    renderer.update_scene(data, camera=cam)
    pixels = renderer.render()
  write_png(out_path, width, height, [pixels[r].tobytes() for r in range(height)])


# ----------------------------------------------------------------------------- main

def main():
  parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                   formatter_class=argparse.RawDescriptionHelpFormatter,
                                   epilog="Run with --list to see the styles.")
  parser.add_argument("--style", default="studio", choices=sorted(STYLES))
  parser.add_argument("--background", default="night", choices=("day", "night"),
                      help="shared day/night background module (default: night)")
  parser.add_argument("-o", "--output", default=None, help="scene file to write (default: update the reference scene)")
  parser.add_argument("--reference", default="scene.xml",
                      help="modular scene whose robot, terrain and options are copied (default: scene.xml)")
  parser.add_argument("--robot", default=None, help="replace the robot include (the reference's first include)")
  parser.add_argument("--terrain", nargs="+", default=None, choices=sorted(TERRAIN_FILES) + ["none"],
                      help="replace the reference's terrain includes (default: keep them)")
  parser.add_argument("--target-body", default="pelvis", help="body the lights aim at")
  parser.add_argument("--cell", type=float, default=None, help="grid spacing [m]")
  parser.add_argument("--reflectance", type=float, default=None, help="floor mirror strength, 0..1")
  parser.add_argument("--azimuth", type=float, default=None, help="viewer start azimuth [deg]; lights follow it")
  parser.add_argument("--elevation", type=float, default=None, help="viewer start elevation [deg]")
  parser.add_argument("--matte-robot", action="store_true", help="keep the robot's plain finish")
  parser.add_argument("--scene-dir", default=DEFAULT_SCENE_DIR, help="directory for relative paths")
  parser.add_argument("--preview", default=None, metavar="PNG", help="also render the viewer's start view")
  parser.add_argument("--preview-size", default="1200x900", metavar="WxH", help="default: the viewer window")
  parser.add_argument("--list", action="store_true", help="list the styles and exit")
  args = parser.parse_args()

  if args.list:
    for name, style in STYLES.items():
      print(f"{name:12s} {style.description}")
    return

  style = STYLES[args.style]
  overrides = {}
  if args.cell is not None:
    overrides["cell"] = args.cell
  if args.reflectance is not None:
    overrides["floor_reflectance"] = args.reflectance
  if args.azimuth is not None:
    overrides["azimuth"] = args.azimuth
  if args.elevation is not None:
    overrides["elevation"] = args.elevation
  if args.matte_robot:
    overrides["robot_specular"] = None
  style = dataclasses.replace(style, **overrides)

  output = resolve(args.output or args.reference, args.scene_dir)
  reference = resolve(args.reference, args.scene_dir)
  out_dir = os.path.dirname(os.path.abspath(output))
  if out_dir != os.path.dirname(os.path.abspath(reference)):
    # Asset paths and compiler directories resolve against the top-level scene.
    raise SystemExit(f"write the scene next to {reference}: relative includes and mesh paths resolve from there")

  # Texture name carries the overrides that change it, so two scenes never share a stale tile.
  texture_name = f"{args.style}_floor.png" if style.cell == STYLES[args.style].cell \
      else f"{args.style}_floor_{style.cell:g}m.png"
  texture_rel = os.path.join("textures", texture_name)
  os.makedirs(os.path.join(out_dir, "textures"), exist_ok=True)
  write_png(os.path.join(out_dir, texture_rel), style.texture_px, style.texture_px, floor_texture_rows(style))
  terrain_texture = os.path.join(out_dir, "textures", "studio_terrain.png")
  write_png(terrain_texture, style.texture_px, style.texture_px, terrain_texture_rows(style))

  # XML comments cannot contain "--", so the options are recorded as key=value.
  defaults = vars(parser.parse_args([]))
  command = " ".join(f"{k}={' '.join(v) if isinstance(v, list) else v}" for k, v in vars(args).items()
                     if v != defaults[k] and k not in ("scene_dir", "preview", "preview_size")) or "(defaults)"
  background = os.path.join("common", "backgrounds", f"background_{args.background}.xml")
  if args.background == "night":
    if args.style != "studio" or overrides or args.target_body != "pelvis":
      background = os.path.join("common", "backgrounds", f"background_night_{os.path.splitext(os.path.basename(output))[0]}.xml")
    background_path = os.path.join(out_dir, background)
    os.makedirs(os.path.dirname(background_path), exist_ok=True)
    build_background(style, args.style, texture_rel, args.target_body, out_dir).write(
        background_path, encoding="unicode", xml_declaration=False)
    print(f"wrote {background_path}")
  tree = build_scene(reference, background, args.robot, args.terrain, command)
  tree.write(output, encoding="unicode", xml_declaration=False)
  with open(output, "a") as f:
    f.write("\n")
  print(f"wrote {output}")
  print(f"wrote {os.path.join(out_dir, texture_rel)}")
  print(f"wrote {terrain_texture}")

  if args.preview:
    w, h = (int(v) for v in args.preview_size.lower().split("x"))
    render_preview(output, args.preview, w, h)
    print(f"wrote {args.preview}")


if __name__ == "__main__":
  main()
