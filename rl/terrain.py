# Copyright (c) 2026, Wei-Hsuan Cheng. All rights reserved.
"""Blind-locomotion terrain for training and evaluation: one MuJoCo heightfield of square tiles,
as in legged_gym. Columns are terrain types, rows are difficulty levels (0 = easiest). Each tile
has a flat platform at its centre to spawn on; the terrain is what the robot meets walking out
from it ("up" types rise away from the centre, "down" types fall away). Slippery ground is the
per-episode friction scale, not a tile type.

  terrain = TerrainMap.training(seed=0)
  scene = terrain.write_scene(out_dir)   # MJCF with the heightfield; floor plane lowered under it
  x, y, z = terrain.spawn(level, column)  # tile-centre platform, world frame
  terrain.height(x, y)                    # ground height for termination and curricula
"""

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

TYPES = ("flat", "rough", "slope_up", "slope_down", "stairs_up", "stairs_down", "discrete", "stepping_stones")
TRAINING_COLUMNS = ("flat", "rough", "rough", "slope_up", "slope_down", "stairs_up", "stairs_down",
                    "discrete", "stepping_stones")


@dataclass
class TerrainConfig:
    tile_size: float = 8.0         # [m]
    resolution: float = 0.05       # [m] heightfield cell
    platform: float = 2.0          # [m] flat spawn square at each tile centre
    border: float = 4.0            # [m] flat margin around the tiles
    levels: int = 8                # difficulty rows
    # Difficulty d in [0, 1] maps linearly onto these (easy, hard) ranges.
    rough_amplitude: tuple = (0.01, 0.05)     # [m] +- of smooth noise on a 0.2 m grid
    slope: tuple = (0.02, 0.17)               # [rad] (1 to 10 deg)
    step_height: tuple = (0.02, 0.10)         # [m]
    step_depth: float = 0.35                  # [m]
    obstacle_height: tuple = (0.02, 0.08)     # [m] +- per block
    stone_size: tuple = (1.0, 0.5)            # [m]
    stone_gap: tuple = (0.05, 0.20)           # [m]
    pit_depth: float = 0.3                    # [m] below the stones
    floor_depth: float = 3.0                  # [m] the scene's floor plane, under everything


def _lerp(r, d):
    return r[0] + (r[1] - r[0]) * d


class TerrainMap:
    """heights[iy, ix] [m] on a regular grid whose cell (0, 0) is at world (x0, y0)."""

    def __init__(self, heights, x0, y0, config, tiles):
        self.heights = heights.astype(np.float64)
        self.x0, self.y0, self.cfg = x0, y0, config
        self.tiles = tiles  # (level, column) -> (type, difficulty, centre x, centre y)

    # -- construction -------------------------------------------------------------------------

    @classmethod
    def training(cls, seed=0, columns=TRAINING_COLUMNS, config=None):
        """Tiles start `border` - 2 m from the world origin, which stays on the flat border: the
        robot is brought up there once and spawned onto tiles from it."""
        cfg = config or TerrainConfig()
        rng = np.random.default_rng(seed)
        n = int(round(cfg.tile_size / cfg.resolution))
        b = int(round(cfg.border / cfg.resolution))
        heights = np.zeros((2 * b + cfg.levels * n, 2 * b + len(columns) * n))
        tiles = {}
        x0 = y0 = -(cfg.border - 2.0)
        for level in range(cfg.levels):
            d = level / max(cfg.levels - 1, 1)
            for column, kind in enumerate(columns):
                iy, ix = b + level * n, b + column * n
                heights[iy:iy + n, ix:ix + n] = make_tile(kind, d, cfg, rng)
                tiles[(level, column)] = (kind, d, x0 + (ix + n / 2) * cfg.resolution, y0 + (iy + n / 2) * cfg.resolution)
        return cls(heights, x0, y0, cfg, tiles)

    @classmethod
    def single(cls, kind, difficulty, seed=0, config=None):
        """One tile centred on the world origin, platform at z = 0, for evaluation through the
        ordinary launch (the robot spawns at the origin)."""
        cfg = config or TerrainConfig()
        rng = np.random.default_rng(seed)
        n = int(round(cfg.tile_size / cfg.resolution))
        b = int(round(cfg.border / cfg.resolution))
        tile = make_tile(kind, difficulty, cfg, rng)
        tile -= tile[n // 2, n // 2]
        heights = np.zeros((2 * b + n, 2 * b + n))
        heights[b:b + n, b:b + n] = tile
        # The border continues the tile's outer edge height so walking off the tile is a step,
        # not a cliff: use the mean edge height.
        edge = np.concatenate([tile[0], tile[-1], tile[:, 0], tile[:, -1]]).mean()
        mask = np.ones_like(heights, bool)
        mask[b:b + n, b:b + n] = False
        heights[mask] = edge
        x0 = y0 = -(b + n / 2) * cfg.resolution
        return cls(heights, x0, y0, cfg, {(0, 0): (kind, difficulty, 0.0, 0.0)})

    # -- queries ------------------------------------------------------------------------------

    def height(self, x, y):
        """Bilinear ground height at world (x, y); outside the map, the floor."""
        fx = (np.asarray(x, float) - self.x0) / self.cfg.resolution
        fy = (np.asarray(y, float) - self.y0) / self.cfg.resolution
        ny, nx = self.heights.shape
        inside = (fx >= 0) & (fx <= nx - 1) & (fy >= 0) & (fy <= ny - 1)
        ix = np.clip(np.floor(fx).astype(int), 0, nx - 2)
        iy = np.clip(np.floor(fy).astype(int), 0, ny - 2)
        tx, ty = np.clip(fx - ix, 0, 1), np.clip(fy - iy, 0, 1)
        h = self.heights
        z = ((1 - tx) * (1 - ty) * h[iy, ix] + tx * (1 - ty) * h[iy, ix + 1]
             + (1 - tx) * ty * h[iy + 1, ix] + tx * ty * h[iy + 1, ix + 1])
        return np.where(inside, z, -self.cfg.floor_depth)

    def spawn(self, level, column):
        """World position of the centre of a tile's platform."""
        _, _, x, y = self.tiles[(level, column)]
        return x, y, float(self.height(x, y))

    @property
    def columns(self):
        return 1 + max(c for _, c in self.tiles)

    # -- MuJoCo -------------------------------------------------------------------------------

    def write_scene(self, out_dir, robot="h2", name="scene_terrain"):
        """The robot's scene with this heightfield and the floor plane lowered under it."""
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests"))
        from mujoco_scene import write_scene

        out_dir = Path(out_dir).resolve()
        out_dir.mkdir(parents=True, exist_ok=True)
        z_min, z_max = float(self.heights.min()), float(self.heights.max())
        span = max(z_max - z_min, 1e-3)
        ny, nx = self.heights.shape
        # MuJoCo's custom heightfield format: int32 nrow, int32 ncol, float32 data, row-major,
        # rows along +y. Normalized here to [0, 1] so MuJoCo's own normalization is a no-op.
        data = ((self.heights - z_min) / span).astype(np.float32)
        hfield_file = out_dir / f"{name}.bin"
        with open(hfield_file, "wb") as f:
            np.array([ny, nx], np.int32).tofile(f)
            data.tofile(f)

        include = ET.Element("mujoco", model="terrain")
        asset = ET.SubElement(include, "asset")
        half_x, half_y = 0.5 * (nx - 1) * self.cfg.resolution, 0.5 * (ny - 1) * self.cfg.resolution
        ET.SubElement(asset, "hfield", name="terrain", file=str(hfield_file),
                      size=f"{half_x} {half_y} {span} 1.0")
        world = ET.SubElement(include, "worldbody")
        ET.SubElement(world, "geom", name="terrain", type="hfield", hfield="terrain",
                      pos=f"{self.x0 + half_x} {self.y0 + half_y} {z_min}", priority="1",
                      friction="0.8 0.1 0.001", rgba="0.55 0.55 0.6 1")
        include_file = out_dir / f"{name}_hfield.xml"
        ET.ElementTree(include).write(include_file, encoding="unicode")

        scene = write_scene(out_dir / f"{name}.xml", robot)
        root = ET.parse(scene).getroot()
        for element in root.findall("include"):
            path = Path(element.get("file"))
            if path.name.startswith("background_"):
                background = ET.parse(path).getroot()
                for nested in background.iter("include"):  # relative to the original's directory
                    nested.set("file", str((path.parent / nested.get("file")).resolve()))
                for geom in background.iter("geom"):
                    if geom.get("type") == "plane":
                        geom.set("pos", f"0 0 {-self.cfg.floor_depth}")
                copy = out_dir / f"{name}_{path.name}"
                ET.ElementTree(background).write(copy, encoding="unicode")
                element.set("file", str(copy))
        ET.SubElement(root, "include", file=str(include_file))
        ET.ElementTree(root).write(scene, encoding="unicode")
        return scene


def make_tile(kind, d, cfg, rng):
    """Heights of one tile [m]: outer edge at 0; the platform at 0 too except for the pyramids."""
    n = int(round(cfg.tile_size / cfg.resolution))
    r = cfg.resolution
    c = (np.arange(n) + 0.5) * r - cfg.tile_size / 2
    gx, gy = np.meshgrid(c, c)
    cheb = np.maximum(np.abs(gx), np.abs(gy))            # pyramid distance from the centre
    outside = np.maximum(cheb - cfg.platform / 2, 0.0)   # 0 on the platform
    on_platform = cheb <= cfg.platform / 2
    if kind == "flat":
        return np.zeros((n, n))
    if kind == "rough":
        # Smooth bumps: uniform noise on a 0.2 m grid, bilinearly interpolated.
        a = _lerp(cfg.rough_amplitude, d)
        coarse = int(round(cfg.tile_size / 0.2)) + 1
        noise = rng.uniform(-a, a, (coarse, coarse))
        cc = np.linspace(-cfg.tile_size / 2, cfg.tile_size / 2, coarse)
        h = _bilinear(noise, cc, gx, gy)
        return np.where(on_platform, 0.0, h)
    # Pyramids are shifted so the tile's outer edge is at 0, as in legged_gym: neighbouring tiles
    # then meet without a cliff, and the centre platform sits in a pit ("up") or on top ("down").
    if kind in ("slope_up", "slope_down"):
        sign = 1.0 if kind == "slope_up" else -1.0
        h = sign * np.tan(_lerp(cfg.slope, d)) * outside
        return h - _edge(h)
    if kind in ("stairs_up", "stairs_down"):
        sign = 1.0 if kind == "stairs_up" else -1.0
        h = sign * _lerp(cfg.step_height, d) * np.ceil(outside / cfg.step_depth)
        return h - _edge(h)
    if kind == "discrete":
        h = np.zeros((n, n))
        height = _lerp(cfg.obstacle_height, d)
        for _ in range(40):
            w, l = rng.uniform(0.4, 1.0, 2)
            cx, cy = rng.uniform(-cfg.tile_size / 2, cfg.tile_size / 2, 2)
            block = (np.abs(gx - cx) < w / 2) & (np.abs(gy - cy) < l / 2)
            h[block] = rng.choice([-1.0, 1.0]) * rng.uniform(0.5, 1.0) * height
        return np.where(on_platform, 0.0, h)
    if kind == "stepping_stones":
        size, gap = _lerp(cfg.stone_size, d), _lerp(cfg.stone_gap, d)
        pitch = size + gap
        # Stones on a grid offset so the platform's edge meets one; the gaps are pits.
        sx = np.mod(gx + cfg.platform / 2, pitch)
        sy = np.mod(gy + cfg.platform / 2, pitch)
        stone = (sx < size) & (sy < size)
        h = np.where(stone, rng.uniform(-0.02, 0.02) * d, -cfg.pit_depth)
        return np.where(on_platform, 0.0, h)
    raise ValueError(f"unknown terrain type {kind}")


def _edge(h):
    return float(np.concatenate([h[0], h[-1], h[:, 0], h[:, -1]]).mean())


def _bilinear(grid, coords, gx, gy):
    step = coords[1] - coords[0]
    fx, fy = (gx - coords[0]) / step, (gy - coords[0]) / step
    ix = np.clip(np.floor(fx).astype(int), 0, len(coords) - 2)
    iy = np.clip(np.floor(fy).astype(int), 0, len(coords) - 2)
    tx, ty = fx - ix, fy - iy
    return ((1 - tx) * (1 - ty) * grid[iy, ix] + tx * (1 - ty) * grid[iy, ix + 1]
            + (1 - tx) * ty * grid[iy + 1, ix] + tx * ty * grid[iy + 1, ix + 1])
