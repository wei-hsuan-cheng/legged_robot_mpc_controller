#!/usr/bin/env python3
"""Render frames of a wb_id_walk_test replay (GT base pose + joints, --record-joints) into one contact sheet.

usage: render_upkie_replay.py SCENE.xml PREFIX_replay.json OUT.png t1 t2 ... [--cam=side|front|top]
Times are the replay's sim time (about 2 s ahead of "time since settle" in the tracking CSV).
SCENE must resolve its includes on this machine, e.g. a copy of the run's scene with local paths.
Use the MuJoCo version the simulator links (3.13.0): collision handling differs between versions.
"""
import json, sys, numpy as np, mujoco
from PIL import Image, ImageDraw
args = [a for a in sys.argv[1:] if not a.startswith("--")]
cam_kind = next((a.split("=")[1] for a in sys.argv[1:] if a.startswith("--cam=")), "side")
scene, replay, out, times = args[0], args[1], args[2], [float(t) for t in args[3:]]
m = mujoco.MjModel.from_xml_path(scene); d = mujoco.MjData(m)
samples = json.load(open(replay)); ts = np.array([s["time"] for s in samples])
frames = []
with mujoco.Renderer(m, height=420, width=560) as r:
    for t in times:
        s = samples[int(np.argmin(abs(ts - t)))]
        d.qpos[:3] = s["position"]; d.qpos[3:7] = s["quaternion_wxyz"]
        for name, q in zip(s["names"], s["q"]):
            d.qpos[m.joint(name).qposadr[0]] = q
        mujoco.mj_forward(m, d)
        cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = [d.qpos[0], d.qpos[1], 0.15]; cam.distance = 0.9
        cam.azimuth, cam.elevation = {"side": (90, -10), "front": (180, -10), "top": (90, -80)}[cam_kind]
        r.update_scene(d, camera=cam)
        img = Image.fromarray(r.render()); ImageDraw.Draw(img).text((8, 8), f"t={s['time']:.2f}s {s['phase']}", fill=(255, 255, 0))
        frames.append(img)
cols = min(4, len(frames)); rows = (len(frames) + cols - 1) // cols
sheet = Image.new("RGB", (560 * cols, 420 * rows))
for i, f in enumerate(frames): sheet.paste(f, ((i % cols) * 560, (i // cols) * 420))
sheet.save(out); print("wrote", out)
