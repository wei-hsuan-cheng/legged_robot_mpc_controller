#!/usr/bin/env python3
"""Render an Upkie run (wb_id_walk_test --record-joints) to MP4 with captions, from its replay and labels.

usage: render_upkie_video.py PREFIX [--out OUT.mp4] [--start T] [--end T] [--speed 1.0] [--fps 30]
                             [--dist 1.25] [--elev -8] [--terrain NAME]
Reads PREFIX_replay.json (GT base pose + joints per sample) and, when present, PREFIX_labels.json written by
tests/upkie_demo.py (terrain, time window, captions, camera). Times are simulation seconds. The states are replayed
into the same scene and rendered offscreen; nothing is re-simulated. The camera follows the base from the side
(azimuth 90 deg) with MuJoCo's default 45 deg vertical field of view.

Needs: mujoco (the version the simulator links, 3.13.0), numpy, Pillow, and ffmpeg on PATH.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mujoco_scene import write_scene  # noqa: E402

W, H = 1280, 720


def font(size):
    for path in ("/System/Library/Fonts/Helvetica.ttc", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        if Path(path).is_file():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default(size=size)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("prefix")
    parser.add_argument("--out", help="default: PREFIX.mp4, or PREFIX_slowmo.mp4 when --speed < 1")
    parser.add_argument("--start", type=float)
    parser.add_argument("--end", type=float)
    parser.add_argument("--speed", type=float, default=1.0, help="playback speed; 0.25 is 4x slow motion")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--dist", type=float, help="camera distance [m] (labels or 1.25)")
    parser.add_argument("--elev", type=float, help="camera elevation [deg] (labels or -8)")
    parser.add_argument("--terrain", help="tests/mujoco_scene.py terrain (labels or none)")
    parser.add_argument("--background", choices=["night", "day"], default="night",
                        help="appearance only: the replay is kinematic (flat scenes are simulated with the day one)")
    args = parser.parse_args()

    samples = json.loads(Path(args.prefix + "_replay.json").read_text())
    labels_path = Path(args.prefix + "_labels.json")
    labels = json.loads(labels_path.read_text()) if labels_path.is_file() else {}
    terrain = args.terrain or labels.get("terrain", "none")
    start = args.start if args.start is not None else labels.get("start", samples[0]["time"])
    end = args.end if args.end is not None else labels.get("end", samples[-1]["time"])
    dist = args.dist or labels.get("camera_distance", 1.25)
    elev = args.elev if args.elev is not None else labels.get("camera_elevation", -8.0)
    out = args.out or args.prefix + ("_slowmo.mp4" if args.speed < 1 else ".mp4")
    phases, subs = labels.get("phases", []), labels.get("subtitles", [])
    label_at = lambda entries, t: next((text for t0, text in reversed(entries) if t >= t0), "")

    with tempfile.TemporaryDirectory() as tmp:
        scene = write_scene(Path(tmp) / "scene.xml", "upkie", terrain)
        if args.background == "night":
            scene.write_text(scene.read_text().replace("background_day.xml", "background_night_upkie.xml"))
        model = mujoco.MjModel.from_xml_path(str(scene))
    model.vis.global_.offwidth, model.vis.global_.offheight = max(model.vis.global_.offwidth, W), max(model.vis.global_.offheight, H)
    data = mujoco.MjData(model)
    times = np.array([s["time"] for s in samples])
    addresses = [model.joint(name).qposadr[0] for name in samples[0]["names"]]
    big, small = font(30), font(24)
    ffmpeg = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
                               "-r", str(args.fps), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", out],
                              stdin=subprocess.PIPE)
    frames = int(round((end - start) / args.speed * args.fps))
    with mujoco.Renderer(model, height=H, width=W) as renderer:
        for k in range(frames + 1):
            t = start + k * args.speed / args.fps
            i = int(np.clip(np.searchsorted(times, t) - 1, 0, len(times) - 2))
            a = float(np.clip((t - times[i]) / (times[i + 1] - times[i]), 0, 1))
            s0, s1 = samples[i], samples[i + 1]
            data.qpos[:3] = (1 - a) * np.array(s0["position"]) + a * np.array(s1["position"])
            q0, q1 = np.array(s0["quaternion_wxyz"]), np.array(s1["quaternion_wxyz"])
            q = (1 - a) * q0 + a * (q1 if q0 @ q1 >= 0 else -q1)
            data.qpos[3:7] = q / np.linalg.norm(q)
            for address, x0, x1 in zip(addresses, s0["q"], s1["q"]):
                data.qpos[address] = (1 - a) * x0 + a * x1
            mujoco.mj_forward(model, data)
            camera = mujoco.MjvCamera()
            camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            camera.lookat[:] = [data.qpos[0], data.qpos[1], 0.2]
            camera.distance, camera.azimuth, camera.elevation = dist, 90.0, elev
            renderer.update_scene(data, camera=camera)
            image = Image.fromarray(renderer.render())
            draw = ImageDraw.Draw(image)
            draw.rectangle([0, 0, W, 96], fill=(0, 0, 0))
            draw.text((20, 10), label_at(phases, t), fill=(255, 230, 80), font=big)
            draw.text((20, 54), label_at(subs, t), fill=(200, 230, 255), font=small)
            draw.rectangle([0, H - 44, W, H], fill=(0, 0, 0))
            info = f"sim t = {t:5.2f} s   base z = {data.qpos[2]:.3f} m" + (f"   ({args.speed:g}x speed)" if args.speed != 1 else "")
            draw.text((20, H - 38), info, fill=(220, 220, 220), font=small)
            ffmpeg.stdin.write(image.tobytes())
    ffmpeg.stdin.close()
    ffmpeg.wait()
    print("wrote", out, frames + 1, "frames")


if __name__ == "__main__":
    main()
