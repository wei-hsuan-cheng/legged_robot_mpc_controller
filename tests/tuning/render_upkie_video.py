#!/usr/bin/env python3
"""Render an Upkie or Pineapple run (wb_id_walk_test --record-joints) to MP4 with captions, from its replay and labels.

usage: render_upkie_video.py PREFIX [--out OUT.mp4] [--start T] [--end T] [--speed 1.0] [--fps 30]
                             [--dist 1.25] [--elev -8] [--terrain NAME] [--no-text]
                             [--follow] [--fovy 45] [--size 1280x720] [--rviz-frames DIR]
                             [--robot upkie|pineapple] [--azimuth 90] [--follow-z -0.1]
Reads PREFIX_replay.json (GT base pose + joints per sample) and, when present, PREFIX_labels.json written by
tests/upkie_demo.py (terrain, time window, captions, camera). Times are simulation seconds. The states are replayed
into the same scene and rendered offscreen; nothing is re-simulated. The camera follows the base from the side
(azimuth 90 deg) with MuJoCo's default 45 deg vertical field of view; labels may move its distance and elevation over
time (camera_keys: [[t, distance, elevation], ...], linear in between). --no-text renders the 3D view only.
--follow uses config/rviz/upkie_video.rviz's camera instead: focal point 0.1 m below the base origin, 1.6 m away, 14 deg
above, from world -y. --rviz-frames stacks the frames of tests/tuning/render_rviz_replay.py (same sim times) on top,
rendering MuJoCo at RViz's render-area size (DIR/rviz_size.json) with the same vertical field of view (--fovy).
Pineapple's RViz view is config/rviz/pineapple_video.rviz: render it with --robot pineapple --azimuth 146 --elev -17
--dist 2.0 --follow-z -0.15 (MuJoCo's azimuth is the RViz Orbit yaw + 180 deg, its elevation minus the pitch).

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
FOLLOW_Z, FOLLOW_DISTANCE, FOLLOW_ELEVATION = -0.1, 1.6, -14.0  # config/rviz/upkie_video.rviz's Orbit view


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
    parser.add_argument("--no-text", action="store_true", help="3D view only: no captions, bars or info line")
    parser.add_argument("--follow", action="store_true", help="camera of config/rviz/upkie_video.rviz (see above)")
    parser.add_argument("--fovy", type=float, help="vertical field of view [deg] (MuJoCo's default 45)")
    parser.add_argument("--size", help="render size WxH (default 1280x720, or RViz's with --rviz-frames)")
    parser.add_argument("--rviz-frames", type=Path, help="stack DIR/rviz_%%05d.png (render_rviz_replay.py) on top")
    parser.add_argument("--robot", choices=["upkie", "pineapple", "pineapple_manipulator"], default="upkie")
    parser.add_argument("--azimuth", type=float, default=90.0, help="camera azimuth [deg]; 90 looks along +y (from -y)")
    parser.add_argument("--follow-z", type=float, default=FOLLOW_Z, help="--follow: focal point below the base origin [m]")
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
    keys = [] if args.dist or args.elev is not None else labels.get("camera_keys", [])
    if args.rviz_frames:
        args.follow = True
    if args.follow:
        keys = []
        dist, elev = args.dist or FOLLOW_DISTANCE, args.elev if args.elev is not None else FOLLOW_ELEVATION
    rw, rh = W, H
    if args.size:
        rw, rh = map(int, args.size.lower().split("x"))
    elif args.rviz_frames:
        size = json.loads((args.rviz_frames / "rviz_size.json").read_text())
        rw, rh = size["width"], size["height"]
    row_h = 2 * round(W * rh / rw / 2)
    out_h = 2 * row_h if args.rviz_frames else row_h

    def camera_at(t):
        if not keys:
            return dist, elev
        times = [k[0] for k in keys]
        return (float(np.interp(t, times, [k[1] for k in keys])), float(np.interp(t, times, [k[2] for k in keys])))

    with tempfile.TemporaryDirectory() as tmp:
        scene = write_scene(Path(tmp) / "scene.xml", args.robot, terrain)
        if args.background == "night":
            scene.write_text(scene.read_text().replace("background_day.xml", f"background_night_{args.robot}.xml"))
        model = mujoco.MjModel.from_xml_path(str(scene))
    model.vis.global_.offwidth, model.vis.global_.offheight = max(model.vis.global_.offwidth, rw), max(model.vis.global_.offheight, rh)
    if args.fovy:
        model.vis.global_.fovy = args.fovy
    data = mujoco.MjData(model)
    times = np.array([s["time"] for s in samples])
    addresses = [model.joint(name).qposadr[0] for name in samples[0]["names"]]
    big, small = font(30), font(24)
    ffmpeg = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{out_h}",
                               "-r", str(args.fps), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", out],
                              stdin=subprocess.PIPE)
    frames = int(round((end - start) / args.speed * args.fps))
    with mujoco.Renderer(model, height=rh, width=rw) as renderer:
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
            camera.lookat[:] = [data.qpos[0], data.qpos[1], data.qpos[2] + args.follow_z if args.follow else 0.2]
            camera.distance, camera.elevation = camera_at(t)
            camera.azimuth = args.azimuth
            renderer.update_scene(data, camera=camera)
            image = Image.fromarray(renderer.render()).resize((W, row_h), Image.LANCZOS)
            if args.rviz_frames:
                top = Image.open(args.rviz_frames / f"rviz_{k:05d}.png").convert("RGB").resize((W, row_h), Image.LANCZOS)
                stacked = Image.new("RGB", (W, out_h))
                stacked.paste(top, (0, 0))
                stacked.paste(image, (0, row_h))
                image = stacked
            if args.no_text:
                ffmpeg.stdin.write(image.tobytes())
                continue
            draw = ImageDraw.Draw(image)
            draw.rectangle([0, 0, W, 96], fill=(0, 0, 0))
            draw.text((20, 10), label_at(phases, t), fill=(255, 230, 80), font=big)
            draw.text((20, 54), label_at(subs, t), fill=(200, 230, 255), font=small)
            if args.rviz_frames:
                draw.text((20, row_h - 34), "RViz: estimated state, MPC optimized trajectory and future ghosts",
                          fill=(220, 220, 220), font=small)
                draw.text((20, row_h + 10), "MuJoCo simulation", fill=(220, 220, 220), font=small)
            H_ = out_h
            draw.rectangle([0, H_ - 44, W, H_], fill=(0, 0, 0))
            info = f"sim t = {t:5.2f} s   base z = {data.qpos[2]:.3f} m" + (f"   ({args.speed:g}x speed)" if args.speed != 1 else "")
            draw.text((20, H_ - 38), info, fill=(220, 220, 220), font=small)
            ffmpeg.stdin.write(image.tobytes())
    ffmpeg.stdin.close()
    ffmpeg.wait()
    print("wrote", out, frames + 1, "frames")


if __name__ == "__main__":
    main()
