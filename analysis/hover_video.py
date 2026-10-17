#!/usr/bin/env python3
"""Side-by-side video of the recorded hover torque test.

    python3 analysis/hover_video.py [runs/hover_video]

Reads <root>/{pid,indi}_{1p44,3p24}/rec/ (frames/<sim_ms>.jpg, pose.csv,
events.csv from sim/record_camera.py) and writes, in <root>/:
  hover_side_by_side.mp4   2 x 2: rows 1.44 / 3.24 N.m, columns PID / INDI
  hover_1p44.mp4, hover_3p24.mp4   PID | INDI for one torque size
All runs are aligned at the start of the roll push (t = 0). Each tile shows
the camera view, a banner while a torque is on, roll / pitch deviation from
the hover attitude before the push and the distance from the origin (Gazebo
ground truth), and a strip chart of roll and pitch with a cursor.
Encoding uses GStreamer (openh264enc, mp4mux).
"""
import csv
import math
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

FPS = 30
PRE, POST = 5.0, 12.0                 # s before the roll push / after the pitch push ends
TILE_W, TILE_H, STRIP_H = 880, 495, 150
HEADER_H = 56
COLORS = {"pid": (42, 120, 214), "indi": (235, 104, 52)}
PITCH_COLOR = (82, 81, 78)
TORQUE_RED = (208, 59, 59)
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_B = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
YLIM = 8.0                            # strip chart range, +-deg


def euler_frd(qw, qx, qy, qz):
    """Roll and pitch (deg, FRD sign) from the ENU/FLU odometry quaternion."""
    roll = math.degrees(math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy)))
    pitch_flu = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx)))))
    return roll, -pitch_flu


class Run:
    def __init__(self, rd):
        rec = rd / "rec"
        self.frames = sorted(rec.glob("frames/*.jpg"))
        self.ft = np.array([int(p.stem) * 1e-3 for p in self.frames])
        rows = list(csv.DictReader(open(rec / "pose.csv")))
        self.pt = np.array([float(r["sim_s"]) for r in rows])
        rp = np.array([euler_frd(*(float(r[k]) for k in ("qw", "qx", "qy", "qz"))) for r in rows])
        self.x = np.array([float(r["x"]) for r in rows])
        self.y = np.array([float(r["y"]) for r in rows])
        ev = list(csv.DictReader(open(rec / "events.csv")))
        on = [float(e["sim_s"]) for e in ev if e["event"].startswith("torque_on")]
        off = [float(e["sim_s"]) for e in ev if e["event"].startswith("torque_off")]
        if len(on) < 2 or len(off) < 2:
            raise SystemExit("%s: expected two pushes in events.csv" % rd)
        self.t0 = on[0]
        self.pushes = [("ROLL", on[0] - self.t0, off[0] - self.t0), ("PITCH", on[1] - self.t0, off[1] - self.t0)]
        pre = (self.pt >= self.t0 - 3) & (self.pt < self.t0)
        self.roll = rp[:, 0] - rp[pre, 0].mean()
        self.pitch = rp[:, 1] - rp[pre, 1].mean()
        self.x0, self.y0 = 0.0, 0.0                       # QLOITER target = take-off point
        self.rel = self.pt - self.t0

    def frame_at(self, tau):
        i = int(np.argmin(np.abs(self.ft - (self.t0 + tau))))
        return Image.open(self.frames[i]).convert("RGB")

    def state_at(self, tau):
        r = float(np.interp(tau, self.rel, self.roll))
        p = float(np.interp(tau, self.rel, self.pitch))
        d = float(np.hypot(np.interp(tau, self.rel, self.x) - self.x0, np.interp(tau, self.rel, self.y) - self.y0))
        return r, p, d

    def torque_at(self, tau):
        return next((name for name, a, b in self.pushes if a <= tau < b), None)


def fonts():
    return {s: ImageFont.truetype(FONT, s) for s in (18, 22, 26)} | {"b26": ImageFont.truetype(FONT_B, 26),
                                                                     "b30": ImageFont.truetype(FONT_B, 30)}


def strip(run, tau, t_end, color, F):
    """Strip chart of roll and pitch deviation over the whole window, with a cursor."""
    im = Image.new("RGB", (TILE_W, STRIP_H), (252, 252, 251))
    d = ImageDraw.Draw(im)
    left, right, top, bot = 54, TILE_W - 10, 10, STRIP_H - 26

    def X(t):
        return left + (t + PRE) / (t_end + PRE) * (right - left)

    def Y(v):
        return top + (1 - (max(-YLIM, min(YLIM, v)) + YLIM) / (2 * YLIM)) * (bot - top)

    for name, a, b in run.pushes:
        d.rectangle([X(a), top, X(b), bot], fill=(236, 233, 228))
    for v in (-YLIM, 0, YLIM):
        d.line([left, Y(v), right, Y(v)], fill=(214, 212, 206) if v else (180, 178, 172), width=1)
        d.text((6, Y(v) - 10), "%+.0f°" % v, fill=(82, 81, 78), font=F[18])
    m = (run.rel >= -PRE) & (run.rel <= t_end)
    step = max(1, int(m.sum() / 900))
    tt = run.rel[m][::step]
    for series, col in ((run.pitch, PITCH_COLOR), (run.roll, color)):
        pts = [(X(t), Y(v)) for t, v in zip(tt, series[m][::step])]
        d.line(pts, fill=col, width=2)
    d.line([X(tau), top, X(tau), bot], fill=(11, 11, 11), width=2)
    d.text((left, bot + 4), "roll", fill=color, font=F[18])
    d.text((left + 50, bot + 4), "pitch", fill=PITCH_COLOR, font=F[18])
    d.text((right - 300, bot + 4), "deviation [deg], torque shaded", fill=(82, 81, 78), font=F[18])
    return im


def tile(run, ctrl, nm, tau, t_end, F):
    im = Image.new("RGB", (TILE_W, TILE_H + STRIP_H), (255, 255, 255))
    im.paste(run.frame_at(tau).resize((TILE_W, TILE_H)), (0, 0))
    d = ImageDraw.Draw(im)
    color = COLORS[ctrl]
    d.rectangle([0, 0, 250, 44], fill=color)
    d.text((12, 6), "%s  %s N·m" % (ctrl.upper(), nm), fill=(255, 255, 255), font=F["b26"])
    tq = run.torque_at(tau)
    if tq:
        d.rectangle([TILE_W - 330, 0, TILE_W, 44], fill=TORQUE_RED)
        d.text((TILE_W - 318, 6), "%s TORQUE ON" % tq, fill=(255, 255, 255), font=F["b26"])
    r, p, dist = run.state_at(tau)
    box = [10, TILE_H - 108, 300, TILE_H - 10]
    d.rectangle(box, fill=(255, 255, 255))
    d.text((22, TILE_H - 102), "roll   %+5.1f°" % r, fill=(11, 11, 11), font=F[26])
    d.text((22, TILE_H - 72), "pitch  %+5.1f°" % p, fill=(11, 11, 11), font=F[26])
    d.text((22, TILE_H - 42), "drift  %4.2f m" % dist, fill=(11, 11, 11), font=F[26])
    im.paste(strip(run, tau, t_end, color, F), (0, TILE_H))
    return im


def render(runs, layout, out_mp4, title, F, workdir):
    """layout: list of rows, each a list of (ctrl, size_label, Run)."""
    t_end = max(max(b for _, _, b in r.pushes) for row in layout for _, _, r in row) + POST
    n = int((t_end + PRE) * FPS)
    W = TILE_W * len(layout[0])
    H = HEADER_H + (TILE_H + STRIP_H) * len(layout)
    if workdir.exists():
        shutil.rmtree(workdir)
    workdir.mkdir(parents=True)
    for k in range(n):
        tau = -PRE + k / FPS
        canvas = Image.new("RGB", (W, H), (255, 255, 255))
        d = ImageDraw.Draw(canvas)
        d.text((16, 12), title, fill=(11, 11, 11), font=F["b30"])
        d.text((W - 360, 16), "t = %+5.1f s from roll push" % tau, fill=(11, 11, 11), font=F[26])
        for i, row in enumerate(layout):
            for j, (ctrl, nm, run) in enumerate(row):
                canvas.paste(tile(run, ctrl, nm, tau, t_end, F), (j * TILE_W, HEADER_H + i * (TILE_H + STRIP_H)))
        canvas.save(workdir / ("f_%05d.jpg" % k), quality=88)
        if k % 150 == 0:
            print("  %s frame %d / %d" % (out_mp4.name, k, n), flush=True)
    encode(workdir, out_mp4)


def encode(workdir, out_mp4):
    """<workdir>/f_00000.jpg ... -> H.264 mp4 at FPS (GStreamer; no ffmpeg here)."""
    cmd = ["gst-launch-1.0", "-q", "multifilesrc", "location=%s/f_%%05d.jpg" % workdir, "index=0",
           "caps=image/jpeg,framerate=%d/1" % FPS, "!", "jpegdec", "!", "videoconvert", "!",
           "video/x-raw,format=I420", "!", "openh264enc", "bitrate=12000000", "!", "h264parse", "!",
           "mp4mux", "!", "filesink", "location=%s" % out_mp4]
    subprocess.run(cmd, check=True)
    shutil.rmtree(workdir)
    print("wrote", out_mp4, flush=True)


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else "runs/hover_video")
    F = fonts()
    runs = {(c, s): Run(root / ("%s_%s" % (c, s))) for c in ("pid", "indi") for s in ("1p44", "3p24")}
    label = {"1p44": "1.44", "3p24": "3.24"}
    work = root / "_frames"
    grid = [[(c, label[s], runs[(c, s)]) for c in ("pid", "indi")] for s in ("1p44", "3p24")]
    render(runs, grid, root / "hover_side_by_side.mp4",
           "Hover torque test: 5 s roll push, then 5 s pitch push", F, work)
    for s in ("1p44", "3p24"):
        render(runs, [grid[0] if s == "1p44" else grid[1]], root / ("hover_%s.mp4" % s),
               "Hover torque test, %s N·m: PID vs INDI" % label[s], F, work)


if __name__ == "__main__":
    main()
