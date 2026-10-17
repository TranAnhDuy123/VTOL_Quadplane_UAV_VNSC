#!/usr/bin/env python3
"""Side-by-side chase video of the recorded 6 m/s square mission, PID | INDI.

    python3 analysis/fw_video.py [runs/fw_video] [--anchor mid_2] [--before 5] [--length 40]

Reads <root>/{pid,indi}_w6/ (rec/ from sim/record_camera.py: frames/<sim_ms>.jpg,
pose.csv, events.csv; mission_status.json; flight.BIN) and writes
<root>/fw_w6.mp4: a clip of --length seconds starting --before seconds ahead
of the roll pulse of window --anchor.

Camera: fixed to the airframe (sim/worlds/chase_camera_sensor.sdf). Each frame
is turned about the aircraft by the camera's roll so the horizon is level, then
cropped to 960 x 540. The aircraft is then seen to bank and pitch against a
level horizon, as from a gimballed camera on a boom behind it.

Clock: t = 0 at the anchor's roll pulse in the PID run. The INDI run is put on
the same clock window by window as in the thesis timelines (analysis/alignment.py:
a pure shift inside each pulse window, a linear stretch between windows), so
both tiles receive every pulse at the same moment.

Overlay per tile: roll and pitch, actual and commanded (ATT in the DataFlash
log), distance from the planned track (Gazebo ground truth against the plan
polyline), a banner while a torque is on, and a strip chart of the tracking
error, commanded - actual, as in analysis/compare_controllers.py.
"""
import argparse
import csv
import json
import math
import re
import shutil
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from pymavlink import mavutil

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from alignment import HOLD_S  # noqa: E402
from hover_video import COLORS, FPS, PITCH_COLOR, TORQUE_RED, encode, fonts  # noqa: E402
from path_error import desired_path, nearest_on_path  # noqa: E402

CAMERA_SDF = HERE.parent / "sim" / "worlds" / "chase_camera_sensor.sdf"
SRC = 1200                              # square source image; the camera looks at the aircraft origin
CROP_W, CROP_H, AX, AY = 960, 540, 480, 324   # output crop, aircraft pixel in it
TILE_W, TILE_H, STRIP_H, HEADER_H = 880, 495, 150, 57
CTRLS = ("pid", "indi")
FONT_MONO = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"


def rot(axis, a):
    c, s = math.cos(a), math.sin(a)
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def camera_mount():
    """Camera orientation in the body frame (FLU) from the sensor <pose>."""
    pose = re.search(r"<pose>([^<]+)</pose>", CAMERA_SDF.read_text()).group(1).split()
    roll, pitch, yaw = (float(v) for v in pose[3:6])
    assert roll == 0.0
    return rot("z", yaw) @ rot("y", pitch)


def quat_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


class Run:
    def __init__(self, rd, mount):
        rec = rd / "rec"
        self.frames = sorted(rec.glob("frames/*.jpg"))
        self.ft = np.array([int(p.stem) * 1e-3 for p in self.frames])
        rows = list(csv.DictReader(open(rec / "pose.csv")))
        self.pt = np.array([float(r["sim_s"]) for r in rows])
        self.q = np.array([[float(r[k]) for k in ("qw", "qx", "qy", "qz")] for r in rows])
        self.mount = mount
        # torque events (sim clock) <-> harness pulses (vehicle clock, with window names)
        ev = list(csv.DictReader(open(rec / "events.csv")))
        on = [float(e["sim_s"]) for e in ev if e["event"].startswith("torque_on")]
        clears = np.array([float(e["sim_s"]) for e in ev if e["event"].startswith("torque_off")])
        off = [float(clears[clears > t].min()) for t in on]     # the harness also clears at start/end
        status = json.load(open(rd / "mission_status.json"))
        pulses = [p for p in status["pulses"] if p["action"] in ("roll", "pitch")]
        if len(on) != len(pulses):
            raise SystemExit("%s: %d torque events, %d harness pulses" % (rd, len(on), len(pulses)))
        d = np.array(on) - np.array([p["tb"] for p in pulses])
        self.offset = float(np.median(d))         # sim time - vehicle time
        self.offset_spread = float(np.ptp(d))
        self.pushes = [(p["action"], p["window"], a, b) for p, a, b in zip(pulses, on, off)]
        self.windows = {}
        for axis, window, a, _ in self.pushes:
            self.windows.setdefault(window, a)
        # attitude tracking error from the log, on the sim clock
        log = mavutil.mavlink_connection(str(rd / "flight.BIN"))
        att = list(iter(lambda: log.recv_match(type="ATT"), None))
        self.at = np.array([m.TimeUS * 1e-6 for m in att]) + self.offset
        self.roll, self.roll_cmd = np.array([[m.Roll, m.DesRoll] for m in att]).T
        self.pitch, self.pitch_cmd = np.array([[m.Pitch, m.DesPitch] for m in att]).T
        self.roll_err = self.roll_cmd - self.roll
        self.pitch_err = self.pitch_cmd - self.pitch
        # distance from the planned track, ground truth (Gazebo x east, y north; origin = plan home)
        V, _, _ = desired_path(status["plan"])
        P = np.column_stack([[float(r["y"]) for r in rows], [float(r["x"]) for r in rows]])
        Q, _, _ = nearest_on_path(P, V)
        self.off_track = np.hypot(*(P - Q).T)

    def quat_at(self, s):
        i = int(np.clip(np.searchsorted(self.pt, s), 1, len(self.pt) - 1))
        a, b = self.q[i - 1], self.q[i]
        if np.dot(a, b) < 0:
            b = -b
        f = np.clip((s - self.pt[i - 1]) / max(self.pt[i] - self.pt[i - 1], 1e-9), 0, 1)
        q = (1 - f) * a + f * b
        return q / np.linalg.norm(q)

    def view(self, s):
        """Levelled 960 x 540 camera view at sim time s."""
        i = int(np.argmin(np.abs(self.ft - s)))
        u = (quat_R(self.quat_at(self.ft[i])) @ self.mount).T @ np.array([0.0, 0.0, 1.0])
        beta = math.degrees(math.atan2(-u[1], u[2]))   # lean of world-up in the image, + = to the right
        im = Image.open(self.frames[i]).convert("RGB")
        im = im.rotate(beta, resample=Image.BILINEAR, center=(SRC / 2, SRC / 2))
        x0, y0 = SRC // 2 - AX, SRC // 2 - AY
        return im.crop((x0, y0, x0 + CROP_W, y0 + CROP_H))


class Clock:
    """Display time tau (PID clock, 0 at the anchor) <-> one run's sim time."""

    def __init__(self, ref, run, anchor):
        t_ref = ref.windows[anchor]
        kr, ko = [], []
        for r, o in sorted((ref.windows[k] - t_ref, run.windows[k]) for k in set(ref.windows) & set(run.windows)):
            for h in (0.0, HOLD_S):
                if kr and (r + h <= kr[-1] or o + h <= ko[-1]):
                    continue
                kr.append(r + h)
                ko.append(o + h)
        self.kr = np.array([kr[0] - 1e6] + kr + [kr[-1] + 1e6])
        self.ko = np.array([ko[0] - 1e6] + ko + [ko[-1] + 1e6])

    def sim(self, tau):
        return np.interp(tau, self.kr, self.ko)

    def tau(self, s):
        return np.interp(s, self.ko, self.kr)


def where(window):
    return {"mid": "on a straight leg", "corner": "at a corner (turning)",
            "transition": "in the transition"}.get(window.split("_")[0], "in hover")


def strip(run, clk, t0, t1, ylim, tau, color, F):
    im = Image.new("RGB", (TILE_W, STRIP_H), (252, 252, 251))
    d = ImageDraw.Draw(im)
    left, right, top, bot = 54, TILE_W - 10, 10, STRIP_H - 26

    def X(t):
        return left + (t - t0) / (t1 - t0) * (right - left)

    def Y(v):
        return top + (1 - (max(-ylim, min(ylim, v)) + ylim) / (2 * ylim)) * (bot - top)

    for _, _, a, b in run.pushes:
        a, b = clk.tau(a), clk.tau(b)
        if b > t0 and a < t1:
            d.rectangle([X(max(a, t0)), top, X(min(b, t1)), bot], fill=(236, 233, 228))
    for v in (-ylim, 0, ylim):
        d.line([left, Y(v), right, Y(v)], fill=(214, 212, 206) if v else (180, 178, 172), width=1)
        d.text((4, Y(v) - 10), "%+.0f°" % v, fill=(82, 81, 78), font=F[18])
    ta = clk.tau(run.at)
    m = (ta >= t0) & (ta <= t1)
    for series, col in ((run.pitch_err, PITCH_COLOR), (run.roll_err, color)):
        d.line([(X(t), Y(v)) for t, v in zip(ta[m], series[m])], fill=col, width=2)
    d.line([X(tau), top, X(tau), bot], fill=(11, 11, 11), width=2)
    d.text((left, bot + 4), "roll", fill=color, font=F[18])
    d.text((left + 50, bot + 4), "pitch", fill=PITCH_COLOR, font=F[18])
    d.text((right - 400, bot + 4), "error, commanded - actual [deg]", fill=(82, 81, 78), font=F[18])
    return im


def tile(run, clk, ctrl, tau, t0, t1, ylim, F):
    s = float(clk.sim(tau))
    im = Image.new("RGB", (TILE_W, TILE_H + STRIP_H), (255, 255, 255))
    im.paste(run.view(s).resize((TILE_W, TILE_H), Image.LANCZOS), (0, 0))
    d = ImageDraw.Draw(im)
    color = COLORS[ctrl]
    d.rectangle([0, 0, 110, 44], fill=color)
    d.text((12, 6), ctrl.upper(), fill=(255, 255, 255), font=F["b26"])
    on = next(((axis, w) for axis, w, a, b in run.pushes if a <= s < b), None)
    if on:
        d.rectangle([TILE_W - 330, 0, TILE_W, 44], fill=TORQUE_RED)
        d.text((TILE_W - 318, 6), "%s TORQUE ON" % on[0].upper(), fill=(255, 255, 255), font=F["b26"])
        d.rectangle([TILE_W - 330, 44, TILE_W, 74], fill=(255, 255, 255))
        d.text((TILE_W - 318, 48), where(on[1]), fill=(11, 11, 11), font=F[18])
    r, rc, p, pc = (float(np.interp(s, run.at, v)) for v in (run.roll, run.roll_cmd, run.pitch, run.pitch_cmd))
    x = float(np.interp(s, run.pt, run.off_track))
    # actual attitude next to the commanded one: a commanded bank (corner, track
    # correction) is not a disturbance response, the error strip below is their difference
    d.rectangle([10, TILE_H - 108, 404, TILE_H - 10], fill=(255, 255, 255))
    d.text((22, TILE_H - 101), "roll  %+6.1f°  cmd %+6.1f°" % (r, rc), fill=(11, 11, 11), font=F["m24"])
    d.text((22, TILE_H - 71), "pitch %+6.1f°  cmd %+6.1f°" % (p, pc), fill=(11, 11, 11), font=F["m24"])
    d.text((22, TILE_H - 41), "off track %5.1f m" % x, fill=(11, 11, 11), font=F["m24"])
    im.paste(strip(run, clk, t0, t1, ylim, tau, color, F), (0, TILE_H))
    return im


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root", nargs="?", default="runs/fw_video")
    ap.add_argument("--anchor", default="mid_2", help="pulse window at t = 0")
    ap.add_argument("--before", type=float, default=5.0)
    ap.add_argument("--length", type=float, default=40.0)
    ap.add_argument("--out", default="fw_w6.mp4")
    a = ap.parse_args()
    root = Path(a.root)
    mount = camera_mount()
    runs = {c: Run(root / ("%s_w6" % c), mount) for c in CTRLS}
    clocks = {c: Clock(runs["pid"], runs[c], a.anchor) for c in CTRLS}
    t0, t1 = -a.before, a.length - a.before
    for c in CTRLS:
        r, k = runs[c], clocks[c]
        lo, hi = k.sim(t0), k.sim(t1)
        if r.ft[0] > lo + 0.1 or r.ft[-1] < hi - 0.1:
            raise SystemExit("%s: frames cover sim %.1f-%.1f s, clip needs %.1f-%.1f s"
                             % (c, r.ft[0], r.ft[-1], lo, hi))
        print("%s: clock offset %.3f s (spread %.3f s), clip = sim %.1f-%.1f s, pulses in clip: %s"
              % (c, r.offset, r.offset_spread, lo, hi,
                 sorted({w for _, w, s, _ in r.pushes if lo <= s <= hi}, key=lambda w: r.windows[w])))
    # one y range for both tiles, from the largest error in the clip
    peak = 0.0
    for c in CTRLS:
        r, ta = runs[c], clocks[c].tau(runs[c].at)
        m = (ta >= t0) & (ta <= t1)
        peak = max(peak, np.abs(r.roll_err[m]).max(), np.abs(r.pitch_err[m]).max())
    ylim = 10.0 * math.ceil(peak / 10.0)
    F = fonts() | {"m24": ImageFont.truetype(FONT_MONO, 24)}
    work = root / "_frames"
    if work.exists():
        shutil.rmtree(work)
    work.mkdir(parents=True)
    n = int(round(a.length * FPS))
    W, H = TILE_W * len(CTRLS), HEADER_H + TILE_H + STRIP_H
    H += H % 2
    for i in range(n):
        tau = t0 + i / FPS
        canvas = Image.new("RGB", (W, H), (255, 255, 255))
        d = ImageDraw.Draw(canvas)
        d.text((16, 12), "Square mission, 6 m/s wind, 1.44 N·m torque pulses", fill=(11, 11, 11), font=F["b30"])
        d.text((W - 210, 16), "t = %+5.1f s" % tau, fill=(11, 11, 11), font=F[26])
        for j, c in enumerate(CTRLS):
            canvas.paste(tile(runs[c], clocks[c], c, tau, t0, t1, ylim, F), (j * TILE_W, HEADER_H))
        canvas.save(work / ("f_%05d.jpg" % i), quality=88)
        if i % 150 == 0:
            print("  frame %d / %d" % (i, n), flush=True)
    encode(work, root / a.out)


if __name__ == "__main__":
    main()
