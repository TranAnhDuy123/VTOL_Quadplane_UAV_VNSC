#!/usr/bin/env python3
"""Analyse the hover-disturbance test (sim/run_hover_disturbance.sh).

    python3 analysis/hover_disturbance.py [runs/hover_test]

For every <root>/<ctrl>_<size>/ with flight.BIN and mission_status.json:

  attitude    roll / pitch, peak deviation from the level held before the push
              and the mean deviation over the last 2 s of the push
  position    horizontal distance from the take-off point (QLOITER target)
  takeover    how much of the torque the rate loop's own command has taken
              over:  (u(t) - u_before) * G * I / M, 1 = fully cancelled.
              u is the normalised roll/pitch mixer command: INDI.RU / PU for
              INDI, PIQR / PIQP P+I+D+FF for PID. G = Q_INDI_RLL_G / PIT_G
              (identified hover effectiveness), I = model inertia.
  t90         time from torque on until takeover first reaches 90 % of its
              value over the last second of the push
  INDI on     share of the push in which the INDI activity bit is set

Writes <root>/summary.md, summary.csv and hover_disturbance.png / .pdf
(time-based: seconds from the roll push, pushes shaded).
"""
import json
import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_log import read_log, arr  # noqa: E402

IXX, IYY = 0.1399, 0.21384856799            # model.sdf
G_HOVER = {"roll": 56.7, "pitch": 44.8}     # sim/indi.param, identified
COLORS = {"pid": "#2a78d6", "indi": "#eb6834"}
R_EARTH = 6378137.0


def load(rd):
    st = json.load(open(rd / "mission_status.json"))
    d = read_log(rd / "flight.BIN")
    t0 = d["ATT"][0]["TimeUS"] * 1e-6
    pushes = {}
    for p in st["pulses"]:
        if p["action"] in ("roll", "pitch"):
            pushes[p["action"]] = [p["tb"] - t0, None]
        elif p["action"] == "off":
            pushes[p["window"].replace("hover_", "")][1] = p["tb"] - t0
    return st, d, t0, pushes


def command(d, t0, ctrl, axis):
    """Normalised mixer command of the rate loop for one axis."""
    if ctrl == "indi" and d["INDI"]:
        return arr(d["INDI"], "RU" if axis == "roll" else "PU", t0)
    rows = d["PIQR" if axis == "roll" else "PIQP"]
    t = np.array([r["TimeUS"] for r in rows]) * 1e-6 - t0
    u = np.array([r["P"] + r["I"] + r["D"] + r.get("FF", 0.0) for r in rows])
    return t, u


def analyse(rd, ctrl):
    st, d, t0, pushes = load(rd)
    nm = st["nm"]
    ta, roll = arr(d["ATT"], "Roll", t0)
    _, pitch = arr(d["ATT"], "Pitch", t0)
    tp, lat = arr(d["POS"], "Lat", t0)
    _, lon = arr(d["POS"], "Lng", t0)
    lat0, lon0 = lat[0], lon[0]
    north = np.radians(lat - lat0) * R_EARTH
    east = np.radians(lon - lon0) * R_EARTH * np.cos(np.radians(lat0))
    dist = np.hypot(north, east)
    out, series = [], dict(ta=ta, roll=roll, pitch=pitch, tp=tp, dist=dist, pushes=pushes)
    for axis, (on, off) in pushes.items():
        ang = roll if axis == "roll" else pitch
        inertia = IXX if axis == "roll" else IYY
        pre = (ta >= on - 3) & (ta < on)
        dur = (ta >= on) & (ta < off)
        post = (ta >= on) & (ta < off + 10)
        base = ang[pre].mean()
        dev = ang - base
        last2 = (ta >= off - 2) & (ta < off)
        tu, u = command(d, t0, ctrl, axis)
        upre = u[(tu >= on - 1) & (tu < on)].mean()
        take = -(u - upre) * G_HOVER[axis] * inertia / nm     # the command opposes the torque
        series["take_" + axis] = (tu, take)
        win = (tu >= on) & (tu < off)
        end = float(np.mean(take[(tu >= off - 1) & (tu < off)]))
        above = np.where(take[win] >= 0.9 * end)[0]
        t90 = float(tu[win][above[0]] - on) if len(above) and end > 0 else np.nan
        pos_win = (tp >= on) & (tp < off + 10)
        indi_on = np.nan
        if d["INDI"]:
            ti, act = arr(d["INDI"], "Act", t0)
            bit = 1 if axis == "roll" else 2
            w = (ti >= on) & (ti < off)
            indi_on = 100.0 * float(((act[w].astype(int) & bit) != 0).mean()) if w.any() else 0.0
        out.append(dict(run=rd.name, ctrl=ctrl, nm=nm, axis=axis,
                        peak_dev_deg=float(np.abs(dev[post]).max()),
                        steady_dev_deg=float(np.abs(dev[last2]).mean()),
                        t90_s=t90,
                        takeover_end=end,
                        max_dist_m=float(dist[pos_win].max()),
                        indi_on_pct=indi_on))
    return out, series


def plot(runs, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 8, "axes.spines.top": False, "axes.spines.right": False})
    sizes = sorted({k[1] for k in runs})
    fig, axs = plt.subplots(4, len(sizes), figsize=(5.2 * len(sizes), 9), sharex="col", squeeze=False)
    for c, nm in enumerate(sizes):
        ref = None
        for ctrl in ("pid", "indi"):
            s = runs.get((ctrl, nm))
            if s is None:
                continue
            t_ref = s["pushes"]["roll"][0]
            ref = ref or s
            lab = ctrl.upper()
            axs[0, c].plot(s["ta"] - t_ref, s["roll"], color=COLORS[ctrl], lw=1, label=lab)
            axs[1, c].plot(s["ta"] - t_ref, s["pitch"], color=COLORS[ctrl], lw=1, label=lab)
            axs[2, c].plot(s["tp"] - t_ref, s["dist"], color=COLORS[ctrl], lw=1, label=lab)
            for axis, ls in (("roll", "-"), ("pitch", "--")):
                t, y = s["take_" + axis]
                axs[3, c].plot(t - t_ref, y, color=COLORS[ctrl], lw=1, ls=ls,
                               label="%s %s" % (lab, axis))
        t_ref = ref["pushes"]["roll"][0]
        for r in range(4):
            a = axs[r, c]
            for axis, (on, off) in ref["pushes"].items():
                a.axvspan(on - t_ref, off - t_ref, color="#c3c2b7", alpha=0.35, lw=0)
            a.grid(True, color="#e6e5e0", lw=0.5)
            a.set_xlim(-8, ref["pushes"]["pitch"][1] - t_ref + 12)
        for axis, (on, off) in ref["pushes"].items():
            axs[0, c].text((on + off) / 2 - t_ref, 1.0, "%s push" % axis, transform=axs[0, c].get_xaxis_transform(),
                           ha="center", va="bottom", fontsize=7, color="#52514e")
        axs[3, c].axhline(1.0, color="#52514e", lw=0.8, ls=":")
        axs[0, c].set_ylabel("roll [deg]")
        axs[1, c].set_ylabel("pitch [deg]")
        axs[2, c].set_ylabel("distance from origin [m]")
        axs[3, c].set_ylabel("torque taken over by\nrate-loop command [-]")
        axs[3, c].set_xlabel("time from roll push [s]   (%.2f N·m pushes shaded)" % nm)
        axs[0, c].legend(frameon=False, loc="lower left")
        axs[3, c].legend(frameon=False, ncol=2, loc="lower right", fontsize=7)
    fig.tight_layout()
    for ext in ("png", "pdf"):
        fig.savefig(out / ("hover_disturbance.%s" % ext), dpi=150)
    plt.close(fig)


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else "runs/hover_test")
    pat = re.compile(r"^(pid|indi)_(\d+p\d+)$")
    rows, runs = [], {}
    for rd in sorted(root.iterdir()):
        m = pat.match(rd.name)
        if not m or not (rd / "flight.BIN").exists():
            continue
        print("reading", rd)
        r, s = analyse(rd, m.group(1))
        rows += r
        runs[(m.group(1), r[0]["nm"])] = s
    S = pd.DataFrame(rows)
    S.to_csv(root / "summary.csv", index=False, float_format="%.3f")
    L = ["# Hover disturbance test", "",
         "Constant torque for 5 s in QLOITER over the origin. takeover = (u - u_before) G I / M "
         "(1 = rate-loop command has cancelled the torque); t90 = time to 90 % of its end value.", "",
         "| torque (N·m) | axis | ctrl | peak dev (deg) | dev last 2 s (deg) | t90 (s) | takeover at end | max distance (m) | INDI on (%) |",
         "|---:|---|---|---:|---:|---:|---:|---:|---:|"]
    for _, r in S.sort_values(["nm", "axis", "ctrl"], ascending=[True, False, False]).iterrows():
        L.append("| %.2f | %s | %s | %.2f | %.2f | %.2f | %.2f | %.2f | %s |" % (
            r.nm, r.axis, r.ctrl.upper(), r.peak_dev_deg, r.steady_dev_deg, r.t90_s,
            r.takeover_end, r.max_dist_m, "—" if np.isnan(r.indi_on_pct) else "%.0f" % r.indi_on_pct))
    (root / "summary.md").write_text("\n".join(L) + "\n")
    print("\n".join(L))
    plot(runs, root)


if __name__ == "__main__":
    main()
