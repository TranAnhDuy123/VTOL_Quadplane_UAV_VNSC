#!/usr/bin/env python3
"""Error between the flown path and the desired path, in x, y and z.

    python3 analysis/path_error.py runs/joint/square [runs/joint/zigzag ...]

For every run directory <root>/<ctrl>_<case>_s<seed>/ with flight.BIN and
mission_status.json:

  desired path  the mission plan named in mission_status.json, loaded with the
                same loader the harness flew it with (sim/fly_plan_mission.py):
                a polyline from the take-off point through the waypoints to the
                land point, at the waypoints' altitude.
  flown path    POS (EKF position) from the DataFlash log, in metres north /
                east of the plan home, and height above home.
  error         for each sample, the vector from the nearest point of the
                desired polyline (in the horizontal plane) to the aircraft:
                  x = north component, y = east component, z = height error
                and the signed cross-track distance (positive = right of track).
                z is only defined where the plan holds an altitude
                (transition, fixed-wing, back-transition); hover climbs and
                descents have no desired height profile.

The desired path has no timing, so there is no along-track error: the nearest
point is used. "Straight" statistics leave out samples whose nearest path point
is within CORNER_M of a waypoint, which separates leg tracking from the corner
cut that the navigation's turn radius produces.

Writes <run>/path_error.csv, <root>/path_error/summary.{csv,md} and
<root>/path_error/path_<profile>.png / .pdf.
"""
import json
import math
import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "sim"))
from validate_log import analyse, arr          # noqa: E402
from fly_plan_mission import load_plan, NAV_CMDS  # noqa: E402
from alignment import run_clock, warp, shade_pulses  # noqa: E402

CORNER_M = 60.0
ZEXAG = 8.0                                        # height exaggeration in the 3D view
PHASES = ["hover_takeoff", "transition", "fixed_wing", "back_transition", "hover_land"]
Z_PHASES = {"transition", "fixed_wing", "back_transition"}
COLORS = {"pid": "#2a78d6", "indi": "#eb6834"}      # validated categorical slots 1, 2
DESIRED = "#52514e"
CASES = [("calm", "calm"), ("w6", "6 m/s + 1.44 N·m"), ("w9", "9 m/s + 3.24 N·m")]
R_EARTH = 6378137.0


def desired_path(plan):
    """(vertices_ne [n,2], altitude [n]) of the planned path, home at (0, 0)."""
    home_lat, home_lon, _, items = load_plan(plan)
    verts, alts = [(0.0, 0.0)], [None]
    for it in items:
        if it[2] not in NAV_CMDS or not (it[9] or it[10]):
            continue
        lat, lon = it[9] * 1e-7, it[10] * 1e-7
        p = (math.radians(lat - home_lat) * R_EARTH,
             math.radians(lon - home_lon) * R_EARTH * math.cos(math.radians(home_lat)))
        if math.hypot(p[0] - verts[-1][0], p[1] - verts[-1][1]) < 1.0:
            alts[-1] = alts[-1] if alts[-1] is not None else float(it[11])
            continue
        verts.append(p)
        alts.append(float(it[11]))
    cruise = max(a for a in alts if a is not None)
    alts = [cruise if (a is None or a <= 0) else a for a in alts]
    return np.array(verts), np.array(alts), (home_lat, home_lon)


def nearest_on_path(P, V):
    """Nearest point on polyline V for every point in P (horizontal).

    Returns (nearest [m,2], segment index [m], fraction along segment [m])."""
    A, B = V[:-1], V[1:]
    D = B - A
    L2 = np.maximum((D ** 2).sum(1), 1e-9)
    # [m, k] projection fractions onto every segment
    f = np.clip(((P[:, None, :] - A[None]) * D[None]).sum(2) / L2[None], 0.0, 1.0)
    Q = A[None] + f[..., None] * D[None]
    d2 = ((P[:, None, :] - Q) ** 2).sum(2)
    k = d2.argmin(1)
    idx = np.arange(len(P))
    return Q[idx, k], k, f[idx, k]


def run_errors(rd):
    status = json.load(open(rd / "mission_status.json"))
    V, alts, (lat0, lon0) = desired_path(status["plan"])
    res, d, t0, phases = analyse(rd / "flight.BIN")
    t, lat = arr(d["POS"], "Lat", t0)
    _, lon = arr(d["POS"], "Lng", t0)
    _, h = arr(d["POS"], "RelHomeAlt", t0)
    P = np.column_stack([np.radians(lat - lat0) * R_EARTH,
                         np.radians(lon - lon0) * R_EARTH * np.cos(np.radians(lat0))])
    Q, k, f = nearest_on_path(P, V)
    E = P - Q
    D = V[k + 1] - V[k]
    Ln = np.maximum(np.hypot(D[:, 0], D[:, 1]), 1e-9)
    xtrack = (D[:, 0] * E[:, 1] - D[:, 1] * E[:, 0]) / Ln
    # distance of the nearest path point from the closest waypoint (corner test)
    dv = np.minimum(f, 1 - f) * Ln
    phase = np.full(len(t), "", dtype=object)
    for name, (lo, hi) in phases.items():
        phase[(t >= lo) & (t < hi)] = name
    z_des = alts[k] + f * (alts[k + 1] - alts[k])
    ez = np.where(np.isin(phase, list(Z_PHASES)), h - z_des, np.nan)
    t_arm = res["events"]["arm"] or 0.0
    df = pd.DataFrame(dict(t=t - t_arm, phase=phase, north=P[:, 0], east=P[:, 1], height=h,
                           ex=E[:, 0], ey=E[:, 1], ez=ez, xtrack=xtrack,
                           straight=dv > CORNER_M))
    df = df[df.phase != ""]
    df.to_csv(rd / "path_error.csv", index=False, float_format="%.3f")
    windows, pulses = run_clock(status, t0, t_arm)
    phases = {k: v[0] - t_arm for k, v in phases.items()}
    return df, dict(V=V, alts=alts, windows=windows, pulses=pulses, phases=phases), status


def rms(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(np.sqrt(np.mean(x ** 2))) if len(x) else np.nan


def amax(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(np.abs(x).max()) if len(x) else np.nan


def summarise(df):
    rows = []
    groups = [(ph, df[df.phase == ph]) for ph in PHASES]
    groups.append(("fixed_wing_straight", df[(df.phase == "fixed_wing") & df.straight]))
    groups.append(("whole_flight", df))
    for name, g in groups:
        if g.empty:
            continue
        rows.append(dict(scope=name, samples=len(g),
                         ex_rms=rms(g.ex), ey_rms=rms(g.ey), ez_rms=rms(g.ez),
                         xtrack_rms=rms(g.xtrack), xtrack_max=amax(g.xtrack), ez_max=amax(g.ez)))
    return rows


def _aligned(runs, case):
    """{ctrl: (df, info, time map)} with the INDI run on the PID clock."""
    here = {c: runs[(c, cs)] for (c, cs) in runs if cs == case}
    ref = here.get("pid", next(iter(here.values()), None))
    out = {}
    for ctrl, item in here.items():
        df, info = item
        if item is ref:
            f = lambda t: np.asarray(t, float)
        else:
            f = warp(ref[1]["windows"], info["windows"], ref[1]["phases"], info["phases"])
        out[ctrl] = (df, info, f)
    return out


def _style(plt):
    plt.rcParams.update({"font.size": 8, "axes.spines.top": False, "axes.spines.right": False,
                         "axes.edgecolor": "#52514e", "axes.labelcolor": "#0b0b0b",
                         "xtick.color": "#52514e", "ytick.color": "#52514e"})


def plot_profile(profile, runs, out):
    """Top view, cross-track and height error against the aligned clock.

    Torque pulses are shaded: grey where both runs received the same window
    (they coincide after alignment), in a run's colour where only it did."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _style(plt)
    fig, axs = plt.subplots(len(CASES), 3, figsize=(11, 2.9 * len(CASES)),
                            gridspec_kw=dict(width_ratios=[1.1, 1.2, 1.2]))
    for r, (case, label) in enumerate(CASES):
        a_top, a_xt, a_z = axs[r]
        here = _aligned(runs, case)
        drew = False
        pulses = {c: [(float(f(t)), a, w) for t, a, w in info["pulses"]] for c, (_, info, f) in here.items()}
        for a in (a_xt, a_z):
            shade_pulses(a, pulses, COLORS)
        lo, hi = np.inf, -np.inf
        for ctrl in ("pid", "indi"):
            if ctrl not in here:
                continue
            df, info, f = here[ctrl]
            V = info["V"]
            if not drew:
                a_top.plot(V[:, 1], V[:, 0], "--", color=DESIRED, lw=1.2, label="desired", zorder=1)
                drew = True
            a_top.plot(df.east, df.north, color=COLORS[ctrl], lw=1.2, label=ctrl.upper(), zorder=2)
            fw = df[df.phase.isin(list(Z_PHASES))]
            ta = f(fw.t.values)
            lo, hi = min(lo, ta.min()), max(hi, ta.max())
            a_xt.plot(ta, fw.xtrack, color=COLORS[ctrl], lw=1.0, label=ctrl.upper())
            a_z.plot(ta, fw.ez, color=COLORS[ctrl], lw=1.0, label=ctrl.upper())
        a_top.set_aspect("equal", adjustable="datalim")
        a_top.set_xlabel("east [m]")
        a_top.set_ylabel("%s\nnorth [m]" % label)
        a_xt.set_ylabel("cross-track [m]")
        a_z.set_ylabel("height error z [m]")
        for a in (a_xt, a_z):
            a.axhline(0, color="#c3c2b7", lw=0.8, zorder=0)
            a.set_xlabel("mission time on the PID clock [s]")
            if np.isfinite(lo):
                a.set_xlim(lo - 5, hi + 5)
        for a in (a_top, a_xt, a_z):
            a.grid(True, color="#e6e5e0", lw=0.5)
        if r == 0:
            a_top.legend(frameon=False, loc="best")
    fig.tight_layout()
    for ext in ("png", "pdf"):
        fig.savefig(out / ("path_%s.%s" % (profile, ext)), dpi=150)
    plt.close(fig)


def desired_3d(V, alts):
    """Planned path in 3D: vertical climb at the take-off point, the legs at
    their altitude, vertical descent at the landing point."""
    n = [V[0, 0]] + list(V[:, 0]) + [V[-1, 0]]
    e = [V[0, 1]] + list(V[:, 1]) + [V[-1, 1]]
    h = [0.0] + list(alts) + [0.0]
    return np.array(n), np.array(e), np.array(h)


def plot_3d(profile, runs, out):
    """3D view of the whole flight, take-off to landing, per condition.
    Dots mark where each torque pulse started (run's colour)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _style(plt)
    fig = plt.figure(figsize=(5.2 * len(CASES), 4.3))
    for c, (case, label) in enumerate(CASES):
        ax = fig.add_subplot(1, len(CASES), c + 1, projection="3d")
        drew = False
        for ctrl in ("pid", "indi"):
            item = runs.get((ctrl, case))
            if item is None:
                continue
            df, info = item
            if not drew:
                n, e, h = desired_3d(info["V"], info["alts"])
                ax.plot(e, n, h, "--", color=DESIRED, lw=1.0, label="desired")
                drew = True
            ax.plot(df.east, df.north, df.height, color=COLORS[ctrl], lw=0.9, label=ctrl.upper())
            if info["pulses"]:
                tp = np.array([t for t, _, _ in info["pulses"]])
                idx = np.clip(np.searchsorted(df.t.values, tp), 0, len(df) - 1)
                ax.scatter(df.east.values[idx], df.north.values[idx], df.height.values[idx],
                           s=10, color=COLORS[ctrl], edgecolor="white", linewidth=0.4, depthshade=False)
        ax.set_xlabel("east [m]")
        ax.set_ylabel("north [m]")
        ax.set_zlabel("height [m]")
        ax.set_zlim(0, 45)
        ax.set_zticks([0, 20, 40])
        # true horizontal proportions; height drawn ZEXAG times its true scale
        xr = np.ptp(ax.get_xlim()); yr = np.ptp(ax.get_ylim())
        ax.set_box_aspect((xr, yr, 45.0 * ZEXAG))
        ax.set_title(label, fontsize=9, color="#0b0b0b", pad=0)
        ax.view_init(elev=22, azim=-62)
        if c == 0:
            ax.legend(frameon=False, loc="upper left", fontsize=7)
    fig.text(0.5, 0.01, "height axis drawn %.0f times its true scale; dots: start of each torque pulse" % ZEXAG,
             ha="center", fontsize=8, color="#52514e")
    fig.subplots_adjust(left=0.0, right=0.97, top=0.91, bottom=0.07, wspace=0.02)
    for ext in ("png", "pdf"):
        fig.savefig(out / ("path3d_%s.%s" % (profile, ext)), dpi=150)
    plt.close(fig)


def main():
    roots = [Path(a) for a in sys.argv[1:]] or [Path("runs/joint/square"), Path("runs/joint/zigzag")]
    pat = re.compile(r"^(pid|indi)_(calm|w6|w9)_s(\d+)$")
    for root in roots:
        out = root / "path_error"
        out.mkdir(parents=True, exist_ok=True)
        rows, runs = [], {}
        for rd in sorted(root.iterdir()):
            m = pat.match(rd.name)
            if not m or not (rd / "flight.BIN").exists() or not (rd / "mission_status.json").exists():
                continue
            ctrl, case, seed = m.group(1), m.group(2), int(m.group(3))
            print("reading", rd)
            df, info, _ = run_errors(rd)
            runs[(ctrl, case)] = (df, info)
            for r in summarise(df):
                r.update(run=rd.name, ctrl=ctrl, case=case, seed=seed)
                rows.append(r)
        if not rows:
            print("no runs under", root)
            continue
        S = pd.DataFrame(rows)
        S.to_csv(out / "summary.csv", index=False, float_format="%.3f")
        L = ["# Path error — %s" % root.name, "",
             "x = north, y = east component of the offset from the nearest point of the desired "
             "path; z = height error (defined in transition, fixed-wing and back-transition only); "
             "cross-track signed, + = right of track. \"fixed_wing_straight\" leaves out points within "
             "%.0f m of a waypoint. Metres." % CORNER_M, "",
             "| case | scope | ctrl | x RMS | y RMS | z RMS | cross-track RMS | cross-track max | z max |",
             "|---|---|---|---:|---:|---:|---:|---:|---:|"]
        for (case, _) in CASES:
            for scope in PHASES + ["fixed_wing_straight", "whole_flight"]:
                for ctrl in ("pid", "indi"):
                    g = S[(S.case == case) & (S.scope == scope) & (S.ctrl == ctrl)]
                    if g.empty:
                        continue
                    r = g.iloc[0]
                    L.append("| %s | %s | %s | %.2f | %.2f | %s | %.2f | %.2f | %s |" % (
                        case, scope, ctrl.upper(), r.ex_rms, r.ey_rms,
                        "—" if np.isnan(r.ez_rms) else "%.2f" % r.ez_rms,
                        r.xtrack_rms, r.xtrack_max, "—" if np.isnan(r.ez_max) else "%.2f" % r.ez_max))
        (out / "summary.md").write_text("\n".join(L) + "\n")
        plot_profile(root.name, runs, out)
        plot_3d(root.name, runs, out)
        print("wrote", out)


if __name__ == "__main__":
    main()
