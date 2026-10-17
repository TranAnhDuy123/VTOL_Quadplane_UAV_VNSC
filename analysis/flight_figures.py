#!/usr/bin/env python3
"""Figures that explain a square-mission run, and PID vs INDI.

    python analysis/flight_figures.py --out runs/figures runs/pid_w6 runs/indi_w6 runs/pid_w9 runs/indi_w9

Per run (<out>/<run>_*.png):
  path     planned path vs flown path, top view and height, plus the path
           error in east / north / up. The error is the flown position minus
           the nearest point of the planned 3-D path. Colour = flight phase.
           Triangles / dashed lines = moment pulses, if the run had any.
  energy   height vs TECS demand, airspeed vs demand, pitch vs demand,
           elevator and throttle. Shows how the aircraft trades height and
           speed (e.g. a nose-down pitch when airspeed is below demand).
  rate     roll and pitch rate: target vs measured, from whichever loop is
           flying the aircraft (VTOL loop PIQx in hover/transition, fixed-wing
           loop PIDR/PIDP otherwise), so there is no gap in the record.
  indi     INDI runs only: desired vs measured angular acceleration (nu, wdot_f)
           and the effector command vs its filtered applied value (u, u0_f).
Across runs (<out>/compare_*.png): tracking errors per phase, PID vs INDI.

Flight phases come from the autopilot's own messages (analysis/validate_log.py).
The ArduPilot mode is AUTO for the whole mission, so the phase is what is
colour coded.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from validate_log import analyse, arr  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

PHASE_COLORS = {"hover_takeoff": "#1f77b4", "transition": "#ff7f0e", "fixed_wing": "#2ca02c",
                "back_transition": "#d62728", "hover_land": "#9467bd"}
PHASE_NAMES = {"hover_takeoff": "VTOL take-off", "transition": "transition", "fixed_wing": "fixed-wing",
               "back_transition": "back-transition", "hover_land": "VTOL landing"}
PLAN = Path(__file__).resolve().parent.parent / "sim" / "square.plan"


def planned_path(lat0, lon0):
    """Planned 3-D path (east, north, up) in m from the take-off point."""
    items = json.load(open(PLAN))["mission"]["items"]
    k = 111320.0
    pts = [(0.0, 0.0, 0.0)]
    for it in items:
        if it["command"] not in (16, 21, 22, 84, 85):
            continue
        lat, lon, alt = it["params"][4], it["params"][5], it["params"][6]
        e = (lon - lon0) * k * math.cos(math.radians(lat0))
        n = (lat - lat0) * k
        if it["command"] in (22, 84):                     # VTOL take-off: straight up
            pts.append((e, n, alt))
        elif it["command"] in (21, 85):                   # VTOL land: arrive at height, then down
            pts.append((e, n, pts[-1][2]))
            pts.append((e, n, 0.0))
        else:
            pts.append((e, n, alt))
    return np.array(pts)


def nearest_on_path(p, path):
    """Nearest point of a polyline to each point in p (N x 3)."""
    best = np.full(len(p), np.inf)
    out = np.zeros_like(p)
    for a, b in zip(path[:-1], path[1:]):
        ab = b - a
        L2 = float(ab @ ab)
        s = np.clip(((p - a) @ ab) / L2, 0, 1) if L2 > 0 else np.zeros(len(p))
        q = a + s[:, None] * ab
        dist = np.linalg.norm(p - q, axis=1)
        m = dist < best
        best[m] = dist[m]
        out[m] = q[m]
    return out


def pulses(d, t0):
    if not d["RCOU"]:
        return []
    tr, c9 = arr(d["RCOU"], "C9", t0)
    idx = np.where(np.diff(c9) != 0)[0] + 1
    return [(tr[i], "roll" if c9[i] == 1900 else "pitch") for i in idx if c9[i] in (1900, 1100)]


def shade(ax, phases, label=False):
    for n, (lo, hi) in phases.items():
        ax.axvspan(lo, hi, color=PHASE_COLORS[n], alpha=0.12, lw=0,
                   label=PHASE_NAMES[n] if label else None)


def mark_pulses(ax, pl):
    for t, axis in pl:
        ax.axvline(t, color="k", ls="--" if axis == "roll" else ":", lw=0.8)


def fig_path(name, res, d, t0, phases, out):
    lat = np.array([r["Lat"] for r in d["POS"]])
    lon = np.array([r["Lng"] for r in d["POS"]])
    t, up = arr(d["POS"], "RelHomeAlt", t0)
    k = 111320.0
    e = (lon - lon[0]) * k * math.cos(math.radians(lat[0]))
    n = (lat - lat[0]) * k
    P = np.c_[e, n, up]
    path = planned_path(lat[0], lon[0])
    err = P - nearest_on_path(P, path)
    pl = pulses(d, t0)

    fig = plt.figure(figsize=(13, 8))
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.plot(path[:, 0], path[:, 1], "k--", lw=1, label="planned")
    for ph, (lo, hi) in phases.items():
        m = (t >= lo) & (t < hi)
        ax1.plot(e[m], n[m], color=PHASE_COLORS[ph], lw=1.6, label=PHASE_NAMES[ph])
    for tp, axis in pl:
        i = np.argmin(abs(t - tp))
        ax1.plot(e[i], n[i], "^" if axis == "roll" else "v", color="k", ms=7)
    if pl:
        ax1.plot([], [], "^k", label="roll pulse")
        ax1.plot([], [], "vk", label="pitch pulse")
    ax1.set_aspect("equal")
    ax1.set_xlabel("east [m]")
    ax1.set_ylabel("north [m]")
    ax1.legend(fontsize=7, loc="best")
    ax1.grid(alpha=0.3)

    ax2 = fig.add_subplot(2, 2, 2)
    q = nearest_on_path(P, path)
    ax2.plot(t, q[:, 2], "k--", lw=1, label="planned height")
    for ph, (lo, hi) in phases.items():
        m = (t >= lo) & (t < hi)
        ax2.plot(t[m], up[m], color=PHASE_COLORS[ph], lw=1.6)
    mark_pulses(ax2, pl)
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("height above home [m]")
    ax2.legend(fontsize=7)
    ax2.grid(alpha=0.3)

    ax3 = fig.add_subplot(2, 1, 2)
    shade(ax3, phases, label=True)
    ax3.plot(t, err[:, 0], lw=1, label="east error")
    ax3.plot(t, err[:, 1], lw=1, label="north error")
    ax3.plot(t, err[:, 2], lw=1.2, label="up error")
    ax3.plot(t, np.linalg.norm(err, axis=1), "k", lw=1.4, label="distance to path")
    mark_pulses(ax3, pl)
    ax3.set_xlabel("time [s]")
    ax3.set_ylabel("flown - planned [m]")
    ax3.legend(fontsize=7, ncol=4)
    ax3.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / ("%s_path.png" % name), dpi=150)
    plt.close(fig)
    dist = np.linalg.norm(err, axis=1)
    return {ph: float(np.sqrt(np.mean(dist[(t >= lo) & (t < hi)] ** 2))) for ph, (lo, hi) in phases.items()}


def fig_energy(name, d, t0, phases, out):
    pl = pulses(d, t0)
    fig, ax = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    t, h = arr(d["TECS"], "h", t0)
    ax[0].plot(t, arr(d["TECS"], "hdem", t0)[1], "k--", lw=1, label="TECS height demand")
    ax[0].plot(t, h, lw=1.2, label="height")
    ax[0].set_ylabel("height [m]")
    ax[1].plot(t, arr(d["TECS"], "spdem", t0)[1], "k--", lw=1, label="airspeed demand")
    ax[1].plot(t, arr(d["TECS"], "sp", t0)[1], lw=1.2, label="airspeed (TECS)")
    ax[1].set_ylabel("airspeed [m/s]")
    tc, np_ = arr(d["CTUN"], "NavPitch", t0)
    ax[2].plot(tc, np_, "k--", lw=1, label="pitch demand")
    ax[2].plot(tc, arr(d["CTUN"], "Pitch", t0)[1], lw=1.2, label="pitch")
    ax[2].set_ylabel("pitch [deg]")
    tr = np.array([r["TimeUS"] for r in d["RCOU"]]) * 1e-6 - t0
    elev = (np.array([r["C2"] for r in d["RCOU"]], float) - 1500) / 500
    thr = np.clip((np.array([r["C3"] for r in d["RCOU"]], float) - 1000) / 1000, 0, 1)
    ax[3].plot(tr, elev, lw=0.8, label="elevator (-1..1)")
    ax[3].plot(tr, thr, lw=1.0, label="forward throttle (0..1)")
    ax[3].set_ylabel("command")
    ax[3].set_xlabel("time [s]")
    for i, a in enumerate(ax):
        shade(a, phases, label=(i == 0))
        mark_pulses(a, pl)
        a.grid(alpha=0.3)
        a.legend(fontsize=7, ncol=3)
    fig.tight_layout()
    fig.savefig(out / ("%s_energy.png" % name), dpi=150)
    plt.close(fig)


def fig_rate(name, d, t0, phases, out):
    pl = pulses(d, t0)
    fig, ax = plt.subplots(3, 1, figsize=(12, 8), sharex=True, gridspec_kw=dict(height_ratios=[3, 3, 1]))
    tq, _ = arr(d["PIQR"], "Tar", t0)

    def vtol_running(times):
        """True where the VTOL rate loop was logging (it runs only when active)."""
        if not len(tq):
            return np.zeros(len(times), bool)
        j = np.searchsorted(tq, times).clip(1, len(tq) - 1)
        return np.minimum(abs(tq[j] - times), abs(tq[j - 1] - times)) < 0.05

    tf, _ = arr(d["PIDR"], "Tar", t0)
    vtol_on = vtol_running(tf)
    for k, (axis, q, f) in enumerate((("roll", "PIQR", "PIDR"), ("pitch", "PIQP", "PIDP"))):
        a = ax[k]
        # VTOL loop (rad/s -> deg/s) where it runs, fixed-wing loop elsewhere
        tq_, tarq = arr(d[q], "Tar", t0)
        _, actq = arr(d[q], "Act", t0)
        tf_, tarf = arr(d[f], "Tar", t0)
        _, actf = arr(d[f], "Act", t0)
        fw = ~vtol_running(tf_)
        a.plot(tq_, np.degrees(tarq), ".", color="0.2", ms=0.8)
        a.plot(tq_, np.degrees(actq), ".", color="#1f77b4", ms=0.8)
        a.plot(tf_[fw], tarf[fw], ".", color="0.2", ms=0.8)
        a.plot(tf_[fw], actf[fw], ".", color="#2ca02c", ms=0.8)
        a.plot([], [], "o", color="0.2", ms=4, label="target")
        a.plot([], [], "o", color="#1f77b4", ms=4, label="measured (VTOL loop)")
        a.plot([], [], "o", color="#2ca02c", ms=4, label="measured (fixed-wing loop)")
        a.set_ylabel("%s rate [deg/s]" % axis)
        shade(a, phases, label=(k == 0))
        mark_pulses(a, pl)
        a.grid(alpha=0.3)
        a.legend(fontsize=7, ncol=4)
    ax[2].fill_between(tf, 0, vtol_on.astype(float), step="post", color="#1f77b4", alpha=0.6, label="VTOL rate loop")
    ax[2].fill_between(tf, 0, (~vtol_on).astype(float), step="post", color="#2ca02c", alpha=0.6, label="fixed-wing rate loop")
    if d["INDI"] or d["INDF"]:
        ti, ai = arr(d["INDI"], "Act", t0) if d["INDI"] else (np.array([]), np.array([]))
        tfi, af = arr(d["INDF"], "Act", t0) if d["INDF"] else (np.array([]), np.array([]))
        ax[2].plot(ti, 0.5 * ((ai.astype(int) & 3) == 3), "k.", ms=0.5)
        ax[2].plot(tfi[(af.astype(int) & 3) == 3], 0.5 * np.ones(((af.astype(int) & 3) == 3).sum()), "k.", ms=0.5)
        ax[2].plot([], [], "k-", label="INDI running (roll+pitch)")
    ax[2].set_yticks([])
    ax[2].set_xlabel("time [s]")
    ax[2].legend(fontsize=7, ncol=3, loc="upper right")
    fig.tight_layout()
    fig.savefig(out / ("%s_rate.png" % name), dpi=150)
    plt.close(fig)


def fig_indi(name, d, t0, phases, out):
    if not (d["INDI"] or d["INDF"]):
        return
    fig, ax = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    for k, (axis, pre) in enumerate((("roll", "R"), ("pitch", "P"))):
        for msg, lab in (("INDI", "motors"), ("INDF", "surfaces")):
            if not d[msg]:
                continue
            t, act = arr(d[msg], "Act", t0)
            bit = 1 if axis == "roll" else 2
            on = (act.astype(int) & bit) != 0
            nu = arr(d[msg], pre + "Nu", t0)[1]
            wd = arr(d[msg], pre + "Wd", t0)[1]
            u = arr(d[msg], pre + "U", t0)[1]
            u0 = arr(d[msg], pre + "U0", t0)[1]
            tt = np.where(on, t, np.nan)
            ax[2 * k].plot(tt, nu, lw=0.6, label="nu, desired (%s)" % lab)
            ax[2 * k].plot(tt, wd, lw=0.6, label="wdot_f, measured (%s)" % lab)
            ax[2 * k + 1].plot(tt, u, lw=0.6, label="u, command (%s)" % lab)
            ax[2 * k + 1].plot(tt, u0, lw=0.6, label="u0_f, applied & filtered (%s)" % lab)
        ax[2 * k].set_ylabel("%s accel [rad/s²]" % axis)
        ax[2 * k + 1].set_ylabel("%s command [-]" % axis)
    for i, a in enumerate(ax):
        shade(a, phases, label=(i == 0))
        mark_pulses(a, pulses(d, t0))
        a.grid(alpha=0.3)
        a.legend(fontsize=7, ncol=3)
    ax[-1].set_xlabel("time [s]")
    fig.tight_layout()
    fig.savefig(out / ("%s_indi.png" % name), dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="run directories (containing flight.BIN)")
    ap.add_argument("--out", default="runs/figures")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    summary = {}
    for rd in map(Path, a.runs):
        name = rd.name
        print("figures for", name)
        res, d, t0, phases = analyse(rd / "flight.BIN")
        path_rms = fig_path(name, res, d, t0, phases, out)
        fig_energy(name, d, t0, phases, out)
        fig_rate(name, d, t0, phases, out)
        fig_indi(name, d, t0, phases, out)
        P = res["phases"]
        summary[name] = dict(
            completed=res["completed"],
            path_err_rms=path_rms,
            roll_rms={ph: (P[ph].get("att_roll_deg") or {}).get("rms") for ph in P},
            pitch_rms={ph: (P[ph].get("att_pitch_deg") or {}).get("rms") for ph in P},
            fw_tecs_h_rms=(P.get("fixed_wing", {}).get("tecs_alt_m") or {}).get("rms"),
            fw_tecs_h_max=(P.get("fixed_wing", {}).get("tecs_alt_m") or {}).get("max"),
            fw_airspeed=P.get("fixed_wing", {}).get("airspeed_stats"),
            fw_assist_pct=P.get("fixed_wing", {}).get("assist_pct_time"),
            fw_throttle_at_max_pct=(P.get("fixed_wing", {}).get("throttle_pct") or {}).get("pct_at_max"),
        )
    (out / "summary.json").write_text(json.dumps(summary, indent=1, default=float))

    # PID vs INDI bar charts, one group per wind case (run names <ctrl>_<case>...)
    cases = sorted({n.split("_", 1)[1] for n in summary if n.startswith(("pid_", "indi_"))})
    if cases:
        phs = ["hover_takeoff", "transition", "fixed_wing", "back_transition", "hover_land"]
        fig, axs = plt.subplots(len(cases), 3, figsize=(14, 3.6 * len(cases)), squeeze=False)
        for r, case in enumerate(cases):
            for c, (key, lab) in enumerate((("roll_rms", "roll error RMS [deg]"),
                                            ("pitch_rms", "pitch error RMS [deg]"),
                                            ("path_err_rms", "path error RMS [m]"))):
                ax = axs[r, c]
                x = np.arange(len(phs))
                for k, (ctrl, col) in enumerate((("pid", "#1f77b4"), ("indi", "#d62728"))):
                    s = summary.get("%s_%s" % (ctrl, case))
                    if not s:
                        continue
                    v = [s[key].get(p) or 0 for p in phs]
                    ax.bar(x + (k - 0.5) * 0.38, v, 0.38, color=col, label=ctrl.upper())
                ax.set_xticks(x)
                ax.set_xticklabels([PHASE_NAMES[p] for p in phs], rotation=20, fontsize=8)
                ax.set_ylabel(lab)
                ax.grid(alpha=0.3, axis="y")
                ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(out / "compare_phases.png", dpi=150)
        plt.close(fig)
    print(json.dumps(summary, indent=1, default=float))


if __name__ == "__main__":
    main()
