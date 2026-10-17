#!/usr/bin/env python3
"""Compare stock PID and INDI over the campaign runs (sim/run_campaign.sh).

    python analysis/compare_controllers.py [runs_dir]

Reads runs/<ctrl>_<case>_s<seed>/flight.BIN (ctrl pid|indi, case calm|w6|w9)
and writes runs/compare/: summary.md, runs.csv, pulses.csv, and plots.

Two kinds of metric:
  per phase  attitude/rate tracking, height, cross-track, saturation, assist
  per pulse  the roll/pitch moment pulses (found from RCOU.C9: 1900 roll,
             1100 pitch, 1500 off): peak deviation of that angle from its
             attitude error (desired - actual) from its value just before
             the pulse, integral of |deviation| over 5 s, time until it
             stays inside the pre-pulse error band (>= 1 deg), peak
             actuator command.

Seeds are repeats of the same case. A difference INDI - PID is marked as
"clear" only when it is larger than the spread (max - min) of both
controllers' seeds, i.e. larger than the run-to-run scatter.
"""
import json
import math
import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent))
from validate_log import analyse, arr  # noqa: E402
from alignment import run_clock, warp, shade_pulses  # noqa: E402

PULSE_WIN = 5.0          # s after pulse start used for pulse metrics (old 4 s-rest schedule)
PULSE_WIN_NEW = 3.0      # new schedule: next pulse starts 3 s after this one
COLORS = {"pid": "#2a78d6", "indi": "#eb6834"}   # validated categorical slots 1, 2
GROUPS = [("hover_takeoff", "take-off"), ("transition", "transition"),
          ("fixed_wing", "fixed-wing"), ("hover_land", "landing")]


def window_group(name):
    """corner_<seq> / mid_<seq> / old "cruise" windows are all fixed-wing."""
    if name.startswith(("corner_", "mid_")) or name == "cruise":
        return "fixed_wing"
    return name
SETTLE_DEG = 1.0
WINDOWS = ["hover_takeoff", "transition", "cruise", "hover_land"]   # pulse pair order
PULSE_NM = {"calm": 0.0, "w6": 1.44, "w9": 3.24}                   # sim/run_campaign.sh
IXX, IYY = 0.1399, 0.21384856799                                    # model.sdf
case_of = {}


def phase_metrics(res):
    P = res["phases"]
    g = lambda ph, key, f="rms": (P.get(ph, {}).get(key) or {}).get(f, np.nan)
    out = dict(completed=bool(res["completed"]))
    for ph in ("hover_takeoff", "hover_land", "transition", "back_transition", "fixed_wing"):
        out["%s_roll_rms" % ph] = g(ph, "att_roll_deg")
        out["%s_pitch_rms" % ph] = g(ph, "att_pitch_deg")
    for ph in ("hover_takeoff", "hover_land"):
        out["%s_alt_rms" % ph] = g(ph, "alt_m")
        out["%s_motor_max_pct" % ph] = (P.get(ph, {}).get("motors") or {}).get("pct_any_at_max", np.nan)
    fw = P.get("fixed_wing", {})
    out["fw_nav_roll_rms"] = g("fixed_wing", "fw_roll_deg")
    out["fw_nav_pitch_rms"] = g("fixed_wing", "fw_pitch_deg")
    out["fw_tecs_alt_rms"] = g("fixed_wing", "tecs_alt_m")
    out["fw_crosstrack_rms"] = (fw.get("crosstrack_m") or {}).get("rms", np.nan)
    out["fw_assist_pct"] = fw.get("assist_pct_time", np.nan)
    out["fw_airspeed_mean"] = (fw.get("airspeed_stats") or {}).get("mean", np.nan)
    for ph in ("hover_takeoff", "transition", "fixed_wing", "hover_land"):
        out["%s_indi_pct" % ph] = P.get(ph, {}).get("indi_active_pct_time", np.nan)
        out["%s_indf_pct" % ph] = P.get(ph, {}).get("indf_active_pct_time", np.nan)
    return out


def status_pulses(status, t0):
    """(t, window, axis) of every pulse start recorded by the mission harness.

    The harness stamps each pulse with the vehicle clock (ATTITUDE.time_boot_ms),
    which is the DataFlash TimeUS clock, so no 10 Hz RCOU marker can be missed."""
    out = []
    for p in (status or {}).get("pulses", []):
        if p.get("action") in ("roll", "pitch") and "tb" in p:
            out.append((float(p["tb"]) - t0, p["window"], p["action"]))
    return out


def pulses(res, d, t0, status=None):
    """One row per moment pulse."""
    if not d["RCOU"]:
        return []
    tr, c9 = arr(d["RCOU"], "C9", t0)
    # attitude tracking error (desired - actual): a commanded turn is not a
    # disturbance response, so work on the error rather than the raw angle
    ta, roll = arr(d["ATT"], "DesRoll", t0)
    roll = roll - arr(d["ATT"], "Roll", t0)[1]
    _, pitch = arr(d["ATT"], "DesPitch", t0)
    pitch = pitch - arr(d["ATT"], "Pitch", t0)[1]
    motors = np.array([[r["C%d" % i] for i in (5, 6, 7, 8)] for r in d["RCOU"]], float)
    ail = np.abs(np.array([r["C1"] for r in d["RCOU"]], float) - 1500) / 500
    elev = np.abs(np.array([r["C2"] for r in d["RCOU"]], float) - 1500) / 500
    phases = res["phases"]
    params = res["params"]
    # RCOU is only about 10 Hz; a marker can be missed if a pulse starts
    # before the next RCOU sample. Derive the trigger window from ArduPilot's
    # own log messages instead of shifting every later label by one pulse.
    triggers = (("Mission: 1 Takeoff", "hover_takeoff"),
                ("Transition started", "transition"),
                ("Reached waypoint #4", "cruise"),
                ("Land descend started", "hover_land"))
    trigger_times = []
    # Disturbance.on_text() uses each trigger only once, even if the vehicle
    # later re-enters transition during fixed-wing assist.
    for prefix, name in triggers:
        first = next((r for r in d["MSG"] if r["Message"].startswith(prefix)), None)
        if first is not None:
            trigger_times.append((first["TimeUS"] * 1e-6 - t0, name))
    trigger_times.sort()
    rows = []
    starts = status_pulses(status, t0)
    pulse_win = PULSE_WIN_NEW if starts else PULSE_WIN
    if not starts:
        for i in np.where(np.diff(c9) != 0)[0]:
            t, v = tr[i + 1], c9[i + 1]
            if v not in (1900, 1100):
                continue
            window = next((name for start, name in reversed(trigger_times) if start <= t), "unknown")
            starts.append((t, window, "roll" if v == 1900 else "pitch"))
    for t, window, axis in starts:
        ang = roll if axis == "roll" else pitch
        pre = (ta >= t - 1.0) & (ta < t)
        win = (ta >= t) & (ta < t + pulse_win)
        if pre.sum() < 3 or win.sum() < 10:
            continue
        base = ang[pre].mean()
        dev = np.abs(ang[win] - base)
        tw = ta[win]
        # settled = back inside the pre-pulse error band (at least 1 deg)
        band = max(SETTLE_DEG, 3.0 * float(ang[pre].std()))
        out_idx = np.where(dev > band)[0]
        settle = (tw[out_idx[-1]] - t) if len(out_idx) else 0.0
        dtw = np.diff(tw, append=tw[-1])
        phase = next((n for n, p in phases.items() if p["t_start"] <= t < p["t_end"]), "none")
        mw = (tr >= t) & (tr < t + 2.0)
        # INDI runs only (motors: INDI log, surfaces: INDF log):
        #  moment_est  moment INDI has taken over while the torque is on,
        #              dU0 * G * I, as a fraction of the injected moment
        #              (1 = the disturbance is fully estimated and cancelled)
        moment_est = np.nan
        for msg in ("INDF", "INDI"):
            if not d.get(msg):
                continue
            ti, act = arr(d[msg], "Act", t0)
            pre_ = "R" if axis == "roll" else "P"
            bit = 1 if axis == "roll" else 2
            on = (act.astype(int) & bit) != 0
            if ((ti >= t + 0.3) & (ti < t + 0.95) & on).sum() < 50:
                continue
            u0 = arr(d[msg], pre_ + "U0", t0)[1]
            before = (ti >= t - 0.5) & (ti < t) & on
            during = (ti >= t + 0.3) & (ti < t + 0.95) & on
            nm = PULSE_NM.get(case_of.get(id(d)), np.nan)
            if before.sum() > 10 and during.sum() > 10 and nm > 0:
                if msg == "INDI":
                    G = params.get("Q_INDI_RLL_G" if axis == "roll" else "Q_INDI_PIT_G") or np.nan
                else:
                    G = float(np.mean(arr(d[msg], "RG" if axis == "roll" else "PG", t0)[1][during]))
                inertia = IXX if axis == "roll" else IYY
                moment_est = float(abs(u0[during].mean() - u0[before].mean()) * G * inertia / nm)
            break
        rows.append(dict(
            t=round(float(t), 2), window=window, group=window_group(window), phase=phase, axis=axis,
            peak_dev_deg=float(dev.max()),
            iae_deg_s=float(np.sum(dev * dtw)),
            settle_s=float(settle),
            motor_peak=float(((motors[mw] - 1000) / 1000).max()) if mw.any() else np.nan,
            surface_peak=float((ail if axis == "roll" else elev)[mw].max()) if mw.any() else np.nan,
            moment_est=moment_est,
        ))
    return rows


def response_trace(d, t0, t, axis, before=1.0, after=PULSE_WIN_NEW):
    ax = "Roll" if axis == "roll" else "Pitch"
    ta, ang = arr(d["ATT"], "Des" + ax, t0)
    ang = ang - arr(d["ATT"], ax, t0)[1]
    m = (ta >= t - before) & (ta < t + after)
    pre = (ta >= t - before) & (ta < t)
    return ta[m] - t, ang[m] - ang[pre].mean()


def main():
    runs = Path(sys.argv[1] if len(sys.argv) > 1 else "runs")
    out = runs / "compare"
    out.mkdir(parents=True, exist_ok=True)
    pat = re.compile(r"^(pid|indi)_(calm|w6|w9)_s(\d+)$")

    run_rows, pulse_rows, traces, timelines = [], [], {}, {}
    for rd in sorted(runs.iterdir()):
        m = pat.match(rd.name)
        if not m or not (rd / "flight.BIN").exists():
            continue
        ctrl, case, seed = m.group(1), m.group(2), int(m.group(3))
        print("reading", rd.name)
        res, d, t0, _ = analyse(rd / "flight.BIN")
        case_of[id(d)] = case
        st_path = rd / "mission_status.json"
        status = json.load(open(st_path)) if st_path.exists() else None
        row = dict(run=rd.name, ctrl=ctrl, case=case, seed=seed)
        row.update(phase_metrics(res))
        run_rows.append(row)
        timelines[(ctrl, case, seed)] = timeline(res, d, t0, status)
        for p in pulses(res, d, t0, status):
            p.update(run=rd.name, ctrl=ctrl, case=case, seed=seed)
            pulse_rows.append(p)
            traces[(ctrl, case, seed, p["group"], p["axis"], p["window"])] = \
                response_trace(d, t0, p["t"], p["axis"])

    if not run_rows:
        raise SystemExit("no campaign runs found under %s" % runs)
    R = pd.DataFrame(run_rows)
    Pu = pd.DataFrame(pulse_rows)
    R.to_csv(out / "runs.csv", index=False)
    Pu.to_csv(out / "pulses.csv", index=False)

    L = ["# PID vs INDI - campaign summary", "",
         "Runs: %d (%s). Mean over seeds, [min, max] in brackets. Δ = INDI - PID; "
         "\"clear\" = |Δ| larger than the seed spread of both controllers." % (
             len(R), ", ".join(sorted(R.run))), ""]

    def table(df, keys, metrics, title):
        L.extend(["## " + title, "", "| " + " | ".join(k.rstrip("_") for k in keys) + " | metric | PID | INDI | Δ | clear |",
                  "|" + "---|" * (len(keys) + 6)])
        for grp, sub in df.groupby(keys):
            grp = grp if isinstance(grp, tuple) else (grp,)
            for met in metrics:
                a = sub[sub.ctrl == "pid"][met].dropna()
                b = sub[sub.ctrl == "indi"][met].dropna()
                if not len(a) and not len(b):
                    continue
                fa = "%.2f [%.2f, %.2f]" % (a.mean(), a.min(), a.max()) if len(a) else "-"
                fb = "%.2f [%.2f, %.2f]" % (b.mean(), b.min(), b.max()) if len(b) else "-"
                if len(a) and len(b):
                    delta = b.mean() - a.mean()
                    spread = max(a.max() - a.min(), b.max() - b.min())
                    clear = "yes" if abs(delta) > spread else "no"
                    fd = "%+.2f" % delta
                else:
                    fd, clear = "-", "-"
                L.append("| " + " | ".join(str(g) for g in grp) + " | %s | %s | %s | %s | %s |" % (met, fa, fb, fd, clear))
        L.append("")

    comp = R.groupby(["ctrl", "case"]).completed.agg(["sum", "count"]).reset_index()
    L += ["## Missions completed", "", "| ctrl | case | completed / runs |", "|---|---|---|"]
    L += ["| %s | %s | %d / %d |" % (r.ctrl, r.case, r["sum"], r["count"]) for _, r in comp.iterrows()]
    L.append("")

    R2 = R.copy()
    R2["case_"] = R2.case
    table(R2, ["case_"], [
        "hover_takeoff_roll_rms", "hover_takeoff_pitch_rms", "hover_land_roll_rms", "hover_land_pitch_rms",
        "transition_roll_rms", "transition_pitch_rms", "fixed_wing_roll_rms", "fixed_wing_pitch_rms",
        "hover_takeoff_alt_rms", "hover_land_alt_rms", "hover_land_motor_max_pct",
        "fw_tecs_alt_rms", "fw_crosstrack_rms", "fw_assist_pct", "fw_airspeed_mean",
        "hover_takeoff_indi_pct", "transition_indi_pct", "fixed_wing_indf_pct", "hover_land_indi_pct",
    ], "Per-phase metrics (deg, m, %)")

    if len(Pu):
        Pu2 = Pu.copy()
        table(Pu2, ["case", "group", "axis"],
              ["peak_dev_deg", "iae_deg_s", "settle_s", "motor_peak", "surface_peak", "moment_est"],
              "Moment-pulse response (deg, deg·s, s, 0-1; group = flight phase the pulse window "
              "belongs to; [min, max] over pulses and seeds)")

    (out / "summary.md").write_text("\n".join(L))
    print("\n".join(L))
    plot_pulses(traces, out)
    plot_timeline(timelines, out)


def timeline(res, d, t0, status):
    """Attitude error vs time since arming, with pulse windows and phase times."""
    ta, droll = arr(d["ATT"], "DesRoll", t0)
    _, roll = arr(d["ATT"], "Roll", t0)
    _, dpitch = arr(d["ATT"], "DesPitch", t0)
    _, pitch = arr(d["ATT"], "Pitch", t0)
    t_arm = res["events"]["arm"] or 0.0
    windows, pulses = run_clock(status, t0, t_arm)
    phases = {k: v["t_start"] - t_arm for k, v in res["phases"].items()}
    step = max(1, len(ta) // 8000)
    return dict(t=ta[::step] - t_arm, roll=(droll - roll)[::step], pitch=(dpitch - pitch)[::step],
                windows=windows, pulses=pulses, phases=phases)


def aligned(timelines, case):
    """{ctrl: (timeline, time map)} for one case, INDI mapped onto the PID clock."""
    runs = {k[0]: timelines[k] for k in sorted(timelines) if k[1] == case}
    ref = runs.get("pid") or next(iter(runs.values()))
    out = {}
    for ctrl, tl in runs.items():
        f = (lambda t: np.asarray(t, float)) if tl is ref else \
            warp(ref["windows"], tl["windows"], ref["phases"], tl["phases"])
        out[ctrl] = (tl, f)
    return out


def plot_timeline(timelines, out):
    """Roll and pitch attitude error on one clock for PID and INDI.

    The INDI run is mapped onto the PID clock (analysis/alignment.py) so that
    every pulse window both runs received starts at the same time; inside a
    window the time scale is true. Pulses both runs received are shaded grey,
    pulses only one run received in that run's colour; ticks mark phases."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 8, "axes.spines.top": False, "axes.spines.right": False})
    marks = [("transition", "T"), ("fixed_wing", "W"), ("back_transition", "B"), ("hover_land", "L")]
    for case in sorted({k[1] for k in timelines}):
        runs = aligned(timelines, case)
        fig, axs = plt.subplots(2, 1, figsize=(11, 5.2), sharex=True)
        pulses = {c: [(float(f(t)), a, w) for t, a, w in tl["pulses"]] for c, (tl, f) in runs.items()}
        for a, axis in zip(axs, ("roll", "pitch")):
            shade_pulses(a, pulses, COLORS, axis=axis)
        for ctrl, (tl, f) in sorted(runs.items(), reverse=True):
            for a, axis in zip(axs, ("roll", "pitch")):
                a.plot(f(tl["t"]), tl[axis], color=COLORS[ctrl], lw=0.7, label=ctrl.upper())
                for ph, lab in marks:
                    if ph in tl["phases"]:
                        x = float(f(tl["phases"][ph]))
                        a.axvline(x, color=COLORS[ctrl], lw=0.6, ls=":", alpha=0.8)
                        if axis == "roll":
                            a.text(x, 1.01, lab, transform=a.get_xaxis_transform(), color=COLORS[ctrl],
                                   ha="center", va="bottom", fontsize=7)
        for a, axis in zip(axs, ("roll", "pitch")):
            a.axhline(0, color="#c3c2b7", lw=0.6)
            a.set_ylabel("%s error, desired - actual [deg]" % axis)
            a.grid(True, color="#e6e5e0", lw=0.5)
        axs[0].legend(frameon=False, loc="upper right", ncol=2)
        axs[1].set_xlabel("mission time on the PID clock [s]; INDI aligned to the same pulse windows "
                          "(grey: 1 s pulse on that axis; T/W/B/L: transition, wing-borne, "
                          "back-transition, landing)")
        fig.tight_layout()
        fig.savefig(out / ("timeline_%s.png" % case), dpi=150)
        fig.savefig(out / ("timeline_%s.pdf" % case))
        plt.close(fig)


def plot_metrics(runs, pulse_rows, out):
    """Paired roll/pitch phase and pulse metrics; ranges show seed scatter."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    phases = GROUPS
    for case in ("w6", "w9"):
        group = runs[(runs.case == case) & runs.completed]
        if not {"pid", "indi"}.issubset(set(group.ctrl)):
            continue
        fig, axes = plt.subplots(2, 2, figsize=(11, 7.4))
        for col, axis in enumerate(("roll", "pitch")):
            ax = axes[0, col]
            x = np.arange(len(phases))
            for ctrl, offset in (("pid", -0.09), ("indi", 0.09)):
                color = COLORS[ctrl]
                df = group[group.ctrl == ctrl]
                values = [df["%s_%s_rms" % (phase, axis)].dropna() for phase, _ in phases]
                mean = np.array([v.mean() if len(v) else np.nan for v in values])
                low = np.array([v.min() if len(v) else np.nan for v in values])
                high = np.array([v.max() if len(v) else np.nan for v in values])
                ax.errorbar(x + offset, mean, yerr=np.array([mean - low, high - mean]),
                            fmt="o-", color=color, capsize=3, label=ctrl.upper())
            ax.set_xticks(x, [label for _, label in phases])
            ax.set_ylabel("%s attitude error RMS [deg]" % axis)
            ax.grid(alpha=0.3)
            ax.legend()

            ax = axes[1, col]
            p = pulse_rows[(pulse_rows.case == case) & (pulse_rows.axis == axis)]
            windows = GROUPS
            for ctrl, offset in (("pid", -0.09), ("indi", 0.09)):
                color = COLORS[ctrl]
                # The phase can differ at the same trigger; use the scheduled
                # pulse window and show scatter, never treat pulses as seeds.
                df = p[p.ctrl == ctrl]
                values = [df[df.group == window].peak_dev_deg.dropna() for window, _ in windows]
                mean = np.array([v.mean() if len(v) else np.nan for v in values])
                low = np.array([v.min() if len(v) else np.nan for v in values])
                high = np.array([v.max() if len(v) else np.nan for v in values])
                ax.errorbar(x + offset, mean, yerr=np.array([mean - low, high - mean]),
                            fmt="o-", color=color, capsize=3, label=ctrl.upper())
            ax.set_xticks(x, [label for _, label in windows])
            ax.set_ylabel("%s pulse peak error [deg]" % axis)
            ax.grid(alpha=0.3)
            ax.legend()
        fig.tight_layout()
        fig.savefig(out / ("metrics_%s.png" % case), dpi=170)
        plt.close(fig)


def plot_pulses(traces, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    keys = sorted({(case, ph, ax) for (_, case, _, ph, ax, _) in traces})
    if not keys:
        return
    cases = sorted({k[0] for k in keys})
    for case in cases:
        ks = [k for k in keys if k[0] == case]
        fig, axs = plt.subplots(len(ks), 1, figsize=(8, 2.4 * len(ks)), squeeze=False)
        for a, (c, ph, ax) in zip(axs[:, 0], ks):
            labelled = set()
            for (ctrl, cc, seed, pp, aa, win), (t, y) in sorted(traces.items()):
                if (cc, pp, aa) != (c, ph, ax):
                    continue
                a.plot(t, y, color=COLORS[ctrl], lw=1, alpha=0.7,
                       label=None if ctrl in labelled else ctrl.upper())
                labelled.add(ctrl)
            a.axvspan(0, 1, color="0.9")
            a.set_ylabel("%s %s\n%s error change [deg]" % (ph.replace("_", " "), ax, case))
            a.grid(alpha=0.3)
            a.legend(fontsize=6, ncol=3)
        axs[-1, 0].set_xlabel("time after pulse start (s); shaded = torque on")
        fig.tight_layout()
        fig.savefig(out / ("pulses_%s.png" % case), dpi=110)
        plt.close(fig)


if __name__ == "__main__":
    main()
