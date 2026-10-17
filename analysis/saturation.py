#!/usr/bin/env python3
"""Effector use and saturation, PID vs INDI, per profile.

    python3 analysis/saturation.py runs/joint/square [runs/joint/zigzag ...]

From RCOU and the logged parameters of every <ctrl>_<case>_s<seed> run:

  lift motors  normalised (PWM - Q_M_PWM_MIN) / range; "on" when any motor is
               above Q_M_SPIN_ARM; "at max" when any motor is at Q_M_SPIN_MAX,
               "at min" when the motors are on and any is at Q_M_SPIN_MIN
  surfaces     aileron / elevator deflection about SERVOn_TRIM, normalised
               to each side's travel; saturated at |u| >= 0.98
  pusher       throttle (SERVO3) at its maximum

Writes <root>/saturation/summary.md, summary.csv (share of time per phase)
and pulses.csv (whether each pulse met a saturated effector within 2 s of its
start), and saturation_<case>.png / .pdf: effector traces on the PID clock
(INDI aligned to the same pulse windows, analysis/alignment.py), pulses shaded.
"""
import json
import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_log import analyse  # noqa: E402
from alignment import run_clock, warp, shade_pulses  # noqa: E402

COLORS = {"pid": "#2a78d6", "indi": "#eb6834"}
CRITICAL = "#d03b3b"
PHASES = ["hover_takeoff", "transition", "fixed_wing", "back_transition", "hover_land"]
SURF_SAT = 0.98
LIMIT_TOL = 0.006


def effectors(d, t0, t_arm):
    P = {r["Name"]: r["Value"] for r in d["PARM"]}
    lo, hi = P.get("Q_M_PWM_MIN", 1000), P.get("Q_M_PWM_MAX", 2000)
    spin_arm, spin_min, spin_max = P.get("Q_M_SPIN_ARM", 0.1), P.get("Q_M_SPIN_MIN", 0.15), P.get("Q_M_SPIN_MAX", 0.95)
    rc = d["RCOU"]
    t = np.array([r["TimeUS"] for r in rc]) * 1e-6 - t0 - t_arm
    m = (np.array([[r["C%d" % i] for i in (5, 6, 7, 8)] for r in rc], float) - lo) / (hi - lo)

    def surface(ch):
        trim, smin, smax = P.get("SERVO%d_TRIM" % ch, 1500), P.get("SERVO%d_MIN" % ch, 1000), P.get("SERVO%d_MAX" % ch, 2000)
        pwm = np.array([r["C%d" % ch] for r in rc], float)
        return np.where(pwm >= trim, (pwm - trim) / max(smax - trim, 1), (pwm - trim) / max(trim - smin, 1))

    thr_min, thr_max = P.get("SERVO3_MIN", 1100), P.get("SERVO3_MAX", 1900)
    thr = (np.array([r["C3"] for r in rc], float) - thr_min) / (thr_max - thr_min)
    on = m.max(1) > spin_arm - 0.02
    return dict(t=t, motors=m, on=on,
                at_max=on & (m.max(1) >= spin_max - LIMIT_TOL),
                at_min=on & (m.min(1) <= spin_min + LIMIT_TOL),
                ail=surface(1), elev=surface(2), thr=thr,
                ail_sat=np.abs(surface(1)) >= SURF_SAT, elev_sat=np.abs(surface(2)) >= SURF_SAT,
                thr_max=thr >= 0.98, spin_min=spin_min, spin_max=spin_max)


def run(rd):
    status = json.load(open(rd / "mission_status.json"))
    res, d, t0, phases = analyse(rd / "flight.BIN")
    t_arm = res["events"]["arm"] or 0.0
    e = effectors(d, t0, t_arm)
    windows, pulses = run_clock(status, t0, t_arm)
    ph = {k: (v[0] - t_arm, v[1] - t_arm) for k, v in phases.items()}
    rows = []
    for name in PHASES + ["whole_flight"]:
        if name == "whole_flight":
            w = np.ones_like(e["t"], bool)
        elif name in ph:
            w = (e["t"] >= ph[name][0]) & (e["t"] < ph[name][1])
        else:
            continue
        if not w.any():
            continue
        pct = lambda k: 100.0 * float(e[k][w].mean())
        rows.append(dict(phase=name, motors_on=pct("on"), motor_at_max=pct("at_max"),
                         motor_at_min=pct("at_min"), aileron_sat=pct("ail_sat"),
                         elevator_sat=pct("elev_sat"), throttle_max=pct("thr_max")))
    prow = []
    for t, axis, window in pulses:
        w = (e["t"] >= t) & (e["t"] < t + 2.0)
        phase = next((k for k, (a, b) in ph.items() if a <= t < b), "none")
        sat_surface = bool((e["ail_sat"] if axis == "roll" else e["elev_sat"])[w].any())
        prow.append(dict(t=round(t, 2), window=window, axis=axis, phase=phase,
                         motors_on=bool(e["on"][w].any()),
                         motor_limit=bool((e["at_max"][w] | e["at_min"][w]).any()),
                         surface_sat=sat_surface))
    return rows, prow, dict(e=e, windows=windows, pulses=pulses, phases=ph)


def plot_case(case, runs, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 8, "axes.spines.top": False, "axes.spines.right": False})
    ref = runs.get("pid") or next(iter(runs.values()))
    maps = {c: ((lambda t: np.asarray(t, float)) if r is ref else
                warp(ref["windows"], r["windows"], {k: v[0] for k, v in ref["phases"].items()},
                     {k: v[0] for k, v in r["phases"].items()}))
            for c, r in runs.items()}
    order = [c for c in ("pid", "indi") if c in runs]
    fig, axs = plt.subplots(len(order) + 3, 1, figsize=(11, 1.9 * (len(order) + 3)), sharex=True)
    pulses = {c: [(float(maps[c](t)), a, w) for t, a, w in runs[c]["pulses"]] for c in order}
    for a in axs:
        shade_pulses(a, pulses, COLORS)
    for i, c in enumerate(order):
        e, f = runs[c]["e"], maps[c]
        t = f(e["t"])
        a = axs[i]
        m = e["motors"]
        a.fill_between(t, m.min(1), m.max(1), color=COLORS[c], alpha=0.35, lw=0, label="%s lift motors, min to max" % c.upper())
        a.plot(t, m.mean(1), color=COLORS[c], lw=0.6)
        for lim in (e["spin_min"], e["spin_max"]):
            a.axhline(lim, color="#52514e", lw=0.6, ls="--")
        sat = e["at_max"] | e["at_min"]
        a.scatter(t[sat], np.full(sat.sum(), 1.04), s=4, marker="|", color=CRITICAL, label="a motor at its limit")
        a.set_ylim(-0.02, 1.32)
        a.set_ylabel("%s lift motors [0-1]" % c.upper())
        a.legend(frameon=False, loc="upper right", ncol=2, fontsize=7)
    for a, key, lab in ((axs[len(order)], "ail", "aileron |u|"), (axs[len(order) + 1], "elev", "elevator |u|"),
                        (axs[len(order) + 2], "thr", "pusher throttle")):
        for c in order:
            e = runs[c]["e"]
            a.plot(maps[c](e["t"]), np.abs(e[key]), color=COLORS[c], lw=0.6, label=c.upper())
        a.axhline(1.0, color="#52514e", lw=0.6, ls="--")
        a.set_ylim(-0.02, 1.08)
        a.set_ylabel(lab)
        a.legend(frameon=False, loc="upper left", ncol=2, fontsize=7)
    for a in axs:
        a.grid(True, color="#e6e5e0", lw=0.5)
    axs[-1].set_xlabel("mission time on the PID clock [s]; INDI aligned to the same pulse windows "
                       "(grey: 1 s torque pulse; red ticks: a lift motor at its minimum or maximum; "
                       "dashed: limits)")
    fig.tight_layout()
    for ext in ("png", "pdf"):
        fig.savefig(out / ("saturation_%s.%s" % (case, ext)), dpi=150)
    plt.close(fig)


def main():
    roots = [Path(a) for a in sys.argv[1:]] or [Path("runs/joint/square"), Path("runs/joint/zigzag")]
    pat = re.compile(r"^(pid|indi)_(calm|w6|w9)_s(\d+)$")
    for root in roots:
        out = root / "saturation"
        out.mkdir(parents=True, exist_ok=True)
        rows, prows, series = [], [], {}
        for rd in sorted(root.iterdir()):
            m = pat.match(rd.name)
            if not m or not (rd / "flight.BIN").exists():
                continue
            ctrl, case = m.group(1), m.group(2)
            print("reading", rd)
            r, p, s = run(rd)
            for x in r:
                x.update(ctrl=ctrl, case=case)
            for x in p:
                x.update(ctrl=ctrl, case=case)
            rows += r
            prows += p
            series.setdefault(case, {})[ctrl] = s
        S, Pp = pd.DataFrame(rows), pd.DataFrame(prows)
        S.to_csv(out / "summary.csv", index=False, float_format="%.2f")
        Pp.to_csv(out / "pulses.csv", index=False)
        L = ["# Effector use and saturation — %s" % root.name, "",
             "Share of time [%%] per phase. Lift motors 'at max/min': any of the four at Q_M_SPIN_MAX / "
             "Q_M_SPIN_MIN while spinning. Surfaces saturated at |u| >= %.2f of their travel." % SURF_SAT, "",
             "| case | phase | ctrl | motors on | motor at max | motor at min | aileron sat | elevator sat | pusher at max |",
             "|---|---|---|---:|---:|---:|---:|---:|---:|"]
        for case in ("calm", "w6", "w9"):
            for ph in PHASES + ["whole_flight"]:
                for ctrl in ("pid", "indi"):
                    g = S[(S.case == case) & (S.phase == ph) & (S.ctrl == ctrl)]
                    if g.empty:
                        continue
                    r = g.iloc[0]
                    L.append("| %s | %s | %s | %.1f | %.1f | %.1f | %.1f | %.1f | %.1f |" % (
                        case, ph, ctrl.upper(), r.motors_on, r.motor_at_max, r.motor_at_min,
                        r.aileron_sat, r.elevator_sat, r.throttle_max))
        if len(Pp):
            L += ["", "## Pulses meeting a saturated effector (within 2 s of the pulse start)", "",
                  "| case | ctrl | phase group | pulses | lift motors on | a motor at its limit | own-axis surface saturated |",
                  "|---|---|---|---:|---:|---:|---:|"]
            Pp["group"] = np.where(Pp.phase == "fixed_wing", "wing-borne", "hover/transition")
            for (case, ctrl, grp), g in Pp.groupby(["case", "ctrl", "group"]):
                L.append("| %s | %s | %s | %d | %d | %d | %d |" % (case, ctrl.upper(), grp, len(g),
                                                                 g.motors_on.sum(), g.motor_limit.sum(), g.surface_sat.sum()))
        (out / "summary.md").write_text("\n".join(L) + "\n")
        print("\n".join(L))
        for case, runs in series.items():
            plot_case(case, runs, out)


if __name__ == "__main__":
    main()
