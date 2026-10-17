#!/usr/bin/env python3
"""How each controller uses its effectors against a wing-borne roll pulse.

    python3 analysis/pulse_effectors.py runs/joint_assist/zigzag [case ...]

For every wing-borne roll pulse (corner and midpoint windows) of the PID and
INDI runs of one case, aligned at the pulse start:
  row 1  roll attitude error change (desired - actual, minus its value before)
  row 2  aileron command, fraction of full travel (signed)
  row 3  lift-motor roll differential, (right pair - left pair) / 2, fraction
         of the motor range (0 when the motors are stopped)
Thin lines: single pulses; thick line: median. Pulse shaded.
Writes <root>/pulse_effectors/effectors_<case>.png / .pdf and a CSV with,
per pulse, peak roll error, time the aileron was saturated, and peak motor
differential.
"""
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_log import analyse, arr  # noqa: E402

COLORS = {"pid": "#2a78d6", "indi": "#eb6834"}
BEFORE, AFTER = 1.0, 4.0
GRID = np.arange(-BEFORE, AFTER, 0.02)


def pulses(rd):
    status = json.load(open(rd / "mission_status.json"))
    res, d, t0, phases = analyse(rd / "flight.BIN")
    fw = phases.get("fixed_wing")
    P = {r["Name"]: r["Value"] for r in d["PARM"]}
    ta, dr = arr(d["ATT"], "DesRoll", t0)
    _, r = arr(d["ATT"], "Roll", t0)
    err = dr - r
    rc = d["RCOU"]
    tr = np.array([x["TimeUS"] for x in rc]) * 1e-6 - t0
    trim, smin, smax = P.get("SERVO1_TRIM", 1500), P.get("SERVO1_MIN", 1000), P.get("SERVO1_MAX", 2000)
    pwm = np.array([x["C1"] for x in rc], float)
    ail = np.where(pwm >= trim, (pwm - trim) / (smax - trim), (pwm - trim) / (trim - smin))
    rng = P.get("Q_M_PWM_MAX", 2000) - P.get("Q_M_PWM_MIN", 1000)
    diff = ((np.array([x["C5"] + x["C8"] for x in rc], float)) - np.array([x["C6"] + x["C7"] for x in rc], float)) / 2 / rng
    out = []
    for p in status["pulses"]:
        if p["action"] != "roll" or not p["window"].startswith(("corner_", "mid_")):
            continue
        t = p["tb"] - t0
        if fw is None or not (fw[0] <= t < fw[1]):
            continue
        pre = (ta >= t - BEFORE) & (ta < t)
        e = np.interp(t + GRID, ta, err) - err[pre].mean()
        a = np.interp(t + GRID, tr, ail)
        m = np.interp(t + GRID, tr, diff)
        win = GRID >= 0
        out.append(dict(window=p["window"], e=e, a=a, m=m,
                        peak_err=float(np.abs(e[win]).max()),
                        ail_sat_s=float((np.abs(a[win]) >= 0.98).sum() * 0.02),
                        motor_diff_peak=float(np.abs(m[win]).max())))
    return out


def main():
    root = Path(sys.argv[1])
    cases = sys.argv[2:] or ["w6", "w9"]
    out = root / "pulse_effectors"
    out.mkdir(parents=True, exist_ok=True)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 8, "axes.spines.top": False, "axes.spines.right": False})
    rows = []
    for case in cases:
        data = {c: pulses(root / ("%s_%s_s1" % (c, case))) for c in ("pid", "indi")}
        fig, axs = plt.subplots(3, 2, figsize=(10, 7), sharex=True, sharey="row")
        for j, c in enumerate(("pid", "indi")):
            ps = data[c]
            for i, key in enumerate(("e", "a", "m")):
                ax = axs[i, j]
                ax.axvspan(0, 1, color="#c3c2b7", alpha=0.35, lw=0)
                for p in ps:
                    ax.plot(GRID, p[key], color=COLORS[c], lw=0.5, alpha=0.35)
                if ps:
                    ax.plot(GRID, np.median([p[key] for p in ps], axis=0), color=COLORS[c], lw=1.8)
                ax.axhline(0, color="#8c8b86", lw=0.6)
                ax.grid(True, color="#e6e5e0", lw=0.5)
            axs[0, j].set_title("%s, %d wing-borne roll pulses" % (c.upper(), len(ps)), fontsize=9)
            axs[2, j].set_xlabel("time from pulse start [s] (shaded: torque on)")
            for p in ps:
                rows.append(dict(case=case, ctrl=c, window=p["window"], peak_err_deg=p["peak_err"],
                                 aileron_saturated_s=p["ail_sat_s"], motor_diff_peak=p["motor_diff_peak"]))
        axs[1, 0].axhline(1.0, color="#52514e", lw=0.6, ls="--")
        axs[1, 0].axhline(-1.0, color="#52514e", lw=0.6, ls="--")
        axs[1, 1].axhline(1.0, color="#52514e", lw=0.6, ls="--")
        axs[1, 1].axhline(-1.0, color="#52514e", lw=0.6, ls="--")
        axs[0, 0].set_ylabel("roll error change [deg]")
        axs[1, 0].set_ylabel("aileron command [-1..1]")
        axs[2, 0].set_ylabel("lift-motor roll differential\n(right - left) [-]")
        fig.tight_layout()
        for ext in ("png", "pdf"):
            fig.savefig(out / ("effectors_%s.%s" % (case, ext)), dpi=150)
        plt.close(fig)
    S = pd.DataFrame(rows)
    S.to_csv(out / "pulses.csv", index=False, float_format="%.3f")
    if len(S):
        print(S.groupby(["case", "ctrl"]).agg(pulses=("peak_err_deg", "size"),
                                              peak_err_median=("peak_err_deg", "median"),
                                              peak_err_max=("peak_err_deg", "max"),
                                              ail_sat_s_mean=("aileron_saturated_s", "mean"),
                                              motor_diff_median=("motor_diff_peak", "median")).round(2).to_string())


if __name__ == "__main__":
    main()
