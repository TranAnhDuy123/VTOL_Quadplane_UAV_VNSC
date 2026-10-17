#!/usr/bin/env python3
"""Short reference table from the real flight log (context only, no comparison).

    python analysis/real_reference.py <real.bin> [sitl.param]

The only real flight of the aircraft is a manual QHOVER -> FBWA -> QHOVER
flight flown with PID. It is used as a general reference for what the
simulation should roughly look like: hover throttle, loop rate, filters,
gains, and how long/fast the FBWA part was. It is NOT compared against the
simulation run by run.

Writes <real.bin dir>/... nothing; prints Markdown to stdout.
"""
import sys
from pathlib import Path

import numpy as np
from pymavlink import mavutil

PARAMS = ["SCHED_LOOP_RATE", "INS_GYRO_FILTER", "INS_HNTCH_ENABLE", "INS_HNTCH_FREQ",
          "Q_M_THST_HOVER", "Q_M_THST_EXPO", "Q_M_SPIN_MIN", "Q_M_SPIN_MAX",
          "Q_A_RAT_RLL_P", "Q_A_RAT_RLL_I", "Q_A_RAT_RLL_D",
          "Q_A_RAT_PIT_P", "Q_A_RAT_PIT_I", "Q_A_RAT_PIT_D",
          "Q_A_RAT_YAW_P", "Q_A_RAT_YAW_I", "Q_A_ANG_RLL_P", "Q_A_ANG_PIT_P",
          "RLL_RATE_P", "RLL_RATE_I", "RLL_RATE_FF", "PTCH_RATE_P", "PTCH_RATE_I", "PTCH_RATE_FF",
          "ARSPD_TYPE", "AIRSPEED_CRUISE"]


def read(path):
    m = mavutil.mavlink_connection(str(path))
    d = {k: [] for k in ("PARM", "MODE", "QTUN", "GPS", "ATT")}
    while True:
        x = m.recv_match(type=list(d))
        if x is None:
            return d
        d[x.get_type()].append(x.to_dict())


def load_param_file(path):
    out = {}
    for line in Path(path).read_text().splitlines():
        f = line.split()
        if len(f) >= 2 and not line.startswith("#"):
            try:
                out[f[0]] = float(f[1])
            except ValueError:
                pass
    return out


def main():
    log = Path(sys.argv[1])
    sitl = load_param_file(sys.argv[2]) if len(sys.argv) > 2 else {}
    d = read(log)
    p = {r["Name"]: r["Value"] for r in d["PARM"]}
    t0 = d["ATT"][0]["TimeUS"] * 1e-6
    names = mavutil.mode_mapping_bynumber(mavutil.mavlink.MAV_TYPE_FIXED_WING)
    modes = [(r["TimeUS"] * 1e-6 - t0, names.get(r["ModeNum"], str(r["ModeNum"]))) for r in d["MODE"]]
    t_end = d["ATT"][-1]["TimeUS"] * 1e-6 - t0

    # Airborne hover = lift-motor throttle clearly above idle.
    tq = np.array([r["TimeUS"] for r in d["QTUN"]]) * 1e-6 - t0
    tho = np.array([r["ThO"] for r in d["QTUN"]])
    air = tho > 0.2
    # GPS ground speed during FBWA.
    tg = np.array([r["TimeUS"] for r in d["GPS"]]) * 1e-6 - t0
    spd = np.array([r["Spd"] for r in d["GPS"]])

    print("# Real flight reference: `%s`\n" % log.name)
    print("Context only - not compared run by run with the simulation.\n")
    print("## Flight modes\n")
    for i, (t, name) in enumerate(modes):
        t2 = modes[i + 1][0] if i + 1 < len(modes) else t_end
        seg = (tg >= t) & (tg < t2)
        extra = ""
        if name == "FBWA" and seg.any():
            extra = " - GPS ground speed mean %.1f / max %.1f m/s" % (spd[seg].mean(), spd[seg].max())
        print("- %.1f-%.1f s (%.1f s): %s%s" % (t, t2, t2 - t, name, extra))
    if air.any():
        print("\n## Hover\n")
        print("- airborne (QTUN ThO > 0.2): %.1f-%.1f s" % (tq[air][0], tq[air][-1]))
        print("- hover throttle (ThO) mean %.3f, Q_M_THST_HOVER %.3f" % (tho[air].mean(), p.get("Q_M_THST_HOVER", float("nan"))))
    print("\n## Settings: real aircraft vs SITL file\n")
    print("| param | real | SITL |\n|---|---|---|")
    for k in PARAMS:
        r, s = p.get(k), sitl.get(k)
        flag = "" if r is None or s is None or abs(r - s) < 1e-6 else " (differs)"
        print("| %s | %s | %s%s |" % (k, "-" if r is None else "%g" % r, "-" if s is None else "%g" % s, flag))


if __name__ == "__main__":
    main()
