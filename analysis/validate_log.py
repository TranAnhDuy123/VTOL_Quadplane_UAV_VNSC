#!/usr/bin/env python3
"""Measure a square-mission DataFlash log and write a validation report.

    python analysis/validate_log.py runs/<name>/flight.BIN

Writes <log dir>/validation.md, validation.json and validation.png.

The flight is split into phases using the autopilot's own STATUSTEXT
messages, so the boundaries are the ones ArduPlane itself used:

    hover_takeoff   arm/AUTO            -> "Transition started"
    transition      "Transition started" -> "Transition done"
    fixed_wing      "Transition done"    -> first "VTOL ..." (back-transition)
    back_transition first "VTOL ..."     -> "VTOL position2 started"
    hover_land      "VTOL position2"     -> "Land complete" / disarm

Every number is computed from the log. Parameters are read back from PARM,
not from the file that was supposed to load them.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
from pymavlink import mavutil

TYPES = ["PARM", "MSG", "MODE", "ATT", "RATE", "PIQR", "PIQP", "PIQY",
         "PIDR", "PIDP", "CTUN", "NTUN", "TECS", "QTUN", "RCOU", "POS", "ARSP", "EV",
         "INDI", "INDF", "GPS"]

PARAMS_OF_INTEREST = [
    "Q_ENABLE", "Q_OPTIONS", "Q_ASSIST_SPEED", "AHRS_EKF_TYPE", "SIM_WIND_SPD",
    "AIRSPEED_CRUISE", "AIRSPEED_MIN", "AIRSPEED_STALL",
    "Q_A_RAT_RLL_P", "Q_A_RAT_RLL_I", "Q_A_RAT_RLL_D", "Q_A_RAT_RLL_FF",
    "Q_A_RAT_PIT_P", "Q_A_RAT_PIT_I", "Q_A_RAT_PIT_D", "Q_A_RAT_PIT_FF",
    "Q_A_RAT_YAW_P", "Q_A_RAT_YAW_I", "Q_A_RAT_YAW_D",
    "Q_A_ANG_RLL_P", "Q_A_ANG_PIT_P", "Q_A_ANG_YAW_P",
    "RLL_RATE_P", "RLL_RATE_I", "RLL_RATE_D", "RLL_RATE_FF",
    "PTCH_RATE_P", "PTCH_RATE_I", "PTCH_RATE_D", "PTCH_RATE_FF",
    "Q_M_PWM_MIN", "Q_M_PWM_MAX", "Q_M_SPIN_MIN", "Q_M_THST_HOVER",
    "Q_INDI_ENABLE", "Q_INDI_AXES", "Q_INDI_FILT_HZ", "Q_INDI_RLL_K", "Q_INDI_PIT_K",
    "Q_INDI_RLL_G", "Q_INDI_PIT_G", "Q_INDI_ACT_TC", "Q_INDI_ACT_DLY",
    "Q_INDI_FW_RLL_G", "Q_INDI_FW_PIT_G", "Q_INDI_FW_TC", "Q_INDI_FW_RLL_K", "Q_INDI_FW_PIT_K",
]


def read_log(path):
    mlog = mavutil.mavlink_connection(str(path))
    data = {t: [] for t in TYPES}
    while True:
        m = mlog.recv_match(type=TYPES)
        if m is None:
            break
        data[m.get_type()].append(m.to_dict())
    return data


def arr(rows, field, t0=0.0):
    """(time_s, values) arrays for one field; time relative to t0."""
    if not rows:
        return np.array([]), np.array([])
    t = np.array([r["TimeUS"] for r in rows]) * 1e-6 - t0
    v = np.array([float(r[field]) for r in rows])
    return t, v


def window(t, v, lo, hi):
    m = (t >= lo) & (t < hi)
    return t[m], v[m]


def rms(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else float("nan")


def wrap180(x):
    return (x + 180.0) % 360.0 - 180.0


def tracking(rows, des, act, lo, hi, t0, wrap=False, interp_from=None):
    """RMS / max |error| and mean |des| of des-act inside [lo, hi)."""
    t, d = arr(rows, des, t0)
    _, a = arr(rows, act, t0)
    m = (t >= lo) & (t < hi)
    e = d[m] - a[m]
    if wrap:
        e = wrap180(e)
    if not len(e):
        return None
    return dict(n=int(len(e)), rms=rms(e), max=float(np.max(np.abs(e))),
                des_rms=rms(d[m]))


def dominant_freq(t, x, lo, hi):
    """Dominant frequency (Hz) of x in [lo, hi) and fraction of power above 5 Hz.

    A limit cycle in the rate loop shows up as a sharp peak carrying most of the
    power; healthy tracking has its power at the (low) command frequencies.
    """
    tt, xx = window(t, x, lo, hi)
    if len(tt) < 256:
        return None
    fs = 1.0 / np.median(np.diff(tt))
    xx = xx - np.mean(xx)
    spec = np.abs(np.fft.rfft(xx * np.hanning(len(xx)))) ** 2
    f = np.fft.rfftfreq(len(xx), 1.0 / fs)
    spec[0] = 0.0
    k = int(np.argmax(spec))
    # Absolute size of the >5 Hz content (RMS, same units as x) via a
    # brick-wall high-pass: a large *fraction* of a tiny signal is just noise.
    X = np.fft.rfft(xx)
    X[f <= 5.0] = 0.0
    hf = np.fft.irfft(X, n=len(xx))
    return dict(fs_hz=float(fs), peak_hz=float(f[k]),
                power_above_5hz=float(spec[f > 5.0].sum() / max(spec.sum(), 1e-12)),
                rms_above_5hz=rms(hf), rms_total=rms(xx))


def analyse(log):
    """Measure one log. Returns (result dict, raw messages, t0, phases)."""
    log = Path(log).resolve()
    d = read_log(log)

    params = {}
    for r in d["PARM"]:
        params[r["Name"]] = r["Value"]
    param_view = {k: params.get(k) for k in PARAMS_OF_INTEREST}

    # Time zero = first ATT sample.
    t0 = d["ATT"][0]["TimeUS"] * 1e-6
    msgs = [(r["TimeUS"] * 1e-6 - t0, r["Message"]) for r in d["MSG"]]
    mode_names = mavutil.mode_mapping_bynumber(mavutil.mavlink.MAV_TYPE_FIXED_WING)
    modes = [(r["TimeUS"] * 1e-6 - t0, mode_names.get(r["ModeNum"], str(r["ModeNum"])),
              r.get("Rsn")) for r in d["MODE"]]

    def first(prefix, after=-1e9):
        for t, s in msgs:
            if t > after and s.startswith(prefix):
                return t
        return None

    # Arm / disarm. With LOG_DISARMED=0 the log starts at arming, so the EV
    # "armed" event is never recorded; the "Throttle armed" text is.
    t_arm = next((r["TimeUS"] * 1e-6 - t0 for r in d["EV"] if r["Id"] == 10), None)
    if t_arm is None:
        t_arm = first("Throttle armed")
    t_disarm = next((r["TimeUS"] * 1e-6 - t0 for r in d["EV"]
                     if r["Id"] == 11 and t_arm is not None
                     and r["TimeUS"] * 1e-6 - t0 > t_arm), None)
    t_end = t_disarm if t_disarm is not None else d["ATT"][-1]["TimeUS"] * 1e-6 - t0

    t_tr0 = first("Transition started")
    t_tr1 = first("Transition done", after=t_tr0 or 0)
    # "VTOL approach" is still fixed-wing flight; the back-transition starts at
    # airbrake (or position1 / "pos1 low speed" if the airbrake stage is skipped).
    # In wind the airbrake stage can be skipped and the back-transition
    # starts with "VTOL pos1 low speed" instead.
    t_bt = min([t for t in (first("VTOL airbrake", after=t_tr1 or 0),
                            first("VTOL position1", after=t_tr1 or 0),
                            first("VTOL pos1", after=t_tr1 or 0)) if t is not None],
               default=None)
    t_p2 = first("VTOL position2", after=t_bt or 0)
    t_land = first("Land complete", after=t_p2 or 0) or t_end

    phases = {}
    if t_arm is not None and t_tr0 is not None:
        phases["hover_takeoff"] = (t_arm, t_tr0)
    if t_tr0 is not None and t_tr1 is not None:
        phases["transition"] = (t_tr0, t_tr1)
    if t_tr1 is not None and t_bt is not None:
        phases["fixed_wing"] = (t_tr1, t_bt)
    if t_bt is not None and t_p2 is not None:
        phases["back_transition"] = (t_bt, t_p2)
    if t_p2 is not None:
        phases["hover_land"] = (t_p2, t_land)

    pwm_min = params.get("Q_M_PWM_MIN", 1000) or 1000
    pwm_max = params.get("Q_M_PWM_MAX", 2000) or 2000
    spin_min = params.get("Q_M_SPIN_MIN", 0.15)

    res = dict(log=str(log), params=param_view,
               events=dict(arm=t_arm, transition_start=t_tr0, transition_done=t_tr1,
                           back_transition=t_bt, position2=t_p2, land_complete=t_land,
                           disarm=t_disarm),
               modes=[dict(t=round(t, 2), mode=m) for t, m, _ in modes],
               phases={})

    # QTUN-derived assist duty as a fraction of *time*: QTUN is only written
    # while the lift motors run, so a row-count fraction would read ~100 %.
    tq, ast = arr(d["QTUN"], "Ast", t0)
    tq_dt = np.diff(tq, append=tq[-1] if len(tq) else 0)
    tq_dt[tq_dt > 0.2] = 0.0   # a gap means QTUN stopped: motors off, not assist

    # Lift-motor PWM (SERVO5-8) and forward throttle (SERVO3).
    tr = np.array([r["TimeUS"] for r in d["RCOU"]]) * 1e-6 - t0
    motors = np.array([[r["C%d" % i] for i in (5, 6, 7, 8)] for r in d["RCOU"]], float)
    motor_frac = (motors - pwm_min) / (pwm_max - pwm_min)

    for name, (lo, hi) in phases.items():
        p = dict(t_start=round(lo, 2), t_end=round(hi, 2), duration_s=round(hi - lo, 2))
        p["att_roll_deg"] = tracking(d["ATT"], "DesRoll", "Roll", lo, hi, t0)
        p["att_pitch_deg"] = tracking(d["ATT"], "DesPitch", "Pitch", lo, hi, t0)
        # DesYaw is only a real command while hovering; in fixed-wing flight
        # yaw is not a controlled variable.
        p["att_yaw_deg"] = (tracking(d["ATT"], "DesYaw", "Yaw", lo, hi, t0, wrap=True)
                            if name.startswith("hover") else None)

        if name.startswith("hover") or name in ("transition", "back_transition"):
            # Quadplane rate PIDs (target vs actual, deg/s).
            for ax, msg in (("roll", "PIQR"), ("pitch", "PIQP"), ("yaw", "PIQY")):
                tr_ = tracking(d[msg], "Tar", "Act", lo, hi, t0)
                if tr_:
                    tr_ = {k: (v * 180 / math.pi if k != "n" else v) for k, v in tr_.items()}
                p["rate_%s_dps" % ax] = tr_
            p["alt_m"] = tracking(d["QTUN"], "DAlt", "Alt", lo, hi, t0)
            p["climb_ms"] = tracking(d["QTUN"], "DCRt", "CRt", lo, hi, t0)
            m = (tr >= lo) & (tr < hi)
            if m.any():
                mf = motor_frac[m]
                p["motors"] = dict(
                    mean=[round(float(x), 3) for x in mf.mean(axis=0)],
                    max=[round(float(x), 3) for x in mf.max(axis=0)],
                    pct_any_at_max=round(100 * float(np.mean((mf >= 0.98).any(axis=1))), 2),
                    pct_any_at_min=round(100 * float(np.mean((mf <= spin_min + 0.01).any(axis=1))), 2),
                )
            # Limit-cycle check on the roll/pitch body rate.
            for ax, fld in (("roll", "R"), ("pitch", "P")):
                t_, v_ = arr(d["RATE"], fld, t0)
                p["spectrum_%s" % ax] = dominant_freq(t_, v_, lo, hi)

        if name in ("fixed_wing", "transition"):
            p["fw_roll_deg"] = tracking(d["CTUN"], "NavRoll", "Roll", lo, hi, t0)
            p["fw_pitch_deg"] = tracking(d["CTUN"], "NavPitch", "Pitch", lo, hi, t0)
            p["airspeed_ms"] = tracking(d["TECS"], "spdem", "sp", lo, hi, t0)
            t_, a_ = arr(d["CTUN"], "As", t0)
            _, a_ = window(t_, a_, lo, hi)
            if len(a_):
                p["airspeed_stats"] = dict(min=float(a_.min()), mean=float(a_.mean()),
                                           max=float(a_.max()))
            p["fw_rate_roll"] = tracking(d["PIDR"], "Tar", "Act", lo, hi, t0)
            p["fw_rate_pitch"] = tracking(d["PIDP"], "Tar", "Act", lo, hi, t0)
            p["tecs_alt_m"] = tracking(d["TECS"], "hdem", "h", lo, hi, t0)
            t_, xt = arr(d["NTUN"], "XT", t0)
            _, xt = window(t_, xt, lo, hi)
            if len(xt):
                p["crosstrack_m"] = dict(rms=rms(xt), max=float(np.max(np.abs(xt))))
            t_, tho = arr(d["CTUN"], "ThO", t0)
            _, tho = window(t_, tho, lo, hi)
            if len(tho):
                p["throttle_pct"] = dict(mean=float(tho.mean()), max=float(tho.max()),
                                         pct_at_max=round(100 * float(np.mean(tho >= 99)), 1))
            mq = (tq >= lo) & (tq < hi)
            p["assist_pct_time"] = round(100 * float(tq_dt[mq][(ast[mq].astype(int) & 1) == 1].sum())
                                         / max(hi - lo, 1e-6), 1)
            p["assist_pct_time"] = min(p["assist_pct_time"], 100.0)
            m = (tr >= lo) & (tr < hi)
            if m.any():
                p["lift_motors_above_idle_pct"] = round(
                    100 * float(np.mean((motor_frac[m] > 0.02).any(axis=1))), 2)

        # Share of the phase's *time* in which INDI ran roll and pitch
        # (INDI = lift motors, INDF = surfaces); both logged every loop.
        for msg in ("INDI", "INDF"):
            if d[msg]:
                t_, act = arr(d[msg], "Act", t0)
                m = (t_ >= lo) & (t_ < hi)
                if m.sum() > 1:
                    dt_ = np.diff(t_[m], append=t_[m][-1])
                    dt_[dt_ > 0.1] = 0.0
                    both = (act[m].astype(int) & 3) == 3
                    p["%s_active_pct_time" % msg.lower()] = round(100 * float(dt_[both].sum()) / max(hi - lo, 1e-6), 1)
        res["phases"][name] = p

    res["completed"] = t_disarm is not None and "hover_land" in phases
    res["statustext"] = [dict(t=round(t, 2), text=s) for t, s in msgs]
    return res, d, t0, phases


def main():
    log = Path(sys.argv[1]).resolve()
    out_dir = log.parent
    res, d, t0, phases = analyse(log)
    (out_dir / "validation.json").write_text(json.dumps(res, indent=1, default=float))
    write_markdown(res, out_dir / "validation.md")
    plot(d, t0, phases, out_dir / "validation.png")
    print((out_dir / "validation.md").read_text())


def fmt(x, nd=2):
    if x is None:
        return "-"
    if isinstance(x, float) and math.isnan(x):
        return "-"
    return ("%%.%df" % nd) % x


def write_markdown(res, path):
    L = ["# Validation: %s" % Path(res["log"]).parent.name, "",
         "Log: `%s`  " % res["log"],
         "Mission completed (landed + disarmed): **%s**" % res["completed"], "",
         "## Parameters read back from the log", "", "| param | value |", "|---|---|"]
    L += ["| %s | %s |" % (k, v) for k, v in res["params"].items()]
    L += ["", "## Events (s from log start)", ""]
    L += ["- %s: %s" % (k, fmt(v, 1)) for k, v in res["events"].items()]
    L += ["", "## Phases", "",
          "| phase | start | end | dur (s) | INDI on motors (% time) | INDI on surfaces (% time) |",
          "|---|---|---|---|---|---|"]
    for n, p in res["phases"].items():
        L.append("| %s | %.1f | %.1f | %.1f | %s | %s |" % (
            n, p["t_start"], p["t_end"], p["duration_s"],
            p.get("indi_active_pct_time", "-"), p.get("indf_active_pct_time", "-")))

    L += ["", "## Attitude tracking (desired - actual, deg)", "",
          "| phase | roll RMS | roll max | pitch RMS | pitch max | yaw RMS | yaw max |",
          "|---|---|---|---|---|---|---|"]
    for n, p in res["phases"].items():
        r, q, y = p["att_roll_deg"], p["att_pitch_deg"], p["att_yaw_deg"]
        L.append("| %s | %s | %s | %s | %s | %s | %s |" % (
            n, fmt(r and r["rms"]), fmt(r and r["max"]), fmt(q and q["rms"]),
            fmt(q and q["max"]), fmt(y and y["rms"]), fmt(y and y["max"])))

    L += ["", "## VTOL rate loop (PIQx target - actual, deg/s)", "",
          "| phase | roll RMS | pitch RMS | yaw RMS | roll peak Hz | roll >5 Hz share / RMS (deg/s) | pitch peak Hz | pitch >5 Hz share / RMS (deg/s) |",
          "|---|---|---|---|---|---|---|---|"]
    for n, p in res["phases"].items():
        if "rate_roll_dps" not in p:
            continue
        sr, sp = p.get("spectrum_roll") or {}, p.get("spectrum_pitch") or {}
        L.append("| %s | %s | %s | %s | %s | %s | %s | %s |" % (
            n, fmt((p["rate_roll_dps"] or {}).get("rms")),
            fmt((p["rate_pitch_dps"] or {}).get("rms")), fmt((p["rate_yaw_dps"] or {}).get("rms")),
            fmt(sr.get("peak_hz")),
            "%s / %s" % (fmt(sr.get("power_above_5hz"), 2), fmt(sr.get("rms_above_5hz"), 3)),
            fmt(sp.get("peak_hz")),
            "%s / %s" % (fmt(sp.get("power_above_5hz"), 2), fmt(sp.get("rms_above_5hz"), 3))))

    L += ["", "## VTOL height and motors", "",
          "| phase | alt err RMS (m) | alt err max (m) | climb err RMS (m/s) | motor mean (0-1) | motor max | % any motor at max | % any motor at min |",
          "|---|---|---|---|---|---|---|---|"]
    for n, p in res["phases"].items():
        if "motors" not in p:
            continue
        a, c, m = p.get("alt_m") or {}, p.get("climb_ms") or {}, p["motors"]
        L.append("| %s | %s | %s | %s | %s | %s | %s | %s |" % (
            n, fmt(a.get("rms")), fmt(a.get("max")), fmt(c.get("rms")),
            m["mean"], m["max"], m["pct_any_at_max"], m["pct_any_at_min"]))

    L += ["", "## Fixed-wing", ""]
    for n in ("transition", "fixed_wing"):
        p = res["phases"].get(n)
        if not p:
            continue
        a = p.get("airspeed_stats") or {}
        L += ["### %s" % n, "",
              "- airspeed min/mean/max: %s / %s / %s m/s; TECS speed-demand tracking RMS %s m/s" % (
                  fmt(a.get("min"), 1), fmt(a.get("mean"), 1), fmt(a.get("max"), 1),
                  fmt((p.get("airspeed_ms") or {}).get("rms"))),
              "- nav roll/pitch tracking RMS (CTUN): %s / %s deg (max %s / %s)" % (
                  fmt((p.get("fw_roll_deg") or {}).get("rms")), fmt((p.get("fw_pitch_deg") or {}).get("rms")),
                  fmt((p.get("fw_roll_deg") or {}).get("max")), fmt((p.get("fw_pitch_deg") or {}).get("max"))),
              "- FW rate loop (PIDR/PIDP) RMS: %s / %s deg/s" % (
                  fmt((p.get("fw_rate_roll") or {}).get("rms")), fmt((p.get("fw_rate_pitch") or {}).get("rms"))),
              "- TECS height error RMS / max: %s / %s m" % (
                  fmt((p.get("tecs_alt_m") or {}).get("rms")), fmt((p.get("tecs_alt_m") or {}).get("max"))),
              "- cross-track RMS / max: %s / %s m" % (
                  fmt((p.get("crosstrack_m") or {}).get("rms")), fmt((p.get("crosstrack_m") or {}).get("max"))),
              "- throttle mean/max: %s / %s %% (%s %% of samples at max)" % (
                  fmt((p.get("throttle_pct") or {}).get("mean"), 1), fmt((p.get("throttle_pct") or {}).get("max"), 1),
                  (p.get("throttle_pct") or {}).get("pct_at_max")),
              "- VTOL assist active: %s %% of time; lift motors above idle: %s %% of samples" % (
                  p.get("assist_pct_time"), p.get("lift_motors_above_idle_pct")),
              ""]

    L += ["## Mode changes", ""] + ["- %.1f s: %s" % (m["t"], m["mode"]) for m in res["modes"]]
    L += ["", "## Autopilot messages", ""] + ["- %.1f s: %s" % (m["t"], m["text"]) for m in res["statustext"]]
    L += ["", "![overview](validation.png)", ""]
    path.write_text("\n".join(L))


def plot(d, t0, phases, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(3, 2, figsize=(14, 11))
    colors = dict(hover_takeoff="#cde", transition="#fec", fixed_wing="#dfd",
                  back_transition="#fec", hover_land="#cde")

    def shade(a):
        for n, (lo, hi) in phases.items():
            a.axvspan(lo, hi, color=colors[n], alpha=0.6, lw=0)

    # Ground track.
    lat = np.array([r["Lat"] for r in d["POS"]])
    lng = np.array([r["Lng"] for r in d["POS"]])
    x = (lng - lng[0]) * 111320 * math.cos(math.radians(lat[0]))
    y = (lat - lat[0]) * 111320
    ax[0, 0].plot(x, y, lw=1)
    ax[0, 0].set_aspect("equal")
    ax[0, 0].set_xlabel("east from start [m]")
    ax[0, 0].set_ylabel("north from start [m]")
    ax[0, 0].grid(alpha=0.3)

    t, alt = arr(d["POS"], "RelHomeAlt", t0)
    ax[0, 1].plot(t, alt, lw=1, label="alt above home")
    t, a = arr(d["CTUN"], "As", t0)
    ax[0, 1].plot(t, a, lw=1, label="airspeed")
    shade(ax[0, 1])
    ax[0, 1].set_ylabel("height [m] / airspeed [m/s]")
    ax[0, 1].legend()

    for k, (des, act, label) in enumerate([("DesRoll", "Roll", "roll [deg]"),
                                           ("DesPitch", "Pitch", "pitch [deg]")]):
        a_ = ax[1, k]
        t, v = arr(d["ATT"], des, t0)
        a_.plot(t, v, lw=0.8, label="desired")
        t, v = arr(d["ATT"], act, t0)
        a_.plot(t, v, lw=0.8, label="actual")
        shade(a_)
        a_.set_ylabel(label)
        a_.legend()

    t, v = arr(d["PIQR"], "Tar", t0)
    ax[2, 0].plot(t, np.degrees(v), ".", ms=1, label="VTOL roll-rate target")
    t, v = arr(d["PIQR"], "Act", t0)
    ax[2, 0].plot(t, np.degrees(v), ".", ms=1, label="VTOL roll-rate actual")
    shade(ax[2, 0])
    ax[2, 0].set_ylabel("VTOL roll rate [deg/s]")
    ax[2, 0].legend(markerscale=8)

    tr = np.array([r["TimeUS"] for r in d["RCOU"]]) * 1e-6 - t0
    for i in (5, 6, 7, 8):
        ax[2, 1].plot(tr, [r["C%d" % i] for r in d["RCOU"]], lw=0.6, label="SERVO%d" % i)
    ax[2, 1].plot(tr, [r["C3"] for r in d["RCOU"]], "k", lw=0.8, label="SERVO3 (fwd thr)")
    shade(ax[2, 1])
    ax[2, 1].set_ylabel("motor PWM [µs]")
    ax[2, 1].legend(fontsize=7)

    for a_ in ax.flat[1:]:
        a_.set_xlabel("t (s)")
        a_.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)


if __name__ == "__main__":
    main()
