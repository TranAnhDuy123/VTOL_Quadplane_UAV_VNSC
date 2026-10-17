#!/usr/bin/env python3
"""Identify control effectiveness G, actuator lag tau and delay T from
SystemID chirp flights (sim/fly_sysid.py).

    python analysis/identify_G.py --hover runs/sysid_hover/flight.BIN \
                                  --fw runs/sysid_fw/flight.BIN \
                                  --out sim/indi.param

Model per axis:  omega_dot(s) / u(s) = G * exp(-s T) / (1 + s tau)
    u         normalised effector command (mixer input, or surface / full travel)
    omega_dot angular acceleration, rad/s^2  ->  G is rad/s^2 per unit command

Estimate. The chirp r(t) is injected on top of the PID output, so u = u_PID + r.
With the loop closed, u and omega are correlated through the controller, and the
plain estimate S_u,wd / S_u,u converges to the *inverse controller*, not the
plant. Using the injected chirp as an instrument,
    H = S_r,wd / S_r,u
removes that bias because r is independent of everything the loop does.
Only frequency bins with coherence(r, omega_dot) >= 0.6 are fitted.

Fixed-wing G changes with dynamic pressure. ArduPlane's speed scaler is
SCALING_SPEED / airspeed, so G(V) = G_ref / scaler^2 and G_ref = G * scaler^2
should come out the same at every airspeed.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from pymavlink import mavutil
from scipy import signal

COH_MIN = 0.6
FIT_FMIN = 0.5           # lowest frequency used in the fit (set by --fit-fmin)
HOVER_AXES = {10: ("roll", "Gx", "ROut"), 11: ("pitch", "Gy", "POut"), 12: ("yaw", "Gz", "YOut")}
FW_AXES = {22: ("roll", "Gx", "Aile"), 23: ("pitch", "Gy", "Elev")}


def read(path, types):
    m = mavutil.mavlink_connection(str(path))
    d = {t: [] for t in types}
    while True:
        x = m.recv_match(type=types)
        if x is None:
            return d
        d[x.get_type()].append(x.to_dict())


def col(rows, f):
    return np.array([float(r[f]) for r in rows])


def segments(d):
    """Split SIDD rows into one block per chirp, labelled by SIDS axis."""
    starts = [(r["TimeUS"], int(r["Ax"])) for r in d["SIDS"]]
    t = col(d["SIDD"], "TimeUS")
    out = []
    for i, (ts, ax) in enumerate(starts):
        te = starts[i + 1][0] if i + 1 < len(starts) else t[-1] + 1
        idx = np.where((t >= ts) & (t < te))[0]
        # the chirp time resets per run; keep the first contiguous block
        if len(idx) > 500:
            out.append((ax, idx))
    return out


def frf(t_us, r, u, wdot, fmin, fmax):
    t = (t_us - t_us[0]) * 1e-6
    fs = 1.0 / np.median(np.diff(t))
    # resample onto a uniform grid (log rows are nearly uniform already)
    tu = np.arange(t[0], t[-1], 1.0 / fs)
    r, u, wdot = (np.interp(tu, t, x) for x in (r, u, wdot))
    nper = int(fs * 8)                      # 8 s windows -> 0.125 Hz bins
    kw = dict(fs=fs, nperseg=nper, noverlap=nper // 2, window="hann")
    f, s_ru = signal.csd(r, u, **kw)
    _, s_ry = signal.csd(r, wdot, **kw)
    _, coh = signal.coherence(r, wdot, **kw)
    H = s_ry / s_ru
    keep = (f >= max(fmin, FIT_FMIN)) & (f <= fmax) & (coh >= COH_MIN)
    return f[keep], H[keep], coh[keep], fs


def fit(f, H, coh):
    """Least squares on the complex FRF: G exp(-jwT)/(1+jw tau)."""
    w = 2 * np.pi * f
    wt = coh / coh.sum()
    best = None
    for tau in np.arange(0.0, 0.2001, 0.001):
        for T in np.arange(0.0, 0.0401, 0.0005):
            m = np.exp(-1j * w * T) / (1 + 1j * w * tau)
            G = np.real(np.sum(wt * H * np.conj(m))) / np.sum(wt * np.abs(m) ** 2)
            err = np.sum(wt * np.abs(H - G * m) ** 2)
            if best is None or err < best[0]:
                best = (err, G, tau, T)
    err, G, tau, T = best
    rel = math.sqrt(err) / max(abs(G), 1e-9)
    return dict(G=float(G), tau=float(tau), T=float(T), fit_rel_err=float(rel),
                n_bins=int(len(f)), f_lo=float(f.min()), f_hi=float(f.max()))


def hover(path):
    d = read(path, ["SIDS", "SIDD", "RATE", "QTUN", "PARM"])
    p = {r["Name"]: r["Value"] for r in d["PARM"]}
    t_rate = col(d["RATE"], "TimeUS")
    res = {}
    for ax, idx in segments(d):
        if ax not in HOVER_AXES:
            continue
        name, gcol, ucol = HOVER_AXES[ax]
        rows = [d["SIDD"][i] for i in idx]
        t = col(rows, "TimeUS")
        r = col(rows, "Targ")
        om = np.radians(col(rows, gcol))
        wdot = np.gradient(om, t * 1e-6)
        u = np.interp(t, t_rate, col(d["RATE"], ucol))
        f, H, coh, fs = frf(t, r, u, wdot, p.get("SID_F_START_HZ", 0.5), 20.0)
        if len(f) < 5:
            res[name] = dict(error="too few coherent bins (%d)" % len(f))
            continue
        res[name] = fit(f, H, coh)
        res[name]["fs_hz"] = fs
    # physics cross-check from model.sdf (quad X, mixer factors +-0.5)
    thh = col(d["QTUN"], "ThH")
    thr_hover = float(np.median(thh[thh > 0.05])) if len(thh) else float("nan")
    t_max = 2.8 * 9.81 / (4 * thr_hover)
    res["model_check"] = dict(
        thrust_hover=thr_hover, T_max_N=t_max,
        G_roll_model=2 * 0.244 * t_max / 0.1399,
        G_pitch_model=2 * 0.294 * t_max / 0.2138,
        note="G = 4 motors * 0.5 mixer factor * arm * T_max / I; rough (linearised thrust curve)")
    return res


def fixed_wing(path):
    d = read(path, ["SIDS", "SIDD", "SIDP", "CTUN", "PARM"])
    t_p = col(d["SIDP"], "TimeUS")
    t_c = col(d["CTUN"], "TimeUS")
    res = {"roll": [], "pitch": []}
    for ax, idx in segments(d):
        if ax not in FW_AXES:
            continue
        name, gcol, ucol = FW_AXES[ax]
        rows = [d["SIDD"][i] for i in idx]
        t = col(rows, "TimeUS")
        r = col(rows, "Targ") / 45.0                     # deg of +-45 -> normalised
        om = np.radians(col(rows, gcol))
        wdot = np.gradient(om, t * 1e-6)
        u = np.interp(t, t_p, col(d["SIDP"], ucol)) / 45.0
        scaler = float(np.mean(np.interp(t, t_p, col(d["SIDP"], "aspd"))))
        aspd = float(np.mean(np.interp(t, t_c, col(d["CTUN"], "As"))))
        f, H, coh, fs = frf(t, r, u, wdot, 0.5, 15.0)
        if len(f) < 5:
            res[name].append(dict(airspeed=aspd, error="too few coherent bins (%d)" % len(f)))
            continue
        e = fit(f, H, coh)
        e.update(airspeed=aspd, scaler=scaler, G_ref=e["G"] * scaler ** 2)
        res[name].append(e)
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hover")
    ap.add_argument("--fw")
    ap.add_argument("--out", help="write an INDI parameter overlay here")
    ap.add_argument("--json", help="write the full result here")
    ap.add_argument("--fit-fmin", type=float, default=0.5,
                    help="lowest frequency in the fit. Aerodynamic damping and angle-of-attack "
                         "terms add to the low-frequency response; above a few Hz only the "
                         "direct effector term is left, which is the G INDI needs")
    a = ap.parse_args()
    global FIT_FMIN
    FIT_FMIN = a.fit_fmin

    out = {}
    if a.hover:
        out["hover"] = hover(a.hover)
    if a.fw:
        out["fixed_wing"] = fixed_wing(a.fw)
    print(json.dumps(out, indent=1))
    if a.json:
        Path(a.json).write_text(json.dumps(out, indent=1))

    if a.out:
        L = ["# INDI parameters identified by analysis/identify_G.py",
             "# hover log: %s" % a.hover, "# fw log:    %s" % a.fw]
        h = out.get("hover", {})
        for ax, pre in (("roll", "RLL"), ("pitch", "PIT"), ("yaw", "YAW")):
            e = h.get(ax, {})
            if "G" in e:
                L.append("Q_INDI_%s_G %.2f" % (pre, e["G"]))
        taus = [h[ax]["tau"] for ax in ("roll", "pitch") if "tau" in h.get(ax, {})]
        dls = [h[ax]["T"] for ax in ("roll", "pitch") if "T" in h.get(ax, {})]
        if taus:
            L.append("Q_INDI_ACT_TC %.4f" % float(np.mean(taus)))
            L.append("Q_INDI_ACT_DLY %.4f" % float(np.mean(dls)))
        fw = out.get("fixed_wing", {})
        ftau = []
        for ax, pre in (("roll", "RLL"), ("pitch", "PIT")):
            refs = [e["G_ref"] for e in fw.get(ax, []) if "G_ref" in e]
            ftau += [e["tau"] for e in fw.get(ax, []) if "tau" in e]
            if refs:
                L.append("Q_INDI_FW_%s_G %.2f" % (pre, float(np.median(refs))))
        if ftau:
            L.append("Q_INDI_FW_TC %.4f" % float(np.mean(ftau)))
        Path(a.out).write_text("\n".join(L) + "\n")
        print("\n".join(L))


if __name__ == "__main__":
    main()
