#!/usr/bin/env python3
"""Stability margins of the INDI rate loop, per axis, before flying it.

    python analysis/indi_margins.py --params sim/stock_pid.param sim/indi.param

Discrete model at the loop rate (same structure as AC_INDI_Axis):

  plant      omega = G_true * A(z) * I(z) * u
             A(z)  first-order actuator (tau) with delay T, I(z) = dt z^-1/(1-z^-1)
  gyro       omega_m = F(z) omega            F = INS low-pass (biquad, INS_GYRO_FILTER)
  accel      wdot_f  = H(z) D(z) omega_m     D = (1-z^-1)/dt, H = two poles at FILT_HZ
  feedback   u0_f    = M(z) u                M = z^-1 A_model(z) (1+z^-1)/2 F(z) H(z)
  law        u = u0_f + (nu - wdot_f)/G,     nu = K (r - omega_m)

Characteristic equation 1 - M + (K + H D) F P / G = 0, so the loop broken at
the plant input is
     L(z) = (K + H D) F P / (G (1 - M)).
Gain margin (GM) and phase margin (PM) come from L on the unit circle.
The script scans K and G_true/G (effectiveness error) and picks the largest K
with GM >= 6 dB and PM >= 45 deg at the nominal G.
"""
import argparse
import math

import numpy as np

AXES = {
    "VTOL roll": ("Q_INDI_RLL_G", "Q_INDI_ACT_TC", "Q_INDI_ACT_DLY"),
    "VTOL pitch": ("Q_INDI_PIT_G", "Q_INDI_ACT_TC", "Q_INDI_ACT_DLY"),
    "FW roll": ("Q_INDI_FW_RLL_G", "Q_INDI_FW_TC", None),
    "FW pitch": ("Q_INDI_FW_PIT_G", "Q_INDI_FW_TC", None),
}


def load(paths):
    p = {}
    for path in paths:
        for line in open(path):
            f = line.split()
            if len(f) >= 2 and not line.startswith("#"):
                try:
                    p[f[0]] = float(f[1])
                except ValueError:
                    pass
    return p


def biquad_lpf(fs, fc, z):
    """ArduPilot LowPassFilter2p (Butterworth biquad), evaluated at z."""
    fr = fs / fc
    ohm = math.tan(math.pi / fr)
    c = 1.0 + 2.0 * math.cos(math.pi / 4.0) * ohm + ohm * ohm
    b0 = ohm * ohm / c
    b1, b2 = 2.0 * b0, b0
    a1 = 2.0 * (ohm * ohm - 1.0) / c
    a2 = (1.0 - 2.0 * math.cos(math.pi / 4.0) * ohm + ohm * ohm) / c
    zi = 1.0 / z
    return (b0 + b1 * zi + b2 * zi * zi) / (1.0 + a1 * zi + a2 * zi * zi)


def first_order(dt, tc, z):
    a = dt / (dt + tc) if tc > 0 else 1.0
    return a / (1.0 - (1.0 - a) / z)


def loop(fs, f_gyro, f_h, tc, dly, K, g_ratio, f):
    dt = 1.0 / fs
    z = np.exp(1j * 2 * np.pi * f * dt)
    zi = 1.0 / z
    n_dly = int(round(dly * fs))
    A = first_order(dt, tc, z) * zi ** n_dly
    F = biquad_lpf(fs, f_gyro, z) if f_gyro > 0 else 1.0
    H = first_order(dt, 1.0 / (2 * np.pi * f_h), z) ** 2
    D = (1 - zi) / dt
    I = dt * zi / (1 - zi)
    P_over_G = g_ratio * A * I          # G_true/G_hat * A * I
    M = zi * A * (1 + zi) / 2 * F * H
    return (K + H * D) * F * P_over_G / (1 - M)


def margins(L, f):
    mag = np.abs(L)
    ph = np.unwrap(np.angle(L))
    gm = pm = float("inf")
    fc = fpc = float("nan")
    i = np.where((mag[:-1] >= 1) & (mag[1:] < 1))[0]
    if len(i):
        k = i[0]
        fc = f[k]
        pm = math.degrees(ph[k]) + 180.0
        pm = (pm + 180.0) % 360.0 - 180.0
    j = np.where(np.diff(np.sign(np.sin(ph))) != 0)[0]
    for k in j:
        if math.cos(ph[k]) < 0:            # phase crosses -180 (mod 360)
            gm = -20 * math.log10(max(mag[k], 1e-12))
            fpc = f[k]
            break
    return gm, pm, fc, fpc


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", nargs="+", required=True)
    ap.add_argument("--fs", type=float, help="actual loop rate, Hz (default SCHED_LOOP_RATE); "
                    "JSON SITL runs a 400 Hz loop every 3 ms, i.e. 333 Hz")
    ap.add_argument("--gm", type=float, default=6.0)
    ap.add_argument("--pm", type=float, default=45.0)
    a = ap.parse_args()
    p = load(a.params)
    fs = a.fs or p.get("SCHED_LOOP_RATE", 400)
    f_gyro = p.get("INS_GYRO_FILTER", 20)
    f_h = p.get("Q_INDI_FILT_HZ", 12.7)
    f = np.logspace(-1, math.log10(fs / 2 * 0.999), 4000)
    print("loop %.0f Hz, INS gyro filter %.0f Hz, INDI filter %.1f Hz" % (fs, f_gyro, f_h))
    print("| axis | tau (s) | T (s) | K chosen | GM (dB) | PM (deg) | crossover (Hz) | GM at G_true = 0.5 G / 2 G |")
    print("|---|---|---|---|---|---|---|---|")
    for name, (gk, tk, dk) in AXES.items():
        if p.get(gk, 0) <= 0:
            print("| %s | - | - | (no G) | | | | |" % name)
            continue
        tc = p.get(tk, 0.03)
        dly = p.get(dk, 0.0) if dk else 0.0
        best = None
        for K in np.arange(1.0, 40.01, 0.5):
            gm, pm, fc, _ = margins(loop(fs, f_gyro, f_h, tc, dly, K, 1.0, f), f)
            if gm >= a.gm and pm >= a.pm:
                best = (K, gm, pm, fc)
        if best is None:
            print("| %s | %.3f | %.3f | none meets GM/PM | | | | |" % (name, tc, dly))
            continue
        K, gm, pm, fc = best
        lo = margins(loop(fs, f_gyro, f_h, tc, dly, K, 0.5, f), f)
        hi = margins(loop(fs, f_gyro, f_h, tc, dly, K, 2.0, f), f)
        print("| %s | %.3f | %.3f | %.1f | %.1f | %.1f | %.2f | %.1f / %.1f |"
              % (name, tc, dly, K, gm, pm, fc, lo[0], hi[0]))


if __name__ == "__main__":
    main()
