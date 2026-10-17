#!/usr/bin/env python3
"""Margins of one roll/pitch axis driven by lift motors AND surfaces with INDI.

    python3 analysis/auxiliary/joint_indi_margins.py --params sim/stock_pid.param sim/indi.param

Same discrete loop model as analysis/indi_margins.py, with two effector sets
i = motors, surfaces acting on one axis. Each runs
    u_i = M_i u_i + w_i (nu_i - H D F omega),   nu_i = K_i (r - F omega)
and the plant is omega = I sum_i rho_i G_i A_i u_i (rho_i = true/modelled G).
Broken at the plant output, the loop is

    L = sum_i  w_i G_i rho_i (K_i + H D) F A_i I / (1 - M_i)

  shared split (the implementation):   w_i = G_i / (G_m^2 + G_s^2)
                                       -> weights G_i^2/sum G^2 add up to 1
  independent laws (no sharing):       w_i = 1 / G_i
                                       -> weights 1 + 1 = 2 while both run

Surface effectiveness G_s = FW_G * (V / SCALING_SPEED)^2, zero below
0.5 * AIRSPEED_MIN, as in Plane::stabilize().
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from indi_margins import load, biquad_lpf, first_order, margins  # noqa: E402


def parts(fs, f_gyro, f_h, tc, dly, z):
    dt = 1.0 / fs
    zi = 1.0 / z
    A = first_order(dt, tc, z) * zi ** int(round(dly * fs))
    F = biquad_lpf(fs, f_gyro, z) if f_gyro > 0 else 1.0
    H = first_order(dt, 1.0 / (2 * np.pi * f_h), z) ** 2
    D = (1 - zi) / dt
    I = dt * zi / (1 - zi)
    M = zi * A * (1 + zi) / 2 * F * H
    return A, F, H, D, I, M


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", nargs="+", required=True)
    ap.add_argument("--fs", type=float, default=333.0, help="actual loop rate (JSON SITL: 333 Hz)")
    ap.add_argument("--rho", type=float, default=1.0, help="true/modelled effectiveness, both sets")
    a = ap.parse_args()
    p = load(a.params)
    fs, f_gyro, f_h = a.fs, p.get("INS_GYRO_FILTER", 20), p.get("Q_INDI_FILT_HZ", 12.7)
    v_ref, v_min = p.get("SCALING_SPEED", 15), p.get("AIRSPEED_MIN", 14)
    f = np.logspace(-1, math.log10(fs / 2 * 0.999), 5000)
    z = np.exp(1j * 2 * np.pi * f / fs)
    motor = parts(fs, f_gyro, f_h, p.get("Q_INDI_ACT_TC", 0), p.get("Q_INDI_ACT_DLY", 0), z)
    surf = parts(fs, f_gyro, f_h, p.get("Q_INDI_FW_TC", 0.02), 0.0, z)

    def branch(pt, K, weight_times_G):
        A, F, H, D, I, M = pt
        return weight_times_G * a.rho * (K + H * D) * F * A * I / (1 - M)

    print("loop %.0f Hz, gyro filter %.0f Hz, INDI filter %.1f Hz, rho %.2f" % (fs, f_gyro, f_h, a.rho))
    for axis, gm_key, gs_key, km_key, ks_key in (
            ("roll", "Q_INDI_RLL_G", "Q_INDI_FW_RLL_G", "Q_INDI_RLL_K", "Q_INDI_FW_RLL_K"),
            ("pitch", "Q_INDI_PIT_G", "Q_INDI_FW_PIT_G", "Q_INDI_PIT_K", "Q_INDI_FW_PIT_K")):
        Gm, Gs_ref, Km, Ks = p[gm_key], p[gs_key], p[km_key], p[ks_key]
        print("\n%s: G_motor %.1f, G_surface(ref %.0f m/s) %.1f, K %.1f / %.1f" % (axis, Gm, v_ref, Gs_ref, Km, Ks))
        print("| airspeed (m/s) | G_s | surface share G_s^2/sum | shared: GM (dB) / PM (deg) | independent: GM (dB) / PM (deg) |")
        print("|---:|---:|---:|---:|---:|")
        for V in (0, 5, 7, 9, 12, 14, 17, 20, 22):
            Gs = Gs_ref * (V / v_ref) ** 2 if V > 0.5 * v_min else 0.0
            S = Gm ** 2 + Gs ** 2
            Lsh = branch(motor, Km, Gm * Gm / S) + (branch(surf, Ks, Gs * Gs / S) if Gs > 0 else 0)
            Lin = branch(motor, Km, 1.0) + (branch(surf, Ks, 1.0) if Gs > 0 else 0)
            gm1, pm1, _, _ = margins(Lsh, f)
            gm2, pm2, _, _ = margins(Lin, f)
            print("| %d | %.1f | %.1f %% | %.1f / %.1f | %.1f / %.1f |"
                  % (V, Gs, 100 * Gs * Gs / S, gm1, pm1, gm2, pm2))


if __name__ == "__main__":
    main()
