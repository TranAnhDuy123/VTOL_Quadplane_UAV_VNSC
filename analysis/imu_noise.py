#!/usr/bin/env python3
"""IMU noise in hover: real aircraft vs simulation.

    python analysis/imu_noise.py --real <real.bin> --real-window 90 102 \
        --sim runs/<clean>/flight.BIN --sim-noisy runs/<noisy>/flight.BIN \
        --out runs/figures/imu_noise.png

Uses the IMU message (gyro and accelerometer after ArduPilot's filters, i.e.
what the controllers see), first IMU only, hover only:
  real  the given window of the real flight (steady manual hover)
  sim   the landing hover of a square-mission run (position2 -> land, trimmed)

The real reference switches to FBWA at 104.7 s: never include it in the
hover comparison. Prints band-limited RMS and writes numerical RMS/PSD errors
to a JSON file next to the figure. This measures what is logged at the IMU;
it cannot recreate the real aircraft's tonal vibration or harmonic notch.
"""
import argparse
import sys
from pathlib import Path

import numpy as np
from pymavlink import mavutil
from scipy import signal

sys.path.insert(0, str(Path(__file__).parent))
from validate_log import analyse  # noqa: E402

BANDS = [(0.5, 5), (5, 20), (20, 60), (60, 150)]
LABELS = ["gyro x (roll)", "gyro y (pitch)", "gyro z (yaw)", "accel x", "accel y", "accel z"]


def imu(path, lo, hi):
    m = mavutil.mavlink_connection(str(path))
    rows, t0 = [], None
    while True:
        x = m.recv_match(type=["IMU", "ATT"])
        if x is None:
            break
        if x.get_type() == "ATT":
            t0 = t0 if t0 is not None else x.TimeUS * 1e-6
            continue
        if x.I == 0 and t0 is not None:
            rows.append((x.TimeUS * 1e-6 - t0, x.GyrX, x.GyrY, x.GyrZ, x.AccX, x.AccY, x.AccZ))
    if not rows:
        raise ValueError("no primary IMU samples in %s" % path)
    a = np.array(rows)
    a = a[(a[:, 0] >= lo) & (a[:, 0] < hi)]
    if len(a) < 1024:
        raise ValueError("not enough IMU samples in %s, interval [%.1f, %.1f)" % (path, lo, hi))
    a[:, 1:4] = np.degrees(a[:, 1:4])        # gyro in deg/s
    return a


def sim_hover(path):
    _, _, _, ph = analyse(path)
    if "hover_land" not in ph:
        raise ValueError("no completed landing hover in %s" % path)
    lo, hi = ph["hover_land"]
    return imu(path, lo + 2.0, hi - 5.0)


def psd(a, j):
    fs = 1.0 / np.median(np.diff(a[:, 0]))
    x = a[:, 1 + j] - a[:, 1 + j].mean()
    return signal.welch(x, fs=fs, nperseg=min(1024, len(x)))


def band_rms(f, P, b0, b1):
    m = (f >= b0) & (f < b1)
    return float(np.sqrt(np.trapezoid(P[m], f[m]))) if m.sum() > 1 else float("nan")


def metrics(sets):
    """Compare logged post-filter IMU spectra, not the injected sensor sigma."""
    spectra = {name: [psd(samples, axis) for axis in range(6)] for name, samples, _ in sets}
    report = {"window_note": "Real: manual airborne QHOVER; simulation: trimmed landing hover. "
                             "No matched maneuvers, motors or filters; this is a noise-floor check.",
              "comparison_band_hz": [20, 120], "axes": {}}
    for axis, label in enumerate(LABELS):
        real_f, real_p = spectra["real aircraft"][axis]
        axis_metrics = {}
        real_rms = band_rms(real_f, real_p, 20, 120)
        for name, series in spectra.items():
            f, P = series[axis]
            row = {"sample_rate_hz": round(1.0 / np.median(np.diff(next(a for n, a, _ in sets if n == name)[:, 0])), 2),
                   "rms_20_120": band_rms(f, P, 20, 120),
                   "rms_25_90": band_rms(f, P, 25, 90)}
            if axis < 3:
                # Differentiation in the frequency domain converts gyro PSD
                # (rad/s)^2/Hz to angular-acceleration PSD (rad/s^2)^2/Hz.
                row["angular_accel_rms_rad_s2_25_90"] = band_rms(
                    f, P * (np.pi / 180) ** 2 * (2 * np.pi * f) ** 2, 25, 90)
            if name != "real aircraft":
                row["rms_relative_error_pct"] = 100 * (row["rms_20_120"] / real_rms - 1)
                # Log power ratio (dB): a large value reveals spectral shape
                # mismatch even if the integrated RMS happens to be right.
                common = (real_f >= 20) & (real_f <= min(120, f[-1]))
                ratio = 10 * np.log10(np.maximum(np.interp(real_f[common], f, P), 1e-16)
                                      / np.maximum(real_p[common], 1e-16))
                row["log_psd_rmse_db"] = float(np.sqrt(np.mean(ratio ** 2)))
            axis_metrics[name] = row
        report["axes"][label] = axis_metrics
    return report


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--real", required=True)
    ap.add_argument("--real-window", type=float, nargs=2, default=[90, 102])
    ap.add_argument("--sim", required=True, help="run with NO injected IMU noise")
    ap.add_argument("--sim-noisy", help="calibration run with known sensor noise")
    ap.add_argument("--noise-config", help="noise JSON used for --sim-noisy")
    ap.add_argument("--suggest-out", help="write a calibrated noise JSON (requires all three logs and --noise-config)")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    sets = [("real aircraft", imu(a.real, *a.real_window), "k"),
            ("simulation", sim_hover(a.sim), "#1f77b4")]
    if a.sim_noisy:
        sets.append(("simulation + IMU noise", sim_hover(a.sim_noisy), "#d62728"))

    report = metrics(sets)
    report["real_window_s"] = a.real_window
    report["inputs"] = {name: (a.real if name == "real aircraft" else
                               a.sim if name == "simulation" else a.sim_noisy) for name, _, _ in sets}
    import json
    if a.suggest_out:
        if not (a.sim_noisy and a.noise_config):
            ap.error("--suggest-out requires --sim-noisy and --noise-config")
        cfg = json.loads(Path(a.noise_config).read_text())
        suggested = {"_comment": "Matched 20-120 Hz post-filter hover IMU RMS using analysis/imu_noise.py; "
                                  "sensor Gaussian stddev at 1 kHz. Clean motion already above real "
                                  "cannot be reduced by adding noise. Does not reproduce tonal vibration "
                                  "or the real 42 Hz gyro filter/notch; validate on a NEW run."}
        for kind, labels in (("gyro_rad_s", LABELS[:3]), ("accel_m_s2", LABELS[3:])):
            levels = []
            for level, label in zip(cfg[kind], labels):
                row = report["axes"][label]
                clean = row["simulation"]["rms_20_120"] ** 2
                noisy = row["simulation + IMU noise"]["rms_20_120"] ** 2
                real = row["real aircraft"]["rms_20_120"] ** 2
                if noisy <= clean:
                    raise ValueError("no noise response in %s; cannot calibrate" % label)
                levels.append(round(float(level) * np.sqrt(max(0.0, real - clean) / (noisy - clean)), 5))
            suggested[kind] = levels
        dest = Path(a.suggest_out)
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_text(json.dumps(suggested, indent=2) + "\n")
        report["suggested_config"] = str(dest)
        print("suggested noise config:", dest)
    path = Path(a.out)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")

    print("| signal | " + " | ".join(n for n, _, _ in sets) + " |  (RMS 20-120 Hz; gyro deg/s, accel m/s²)")
    print("|---|" + "---|" * len(sets))
    for j, lab in enumerate(LABELS):
        vals = [report["axes"][lab][name]["rms_20_120"] for name, _, _ in sets]
        print("| %s | %s |" % (lab, " | ".join("%.3f" % v for v in vals)))
    for lab in LABELS:
        if "simulation + IMU noise" in report["axes"][lab]:
            row = report["axes"][lab]["simulation + IMU noise"]
            print("%s: RMS error %+.1f%%, PSD error %.1f dB" % (
                lab, row["rms_relative_error_pct"], row["log_psd_rmse_db"]))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(2, 3, figsize=(13, 6.5), sharex=True)
    for j, lab in enumerate(LABELS):
        ax = axs[j // 3, j % 3]
        for name, arr_, col in sets:
            f, P = psd(arr_, j)
            ax.semilogy(f, P, color=col, lw=0.9, label=name)
        ax.axvspan(20, 120, color="0.93", zorder=0)
        ax.grid(alpha=0.3)
        ax.set_ylabel(lab + ("\nPSD [(deg/s)²/Hz]" if j < 3 else "\nPSD [(m/s²)²/Hz]"), fontsize=8)
    for ax in axs[1]:
        ax.set_xlabel("frequency [Hz]")
    axs[0, 0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(a.out, dpi=150)
    plt.close(fig)
    print("figure:", a.out)
    print("metrics:", path.with_suffix(".json"))


if __name__ == "__main__":
    main()
