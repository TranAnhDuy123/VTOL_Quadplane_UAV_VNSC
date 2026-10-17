#!/usr/bin/env python3
"""3D path of the hover torque test, PID vs INDI.

    python3 analysis/hover_path3d.py [runs/hover_test]

Writes <root>/hover_path3d.png / .pdf. Dots mark the start of each push.
"""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_log import read_log, arr  # noqa: E402

COLORS = {"pid": "#2a78d6", "indi": "#eb6834"}
R_EARTH = 6378137.0


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    root = Path(sys.argv[1] if len(sys.argv) > 1 else "runs/hover_test")
    fig = plt.figure(figsize=(11, 5))
    for c, (size, label) in enumerate((("1p44", "1.44 N·m"), ("3p24", "3.24 N·m"))):
        ax = fig.add_subplot(1, 2, c + 1, projection="3d")
        for ctrl in ("pid", "indi"):
            rd = root / ("%s_%s" % (ctrl, size))
            d = read_log(rd / "flight.BIN")
            t0 = d["ATT"][0]["TimeUS"] * 1e-6
            t, lat = arr(d["POS"], "Lat", t0)
            _, lon = arr(d["POS"], "Lng", t0)
            _, h = arr(d["POS"], "RelHomeAlt", t0)
            n = np.radians(lat - lat[0]) * R_EARTH
            e = np.radians(lon - lon[0]) * R_EARTH * np.cos(np.radians(lat[0]))
            ax.plot(e, n, h, color=COLORS[ctrl], lw=1.0, label=ctrl.upper())
            st = json.load(open(rd / "mission_status.json"))
            for p in st["pulses"]:
                if p["action"] in ("roll", "pitch"):
                    i = min(np.searchsorted(t, p["tb"] - t0), len(t) - 1)
                    ax.scatter(e[i], n[i], h[i], s=18, color=COLORS[ctrl], edgecolor="black", linewidth=0.5)
        ax.set_xlabel("east [m]")
        ax.set_ylabel("north [m]")
        ax.set_zlabel("height [m]")
        ax.set_title(label)
        ax.legend(frameon=False, loc="upper left")
    fig.text(0.5, 0.02, "hover over the origin; 5 s roll then 5 s pitch torque, dots mark each push start",
             ha="center", fontsize=9)
    fig.tight_layout()
    for ext in ("png", "pdf"):
        fig.savefig(root / ("hover_path3d.%s" % ext), dpi=150)
    print("wrote", root / "hover_path3d.png")


if __name__ == "__main__":
    main()
