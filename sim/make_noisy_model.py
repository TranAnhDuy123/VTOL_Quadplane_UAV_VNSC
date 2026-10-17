#!/usr/bin/env python3
"""Copy the quadplane model with Gaussian noise on its Gazebo IMU sensor.

    python sim/make_noisy_model.py sim/imu_noise.json <out_models_dir>

Writes <out_models_dir>/VTOL_Quadplane/ (model.sdf with an <imu> noise block,
meshes and materials linked from sim/gazebo_models/VTOL_Quadplane). The run
script puts <out_models_dir> first on GZ_SIM_RESOURCE_PATH, so only that run
sees the noisy IMU; the shared model is not changed.

SITL's own IMU noise (SIM_GYR*_RND, SIM_ACC*_RND) does not work in hover with
the JSON backend: it is scaled by a throttle that SITL computes from a motor
mask the JSON backend never sets, so it is zero while hovering. The noise is
therefore added where the real noise comes from, at the sensor in the physics
model (1 kHz, before ArduPilot's gyro/accel filters).
"""
import json
import os
import sys
from pathlib import Path

SRC = Path(__file__).resolve().parent / "gazebo_models" / "VTOL_Quadplane"


def noise_xyz(sd):
    return "".join("<%s><noise type=\"gaussian\"><mean>0</mean><stddev>%g</stddev></noise></%s>"
                   % (a, s, a) for a, s in zip("xyz", sd))


def main():
    cfg = json.load(open(sys.argv[1]))
    out = Path(sys.argv[2]) / "VTOL_Quadplane"
    out.mkdir(parents=True, exist_ok=True)
    for item in SRC.iterdir():
        if item.name in ("model.sdf",) or item.suffix in (".save", ".erb") or ".save" in item.name:
            continue
        link = out / item.name
        if not link.exists():
            os.symlink(item, link)
    sdf = (SRC / "model.sdf").read_text()
    anchor = "<update_rate>1000.0</update_rate>\n      </sensor>"
    assert sdf.count(anchor) == 1, "IMU sensor block not found"
    ang = noise_xyz(cfg["gyro_rad_s"])
    lin = noise_xyz(cfg["accel_m_s2"])
    imu = ("<update_rate>1000.0</update_rate>\n        <imu>\n"
           "          <angular_velocity>%s</angular_velocity>\n"
           "          <linear_acceleration>%s</linear_acceleration>\n"
           "        </imu>\n      </sensor>" % (ang, lin))
    (out / "model.sdf").write_text(sdf.replace(anchor, imu))
    print("noisy model:", out)


if __name__ == "__main__":
    main()
