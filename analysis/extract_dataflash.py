#!/usr/bin/env python3
"""Stream ArduPilot DataFlash messages to separate, compressed CSV tables.

    python analysis/extract_dataflash.py runs/noisy_wind/pid_w6_s1/flight.BIN
    python analysis/extract_dataflash.py --types IMU,ATT,RCOU path/to/flight.BIN

Each message type has its own schema and timestamp (TimeUS, autopilot clock).
Avoid resampling or joining unlike clocks when investigating pulses or the IMU.
The manifest includes the mission settings and row counts for traceability.
"""
import argparse
import csv
import gzip
import json
from contextlib import ExitStack
from pathlib import Path

from pymavlink import mavutil

DEFAULT_TYPES = ("ATT", "PIQR", "PIQP", "PIDR", "PIDP", "RCOU",
                 "INDI", "INDF", "IMU", "CTUN", "TECS", "QTUN",
                 "POS", "ARSP", "MSG", "MODE", "PARM")


def extract(log, output, types):
    log = Path(log).resolve()
    output = Path(output).resolve()
    if not log.is_file():
        raise FileNotFoundError(log)
    output.mkdir(parents=True, exist_ok=True)
    counts = {}
    writers = {}
    connection = mavutil.mavlink_connection(str(log))
    with ExitStack() as stack:
        while (message := connection.recv_match(type=types)) is not None:
            kind = message.get_type()
            row = message.to_dict()
            row.pop("mavpackettype", None)
            if kind not in writers:
                file = stack.enter_context(gzip.open(output / (kind + ".csv.gz"), "wt", newline=""))
                writer = csv.DictWriter(file, fieldnames=list(row))
                writer.writeheader()
                writers[kind] = writer
                counts[kind] = 0
            writers[kind].writerow(row)
            counts[kind] += 1
    connection.close()
    metadata = {"source": str(log), "types_requested": types, "row_counts": counts}
    for name in ("params.param", "world.sdf", "imu_noise.json", "mission_status.json"):
        path = log.parent / name
        if path.exists():
            metadata[name] = str(path)
    (output / "manifest.json").write_text(json.dumps(metadata, indent=2) + "\n")
    return counts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, help="DataFlash .BIN flight log")
    parser.add_argument("--out", type=Path, help="output folder (default: log's extracted/)")
    parser.add_argument("--types", default=",".join(DEFAULT_TYPES),
                        help="comma-separated DataFlash message types, e.g. IMU,ATT,RCOU")
    args = parser.parse_args()
    types = [name.strip().upper() for name in args.types.split(",") if name.strip()]
    counts = extract(args.log, args.out or args.log.parent / "extracted", types)
    print(json.dumps(counts, indent=2))


if __name__ == "__main__":
    main()