#!/usr/bin/env python3
"""Print the geometry of a QGroundControl .plan file in local metres.

Flattens Survey complex items into their waypoints, converts every position to
north/east metres from the planned home, and prints each leg's length and
heading, the turn angle at each waypoint, and the landing point.

    python3 analysis/auxiliary/plan_geometry.py "sim/Default square zigzag path.plan"
"""
import json
import math
import sys

R_EARTH = 6378137.0
NAV_WAYPOINT, NAV_LAND, NAV_TAKEOFF, DO_CHANGE_SPEED = 16, 21, 22, 178


def items(mission):
    for it in mission["items"]:
        if it.get("type") == "ComplexItem":
            yield from it["TransectStyleComplexItem"]["Items"]
        else:
            yield it


def main(path):
    mission = json.load(open(path))["mission"]
    lat0, lon0, _ = mission["plannedHomePosition"]

    def ne(lat, lon):
        return (math.radians(lat - lat0) * R_EARTH,
                math.radians(lon - lon0) * R_EARTH * math.cos(math.radians(lat0)))

    pts, land = [], None
    for it in items(mission):
        cmd, p = it["command"], it["params"]
        if cmd == DO_CHANGE_SPEED:
            print(f"speed command: {p[1]:.1f} m/s")
        elif cmd == NAV_TAKEOFF:
            print(f"take-off to {p[6]:.0f} m at home")
        elif cmd == NAV_WAYPOINT:
            pts.append((*ne(p[4], p[5]), p[6]))
        elif cmd == NAV_LAND:
            land = ne(p[4], p[5])

    print(f"{len(pts)} waypoints, altitude {sorted({round(p[2]) for p in pts})} m")
    prev_hdg, total = None, 0.0
    for a, b in zip(pts, pts[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        hdg = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) % 360
        turn = "" if prev_hdg is None else f"  turn {((hdg - prev_hdg + 180) % 360) - 180:+6.1f} deg"
        print(f"  ({a[0]:7.1f},{a[1]:7.1f}) -> ({b[0]:7.1f},{b[1]:7.1f})  {d:6.1f} m  hdg {hdg:5.1f}{turn}")
        prev_hdg, total = hdg, total + d
    print(f"path length {total:.0f} m")
    if land:
        print(f"landing at N {land[0]:.0f} m, E {land[1]:.0f} m, {math.hypot(*land):.0f} m from home")


if __name__ == "__main__":
    main(sys.argv[1])
