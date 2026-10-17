#!/usr/bin/env python3
"""Rank the lift-drag panels of a Gazebo model by maximum-lift contribution.

Reads every gz-sim LiftDrag plugin in the SDF, classifies each panel from its
upward vector and link (wing-type horizontal panel, vertical fin, rotor blade),
and reports S*CL_max = area * cla * alpha_stall, which is the CL_max the
gz-sim LiftDrag system produces (CL = cla * (alpha_geom + a0) up to alpha_stall).
Mass is summed over every link, not only base_link.

Optionally compares the main-wing panel with an OpenVSP VSPAERO_Polar export.

    python3 analysis/auxiliary/rank_lifting_surfaces.py \
        sim/gazebo_models/VTOL_Quadplane/model.sdf \
        --openvsp "../analysis/python/twinbooms_data(1).csv"
"""
import argparse
import csv
import math
import re

G = 9.80665


def tag(block, name):
    m = re.search(rf"<{name}>(.*?)</{name}>", block, re.S)
    return m.group(1).strip() if m else None


def vec(text):
    return [float(v) for v in text.split()]


def load(sdf_path):
    text = open(sdf_path).read()
    masses = [float(m) for m in re.findall(r"<mass>(.*?)</mass>", text)]
    panels = []
    for block in re.findall(r"<plugin[^>]*lift-drag[^>]*>(.*?)</plugin>", text, re.S):
        p = dict(link=tag(block, "link_name"),
                 a0=float(tag(block, "a0")), cla=float(tag(block, "cla")),
                 cda=float(tag(block, "cda")), cma=float(tag(block, "cma")),
                 alpha_stall=float(tag(block, "alpha_stall")),
                 area=float(tag(block, "area")), cp=vec(tag(block, "cp")),
                 up=vec(tag(block, "upward")), joint=tag(block, "control_joint_name"))
        panels.append(p)
    return masses, panels


def classify(p):
    if p["link"].startswith("motor"):
        return "rotor blade"
    up = p["up"]
    if abs(up[2]) > 0.9:
        x, y = p["cp"][0], abs(p["cp"][1])
        if x < -0.3:
            return "elevator" if p["joint"] else "horizontal tail"
        return "aileron section" if p["joint"] else "main wing"
    if p["joint"]:
        return "rudder"
    return "winglet" if abs(p["cp"][1]) > 0.5 else "vertical fin"


def openvsp_wing(path):
    """Return (CL_alpha per rad, zero-lift alpha rad, Sref m^2) from a VSPAERO polar."""
    fields, name, sref = {}, None, None
    for row in csv.reader(open(path)):
        if not row:
            continue
        if row[0] == "Results_Name":
            name = row[1]
        elif name == "VSPAERO_Polar":
            fields[row[0]] = [float(v) for v in row[1:] if v.strip()]
        elif row[0] == "FC_Sref_" and sref is None:
            sref = float(row[1]) * 1e-4          # OpenVSP model is in cm
    a, cl = fields["Alpha"], fields["CLtot"]
    n = len(a) // 2 if a[:len(a) // 2] == a[len(a) // 2:2 * (len(a) // 2)] else len(a)
    pts = [(ai, ci) for ai, ci in zip(a[:n], cl[:n]) if -5 <= ai <= 8]
    ma = sum(x for x, _ in pts) / len(pts)
    mc = sum(y for _, y in pts) / len(pts)
    slope = sum((x - ma) * (y - mc) for x, y in pts) / sum((x - ma) ** 2 for x, _ in pts)
    alpha0 = ma - mc / slope
    return math.degrees(slope), math.radians(-alpha0), sref


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sdf")
    ap.add_argument("--rho", type=float, default=1.2041)
    ap.add_argument("--openvsp", default=None)
    args = ap.parse_args()

    masses, panels = load(args.sdf)
    mass = sum(masses)
    weight = mass * G
    print(f"links: {len(masses)}   total mass {mass:.3f} kg   weight {weight:.2f} N"
          f"   (base_link alone {masses[0]:.3f} kg = {masses[0] * G:.2f} N)\n")

    for p in panels:
        p["kind"] = classify(p)
        p["clmax"] = p["cla"] * p["alpha_stall"]
        p["s_clmax"] = p["area"] * p["clmax"]

    wing_like = [p for p in panels if p["kind"] in ("main wing", "aileron section", "horizontal tail", "elevator")]
    total = sum(p["s_clmax"] for p in wing_like)
    print(f"{'rank':>4} {'surface':16} {'area m2':>9} {'cla':>6} {'a0 rad':>7} {'stall deg':>9} "
          f"{'CLmax':>6} {'S*CLmax':>8} {'share':>6}  flags")
    for i, p in enumerate(sorted(panels, key=lambda q: -q["s_clmax"]), 1):
        share = f"{100 * p['s_clmax'] / total:5.1f}%" if p in wing_like else "   -- "
        flags = []
        if p["kind"] != "rotor blade" and p["clmax"] > 1.8:
            flags.append("CLmax above any plain airfoil")
        if p["kind"] != "rotor blade" and math.degrees(p["alpha_stall"]) > 20:
            flags.append("stall angle > 20 deg")
        if p["cma"] == 0.0 and p["kind"] in ("main wing", "horizontal tail"):
            flags.append("no pitching moment (cma=0)")
        print(f"{i:4d} {p['kind']:16} {p['area']:9.5f} {p['cla']:6.2f} {p['a0']:7.3f} "
              f"{math.degrees(p['alpha_stall']):9.1f} {p['clmax']:6.2f} {p['s_clmax']:8.4f} {share}  {'; '.join(flags)}")

    wing = [p for p in panels if p["kind"] in ("main wing", "aileron section")]
    s_wing = sum(p["area"] for p in wing)
    sc_wing = sum(p["s_clmax"] for p in wing)
    sc_main = sum(p["s_clmax"] for p in panels if p["kind"] == "main wing")

    def vs(sc):
        return math.sqrt(2 * weight / (args.rho * sc))

    print(f"\nwing planform (main + aileron sections): {s_wing:.4f} m^2")
    print(f"sum S*CLmax, wing + tail panels:   {total:.4f} m^2 -> 1-g stall {vs(total):5.1f} m/s (optimistic bound)")
    print(f"sum S*CLmax, wing + aileron panels:{sc_wing:.4f} m^2 -> 1-g stall {vs(sc_wing):5.1f} m/s")
    print(f"sum S*CLmax, centre wing panel only:{sc_main:.4f} m^2 -> 1-g stall {vs(sc_main):5.1f} m/s (conservative)")

    if args.openvsp:
        cla, a0, sref = openvsp_wing(args.openvsp)
        main = next(p for p in panels if p["kind"] == "main wing")
        print(f"\nOpenVSP polar: CL_alpha {cla:.2f} /rad, zero-lift offset a0 {a0:.3f} rad, Sref {sref:.4f} m^2")
        print(f"Gazebo wing:   CL_alpha {main['cla']:.2f} /rad, a0 {main['a0']:.3f} rad, area {s_wing:.4f} m^2")
        print("(VSPAERO is a potential-flow solver: it gives the slope and offset but no stall.)")


if __name__ == "__main__":
    main()
