"""Needle for the pot node: a push-fit pointer for the Alps RK16812MG's 6 mm D shaft.

A round hub on the shaft, a low arm, and a raised tab at the far end that is both the pointer
(its tip reads against the -50 / 0 / +50 marks) and the place a hand takes hold. Prints flat on
its back, no supports: hub, arm and tab all sit on z = 0.

    python node/print/needle_v1_0.py                    -> node/print/needle_v1_0.stl
    python node/print/needle_v1_0.py --bore-d 6.2       loosen the fit after a test print

Coordinates: shaft axis at the origin, Z up, needle along +X. The STL's 80-byte header and
claims.json carry the git version it was built from.

Claims, not print statements: the build exits non-zero if the part is not one closed body that
survives the STL write, or if the bore, tip or heights are not what was asked for.
"""

import argparse
import datetime
import json
import math
import os
import subprocess
import sys

import build123d as bd
import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import claims as K  # noqa: E402

VERSION = "1.0"


def git_version():
    """'commit · dd Mon HH:mm [+N uncommitted]', as the firmware's version.h."""
    try:
        run = lambda *a: subprocess.run(a, cwd=HERE, capture_output=True, text=True).stdout.strip()
        commit = run("git", "rev-parse", "--short", "HEAD")
        when = datetime.datetime.fromisoformat(run("git", "log", "-1", "--format=%cI"))
        dirty = len([l for l in run("git", "status", "--porcelain", "--", ".").splitlines()
                     if not l.endswith((".stl", "claims.json"))])
        return "%s · %s%s" % (commit, when.strftime("%d %b %H:%M"),
                                   " +%d uncommitted" % dirty if dirty else "")
    except Exception as e:  # noqa: BLE001
        return "unversioned (%s)" % e


def build(a):
    hub = bd.Cylinder(a.hub_d / 2, a.hub_h, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

    tab_x0 = a.tip - a.tab_len
    arm = bd.Pos(tab_x0 / 2, 0, 0) * bd.Box(tab_x0, a.arm_w, a.arm_h,
                                             align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))

    # tab: plan outline with a V point at x = tip, extruded to tab height
    w = a.tab_w / 2
    point = a.tip - a.point_len
    tab_plan = bd.Polygon((tab_x0, -w), (point, -w), (a.tip, 0), (point, w), (tab_x0, w),
                          align=None)
    tab = bd.extrude(tab_plan, amount=a.tab_h)

    body = hub + arm + tab

    # D bore, through: circle cut by a flat. bore_flat = flat face to the far side of the circle.
    r = a.bore_d / 2
    d_flat = a.bore_flat - r                    # flat's distance from the axis
    circle = bd.Cylinder(r, a.hub_h + 2, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    keep = bd.Box(2 * r + 2, 2 * r + 2, a.hub_h + 2,
                  align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    keep = bd.Pos(d_flat - (r + 1), 0, 0) * keep          # everything with x <= d_flat
    d_hole = bd.Pos(0, 0, -1) * (circle & keep)
    d_hole = bd.Rot(0, 0, a.flat_angle) * d_hole            # 0 deg: flat faces the tip (+X)

    # lead-in at the bed side, so elephant's foot doesn't close the mouth of the bore
    lead = bd.Cone(r + a.lead_in, r, a.lead_in, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    return body - d_hole - lead


def bore_section(mesh, z, flat_angle):
    """The hole in a slice at height z, in the bore's own frame (flat normal along +X)."""
    sec = mesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    planar, to3d = sec.to_2D()
    holes = [np.asarray(h.coords) for p in planar.polygons_full for h in p.interiors]
    if not holes:
        return None
    # back to world XY, then rotate so the flat's normal is +X
    pts = []
    for h in holes:
        xyz = trimesh.transform_points(np.column_stack([h, np.zeros(len(h))]), to3d)
        pts.append(xyz[:, :2])
    h = min(pts, key=lambda p: np.linalg.norm(p.mean(axis=0)))   # the one on the axis
    t = math.radians(-flat_angle)
    rot = np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])
    return h.dot(rot.T)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--tip", type=float, default=90.0, help="shaft axis to the pointer tip, mm")
    p.add_argument("--hub-d", type=float, default=14.0)
    p.add_argument("--hub-h", type=float, default=10.0)
    p.add_argument("--arm-w", type=float, default=6.0)
    p.add_argument("--arm-h", type=float, default=5.0)
    p.add_argument("--tab-len", type=float, default=14.0, help="tab length along the arm, incl. point")
    p.add_argument("--tab-w", type=float, default=9.0)
    p.add_argument("--tab-h", type=float, default=10.0)
    p.add_argument("--point-len", type=float, default=5.0, help="length of the V point")
    p.add_argument("--bore-d", type=float, default=6.15, help="bore diameter: 6 mm shaft + push-fit allowance")
    p.add_argument("--bore-flat", type=float, default=4.6,
                   help="flat face to far side of the bore (shaft is ~4.5)")
    p.add_argument("--flat-angle", type=float, default=0.0,
                   help="where the bore's flat faces, deg from the tip direction")
    p.add_argument("--lead-in", type=float, default=0.4)
    p.add_argument("--min-wall", type=float, default=2.5, help="claim: thinnest hub wall around the bore, mm")
    p.add_argument("--fit-tol", type=float, default=0.06, help="claim: bore sizes within this, mm")
    p.add_argument("--plate", default="220x220x250")
    p.add_argument("--out", default=os.path.join(HERE, "needle_v%s.stl" % VERSION.replace(".", "_")))
    a = p.parse_args()

    ver = "needle v%s %s" % (VERSION, git_version())
    C = K.Claims(ver)
    C.note("parameters", "", **{k: v for k, v in vars(a).items() if k != "out"})

    part = build(a)
    bd.export_stl(part, a.out, tolerance=0.005, angular_tolerance=0.05)

    # stamp the version into the binary STL's 80-byte header
    with open(a.out, "r+b") as f:
        f.write(ver.encode("utf-8")[:80].ljust(80, b" "))

    mesh = trimesh.load(a.out, file_type="stl", process=True)   # claims are about the bytes on disk
    ok = K.closed_volume(C, mesh, "needle")
    ok &= K.single_body(C, mesh, "needle", floor=50.0)
    ok &= K.survives_the_file(C, mesh, "needle")
    ok &= K.fits_plate(C, mesh, a.plate, "needle")

    lo, hi = mesh.bounds
    ok &= C.require("tip is %.1f mm from the shaft axis" % a.tip, abs(hi[0] - a.tip) < 0.05,
                    "furthest point at x = %.3f" % hi[0], tip_x=round(float(hi[0]), 3))
    ok &= C.require("sits flat on the bed", abs(lo[2]) < 1e-6,
                    "lowest point z = %.4f" % lo[2], min_z=round(float(lo[2]), 4))
    ok &= C.require("tab is %.1f mm high" % a.tab_h, abs(hi[2] - max(a.tab_h, a.hub_h)) < 0.02,
                    "top at z = %.3f" % hi[2], top_z=round(float(hi[2]), 3))
    bed = mesh.section(plane_origin=[0, 0, 0.05], plane_normal=[0, 0, 1]).to_2D()[0].area
    C.note("footprint on the bed", "%.0f mm2 touching the plate" % bed, footprint_mm2=round(float(bed), 1))

    # the bore, measured mid-hub (clear of the lead-in), in the bore's own frame
    h = bore_section(mesh, a.hub_h / 2, a.flat_angle)
    if h is None:
        ok &= C.require("hub has a bore", False, "no hole in the slice at mid-hub")
    else:
        across_flat = h[:, 0].max() - h[:, 0].min()
        across_round = h[:, 1].max() - h[:, 1].min()
        flat_x = h[:, 0].max()
        ok &= C.require("bore is %.2f across the flat" % a.bore_flat,
                        abs(across_flat - a.bore_flat) <= a.fit_tol,
                        "measured %.3f mm" % across_flat, across_flat=round(float(across_flat), 3))
        ok &= C.require("bore is %.2f across the round" % a.bore_d,
                        abs(across_round - a.bore_d) <= a.fit_tol,
                        "measured %.3f mm" % across_round, across_round=round(float(across_round), 3))
        ok &= C.require("flat faces %.0f deg" % a.flat_angle,
                        abs(flat_x - (a.bore_flat - a.bore_d / 2)) <= a.fit_tol,
                        "flat %.3f mm from the axis" % flat_x, flat_from_axis=round(float(flat_x), 3))
        wall = a.hub_d / 2 - np.linalg.norm(h, axis=1).max()
        ok &= C.require("hub wall at least %.1f mm" % a.min_wall, wall >= a.min_wall,
                        "thinnest %.2f mm" % wall, wall_mm=round(float(wall), 2))

    C.note("weight", "%.1f cm3, ~%.1f g in PLA" % (mesh.volume / 1000, mesh.volume / 1000 * 1.24),
           volume_cm3=round(float(mesh.volume) / 1000, 2))
    passed = C.report(os.path.splitext(a.out)[0] + ".claims.json")
    print("wrote", a.out)
    if not (passed and ok):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
