#!/usr/bin/env python3
"""Plan the Kinova extrusion of the homeostat path so nothing touches what is already printed.

For every point along the strand, in print order (outside in), it picks a nozzle direction
(tilt from straight down + azimuth, roll free) that keeps

  * the hot end parts (nozzle, heater block, heatsink + fan duct, mount) and
  * every arm link from the upper arm to the bracelet

clear of all strand laid so far and of the table, then solves IK warm-started from the
previous point so the joints move smoothly. Needs numpy only, so it also runs on armster.

    python plan.py                    # writes plan.json
    python plan.py --scale 160        # bigger print (mm per 90 deg of needle travel)

The hot end is a guess until the real one is measured: see HOTEND below.
"""
import argparse, json, math, os, sys
import numpy as np
import gen3, wrist
from homeostat_path import load_path

# ---------------------------------------------------------------- geometry (metres)
# Tool frame = the nozzle tip: flange + gen3.NOZZLE along the flange z. The arm itself is still set to a
# 0.200 m tool (gen3.ARM_TCP), so poses sent to it go through gen3.arm_tcp() (the "tcp" field below).
# Tool z points out of the nozzle, so the hot end body occupies tool z in [-NOZZLE, 0].
# (name, z_from, z_to, radius, clearance wanted from strand, ignore the last N mm of fresh strand)
HOTEND = [
    ("nozzle",  -0.006, 0.000, 0.0025, 0.0005, 4.0),
    ("heater",  -0.018, -0.006, 0.0140, 0.0020, 4.0),
    ("duct",    -0.060, -0.018, 0.0300, 0.0050, 0.0),
    ("mount",   -gen3.NOZZLE, -0.060, 0.0350, 0.0080, 0.0),
]
# Arm capsules between consecutive frames from gen3.frames(): index pairs, radius, clearance.
# frames: 0..5 = after joints 1..6, 6 = flange.  (Base and shoulder are below/behind the work.)
ARM = [("upper arm", 1, 2, 0.050, 0.015), ("forearm", 2, 3, 0.045, 0.015),
       ("wrist 1", 3, 4, 0.045, 0.015), ("wrist 2", 4, 5, 0.040, 0.015),
       ("bracelet", 5, 6, 0.045, 0.015)]
# Same hot end with a 20 mm nozzle extension: the heater sits higher, so it can reach between
# the tight inner loops. Selected with --long-nozzle.
HOTEND_LONG = [
    ("nozzle",  -0.020, 0.000, 0.0030, 0.0005, 4.0),
    ("heater",  -0.032, -0.020, 0.0140, 0.0020, 0.0),
    ("duct",    -0.075, -0.032, 0.0300, 0.0050, 0.0),
    ("mount",   -gen3.NOZZLE, -0.075, 0.0350, 0.0080, 0.0),
]
BEAD_R = 0.0012          # free-formed strand radius
TOOL = np.eye(4); TOOL[2, 3] = gen3.NOZZLE
DOWN_EULER = (180.0, 0.0, 90.0)   # paint_sim's "tool pointing at the desk"

# ---------------------------------------------------------------- distances
def pts_to_cyl(P, base, axis, z0, z1, r):
    """Signed-ish distance from points to a solid finite cylinder (0 inside)."""
    v = P - base
    a = v @ axis
    rad = np.linalg.norm(v - np.outer(a, axis), axis=1)
    dr = np.maximum(rad - r, 0.0)
    da = np.maximum(np.maximum(z0 - a, a - z1), 0.0)
    return np.hypot(dr, da)

def pts_to_seg(P, A, B):
    AB = B - A; L2 = AB @ AB
    t = np.clip(((P - A) @ AB) / L2, 0, 1) if L2 > 0 else np.zeros(len(P))
    return np.linalg.norm(P - (A + np.outer(t, AB)), axis=1)

def tool_clearance(R, tip, strand, arc_back, table_z):
    """Min clearance (m, after radii and wanted margin) of each hot end part. Negative = too close."""
    axis = R[:, 2]; out = []
    for name, z0, z1, r, want, ignore_mm in HOTEND:
        P = strand[arc_back > ignore_mm / 1000.0] if ignore_mm else strand
        d = pts_to_cyl(P, tip, axis, z0, z1, r).min() - BEAD_R - want if len(P) else 1.0
        if name != "nozzle":   # lowest point of this cylinder vs the table
            ends = [tip + axis * z0, tip + axis * z1]
            low = min(e[2] for e in ends) - r * math.sqrt(max(0.0, 1 - axis[2] ** 2))
            d = min(d, low - table_z - want)
        out.append(d)
    return out

def arm_clearance(F, strand, table_z):
    out = []
    for name, i, j, r, want in ARM:
        A, B = F[i][:3, 3], F[j][:3, 3]
        d = (pts_to_seg(strand, A, B).min() - BEAD_R if len(strand) else 1.0) - r - want
        d = min(d, min(A[2], B[2]) - r - table_z - 0.02)
        out.append(d)
    return out

# ---------------------------------------------------------------- IK
def _err(T, Tt):
    dp = Tt[:3, 3] - T[:3, 3]
    Re = Tt[:3, :3] @ T[:3, :3].T
    ang = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]]) * 0.5
    return np.r_[dp, ang]

def ik(Tt, q0, iters=60):
    q = np.array(q0, float)
    for _ in range(iters):
        T = gen3.fk(q, TOOL); e = _err(T, Tt)
        if np.linalg.norm(e[:3]) < 1e-5 and np.linalg.norm(e[3:]) < 1e-4:
            break
        J = np.zeros((6, 6)); h = 1e-4
        for k in range(6):
            dq = q.copy(); dq[k] += math.degrees(h)
            J[:, k] = _err(T, gen3.fk(dq, TOOL)) / h
        lam = 0.02
        dx = np.linalg.solve(J.T @ J + lam ** 2 * np.eye(6), J.T @ e)
        q = q + np.degrees(np.clip(dx, -0.2, 0.2))
    T = gen3.fk(q, TOOL); e = _err(T, Tt)
    ok = np.linalg.norm(e[:3]) < 2e-4 and np.linalg.norm(e[3:]) < 2e-3
    for k, lim in enumerate(gen3.LIM):
        if lim is not None and abs(((q[k] + 180) % 360) - 180) > lim - 2:
            ok = False
    if abs(((q[2] + 180) % 360) - 180) < J3_MIN - 5:     # elbow nearly straight: the estimate let it through
        ok = False
    return q, ok

# ---------------------------------------------------------------- orientation candidates
def frame_from_axis(z, x_hint):
    z = z / np.linalg.norm(z)
    x = x_hint - (x_hint @ z) * z
    if np.linalg.norm(x) < 1e-6:
        x = np.cross(z, [0, 1, 0]) if abs(z[1]) < 0.9 else np.cross(z, [1, 0, 0])
    x /= np.linalg.norm(x)
    return np.c_[x, np.cross(z, x), z]

TILTS = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90]
AZIS = list(range(0, 360, 30))
MAX_JUMP = 45.0   # deg any joint may move between neighbouring points 2 mm apart
CANDS = [(0, 0)] + [(t, a) for t in TILTS[1:] for a in AZIS]

def axis_of(tilt, azi):
    t, a = math.radians(tilt), math.radians(azi)
    return np.array([math.sin(t) * math.cos(a), math.sin(t) * math.sin(a), -math.cos(t)])

# ---------------------------------------------------------------- placement
def rot_xy(roll, pitch):
    """Rotation about world x by roll, then about world y by pitch (deg)."""
    cr, sr, cp, sp = math.cos(math.radians(roll)), math.sin(math.radians(roll)), math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]); Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return Ry @ Rx

def place_base(P_mm):
    """Page axes -> world (x right, page y up -> world z up, page z -> world -y), rotated so the first
    (outermost) point is the lowest, then centred in x/y with its lowest point at z = 0."""
    W = np.c_[P_mm[:, 0], -P_mm[:, 2], P_mm[:, 1]] / 1000.0
    c = W.mean(0); u = W[0] - c; u /= np.linalg.norm(u); d = np.array([0, 0, -1.0])
    v = np.cross(u, d); s = np.linalg.norm(v); cth = u @ d
    if s > 1e-9:
        K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]]) / s
        Rm = np.eye(3) + s * K + (1 - cth) * K @ K
        W = (W - c) @ Rm.T + c
    return W - [W[:, 0].mean(), W[:, 1].mean(), W[:, 2].min()]

def place_from_base(B, centre, table_z, lift, yaw=0.0, roll=0.0, pitch=0.0):
    """Tilt (roll about x, then pitch about y) about the middle, set back down on the table (lowest
    point at table + lift), turn about the vertical by yaw, move to centre. editor.html does the same."""
    W = B
    if roll or pitch:
        W = W @ rot_xy(roll, pitch).T
        W = W - [W[:, 0].mean(), W[:, 1].mean(), W[:, 2].min()]
    cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    W = W @ np.array([[cy, sy, 0], [-sy, cy, 0], [0, 0, 1]])     # turn about the vertical through its middle
    return W + [centre[0], centre[1], table_z + lift]

def needs_post(W, table_z):
    """The anchor (first point) has to stand on something: a post when it is above the table."""
    return W[0][2] > table_z + 1e-3

def place(P_mm, centre, table_z, lift, yaw=0.0, roll=0.0, pitch=0.0):
    """Page axes -> world (x right, page y up -> world z up, page z -> world -y),
    rotated so the first (outermost) point is the lowest, i.e. the anchor on the table."""
    return place_from_base(place_base(P_mm), centre, table_z, lift, yaw, roll, pitch)

# ---------------------------------------------------------------- look-ahead planning
def fib_dirs(n=1600, max_up=0.26):
    """Nozzle directions spread evenly over the sphere, from straight down to ~15 deg above horizontal."""
    k = np.arange(n) + 0.5; phi = np.arccos(1 - 2 * k / n); th = math.pi * (1 + 5 ** 0.5) * k
    D = np.c_[np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)]
    return D[D[:, 2] <= max_up]

def clearance_all_dirs(D, tip, strand, back, table_z):
    """Min hot end clearance (m) for every direction at once. Same rules as tool_clearance."""
    best = np.full(len(D), 1.0)
    for name, z0, z1, r, want, ignore_mm in HOTEND:
        P = strand[back > ignore_mm / 1000.0] if ignore_mm else strand
        if len(P):
            v = P - tip                                   # (p,3)
            a = v @ D.T                                   # (p,d) axial
            rad = np.sqrt(np.maximum((v * v).sum(1)[:, None] - a * a, 0))
            dist = np.hypot(np.maximum(rad - r, 0), np.maximum(np.maximum(z0 - a, a - z1), 0)).min(0)
            best = np.minimum(best, dist - BEAD_R - want)
        if name != "nozzle":
            low = np.minimum(tip[2] + D[:, 2] * z0, tip[2] + D[:, 2] * z1) - r * np.sqrt(np.maximum(0, 1 - D[:, 2] ** 2))
            best = np.minimum(best, low - table_z - want)
    return best

ROLLS = [0, 30, -30, 60, -60, 90, -90, 135, -135, 180]
J5_MAX = 112.0     # deg; the joint stops at 120.3
J5_MIN = 6.0       # keep off the wrist singularity, where J4 and J6 spin
J3_MAX = 143.0     # deg; the elbow stops at 147.8
J3_MIN = 25.0      # deg; keep the elbow off straight (the reach-edge singularity). At ~10 deg the arm's own
                   # IK gives up ("second convergence failed"), and the model's few mm of error count most there.

def arm_limits_ok(tip, D):
    """Directions the arm can take at tip as far as reach and joint limits go: (ok, j5)."""
    j5, reach = wrist.j5_est(tip, D)
    j3 = wrist.j3_est(tip, D)
    return reach & (j5 <= J5_MAX) & (j5 >= J5_MIN) & (j3 <= J3_MAX) & (j3 >= J3_MIN), j5
STEP_DEG = 9.0      # most the nozzle direction may swing between points 2 mm apart

def arm_clearance_all_dirs(D, tip, strand, table_z):
    """Upper arm, forearm and wrist (J5 to flange, on the tool axis) vs strand and table, every direction."""
    E, Wc, _ = wrist.arm_points(tip, D)
    F0 = tip[None, :] - gen3.NOZZLE * D                 # flange
    segs = [(np.broadcast_to(wrist.SH, E.shape), E, 0.050), (E, Wc, 0.045), (Wc, F0, 0.045)]
    best = np.full(len(D), 1.0)
    for A, B, r in segs:
        if len(strand):
            best = np.minimum(best, wrist.seg_dist(strand, A, B).min(0) - BEAD_R - r - 0.015)
        best = np.minimum(best, np.minimum(A[:, 2], B[:, 2]) - r - table_z - 0.02)
    return best

def choose_dirs(D, C, banned, start):
    """Dynamic programme: one direction per point, each clear, neighbours within STEP_DEG,
    minimising total swing + tilt + time spent close to the strand."""
    n, nd = C.shape
    ang = np.degrees(np.arccos(np.clip(D @ D.T, -1, 1)))
    K = int((ang <= STEP_DEG).sum(1).max())
    nb = np.argsort(ang, axis=1)[:, :K]; nba = np.take_along_axis(ang, nb, 1)
    nba[nba > STEP_DEG] = np.inf
    tilt = np.degrees(np.arccos(np.clip(-D[:, 2], -1, 1)))
    feas = (C >= 0) & ~banned
    local = tilt / 30.0 + np.clip(0.006 - C, 0, None) / 0.006 * 2.0
    cost = np.where(feas[0], local[0] + np.degrees(np.arccos(np.clip(D @ start, -1, 1))) / 30.0, np.inf)
    back = np.zeros((n, nd), int)
    for i in range(1, n):
        cand = cost[nb] + nba / 3.0                 # (nd, K)
        j = np.argmin(cand, 1)
        best = cand[np.arange(nd), j]
        back[i] = nb[np.arange(nd), j]
        cost = np.where(feas[i], best + local[i], np.inf)
        if not np.isfinite(cost).any():
            return None, i
    path = [int(np.argmin(cost))]
    for i in range(n - 1, 0, -1):
        path.append(int(back[i][path[-1]]))
    return path[::-1], n

def strand_samples(W, table_z, light=False):
    """1 mm samples along the path (arc length S), with the post under the anchor: what the hot end and
    arm must keep clear of. Drawing with light leaves nothing behind, so then there is nothing to avoid
    but the table (checked separately)."""
    if light:
        return np.zeros((0, 3)), np.zeros(0)
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(W, axis=0), axis=1))]
    S = np.arange(0, s[-1] + 1e-9, 0.001)
    dense = np.c_[[np.interp(S, s, W[:, k]) for k in range(3)]].T
    if needs_post(W, table_z):
        dense, S = with_post(W, dense, S, table_z)
    return dense, S

def with_post(W, dense, S, table_z):
    post = np.array([[W[0][0] + 0.010 * math.cos(a), W[0][1] + 0.010 * math.sin(a), z]
                     for z in np.arange(table_z, W[0][2], 0.005) for a in np.arange(0, 6.28, 0.785)])
    return np.r_[post, dense], np.r_[np.full(len(post), -1.0), S]

def plan(args):
    P_mm, T = load_path(args.path_file, args.preset, args.seconds, args.scale, args.step)
    if args.from_t > 0:   # just the attractor: from the last uniselector step on
        keep = T >= args.from_t; P_mm, T = P_mm[keep], T[keep]
        if len(P_mm) < 2:
            raise SystemExit("Nothing in the path after t = %g s." % args.from_t)
    W = place(P_mm, (args.x, args.y), args.table, args.lift, args.yaw, args.roll, args.pitch)
    if args.trim > 0:
        W = W[: max(2, len(W) - int(round(args.trim / args.step)))]
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(W, axis=0), axis=1))]
    # the strand so far (and the post the anchor sits on), or nothing at all when drawing with light
    dense, S = strand_samples(W, args.table, args.light)
    D = fib_dirs(max_up=1.0) if args.light else fib_dirs()   # a light may point any way, up included
    if args.light:
        # A light drawing should flow: its tip slows to a crawl wherever the joints must swing a lot for a little
        # path, which happens near the wrist singularity (J5 near 0) and where the light turns sharply.
        global J5_MIN, STEP_DEG
        J5_MIN = float(os.environ.get("LIGHT_J5_MIN", 25.0)); STEP_DEG = float(os.environ.get("LIGHT_STEP_DEG", 6.0))
    # 1. every clear direction at every point
    C = np.zeros((len(W), len(D)))
    for i, tip in enumerate(W):
        if args.progress and i % 100 == 0:
            print("finding clear directions: point %d of %d" % (i, len(W)), flush=True)
        tan = W[min(i + 1, len(W) - 1)] - W[max(i - 1, 0)]; tan /= (np.linalg.norm(tan) or 1)
        m = S <= s[i] + 1e-9
        C[i] = clearance_all_dirs(D, tip, dense[m], s[i] - S[m], args.table)
        if not args.light:
            C[i][D @ tan > 0.5] = -1.0    # moving into the nozzle's own direction buries the strand
        C[i] = np.minimum(C[i], arm_clearance_all_dirs(D, tip, dense[m], args.table))
        C[i][~arm_limits_ok(tip, D)[0]] = -1.0   # out of reach, wrist or elbow past its limit, elbow too straight
    free = (C >= 0).sum(1)
    blocked = np.nonzero(free == 0)[0]
    end = int(blocked[0]) if len(blocked) else len(W)
    if args.verbose:
        print("clear directions per point: median %d, min %d; first point with none: %s of %d"
              % (np.median(free), free.min(), blocked[:1].tolist(), len(W)), file=sys.stderr)
    # 2. smooth direction sequence, 3. IK along it; ban what fails and go again
    banned = np.zeros_like(C, bool); start = np.array([0, 0, -1.0])
    # Light drawings: the direction grid is ~5 deg apart, and each 5 deg turn of the light swings the wrist
    # (40 cm back from the tip) 3-4 cm in one 2 mm step: the elbow lurches and the tip crawls. So the chosen
    # directions are smoothed along the path (window in points), narrower each time a smoothed pose fails.
    win = int(os.environ.get("LIGHT_SMOOTH", 41)) if args.light else 0
    for attempt in range(40):
        if args.progress:
            print("smoothing the directions and solving the arm's joints, pass %d" % (attempt + 1), flush=True)
        path, reached = choose_dirs(D, C[:end], banned[:end], start)
        if path is None:
            end = reached; continue        # nothing smooth gets past `reached`: print stops there
        Zs = D[np.array(path[:end])]
        if win >= 3:
            k = np.ones(win) / win; pad = win // 2
            Zp = np.r_[np.repeat(Zs[:1], pad, 0), Zs, np.repeat(Zs[-1:], pad, 0)]
            Zs = np.c_[[np.convolve(Zp[:, c], k, "valid") for c in range(3)]].T
            Zs /= np.linalg.norm(Zs, axis=1)[:, None]
        q = np.array(args.start_joints, float); x_hint = gen3.from_euler(*DOWN_EULER)[:, 0]
        out, bad = [], None
        for i in range(end):
            if args.progress and i % 100 == 0:
                print("solving the arm's joints: point %d of %d" % (i, end), flush=True)
            z = Zs[i]; tip = W[i]; m = S <= s[i] + 1e-9
            ok = False; why = []
            for roll in ROLLS:          # the hot end is round, so roll is free: take the one the wrist likes
                ca, sa = math.cos(math.radians(roll)), math.sin(math.radians(roll))
                h = ca * x_hint + sa * np.cross(z, x_hint)
                R = frame_from_axis(z, h)
                Tt = np.eye(4); Tt[:3, :3] = R; Tt[:3, 3] = tip
                qn, conv = ik(Tt, q, 200 if i else 400)
                jump = np.max(np.abs(((qn - q + 180) % 360) - 180))
                if not conv: why.append("ik r%d" % roll); continue
                if i and jump > MAX_JUMP: why.append("jump %.0f r%d" % (jump, roll)); continue
                ac = arm_clearance(gen3.frames(qn), dense[m], args.table)
                if min(ac) < 0: why.append("%s %.0fmm r%d" % (ARM[int(np.argmin(ac))][0], min(ac) * 1000, roll)); continue
                if win >= 3 and min(tool_clearance(R, tip, dense[m], s[i] - S[m], args.table)) < 0:
                    why.append("table r%d" % roll); continue      # a smoothed direction, not one the grid checked
                ok = True; break
            if not ok:
                bad = i; break
            q = qn; x_hint = R[:, 0]
            tc = tool_clearance(R, tip, dense[m], s[i] - S[m], args.table)
            tilt = math.degrees(math.acos(np.clip(-z[2], -1, 1)))
            tcp = tip - (gen3.NOZZLE - gen3.ARM_TCP) * z      # what the arm (0.200 m tool) is told
            out.append(dict(i=i, t=round(float(T[i]), 3), tip=tip.round(5).tolist(), tcp=tcp.round(5).tolist(),
                            ok=True, tilt=round(tilt, 1),
                            azi=round(math.degrees(math.atan2(z[1], z[0])), 1),
                            euler=gen3.euler_xyz(R).round(3).tolist(), joints=np.round(q, 3).tolist(),
                            hotend_mm=[round(v * 1000, 1) for v in tc], arm_mm=[round(v * 1000, 1) for v in ac]))
        if bad is None:
            break
        if win >= 3:              # a smoothed pose failed: smooth less, and try again before banning anything
            win = win // 2 if win // 2 >= 3 else 0
            if args.verbose:
                print("attempt %d: smoothed pose at point %d failed (%s); smoothing over %d points" % (attempt, bad, "; ".join(why[:2]), win), file=sys.stderr)
            continue
        if args.verbose:
            print("attempt %d: point %d (tilt %.0f): %s" % (attempt, bad, math.degrees(math.acos(-D[path[bad]][2])), "; ".join(why[:4])), file=sys.stderr)
        near = np.degrees(np.arccos(np.clip(D @ D[path[bad]], -1, 1))) < 6
        banned[max(0, bad - 2): bad + 3, near] = True
    for i in range(len(out), len(W)):
        out.append(dict(i=i, tip=W[i].round(5).tolist(), ok=False))
    return dict(params=dict(vars(args), nozzle=gen3.NOZZLE), strand=W.round(5).tolist(), waypoints=out, fails=len(W) - sum(w["ok"] for w in out),
                printable_mm=round(float(s[len([w for w in out if w["ok"]]) - 1]) * 1000, 1), total_mm=round(float(s[-1]) * 1000, 1),
                hotend=[h[0] for h in HOTEND], arm=[a[0] for a in ARM])

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--preset", type=int, default=0)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--path-file", default=None, help="a path sent from the simulator page (JSON); replaces --preset and --seconds")
    ap.add_argument("--scale", type=float, default=120.0, help="mm per 90 deg, as the page's export box")
    ap.add_argument("--step", type=float, default=2.0, help="mm between waypoints")
    ap.add_argument("--x", type=float, default=0.45); ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--from-t", type=float, default=0.0, help="start at this model time, s (10.0 = last uniselector step, spiral only)")
    ap.add_argument("--yaw", type=float, default=0.0, help="turn the sculpture about the vertical, deg")
    ap.add_argument("--roll", type=float, default=0.0, help="tilt about the world x axis before the yaw, deg")
    ap.add_argument("--pitch", type=float, default=0.0, help="tilt about the world y axis before the yaw, deg")
    ap.add_argument("--table", type=float, default=0.0, help="table height in the arm's base frame, m")
    ap.add_argument("--lift", type=float, default=0.0, help="print this far above the table (air run), m")
    ap.add_argument("--start-joints", type=float, nargs=6, default=[0, 15, 230, 0, 55, 90])
    ap.add_argument("--out", default="plan.json")
    ap.add_argument("--long-nozzle", action="store_true", help="hot end with a 20 mm nozzle extension")
    ap.add_argument("--trim", type=float, default=0.0, help="stop this many mm before the end of the spiral")
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument("--progress", action="store_true", help="print progress lines as it goes")
    ap.add_argument("--light", action="store_true", help="drawing with light: nothing is left behind, so no strand to avoid")
    a = ap.parse_args()
    if a.long_nozzle: HOTEND[:] = HOTEND_LONG
    res = plan(a)
    json.dump(res, open(a.out, "w"))
    wp = [w for w in res["waypoints"] if w["ok"]]
    print("%d waypoints, %d with no safe pose; printable %.0f of %.0f mm of strand"
          % (len(res["waypoints"]), res["fails"], res["printable_mm"], res["total_mm"]))
    if wp:
        print("worst hot end clearance %.1f mm, worst arm clearance %.1f mm, max tilt %d deg"
              % (min(min(w["hotend_mm"]) for w in wp), min(min(w["arm_mm"]) for w in wp), max(w["tilt"] for w in wp)))
