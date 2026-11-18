"""What the layout editor (editor.py + editor.html) asks of the planner, kept quick.

reach_map(table)   where the nozzle can get to at all, ignoring the print: the arm's base turns all the way
                   round, so this only depends on distance from the base axis (r) and height (z). A 2-D map,
                   revolved into a ring by the page.
quick_check(...)   per point of a placed path: can some direction reach it past the strand printed so far?
                   The same tests as the planner's first pass (plan.plan step 1), without the smoothing and IK
                   passes, so it is optimistic: plan.py + arm_sim.py are the result to trust.
"""
import math
import numpy as np
import plan, wrist
from homeostat_path import path_mm

DIRS = plan.fib_dirs(900)          # the sweep's direction set: down to ~15 deg above horizontal
R_MAX, Z_MAX, CELL = 1.20, 0.80, 0.01
BASE_R = 0.12                      # the arm's own base and shoulder column: nothing gets printed in there

HOT_R = 0.26                       # strand further than this from the tip cannot touch the hot end (230 mm long)
ARM_EVERY = 4                      # arm links are tested against every 4th mm of strand (15 mm margin; <= 2 mm optimistic)

OK, BLOCKED, NO_REACH, OUTSIDE_SWEEP = 0, 1, 2, 3


class Cancelled(Exception):
    pass


def arm_only(tip, D, table):
    """Directions the arm itself can take at tip: reach, wrist limits, links and hot end above the table."""
    if math.hypot(tip[0], tip[1]) < BASE_R:
        return np.zeros(len(D), bool)
    j5, reach = wrist.j5_est(tip, D)
    ok = reach & (j5 <= plan.J5_MAX) & (j5 >= plan.J5_MIN) & (wrist.j3_est(tip, D) <= plan.J3_MAX)
    if ok.any():
        idx = np.nonzero(ok)[0]
        empty = np.zeros((0, 3))
        c = np.minimum(plan.clearance_all_dirs(D[idx], tip, empty, np.zeros(0), table),
                       plan.arm_clearance_all_dirs(D[idx], tip, empty, table))
        ok[idx[c < 0]] = False
    return ok


def reach_map(table):
    """Fraction of DIRS the arm can take, on an r x z grid (CELL apart) from the table up."""
    rs = np.arange(0, R_MAX + 1e-9, CELL); zs = np.arange(table, Z_MAX + 1e-9, CELL)
    M = np.zeros((len(zs), len(rs)))
    for a, z in enumerate(zs):
        for b, r in enumerate(rs):
            M[a, b] = arm_only(np.array([r, 0.0, z]), DIRS, table).mean()
    return dict(r0=0.0, z0=float(table), cell=CELL, nr=len(rs), nz=len(zs), frac=np.round(M, 3).tolist())


def base_shape(preset, seconds, scale, from_t, step=2.0):
    """The path at this scale, oriented with its anchor lowest, centred, lowest point at z = 0."""
    P_mm, T = path_mm(preset, seconds, scale, step)
    if from_t > 0:
        keep = T >= from_t; P_mm, T = P_mm[keep], T[keep]
    return plan.place_base(P_mm), T


def in_sweep(p, sweep):
    if not sweep:
        return True
    a = math.degrees(math.atan2(p[1], p[0]))
    return sweep[0] <= a <= sweep[1]


def quick_check(W, table, sweep=None, cancel=None):
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(W, axis=0), axis=1))]
    S = np.arange(0, s[-1] + 1e-9, 0.001)
    dense = np.c_[[np.interp(S, s, W[:, k]) for k in range(3)]].T
    if plan.needs_post(W, table):
        dense, S = plan.with_post(W, dense, S, table)
    status, free = [], []
    for i, tip in enumerate(W):
        if cancel and i % 16 == 0 and cancel():
            raise Cancelled()
        if not in_sweep(tip, sweep):
            status.append(OUTSIDE_SWEEP); free.append(0); continue
        tan = W[min(i + 1, len(W) - 1)] - W[max(i - 1, 0)]; tan /= (np.linalg.norm(tan) or 1)
        ok = arm_only(tip, DIRS, table) & (DIRS @ tan <= 0.5)
        if not ok.any():
            status.append(NO_REACH); free.append(0); continue
        D = DIRS[ok]; m = S <= s[i] + 1e-9
        P1, back = dense[m], s[i] - S[m]
        near = ((P1 - tip) ** 2).sum(1) < HOT_R ** 2
        C = np.minimum(plan.clearance_all_dirs(D, tip, P1[near], back[near], table),
                       plan.arm_clearance_all_dirs(D, tip, P1[::ARM_EVERY], table))
        n = int((C >= 0).sum())
        status.append(OK if n else BLOCKED); free.append(n)
    stop = next((i for i, v in enumerate(status) if v != OK), len(W))
    return dict(status=status, free=free, stop=stop, total_mm=round(float(s[-1]) * 1000, 1),
                printable_mm=round(float(s[stop - 1]) * 1000, 1) if stop else 0.0, post=bool(plan.needs_post(W, table)))
