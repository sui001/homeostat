"""A safe way out of a print, checked before anything moves.

From wherever the arm is (the end of a print, or wherever a Stop left it):
  1. back out along the nozzle, the way it came in (the one direction the planner knows is clear there),
  2. rise straight up to above the print's highest point plus a margin,
  3. a joint move to Home.
Every pose on the way, joint moves sampled every 2 deg, gets the planner's own clearance tests (hot end
and arm links against the strand printed so far, and the table). If any sample is too close, nothing is
returned to run: the arm stays where it is, to be moved by hand.

Pure planning on the arm model (gen3 + plan): air_run.py and safe_park.py turn it into motion.
"""
import math
import numpy as np
import gen3, plan

BACK_M = 0.05          # back out this far along the nozzle
RISE_MARGIN = 0.10     # then rise to this far above the print's highest point
STEP_M = 0.005         # IK every 5 mm on the way
SAMPLE_DEG = 2.0       # joint moves checked every 2 deg
HOME = (0.0, 15.0, 230.0, 0.0, 55.0, 90.0)


def strand_so_far(W, table, upto=None):
    """1 mm samples of the strand up to point `upto` (all of it if None), with the post under the anchor,
    and each sample's distance back from the newest end (the hot end may touch the freshest few mm)."""
    W = np.asarray(W, float)
    if upto is not None:
        W = W[: max(1, upto + 1)]
    if len(W) < 2:
        return np.zeros((0, 3)), np.zeros(0)
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(W, axis=0), axis=1))]
    S = np.arange(0, s[-1] + 1e-9, 0.001)
    dense = np.c_[[np.interp(S, s, W[:, k]) for k in range(3)]].T
    if plan.needs_post(W, table):
        dense, S = plan.with_post(W, dense, S, table)
    return dense, s[-1] - np.where(S < 0, -1.0, S)       # post samples (S = -1) count as old


def clearance(q, dense, back, table):
    """Smallest clearance (m, after margins) of the hot end and arm links at joints q, and what it is."""
    F = gen3.frames(q, plan.TOOL)
    hc = plan.tool_clearance(F[-1][:3, :3], F[-1][:3, 3], dense, back, table)
    ac = plan.arm_clearance(F[:-1], dense, table)
    i, j = int(np.argmin(hc)), int(np.argmin(ac))
    return (hc[i], plan.HOTEND[i][0]) if hc[i] < ac[j] else (ac[j], plan.ARM[j][0])


def joint_samples(qa, qb, step=SAMPLE_DEG):
    """Poses along a joint move from qa to qb (each joint the short way round), every `step` deg."""
    d = ((np.asarray(qb, float) - np.asarray(qa, float) + 180) % 360) - 180
    n = max(1, int(math.ceil(np.abs(d).max() / step)))
    return [np.asarray(qa, float) + d * (k / n) for k in range(1, n + 1)]


def nearest_point(W, tip):
    """Index of the strand point nearest the tip: how far a stopped print had got."""
    return int(np.argmin(np.linalg.norm(np.asarray(W) - tip, axis=1)))


def plan_exit(q_now, W, table, upto=None, home=HOME, back_m=BACK_M):
    """Returns dict(ok, chain, home, worst_mm, what, rise_to, reason). chain = joint poses to go through
    (backing out, then rising), not including q_now; then a joint move to `home`."""
    dense, back = strand_so_far(W, table, upto)
    q_now = np.asarray(q_now, float)
    F0 = gen3.fk(q_now, plan.TOOL); R0, p0, z = F0[:3, :3], F0[:3, 3], F0[:3, 2]
    chain = []

    def fail(why):
        return dict(ok=False, reason=why, chain=chain, home=list(home))

    # 1. back out along the nozzle
    q = q_now
    for s in np.arange(STEP_M, back_m + 1e-9, STEP_M):
        T = np.eye(4); T[:3, :3] = R0; T[:3, 3] = p0 - z * s
        q, ok = plan.ik(T, q, 120)
        if not ok:
            return fail("the arm can't back out along the nozzle past %.0f mm" % ((s - STEP_M) * 1000))
        chain.append(q)
    # 2. rise straight up, as far as needed (or as far as the arm goes here)
    p = gen3.fk(q, plan.TOOL)[:3, 3]
    top = max(dense[:, 2].max() if len(dense) else table, p0[2]) + RISE_MARGIN
    for zz in np.arange(p[2] + 0.01, top + 1e-9, 0.01):
        T = np.eye(4); T[:3, :3] = R0; T[:3, 3] = [p[0], p[1], zz]
        qn, ok = plan.ik(T, q, 120)
        if not ok:
            break
        q = qn; chain.append(q)
    rise_to = gen3.fk(q, plan.TOOL)[2, 3]
    # check every pose on the way, and the joint move to Home
    worst, what = 1.0, ""
    prev = q_now
    for qk in chain + [np.asarray(home, float)]:
        for qs in joint_samples(prev, qk):
            c, w = clearance(qs, dense, back, table)
            if c < worst:
                worst, what = c, w
        prev = qk
    res = dict(ok=worst >= 0, chain=[list(map(float, c)) for c in chain], home=list(home), worst_mm=round(worst * 1000, 1),
               what=what, rise_to=round(float(rise_to), 3))
    if worst < 0:
        res["reason"] = "the %s would come within %.0f mm of the print (or the table) on the way out" % (what, -worst * 1000)
    return res
