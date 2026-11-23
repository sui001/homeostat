"""For each strand point: how many approach directions (whole sphere) leave the hot end clear of
strand laid before it (and the table). Answers 'is this print order possible at all' before IK."""
import sys, math, numpy as np
import plan
from homeostat_path import path_mm
def fib(n):
    k = np.arange(n) + 0.5; phi = np.arccos(1 - 2 * k / n); th = math.pi * (1 + 5 ** 0.5) * k
    return np.c_[np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)]
DIRS = fib(400)   # tool z (out of the nozzle) candidates
def run(scale, reverse, preset=0):
    P, _ = path_mm(preset, 30, scale, 2.0)
    W = plan.place(P, (0.45, 0), 0.0, 0.0)
    if reverse: W = W[::-1].copy()
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(W, axis=0), axis=1))]
    S = np.arange(0, s[-1], 0.001); D = np.c_[[np.interp(S, s, W[:, k]) for k in range(3)]].T
    hint = np.array([1.0, 0, 0]); bad = []; nfree = []
    for i in range(0, len(W), 3):
        m = S <= s[i]; st, back = D[m], s[i] - S[m]
        tan = W[min(i+1, len(W)-1)] - W[max(i-1, 0)]; tan /= np.linalg.norm(tan) or 1
        n = 0
        for z in DIRS:
            if z[2] > 0.3 or tan @ z > 0.5: continue   # nozzle pointing more than ~17 deg upward: not with this arm
            R = plan.frame_from_axis(z, hint)
            if min(plan.tool_clearance(R, W[i], st, back, -1.0 if reverse else 0.0)) >= 0: n += 1
        nfree.append(n)
        if n == 0: bad.append(i)
    nfree = np.array(nfree)
    print("scale %3d %s: %3d pts, %3d with NO clear direction (first at %s), median free dirs %d"
          % (scale, "inside-out" if reverse else "outside-in", len(nfree), len(bad), bad[:1], np.median(nfree)))
if __name__ == "__main__":
  for sc in (120, 200, 300):
    for rev in (False, True): run(sc, rev)
