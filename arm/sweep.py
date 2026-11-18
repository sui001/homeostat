"""Phase 1 only: how far along the strand is there at least one clear, in-limits direction?"""
import sys, math, numpy as np, plan, wrist
from homeostat_path import path_mm
scale, x, yaw, longn = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), sys.argv[4] == "1"
lift = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
rev = len(sys.argv) > 6 and sys.argv[6] == "1"
if longn: plan.HOTEND[:] = plan.HOTEND_LONG
P, _ = path_mm(0, 30, scale, 2.0)
W = plan.place(P, (x, 0), 0.0, lift, yaw)
if rev: W = W[::-1].copy()
s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(W, axis=0), axis=1))]
S = np.arange(0, s[-1] + 1e-9, 0.001); dense = np.c_[[np.interp(S, s, W[:, k]) for k in range(3)]].T
if lift > 0:   # a post from the table up to the anchor point, 20 mm across, always there
    post = np.array([[W[0][0] + 0.010 * math.cos(a), W[0][1] + 0.010 * math.sin(a), z] for z in np.arange(0, W[0][2], 0.005) for a in np.arange(0, 6.28, 0.785)])
    dense = np.r_[post, dense]; S = np.r_[np.full(len(post), -1.0), S]
D = plan.fib_dirs(900)
for i in range(0, len(W)):
    tip = W[i]; m = S <= s[i] + 1e-9
    tan = W[min(i + 1, len(W) - 1)] - W[max(i - 1, 0)]; tan /= np.linalg.norm(tan) or 1
    C = np.minimum(plan.clearance_all_dirs(D, tip, dense[m], s[i] - S[m], 0.0), plan.arm_clearance_all_dirs(D, tip, dense[m], 0.0))
    j5, reach = wrist.j5_est(tip, D)
    ok = (C >= 0) & (D @ tan <= 0.5) & reach & (j5 <= plan.J5_MAX) & (j5 >= plan.J5_MIN) & (wrist.j3_est(tip, D) <= plan.J3_MAX)
    if not ok.any():
        break
else:
    i = len(W)
print(("INSIDE-OUT " if rev else "outside-in ") + "lift %.2f " % lift + "scale %d x %.2f yaw %3d %s: clear to %.0f%% (%.0f of %.0f mm)" % (scale, x, yaw, "long " if longn else "stock", 100 * s[min(i, len(W) - 1)] / s[-1] if i < len(W) else 100, s[min(i, len(W)-1)] * 1000, s[-1] * 1000))
