"""Cheap estimate of |J5| (wrist bend) for a nozzle direction at a tip position, elbow-up arm.
J5 sits on the tool axis gen3.NOZZLE + 0.1674 m behind the tip (nozzle + 0.0615 + 0.10593); J5 is the angle
between the forearm (elbow -> J5 centre) and the tool axis."""
import numpy as np
import gen3
SH = np.array([0, 0, 0.2848]); L1, L2, BACK = 0.410, 0.3144, gen3.NOZZLE + 0.16743
def j5_est(tip, D):
    Wc = tip[None, :] - BACK * D                       # (d,3) J5 centres
    v = Wc - SH; r = np.hypot(v[:, 0], v[:, 1]); h = v[:, 2]; L = np.hypot(r, h)
    reach = L <= (L1 + L2) * 0.995
    c = np.clip((L1**2 + L**2 - L2**2) / (2 * L1 * L), -1, 1)
    a = np.arctan2(h, r) + np.arccos(c)                 # elbow-up upper-arm elevation
    E = np.c_[np.cos(a) * v[:, 0] / np.maximum(r, 1e-9), np.cos(a) * v[:, 1] / np.maximum(r, 1e-9), np.sin(a)] * L1 + SH
    f = Wc - E; f /= np.linalg.norm(f, axis=1)[:, None]
    return np.degrees(np.arccos(np.clip((f * D).sum(1), -1, 1))), reach

def j3_est(tip, D):
    """|J3| = 180 - elbow interior angle (fits the arm's IK to 0.4 deg)."""
    L = np.linalg.norm(tip[None, :] - BACK * D - SH, axis=1)
    inner = np.degrees(np.arccos(np.clip((L1**2 + L2**2 - L**2) / (2 * L1 * L2), -1, 1)))
    return 180.0 - inner

def arm_points(tip, D):
    """Elbow and J5-centre positions for each direction (elbow up), and reachability."""
    Wc = tip[None, :] - BACK * D
    v = Wc - SH; r = np.hypot(v[:, 0], v[:, 1]); h = v[:, 2]; L = np.hypot(r, h)
    reach = L <= (L1 + L2) * 0.995
    c = np.clip((L1**2 + L**2 - L2**2) / (2 * L1 * L), -1, 1)
    a = np.arctan2(h, r) + np.arccos(c)
    rr = np.maximum(r, 1e-9)
    E = np.c_[np.cos(a) * v[:, 0] / rr, np.cos(a) * v[:, 1] / rr, np.sin(a)] * L1 + SH
    return E, Wc, reach

def seg_dist(P, A, B):
    """Distance from every point P (p,3) to every segment A[d]-B[d]: (p,d)."""
    AB = B - A; L2_ = np.maximum((AB * AB).sum(1), 1e-12)
    PA = P @ AB.T - (A * AB).sum(1)                       # (P-A).AB
    t = np.clip(PA / L2_, 0, 1)
    PP = (P * P).sum(1)[:, None] - 2 * P @ A.T + (A * A).sum(1)   # |P-A|^2
    return np.sqrt(np.maximum(PP - 2 * t * PA + t * t * L2_, 0))
