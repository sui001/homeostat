"""PT's essential homeostat model (same maths and RNG order as homeostat.html / headless.py)
and the page's 4-D -> 3-D step, giving the path the arm extrudes, in mm."""
import math, random
import numpy as np

N, POS, LIMIT, STOP, DT = 4, 25, 45.0, 50.0, 0.01
GAIN, DAMP, SAT, I_MAX = 4.0, 4.0, 80.0, 80.0
PRESETS = [
    dict(seed=435017, D=[-0.8, -0.8, -0.5, -0.3], x0=[-5, 10, -17, 49], period=2.0),
    dict(seed=104283, D=[0.1, 0.0, -0.9, -0.5], x0=[-43, -6, 28, -17], period=1.5),
    dict(seed=489235, D=[-0.3, -0.5, 0.4, -0.7], x0=[-47, -47, 31, -35], period=2.5),
    dict(seed=186963, D=[-0.2, -0.9, 0.1, -0.5], x0=[28, 28, 3, 33], period=2.0),
    dict(seed=476128, D=[-0.7, -0.8, 0.0, -0.3], x0=[-49, 46, 22, 24], period=3.0),
    dict(seed=634613, D=[-0.3, -0.9, -0.9, -0.7], x0=[-45, 10, -8, 22], period=3.0),
]

def simulate(preset=0, seconds=30.0):
    p = PRESETS[preset]
    rng = random.Random(p["seed"])
    sched = [[[rng.choice([-1, 1]) * rng.uniform(0.1, 1.0) if j != i else 0.0 for j in range(N)]
              for _ in range(POS)] for i in range(N)]
    D, q, x, v = p["D"], [0] * N, [float(a) for a in p["x0"]], [0.0] * N
    t, nxt = 0.0, p["period"]
    out = lambda th: I_MAX * math.tanh(th / SAT)
    def acc(th, om):
        g = [out(a) for a in th]
        return [GAIN * math.cos(math.radians(th[i])) * (sum(sched[i][q[i]][j] * g[j] for j in range(N)) + D[i] * g[i]) - DAMP * om[i] for i in range(N)]
    hist = []
    for _ in range(int(round(seconds / 0.03))):
        for _ in range(3):
            a1 = acc(x, v)
            x1 = [x[i] + 0.5 * DT * v[i] for i in range(N)]; v1 = [v[i] + 0.5 * DT * a1[i] for i in range(N)]; a2 = acc(x1, v1)
            x2 = [x[i] + 0.5 * DT * v1[i] for i in range(N)]; v2 = [v[i] + 0.5 * DT * a2[i] for i in range(N)]; a3 = acc(x2, v2)
            x3 = [x[i] + DT * v2[i] for i in range(N)]; v3 = [v[i] + DT * a3[i] for i in range(N)]; a4 = acc(x3, v3)
            for i in range(N):
                x[i] += DT / 6 * (v[i] + 2 * v1[i] + 2 * v2[i] + v3[i]); v[i] += DT / 6 * (a1[i] + 2 * a2[i] + 2 * a3[i] + a4[i])
                if abs(x[i]) > STOP: x[i] = max(-STOP, min(STOP, x[i])); v[i] = 0.0
            t += DT
            if t >= nxt - 1e-9:
                nxt = t + p["period"]
                F = [abs(out(a)) > out(LIMIT) for a in x]
                for i in range(N):
                    if F[i]: q[i] = (q[i] + 1) % POS
        hist.append([t] + list(x))
    return np.array(hist)

def to3(p4, xw=0.0, yw=0.4):
    """The page's to3(): two 4-D turns then perspective along w. Degrees in, degrees out."""
    px, py, pz, pw = p4.T
    px, pw = px * math.cos(xw) - pw * math.sin(xw), px * math.sin(xw) + pw * math.cos(xw)
    py, pw = py * math.cos(yw) - pw * math.sin(yw), py * math.sin(yw) + pw * math.cos(yw)
    k = 3.5 * STOP / (3.5 * STOP - pw)
    return np.c_[px * k, py * k, pz * k]

def path_mm(preset=0, seconds=30.0, mm_per_90=120.0, step_mm=2.0, xw=0.0, yw=0.4):
    """Resampled 3-D path in mm, page axes (x right, y up, z toward viewer), plus model time per point.
    Stops where the spiral has settled (moves < 0.25 deg per frame, as the page's movingHist)."""
    h = simulate(preset, seconds)
    d4 = np.r_[np.inf, np.linalg.norm(np.diff(h[:, 1:], axis=0), axis=1)]
    moving = np.nonzero(d4 > 0.25 / 3)[0]          # page thins at 0.25 deg between kept samples
    h = h[: moving[-1] + 1]
    P = to3(h[:, 1:], xw, yw) * (mm_per_90 / (2 * LIMIT))
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    S = np.arange(0, s[-1], step_mm)
    return np.c_[[np.interp(S, s, P[:, k]) for k in range(3)]].T, np.interp(S, s, h[:, 0])

def path_from_export(d, mm_per_90, step_mm=2.0):
    """A path sent from the simulator page ("Send to the arm"): rows of t, x, y, z in mm at the page's
    own mm per 90 deg, page axes, already thinned like path_mm. Rescaled to mm_per_90, resampled every
    step_mm. Returns (P_mm, T) like path_mm."""
    A = np.asarray(d["points"], float)
    P = A[:, 1:4] * (mm_per_90 / float(d["mm_per_90"]))
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    S = np.arange(0, s[-1], step_mm)
    return np.c_[[np.interp(S, s, P[:, k]) for k in range(3)]].T, np.interp(S, s, A[:, 0])


def load_path(path_file, preset, seconds, mm_per_90, step_mm=2.0):
    """The planner's path: from a simulator export when path_file is given, else from the built-in model."""
    if path_file:
        import json
        return path_from_export(json.load(open(path_file)), mm_per_90, step_mm)
    return path_mm(preset, seconds, mm_per_90, step_mm)


if __name__ == "__main__":
    P, T = path_mm()
    print(len(P), "points; t", T[0], "->", T[-1], "; extent mm", P.min(0).round(1), P.max(0).round(1))
    r = np.linalg.norm(P - P[-1], axis=1)
    for f in (0, .25, .5, .75, .9, 1): print("frac", f, "radius from end %.1f mm" % r[int(f * (len(P) - 1))])
