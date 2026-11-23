"""Classic 3-D strange attractors as paths for the arm, in the same form as a path sent from the homeostat
simulator (format homeostat-path/1), so the layout editor places, plans and draws them like any other.

    python attractors.py lorenz 40 > lorenz.json        # kind, how long to run it (the system's own time)

Each is integrated (RK4) from a start point nudged by `seed`, its settling-in transient dropped, then scaled
so its largest extent is EXTENT_MM at MM_PER_90: in the editor, the scale field (mm per 90 deg) then makes
the attractor about twice that many mm across. They are stood upright (their own z up) and kept that way.
"""
import json, math, sys, time
import numpy as np

EXTENT_MM, MM_PER_90 = 180.0, 90.0

# name: (label, derivative f(v, p), default params, start, dt, transient, default duration, what it looks like)
ATTRACTORS = {
    "lorenz": ("Lorenz", lambda v, p: np.array([p["sigma"] * (v[1] - v[0]), v[0] * (p["rho"] - v[2]) - v[1], v[0] * v[1] - p["beta"] * v[2]]),
               dict(sigma=10.0, rho=28.0, beta=8 / 3), (1.0, 1.0, 1.0), 0.005, 5.0, 20.0,
               "two wings, the trajectory flipping between them"),
    "rossler": ("Rössler", lambda v, p: np.array([-v[1] - v[2], v[0] + p["a"] * v[1], p["b"] + v[2] * (v[0] - p["c"])]),
                dict(a=0.2, b=0.2, c=5.7), (1.0, 1.0, 0.0), 0.01, 50.0, 60.0,
                "a flat spiral that lifts and folds back"),
    "thomas": ("Thomas", lambda v, p: np.array([math.sin(v[1]) - p["b"] * v[0], math.sin(v[2]) - p["b"] * v[1], math.sin(v[0]) - p["b"] * v[2]]),
               dict(b=0.208186), (1.1, 1.1, -0.01), 0.05, 100.0, 150.0,
               "a cyclically symmetric lattice of loops"),
    "aizawa": ("Aizawa", lambda v, p: np.array([(v[2] - p["b"]) * v[0] - p["d"] * v[1], p["d"] * v[0] + (v[2] - p["b"]) * v[1],
                                               p["c"] + p["a"] * v[2] - v[2] ** 3 / 3 - (v[0] ** 2 + v[1] ** 2) * (1 + p["e"] * v[2]) + p["f"] * v[2] * v[0] ** 3]),
               dict(a=0.95, b=0.7, c=0.6, d=3.5, e=0.25, f=0.1), (0.1, 0.0, 0.0), 0.01, 20.0, 25.0,
               "a sphere-like shell with a tube through its axis"),
    "halvorsen": ("Halvorsen", lambda v, p: np.array([-p["a"] * v[0] - 4 * v[1] - 4 * v[2] - v[1] ** 2,
                                                     -p["a"] * v[1] - 4 * v[2] - 4 * v[0] - v[2] ** 2,
                                                     -p["a"] * v[2] - 4 * v[0] - 4 * v[1] - v[0] ** 2]),
                  dict(a=1.89), (-1.48, -1.51, 2.04), 0.005, 10.0, 12.0,
                  "three linked lobes, turned about one axis"),
    "chen": ("Chen", lambda v, p: np.array([p["a"] * (v[1] - v[0]), (p["c"] - p["a"]) * v[0] - v[0] * v[2] + p["c"] * v[1], v[0] * v[1] - p["b"] * v[2]]),
             dict(a=35.0, b=3.0, c=28.0), (-10.0, 0.0, 37.0), 0.002, 5.0, 6.0,
             "a Lorenz cousin with tighter, busier wings"),
}


def integrate(kind, duration=None, params=None, seed=0):
    """Points (n, 3) in the system's own units after the transient, and their times."""
    label, f, p0, x0, dt, transient, d0, _ = ATTRACTORS[kind]
    p = dict(p0, **(params or {}))
    duration = float(duration or d0)
    rng = np.random.default_rng(int(seed))
    v = np.array(x0, float) + (rng.normal(0, 1e-3, 3) if seed else 0)
    n_skip, n = int(transient / dt), int(duration / dt)
    out = np.empty((n, 3))
    for k in range(n_skip + n):
        k1 = f(v, p); k2 = f(v + dt / 2 * k1, p); k3 = f(v + dt / 2 * k2, p); k4 = f(v + dt * k3, p)
        v = v + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        if not np.isfinite(v).all() or np.abs(v).max() > 1e6:
            raise ValueError("%s blew up with these settings" % label)
        if k >= n_skip:
            out[k - n_skip] = v
    return out, np.arange(n) * dt


def export(kind, duration=None, params=None, seed=0):
    """The attractor as a homeostat-path/1 dict: rows of t, x, y, z in page mm (x right, y up, z toward
    you), the attractor's own z pointing up, largest extent EXTENT_MM."""
    V, T = integrate(kind, duration, params, seed)
    V = V - (V.max(0) + V.min(0)) / 2
    V *= EXTENT_MM / (V.max(0) - V.min(0)).max()
    page = np.c_[V[:, 0], V[:, 2], -V[:, 1]]
    # thin to about 0.4 mm apart (the planner resamples every 2 mm at the size it is drawn)
    keep = [0]; last = page[0]
    for i in range(1, len(page)):
        if np.linalg.norm(page[i] - last) >= 0.4:
            keep.append(i); last = page[i]
    keep = np.array(keep)
    label = ATTRACTORS[kind][0]
    return dict(format="homeostat-path/1", kind="attractor", attractor=kind, seed=int(seed),
                preset="%s attractor" % label, params=dict(ATTRACTORS[kind][2], **(params or {})),
                t_end=float(T[keep[-1]]), view={}, mm_per_90=MM_PER_90, upright=True,
                points=np.c_[T[keep], page[keep]].round(4).tolist())


if __name__ == "__main__":
    kind = sys.argv[1] if len(sys.argv) > 1 else "lorenz"
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else None
    t0 = time.time(); d = export(kind, dur)
    P = np.array(d["points"])[:, 1:]
    print("%s: %d points, %.0f mm of path at %g mm per 90 (%.1f s)" % (d["preset"], len(P),
          np.linalg.norm(np.diff(P, axis=0), axis=1).sum(), MM_PER_90, time.time() - t0), file=sys.stderr)
    json.dump(d, sys.stdout)
