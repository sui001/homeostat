"""Streaming a joint path to the arm as speed commands: one continuous motion, no waypoint lists.

Waypoint lists (armmove.py) must start and end at rest, and the arm gets slow to accept long ones (~20 s
for 850 waypoints, 2026-11-20), so a long drawing became 24 parts with a stop at every join. Here the Pi
works out where every joint should be at every moment (a timed joint path: even tip speed, joint speed and
acceleration held under limits) and sends joint speeds about 40 times a second: the planned speed plus a
small correction toward where the joint should be.

Safety, since a speed command has no time limit of its own:
  - any error, Ctrl-C (the editor's Stop sends one) or the end of the path sends zero speeds and Stop();
  - if any joint gets more than ERR_MAX deg from where it should be (something stopped or blocked it, or
    the arm can't keep up), it stops at once;
  - if a tick comes more than LATE_MAX s late, it stops;
  - the session's connection timeout is short (the caller sets it), so if this process dies the arm's
    controller drops the session.
"""
import math, time
import numpy as np
from kortex_api.autogen.messages import Base_pb2

RATE = 40.0           # commands per second
KP = 2.0              # 1/s: correction toward the planned position
ERR_MAX = 4.0         # deg any joint may lag or lead before stopping
LATE_MAX = 0.25       # s a tick may be late before stopping
VCMD_MAX = 40.0       # deg/s most any joint is ever commanded


def unwrap(Q):
    """Joint angles as a continuous path (no jumps at +-180)."""
    Q = np.asarray(Q, float).copy()
    for k in range(1, len(Q)):
        Q[k] = Q[k - 1] + ((Q[k] - Q[k - 1] + 180) % 360 - 180)
    return Q


def timing(Q, L, speed, vmax, amax):
    """Times at each pose: tip at `speed` m/s (L: tip travel of each step, m), no joint over vmax deg/s,
    joints speeding up and slowing down at about amax deg/s^2, from rest and to rest. A feed-rate planner:
    each step's speed cap, then a forward pass (how fast it can be after speeding up from the start) and a
    backward pass (how fast it can be and still slow down for the end). Returns cumulative times (len n)."""
    dQ = np.diff(np.asarray(Q, float), axis=0); L = np.asarray(L, float)
    ds = np.abs(dQ).max(1)                                   # deg the busiest joint moves in each step
    cap = np.minimum(vmax, np.where(L > 1e-9, ds * speed / np.maximum(L, 1e-9), vmax))
    u = cap.copy()
    prev = 0.0
    for k in range(len(u)):
        u[k] = min(u[k], math.sqrt(prev ** 2 + 2 * amax * ds[k])); prev = u[k]
    nxt = 0.0
    for k in range(len(u) - 1, -1, -1):
        u[k] = min(u[k], math.sqrt(nxt ** 2 + 2 * amax * ds[k])); nxt = u[k]
    dt = np.where(ds > 1e-9, ds / np.maximum(u, 1e-6), np.maximum(L / speed, 1e-3))
    return np.r_[0.0, np.cumsum(np.maximum(dt, 1e-3))]


class Path:
    """Joint positions and speeds at any time. The timed poses are sampled every DT s and smoothed with a
    Gaussian of SIGMA s (with a moment of rest padded at each end, so it starts and ends exactly on the
    poses, at rest): the step-to-step jolts of the timing, worst starting from rest, are spread out."""
    DT, SIGMA = 0.01, 0.2

    def __init__(self, Q, t):
        Q, t = np.asarray(Q, float), np.asarray(t, float)
        pad = 4 * self.SIGMA
        tg = np.arange(-pad, t[-1] + pad + self.DT / 2, self.DT)
        G = np.c_[[np.interp(tg, t, Q[:, j]) for j in range(Q.shape[1])]].T     # flat before 0 and after the end
        x = np.arange(-int(3 * self.SIGMA / self.DT), int(3 * self.SIGMA / self.DT) + 1) * self.DT
        k = np.exp(-0.5 * (x / self.SIGMA) ** 2); k /= k.sum(); h = len(k) // 2
        Gp = np.r_[np.repeat(G[:1], h, 0), G, np.repeat(G[-1:], h, 0)]
        self.G = np.c_[[np.convolve(Gp[:, j], k, "valid") for j in range(G.shape[1])]].T
        self.Vg = np.gradient(self.G, self.DT, axis=0)
        self.Q, self.T = Q, float(tg[-1] - tg[0])

    def at(self, tt):
        u = min(max(tt, 0.0), self.T) / self.DT
        k = int(min(u, len(self.G) - 2)); f = u - k
        return self.G[k] * (1 - f) + self.G[k + 1] * f, self.Vg[k] * (1 - f) + self.Vg[k + 1] * f

    def stats(self, dt=0.01):
        """Largest joint speed and acceleration along the smooth path (what the arm will be asked for)."""
        ts = np.arange(0, self.T, dt)
        V = np.array([self.at(x)[1] for x in ts])
        A = np.diff(V, axis=0) / dt
        return float(np.abs(V).max()), float(np.abs(A).max()) if len(A) else 0.0


A_LIMIT = 40.0        # deg/s^2 the smoothed path may ask of any joint (the arm's own limit is ~51.6)


def plan_stream(Q, L, speed, vmax=30.0, amax=25.0, a_limit=A_LIMIT):
    """A Path for joint poses Q (L: tip travel per step, m) at up to `speed` m/s; slowed evenly until the
    smoothed path stays under a_limit (tight curves ask more of the joints than speeding up does)."""
    Q = unwrap(Q)
    for _ in range(8):
        p = Path(Q, timing(Q, L, speed, vmax, amax))
        v, a = p.stats()
        if a <= a_limit:
            return p, speed
        speed /= math.sqrt(a / a_limit) * 1.02
    return p, speed


_FORGET = None


def send(base, v):
    """Joint speeds, sent without waiting for the arm's reply: waiting takes 25 ms a command, up to 230 ms
    (2026-11-20), longer than a tick; sent and forgotten, ~5 ms."""
    global _FORGET
    if _FORGET is None:
        from kortex_api.RouterClient import RouterClientSendOptions
        _FORGET = RouterClientSendOptions(); _FORGET.andForget = True
    cmd = Base_pb2.JointSpeeds()
    for i, x in enumerate(v):
        js = cmd.joint_speeds.add(); js.joint_identifier = i; js.value = float(x)
    base.SendJointSpeedsCommand(cmd, options=_FORGET)


def halt(base):
    try:
        send(base, np.zeros(6))
    except Exception:
        pass
    try:
        base.Stop()
    except Exception:
        pass


def run(base, cyc, path, on_tick=None):
    """Drive the arm along `path` (it must be standing at path.Q[0]). Raises SystemExit after Stop() if
    anything is off. on_tick(t, T, feedback) is called each tick."""
    def measured(ref):
        fb = cyc.RefreshFeedback()
        q = np.array([a.position for a in fb.actuators])
        return ref + ((q - ref + 180) % 360 - 180), fb  # the measured angles, unwrapped next to the reference
    q0, _ = measured(path.Q[0])
    off = np.abs(q0 - path.Q[0]).max()
    if off > 1.0:
        raise SystemExit("Not streaming: the arm is %.1f deg from where the path starts." % off)
    period = 1.0 / RATE
    send(base, np.zeros(6))                   # the first speed command switches the arm's mode: not on the clock
    time.sleep(0.2)
    t_start = time.monotonic(); nxt = t_start; worst = 0.0; last_send = None
    took = {"feedback": [], "send": [], "tick": []}
    def report():
        return "; ".join("%s median %.0f ms, max %.0f" % (k, 1e3 * np.median(v), 1e3 * max(v)) for k, v in took.items() if v)
    try:
        while True:
            t0 = time.monotonic()
            if last_send is not None and t0 - last_send > LATE_MAX:
                raise SystemExit("Stopped: %.2f s between two commands, at %.1f s of %.0f (%s)." % (t0 - last_send, t0 - t_start, path.T, report()))
            tt = t0 - t_start
            ref, vref = path.at(tt)
            q, fb = measured(ref)
            t1 = time.monotonic(); took["feedback"].append(t1 - t0)
            err = ref - q
            worst = max(worst, float(np.abs(err).max()))
            if np.abs(err).max() > ERR_MAX:
                j = int(np.argmax(np.abs(err)))
                raise SystemExit("Stopped: joint %d was %.1f deg from where it should be (stopped, blocked, or too fast)."
                                 % (j + 1, err[j]))
            if tt >= path.T and np.abs(err).max() < 0.3:
                break
            if tt >= path.T + 3.0:
                break                                    # settled as well as it will
            v = np.clip(vref + KP * err, -VCMD_MAX, VCMD_MAX)
            send(base, v); last_send = time.monotonic(); took["send"].append(last_send - t1)
            if on_tick:
                on_tick(tt, path.T, fb)
            took["tick"].append(time.monotonic() - t0)
            nxt = max(nxt + period, time.monotonic())   # never pile up lateness
            time.sleep(max(0.0, nxt - time.monotonic()))
    finally:
        halt(base)
    print("  stream timing: %s" % report(), flush=True)
    return worst
