"""Moving the arm along joint waypoint lists, shared by air_run.py and safe_park.py, and the live feed
the editor draws the moving arm from.

Lessons from the arm, 2026-11-19, built in here:
  - at most 1000 waypoints per trajectory (callers split longer ones),
  - the arm fits its own curve through angular waypoints and judges the timing itself; when it refuses,
    slow the whole list down evenly: uneven step times are what make its curve overshoot.
"""
import json, math, os, threading, time
import numpy as np
from kortex_api.autogen.messages import Base_pb2
import waypoint          # ~/kinova/waypoint.py: callers put ~/kinova on sys.path

LIVE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "live.json")
TIMING = (Base_pb2.TRAJECTORY_ERROR_TYPE_INVALID_JOINT_ACCELERATION, Base_pb2.TRAJECTORY_ERROR_TYPE_INVALID_JOINT_SPEED,
          Base_pb2.TRAJECTORY_ERROR_TYPE_INVALID_JOINT_POSITION)


def joint_list(qs, durations, blend=True):
    wl = Base_pb2.WaypointList(); wl.use_optimal_blending = blend
    for k, (q, du) in enumerate(zip(qs, durations)):
        wp = wl.waypoints.add(); wp.name = "j%d" % k
        wp.angular_waypoint.angles.extend([waypoint.wrap(v) for v in q])
        wp.angular_waypoint.duration = float(du)
    return wl


def smooth_times(steps, dt, acc, passes=400):
    """Stretch step times until no joint changes speed faster than acc between steps (from rest, to rest)."""
    d = [float(t) for t in dt]
    z = np.zeros_like(np.asarray(steps[0], float))
    for _ in range(passes):
        v = [np.asarray(st, float) / t for st, t in zip(steps, d)]
        worst = 0.0
        for k in range(len(d) + 1):
            vp = v[k - 1] if k > 0 else z
            vn = v[k] if k < len(d) else z
            span = ((d[k - 1] if k > 0 else 0) + (d[k] if k < len(d) else 0)) / (2 if 0 < k < len(d) else 1)
            a = np.abs(vn - vp).max() / span
            if a > acc:
                f = min(1.5, math.sqrt(a / acc) * 1.01)
                if k > 0: d[k - 1] *= f
                if k < len(d): d[k] *= f
            worst = max(worst, a)
        if worst <= acc * 1.001:
            break
    return d


def step_times(start, qs, jv=10.0, min_dt=0.3, acc=30.0):
    """Times for a joint list from `start` through qs: each step at most jv deg/s, then acceleration-limited."""
    prev, steps, dt = np.asarray(start, float), [], []
    for q in qs:
        d = ((np.asarray(q, float) - prev + 180) % 360) - 180
        steps.append(d); dt.append(max(np.abs(d).max() / jv, min_dt)); prev = np.asarray(q, float)
    return smooth_times(steps, dt, acc)


def fit(base, qs, t0, first=1):
    """Ask the arm about a joint list whose first waypoint is reached from where the arm stands. Where it
    finds a joint too fast, accelerating too hard, or its curve overshooting a joint limit, slow the whole
    list down evenly and ask again."""
    t = list(t0)
    for rounds in range(1, 31):
        rep = base.ValidateWaypointList(joint_list(qs, t)).trajectory_error_report.trajectory_error_elements
        if not rep or any(e.error_type not in TIMING for e in rep):
            break
        f = 1.0
        for e in rep:
            if e.error_type == TIMING[2]:
                f = max(f, 1.15)
            else:
                over = e.error_value / (e.max_value or 1)
                f = max(f, (math.sqrt(over) if e.error_type == TIMING[0] else over) * 1.03)
        t = t[:first] + [min(x * f, 20.0) for x in t[first:]]
    return t, rounds


def run(base, name, wl, timeout):
    """Execute a waypoint list and wait for it. Raises SystemExit on timeout (after Stop) or abort."""
    done, res = threading.Event(), {}

    def on_event(n):
        ev = Base_pb2.ActionEvent.Name(n.action_event)
        if ev in ("ACTION_END", "ACTION_ABORT"):
            res["ev"] = ev; res["n"] = n; done.set()
    h = base.OnNotificationActionTopic(on_event, Base_pb2.NotificationOptions())
    t0 = time.time()
    base.ExecuteWaypointTrajectory(wl)
    ok = done.wait(timeout); base.Unsubscribe(h)
    if not ok:
        base.Stop(); raise SystemExit("TIMED OUT on %s after %.0f s. Sent Stop()." % (name, timeout))
    print("  %-34s %s in %.1f s" % (name, res["ev"], time.time() - t0), flush=True)
    if res["ev"] == "ACTION_ABORT":
        raise SystemExit("ABORTED on %s: %s" % (name, res["n"]))


def run_exit(base, cyc, ex, label="safe exit"):
    """Run a plan_exit() result: through its chain (backing out, rising), then a joint move to Home."""
    if ex["chain"]:
        cur = [a.position for a in cyc.RefreshFeedback().actuators]
        t, rounds = fit(base, ex["chain"], step_times(cur, ex["chain"]), first=0)
        wl = joint_list(ex["chain"], t)
        p = waypoint.problems(base, wl)
        if p:
            raise SystemExit("REFUSED the %s: %s. The arm has not moved." % (label, "; ".join(p[:2])))
        run(base, "%s: back out and rise" % label, wl, 60 + 2 * sum(t))
    cur = [a.position for a in cyc.RefreshFeedback().actuators]
    run(base, "%s: to Home" % label, waypoint.build(cur, ex["home"], 20.0, "home"), 90)


class WristLight:
    """A light on one of the wrist's four digital pins (GPIO 1-4 on the bracelet's expansion connector):
    3.3 V push-pull, so switch the light through a small logic-level MOSFET or transistor. pin 0 = no
    light wired: on() and off() do nothing."""
    def __init__(self, router, pin):
        self.pin, self.ic, self.dev = int(pin), None, None
        if not self.pin:
            return
        from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
        from kortex_api.autogen.client_stubs.InterconnectConfigClientRpc import InterconnectConfigClient
        from kortex_api.autogen.messages import InterconnectConfig_pb2 as IC, Common_pb2
        self.IC = IC
        self.dev = [h.device_identifier for h in DeviceManagerClient(router).ReadAllDevices().device_handle
                    if h.device_type == Common_pb2.INTERCONNECT][0]
        self.ic = InterconnectConfigClient(router)
        c = IC.GPIOConfiguration(); c.identifier = self.pin
        c.mode = IC.GPIO_MODE_OUTPUT_PUSH_PULL; c.pull = IC.GPIO_PULL_NONE; c.default_value = IC.GPIO_VALUE_LOW
        self.ic.SetGPIOConfiguration(c, self.dev)
        self.off()

    def _set(self, high):
        if not self.pin:
            return
        st = self.IC.GPIOState(); st.identifier = self.pin
        st.value = self.IC.GPIO_VALUE_HIGH if high else self.IC.GPIO_VALUE_LOW
        self.ic.SetGPIOState(st, self.dev)

    def on(self):
        self._set(True)

    def off(self):
        try:
            self._set(False)
        except Exception as e:
            print("  could not switch the light off: %s" % str(e)[:80])


class Live:
    """Samples the arm 20 times a second: keeps a log of tool positions (for tracking) and writes
    live.json (joints, tool, what it is doing) for the editor's live view."""
    def __init__(self, cyc, what="", **extra):
        self.cyc, self.what, self.extra, self.log = cyc, what, extra, []
        self._stop = threading.Event(); self._th = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._th.start(); return self

    def _write(self, d):
        tmp = LIVE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(d, f)
        os.replace(tmp, LIVE)

    def _loop(self):
        k = 0
        while not self._stop.is_set():
            try:
                fb = self.cyc.RefreshFeedback()
            except Exception:
                return
            now = time.time(); b = fb.base
            self.log.append((now, b.tool_pose_x, b.tool_pose_y, b.tool_pose_z))
            if k % 2 == 0:
                try:
                    self._write(dict(self.extra, active=True, t=now, what=self.what,
                                     joints=[a.position for a in fb.actuators], tool=[b.tool_pose_x, b.tool_pose_y, b.tool_pose_z]))
                except OSError:
                    pass
            k += 1
            time.sleep(0.05)

    def stop(self):
        self._stop.set(); self._th.join(timeout=2)
        try:
            d = json.load(open(LIVE)); d.update(active=False, t=time.time()); self._write(d)
        except Exception:
            pass
