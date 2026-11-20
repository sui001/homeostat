#!/usr/bin/env python3
"""Air-run a homeostat print plan on the arm: the planned path, raised clear of the desk, no extrusion.

Without KINOVA_CONFIRM=yes it validates every move with the arm and moves nothing.

    ~/kinova-py310/bin/python air_run.py spiral_std200_checked.json            # check only
    KINOVA_CONFIRM=yes ~/kinova-py310/bin/python air_run.py spiral_std200_checked.json

Env:
  AIR_RAISE_MM   how far above the planned path to run it (default 20; more pushes J5 toward its limit)
  AIR_SPEED      mm/s along the strand (default 10, the planned print speed)
  AIR_BLEND_MM   blending radius at each waypoint so it flows instead of stopping every 2 mm (default 0.8)

  AIR_LIGHT_PIN  light drawings: the wrist pin (1-4) the light is switched on, 0 = none (default 0)
  AIR_LIGHT_HOLD_S  light drawings: the pause at the start and at the end, for the camera's shutter (default 5)

A light drawing (a plan made with --light) runs dark up to the start, holds there while you open the
shutter, switches the light on along the path only, switches it off and holds again while you close the
shutter, then leaves the usual safe way. SHUTTER lines in the output mark those moments.

Follows the ~/kinova conventions: takes the arm lock, refuses unless the arm is idle at Home,
and Ctrl-C (or any error) sends Stop(). Sequence: Home -> above the start -> along the path ->
the safe exit (back out along the nozzle, rise above the print, then Home; exit_path.py).
"""
import os, sys, json, math, time, threading
import numpy as np
sys.path.insert(0, os.path.expanduser("~/kinova"))
import kenv; kenv.load()
import armlock, waypoint, gen3, exit_path, armmove, plan
from armmove import joint_list, smooth_times
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.ControlConfigClientRpc import ControlConfigClient
from kortex_api.autogen.messages import Base_pb2, Session_pb2

PLAN = sys.argv[1] if len(sys.argv) > 1 else "spiral_std200_checked.json"
RAISE = float(os.environ.get("AIR_RAISE_MM", 20)) / 1000
SPEED = float(os.environ.get("AIR_SPEED", 10)) / 1000
FAST = float(os.environ.get("AIR_FAST", 1))        # run the whole thing this many times faster than printing (testing)
BLEND = float(os.environ.get("AIR_BLEND_MM", 0.8)) / 1000
ARMED = os.environ.get("KINOVA_CONFIRM") == "yes"
CHUNK = 40
HOVER = float(os.environ.get("AIR_HOVER_MM", 15)) / 1000   # extra height for the approach and the retreat
HOME_TOL = 2.0          # deg

d = json.load(open(PLAN))
LIGHT = bool(d["params"].get("light"))
if LIGHT:
    RAISE = 0.0          # a light drawing runs where it was planned: there's nothing to keep the claw off
LIGHT_PIN = int(os.environ.get("AIR_LIGHT_PIN", 0)) if LIGHT else 0
HOLD = float(os.environ.get("AIR_LIGHT_HOLD_S", 5)) if LIGHT else 0.0
if not 0 <= LIGHT_PIN <= 4:
    sys.exit("AIR_LIGHT_PIN must be 0 (no light wired) or a wrist pin, 1 to 4.")
wps = [w for w in d["waypoints"] if w["ok"]]
if FAST > 1:   # faster runs: fewer, longer steps (every FAST-th point) rather than the same steps squeezed
    step = int(round(FAST))
    wps = [wps[i] for i in sorted(set(range(0, len(wps), step)) | {len(wps) - 1})]
if not all(w["ok"] for w in d["waypoints"]):
    print("NOTE: %d planned points are unreachable and are skipped" % sum(not w["ok"] for w in d["waypoints"]))
tips = np.array([w["tip"] for w in wps]) + [0, 0, RAISE]
eul = [w["euler"] for w in wps]
NOZ = d["params"].get("nozzle", gen3.ARM_TCP)        # plans from before the nozzle was measured: tcp == tip
tcp_of = lambda pt, e: gen3.arm_tcp(pt, e, NOZ)
print("plan %s: %d points, %.0f mm, raised %.0f mm -> tip z %.0f..%.0f mm"
      % (PLAN, len(wps), 1000 * np.linalg.norm(np.diff(tips, axis=0), axis=1).sum(), RAISE * 1000,
         tips[:, 2].min() * 1000, tips[:, 2].max() * 1000))


def cart_list(points, eulers, speed, blend, ang=15.0):
    wl = Base_pb2.WaypointList(); wl.use_optimal_blending = False
    for k, (p, e) in enumerate(zip(points, eulers)):
        wp = wl.waypoints.add(); wp.name = "w%d" % k
        cw = wp.cartesian_waypoint
        cw.pose.x, cw.pose.y, cw.pose.z = [float(v) for v in p]
        cw.pose.theta_x, cw.pose.theta_y, cw.pose.theta_z = [float(v) for v in e]
        cw.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
        cw.maximum_linear_velocity = speed; cw.maximum_angular_velocity = ang
        cw.blending_radius = blend if k < len(points) - 1 else 0.0
    return wl


above = tips[0] + [0, 0, HOVER]
LIMITS = {1: 128.9, 2: 147.8, 4: 120.3}   # joint index -> |limit| deg, from the arm
MARGIN = 3.0
MAX_STEP = 6.0                            # deg J1-J5 may move between neighbouring points
JV = float(os.environ.get("AIR_JV", 5.0)) * FAST   # deg/s cap per joint, keeps joint acceleration under the arm's 51 deg/s^2
MIN_DT = float(os.environ.get("AIR_MIN_DT", 0.3))  # s per waypoint at least
ACC = float(os.environ.get("AIR_ACC", 30.0))      # deg/s^2 planned to; the arm refuses over 51.6 (and measures it its own way)


SPEED *= FAST
FINE = min(5.0, 2.0 * FAST)                                # deg per waypoint after splitting
MAX_ROLL = 30.0                           # J6 only turns the tool about its own axis


t = TCPTransport(); r = RouterClient(t, RouterClient.basicErrorCallback); t.connect("192.168.1.10", 10000)
info = Session_pb2.CreateSessionInfo(); info.username = os.environ["KINOVA_USER"]; info.password = os.environ["KINOVA_PASS"]
info.session_inactivity_timeout = 60000; info.connection_inactivity_timeout = 2000
sm = SessionManager(r); sm.CreateSession(info)
base = BaseClient(r); cyc = BaseCyclicClient(r)
lock = live = light = None
try:
    tt = ControlConfigClient(r).GetToolConfiguration().tool_transform
    if abs(tt.z - gen3.ARM_TCP) > 1e-3 or abs(tt.x) + abs(tt.y) > 1e-3:
        sys.exit("Tool transform on the arm is %s, the plan assumes z = %.3f m. Stopping." % ([tt.x, tt.y, tt.z], gen3.ARM_TCP))
    home = None
    req = Base_pb2.RequestedActionType(); req.action_type = Base_pb2.REACH_JOINT_ANGLES
    for a in base.ReadAllActions(req).action_list:
        if a.name.strip().lower() == "home":
            home = [j.value for j in base.ReadAction(a.handle).reach_joint_angles.joint_angles.joint_angles]
    cur = [a.position for a in cyc.RefreshFeedback().actuators]
    off = max(abs(x) for x in waypoint.travel(cur, home))
    print("arm at %s, %.1f deg from Home" % ([round(a, 1) for a in cur], off))
    if off > HOME_TOL:
        sys.exit("Refusing: not at Home. Park it first, with Park in the editor, or safe_park.py.")

    # The arm's own IK for every raised point, each seeded with the one before, so the joint path is
    # continuous and is exactly what gets run. (Cartesian waypoints let the arm pick its own branch
    # mid-path once the path is raised: it ran J5 into its limit.)
    def ik(pt, e, seed):
        ikd = Base_pb2.IKData(); p = ikd.cartesian_pose
        p.x, p.y, p.z = [float(v) for v in tcp_of(pt, e)]; p.theta_x, p.theta_y, p.theta_z = [float(v) for v in e]
        for q in seed: ikd.guess.joint_angles.add().value = q
        try:
            return np.array([waypoint.wrap(j.value) for j in base.ComputeInverseKinematics(ikd).joint_angles])
        except Exception as ex:
            # the arm's IK gives up on some poses it can be in (tool above horizontal, high up): our model's
            # IK from the same seed, used only if the arm's own FK puts those joints on the pose
            R = gen3.from_euler(*e); Tt = np.eye(4); Tt[:3, :3] = R; Tt[:3, 3] = pt
            qm, conv = plan.ik(Tt, np.asarray(seed, float), 200)
            if conv:
                qm = np.array([waypoint.wrap(v) for v in qm])
                ok_fk, mm, deg = gen3.arm_fk_check(base, qm, tcp_of(pt, e), e)
                if ok_fk:
                    model_ik[0] += 1
                    return qm
            sys.exit("The arm has no IK solution at tip %s (raised %.0f mm): %s. Not moving; try a smaller AIR_RAISE_MM."
                     % (np.round(pt, 4).tolist(), RAISE * 1000, str(ex).split("=>")[-1].strip()[:90]))
    def rot_lerp(R0, R1, f):
        Rr = R0.T @ R1; ang = math.acos(max(-1.0, min(1.0, (np.trace(Rr) - 1) / 2)))
        if ang < 1e-9: return R0
        ax = np.array([Rr[2, 1] - Rr[1, 2], Rr[0, 2] - Rr[2, 0], Rr[1, 0] - Rr[0, 1]]) / (2 * math.sin(ang))
        K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]]); a = f * ang
        return R0 @ (np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K)

    def dstep(a_, b_):
        return np.abs(((b_ - a_ + 180) % 360) - 180)

    model_ik = [0]
    # every pose along the way: above the start, then the path (the way out is the safe exit, below)
    P = [above] + list(tips); E = [eul[0]] + eul
    q = ik(P[0], E[0], wps[0]["joints"]); chain = [q]; seglen = []
    for k in range(1, len(P)):
        qk = ik(P[k], E[k], q); n = int(math.ceil(dstep(q, qk)[:5].max() / FINE))
        if n > 1:   # split the segment so no joint swings more than FINE deg between waypoints
            R0, R1 = gen3.from_euler(*E[k - 1]), gen3.from_euler(*E[k])
            # the IK at a split point can land a little off the straight line in joint space, so check the
            # steps that came out and split finer until they really are under FINE (or give up at 64)
            while True:
                sub, prev = [], chain[-1]
                for j in range(1, n):
                    fr = j / n; qj = ik(P[k - 1] + fr * (P[k] - P[k - 1]), gen3.euler_xyz(rot_lerp(R0, R1, fr)), prev)
                    sub.append(qj); prev = qj
                qk = ik(P[k], E[k], prev)
                pts = [chain[-1]] + sub + [qk]
                if max(dstep(a_, b_)[:5].max() for a_, b_ in zip(pts[:-1], pts[1:])) <= FINE * 1.1 or n >= 64:
                    break
                n *= 2
            chain.extend(sub); seglen.extend([np.linalg.norm(P[k] - P[k - 1]) / n] * n)
        else:
            seglen.append(np.linalg.norm(P[k] - P[k - 1]))
        chain.append(qk); q = qk
        if k == 1:
            k0 = len(chain) - 1          # chain[k0] is the first point of the path; before it, the way down onto it
    q_above = chain[0]
    steps = [dstep(a_, b_) for a_, b_ in zip(chain[:-1], chain[1:])]
    worst = max(st[:5].max() for st in steps); roll = max(st[5] for st in steps)
    near = min(lim - abs(q[j]) for q in chain for j, lim in LIMITS.items())
    j5min = min(abs(q[4]) for q in chain)
    if model_ik[0]:
        print("  %d poses from our model's IK where the arm's gave up, each confirmed by the arm's own FK" % model_ik[0])
    print("IK chain: %d poses, biggest step J1-J5 %.1f deg, J6 %.1f deg, closest to a joint limit %.1f deg, |J5| >= %.1f"
          % (len(chain), worst, roll, near, j5min))
    if worst > MAX_STEP or roll > MAX_ROLL or near < MARGIN:
        sys.exit("Joint path not safe to run (step > %.0f deg or within %.0f deg of a limit). Not moving." % (MAX_STEP, MARGIN))
    dur = [max(L / SPEED, st.max() / JV, MIN_DT) for L, st in zip(seglen, steps)]
    approach = waypoint.duration(cur, q_above, 20.0)

    def fit(qs, t0, first=1):
        return armmove.fit(base, qs, t0, first)

    # Steps timed against the joint acceleration limit (51.6 deg/s^2), stretched only where a joint changes
    # speed or direction sharply, from rest and to rest.
    path_signed = [((b_ - a_ + 180) % 360) - 180 for a_, b_ in zip(chain[:-1], chain[1:])]
    MAX_WP = int(os.environ.get("AIR_MAX_WP", 900))   # the arm refuses a trajectory of more than 1000 waypoints
    if FAST == 1 and len(chain) + 1 <= MAX_WP and not LIGHT:
        # one list from Home: joint move to above the start, a short hold, down onto the path, along it, up
        qs = [chain[0]] + chain
        times = smooth_times([np.array(waypoint.travel(cur, chain[0])), np.zeros(6)] + path_signed, [approach, 0.6] + dur, ACC)
        times, rounds = fit(qs, times, first=1)          # never stretch the move out of Home (0)
        print("timing fitted to the arm's limits in %d round(s)" % rounds)
        moves = [("Home -> start -> path -> up (%d joint waypoints)" % len(qs), joint_list(qs, times))]
        est = sum(times)
    else:
        # Faster than print speed, or more waypoints than the arm takes in one list: the move out of Home on
        # its own (it ends at rest), then the path in parts of up to MAX_WP waypoints, each timed and checked
        # by the arm from where it then stands, just before it runs. (As one list, the arm's curve through the
        # join with the move out of Home overshoots the wrist past its limit at anything faster.)
        path_times = smooth_times(path_signed, dur, ACC)
        n = len(chain) - 1
        cuts = list(range(0, n, MAX_WP)) + [n]
        if LIGHT:   # the way down onto the start on its own, dark; then the path, lit, in as few parts as it takes
            cuts = [0, k0] + list(range(k0 + MAX_WP, n, MAX_WP)) + [n]
        moves = [("Home -> above the start (joint move)", waypoint.build(cur, q_above, 20.0, "approach"))]
        for c, (i0, i1) in enumerate(zip(cuts[:-1], cuts[1:])):
            label = "path part %d of %d" % (c + 1, len(cuts) - 1)
            if LIGHT:
                label = "down onto the start" if c == 0 else "drawing, part %d of %d" % (c, len(cuts) - 2)
            moves.append(("%s%s (%d joint waypoints)" % (label, "" if FAST == 1 else " at %gx" % FAST, i1 - i0), (i0, i1)))
        est = approach + sum(path_times) + 2 * HOLD
        if LIGHT:
            print("light drawing: the exposure (light on) lasts about %.0f s; light %s"
                  % (sum(path_times[k0:]), "on wrist pin %d" % LIGHT_PIN if LIGHT_PIN else "not switched (no pin set): it stays on"))
    # validate before any motion
    bad = 0
    for name, wl in moves:
        if isinstance(wl, tuple):
            print("         %-34s timed and checked by the arm when it gets there" % name)
            continue
        p = armmove.problems(base, wl)
        if p:
            bad += 1; print("REFUSED  %-34s %s" % (name, "; ".join(p[:3])))
    print("validated %d moves with the arm: %d refused" % (len(moves), bad))
    if bad:
        sys.exit("Not moving.")
    if not ARMED:
        print("Check only (about %.0f s from Home to the end of the path%s). Set KINOVA_CONFIRM=yes to run it."
              % (est, "" if FAST == 1 else ", asked for %gx print speed" % FAST))
        sys.exit(0)

    lock = armlock.take("homeostat air-run")
    light = armmove.WristLight(r, LIGHT_PIN) if LIGHT else None    # set up now, and off
    print("ARMED: running in about 3 s. Ctrl-C stops the arm.")
    time.sleep(3)

    # log where the tool really goes, and feed the editor's live view
    live = armmove.Live(cyc, "air-run", plan=PLAN, raise_mm=RAISE * 1000, fast=FAST).start()
    log = live.log

    def run(name, wl, timeout):
        live.what = name
        armmove.run(base, name, wl, timeout)

    def shutter(word, then=None):
        """A pause for the camera: the editor beeps and counts down on the SHUTTER line."""
        print("SHUTTER %s: %.0f s" % (word, HOLD), flush=True)
        live.what = "%s the shutter" % word.lower()
        time.sleep(HOLD)
        if then: then()

    t_start = time.time()
    for name, wl in moves:
        if isinstance(wl, tuple):   # a part of the path, from where the arm now stands (at rest)
            i0, i1 = wl
            lit_part = LIGHT and i0 >= k0
            if lit_part and i0 > k0:  # between parts of a drawing: the light goes off while the arm checks the next
                light.off(); print("LIGHT OFF while the arm checks the next part%s" % ("" if LIGHT_PIN else " (no pin: it stays on, a brighter dot here)"), flush=True)
            live.what = "the arm is checking %s" % name
            t, rounds = fit(chain[1 + i0:1 + i1], path_times[i0:i1], first=0)   # from rest: its first step may stretch too
            wl = joint_list(chain[1 + i0:1 + i1], t)
            p = armmove.problems(base, wl)
            if p:
                fb = cyc.RefreshFeedback()
                json.dump(dict(name=name, here=[a.position for a in fb.actuators], joints=[list(map(float, q)) for q in chain[1 + i0:1 + i1]],
                               times=list(map(float, t)), problems=p), open("air_run_refused.json", "w"))
                raise SystemExit("REFUSED %s: %s. The arm has stopped where that part starts: park it at Home." % (name, "; ".join(p[:2])))
            print("  %s: timing fitted in %d round(s), %.0f s" % (name, rounds, sum(t)), flush=True)
            if lit_part:
                if i0 == k0:          # checked and ready at the start: now the camera
                    shutter("OPEN")
                light.on(); print("LIGHT ON", flush=True)
            run(name, wl, 60 + 2 * sum(t))
            continue
        run(name, wl, 60 + 2 * est)
    if LIGHT:
        light.off(); print("LIGHT OFF", flush=True)
        shutter("CLOSE")
    # the way out: back out along the nozzle, rise above the (here imaginary, raised) print, then Home,
    # every pose checked against it first (exit_path.py)
    cur = [a.position for a in cyc.RefreshFeedback().actuators]
    ex = exit_path.plan_exit(cur, np.array(d["strand"]) + [0, 0, RAISE], d["params"]["table"], None, home, light=LIGHT)
    if not ex["ok"]:
        raise SystemExit("Finished the path, but there is no safe way out: %s. The arm is waiting at the end of the path: "
                         "move it clear by hand." % ex["reason"])
    print("  way out: back out %.0f mm along the nozzle, up to z %.2f m, then Home; closest %.1f mm (%s)"
          % (exit_path.BACK_M * 1000, ex["rise_to"], ex["worst_mm"], ex["what"]), flush=True)
    live.what = "safe exit"
    armmove.run_exit(base, cyc, ex)
    live.stop(); live = None
    print("done in %.0f s" % (time.time() - t_start))

    # how closely the tool followed the raised path, over the path part only
    L = np.array(log)[:, 1:]
    tc = np.array([tcp_of(p, e) for p, e in zip(tips, eul)])   # the arm reports its own 0.200 m TCP
    dense = np.concatenate([np.linspace(a, b, 20, endpoint=False) for a, b in zip(tc[:-1], tc[1:])] + [tc[-1:]])
    near = np.array([np.linalg.norm(dense - p, axis=1).min() for p in L])
    on = near < 0.02     # samples taken while on the path, not during approach / return
    print("tracking: %d samples on the path, deviation median %.2f mm, max %.2f mm"
          % (on.sum(), np.median(near[on]) * 1000, near[on].max() * 1000))
    json.dump(dict(plan=PLAN, raise_mm=RAISE * 1000, speed=SPEED * 1000, log=log), open("air_run_log.json", "w"))
except KeyboardInterrupt:
    base.Stop(); print("\nCtrl-C: sent Stop().")
except SystemExit:
    raise
except Exception:
    try: base.Stop()
    except Exception: pass
    raise
finally:
    if light: light.off()
    if live: live.stop()
    if lock: lock.release()
    sm.CloseSession(); r.SetActivationStatus(False)
