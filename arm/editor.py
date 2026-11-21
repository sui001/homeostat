#!/usr/bin/env python3
"""Layout editor for the homeostat print: place, turn, tilt and scale the path inside the arm's reach,
see what is printable, then plan it and check it on the arm. Cura's build plate, for the Kinova.

    python editor.py            then open http://localhost:8792

"Plan" and "Plan and check" never move the arm: arm_sim.py only asks the arm for IK and trajectory
validation. On armster (EDITOR_ON_ARM=1) the page can also air-run a checked plan (air_run.py: the path,
raised, no extrusion) and park the arm at Home. Those move the arm, only after a confirm on the page,
and the page's Stop button interrupts the script and sends the arm Stop (arm_stop.py).

Where it runs:
  on a laptop        binds 127.0.0.1; the arm check copies the scripts to armster and runs it there by ssh.
  on armster         EDITOR_ON_ARM=1: everything runs locally (homeostat-editor.service does this).
EDITOR_HOST        address to listen on: 127.0.0.1 (default), or "tailscale" for this machine's
                   Tailscale address only, so it is not on the arm's wired network or the Wi-Fi.
"""
import json, math, os, re, signal, subprocess, sys, threading, time, traceback
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
import numpy as np
import gen3, plan, editor_core as ec, attractors, armstream, version

VERSION = version.get()       # what this editor is running: the pages show it, and reload when it changes

HERE = os.path.dirname(os.path.abspath(__file__))
PLANS = os.path.join(HERE, "plans")
IMPORTS = os.path.join(PLANS, "imports")      # paths sent from the simulator page
PORT = int(os.environ.get("EDITOR_PORT", 8792))
HOST = os.environ.get("EDITOR_HOST", "127.0.0.1")
ON_ARM = os.environ.get("EDITOR_ON_ARM") == "1"
LAYOUT = os.path.expanduser("~/kinova/layout.json")
PI = os.environ.get("ARM_HOST", "pi@armster")
PI_DIR = "tools/claude/homeostat"
DEPLOY = ["gen3.py", "wrist.py", "plan.py", "homeostat_path.py", "arm_sim.py", "air_run.py"]
NUM = ("preset", "seconds", "scale", "from_t", "x", "y", "table", "lift", "yaw", "roll", "pitch")

_reach, _reach_lock = {}, threading.Lock()
_sweep = {"state": "loading", "sweep": None}
# one job at a time. kind: plan | airrun | park. "result" is the last plan's, kept across air-runs.
job = {"state": "idle", "kind": None, "log": [], "result": None, "run": None, "stop": False}
_proc = [None]            # the motion script running now, for Stop
KINOVA = os.path.expanduser("~/kinova")
HOME = [0, 15, 230, 0, 55, 90]          # the arm's factory Home pose, deg
LAST_RESULT = os.path.join(PLANS, "last_result.json")   # the newest plan's result, kept across restarts
LAST_RUN = os.path.join(PLANS, "last_run.json")   # the plan last air-run, so Park knows what may be in the way
LIVE = os.path.join(HERE, "live.json")           # written by air_run.py / safe_park.py while they move the arm
_idle = {"t": 0.0, "d": None}
TABLE = os.path.join(PLANS, "table.json")        # the measured table height, the default for new layouts
_touches = []                                    # tip positions captured by hand-guiding, this session


def table_state():
    out = dict(touches=_touches, saved=json.load(open(TABLE)) if os.path.exists(TABLE) else None)
    if _touches:
        z = np.array([t["tip"][2] for t in _touches])
        out.update(mean=round(float(z.mean()), 4), spread_mm=round(float(np.ptp(z)) * 1000, 1))
        xy = np.array([t["tip"][:2] for t in _touches])
        spread_xy = float(np.linalg.svd(xy - xy.mean(0), compute_uv=False)[-1]) / math.sqrt(len(xy)) if len(xy) >= 3 else 0.0
        out["spread_xy_mm"] = round(spread_xy * 1000, 1)
        if len(_touches) >= 3 and spread_xy > 0.02:     # a plane through them (only if they span an area): how far off level
            A = np.c_[[t["tip"][0] for t in _touches], [t["tip"][1] for t in _touches], np.ones(len(_touches))]
            a, b, _ = np.linalg.lstsq(A, z, rcond=None)[0]
            out["tilt_deg"] = round(float(np.degrees(np.arctan(np.hypot(a, b)))), 2)
    return out


_arm_read = threading.Lock()


def read_arm_joints():
    """The arm's joints, read directly (only when nothing of ours is moving it). One reading at a time:
    two sessions opened at once can time out. One retry."""
    with _arm_read:
        try:
            return _read_arm_joints()
        except Exception:
            time.sleep(0.5)
            return _read_arm_joints()


def _read_arm_joints():
    sys.path.insert(0, KINOVA)
    import kenv; kenv.load()
    from kortex_api.TCPTransport import TCPTransport
    from kortex_api.RouterClient import RouterClient
    from kortex_api.SessionManager import SessionManager
    from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
    from kortex_api.autogen.messages import Session_pb2
    t = TCPTransport()
    try:                       # the socket is always closed: the editor runs for days and reads every few seconds
        r = RouterClient(t, RouterClient.basicErrorCallback); t.connect("192.168.1.10", 10000)
        info = Session_pb2.CreateSessionInfo(); info.username = os.environ["KINOVA_USER"]; info.password = os.environ["KINOVA_PASS"]
        info.session_inactivity_timeout = 10000; info.connection_inactivity_timeout = 2000
        sm = SessionManager(r); sm.CreateSession(info)
        try:
            return [a.position for a in BaseCyclicClient(r).RefreshFeedback().actuators]
        finally:
            sm.CloseSession(); r.SetActivationStatus(False)
    finally:
        t.disconnect()


def live_state():
    """Where the arm is: from the moving script's feed while one runs, else read from the arm (every 3 s at most)."""
    d = None
    try:
        d = json.load(open(LIVE))
    except Exception:
        pass
    moving = bool(d and d.get("active") and time.time() - d.get("t", 0) < 3)
    if not moving and ON_ARM and not (job["state"] == "running" and job["kind"] != "plan"):
        if d and d.get("joints") and d.get("t", 0) > _idle["t"]:      # a run just ended: its last reading is newest
            _idle.update(t=d["t"], d=dict(d, active=False))
        if time.time() - _idle["t"] > 3:
            _idle["t"] = time.time()
            try:
                _idle["d"] = dict(active=False, what="", joints=read_arm_joints(), t=time.time())
            except Exception as e:
                _idle["d"] = dict(active=False, what="", error=str(e).splitlines()[0][:100] if str(e) else "no arm")
        d = _idle["d"]
    if not d or not d.get("joints"):
        return dict(active=False, points=None, error=(d or {}).get("error", "no reading"))
    F = gen3.frames(d["joints"], plan.TOOL)
    return dict(active=moving, what=d.get("what", ""), joints=[round(v, 2) for v in d["joints"]],
                points=[[0, 0, 0]] + [f[:3, 3].round(4).tolist() for f in F], t=d.get("t"))
_quick_gen = [0]          # a newer quick check makes older ones stop early
_job_lock = threading.Lock()


def nums(d):
    p = {k: float(d[k]) for k in NUM if k in d}
    for k, v in p.items():
        if not math.isfinite(v):
            raise ValueError("%s is not a number" % k)
    p["preset"] = int(p.get("preset", 0))
    if not 10 <= p.get("scale", 200) <= 4000:
        raise ValueError("scale must be 10-4000 mm per 90 deg")
    return p


FASTS = (1, 2, 3, 4, 6, 8)
_times_lock = threading.Lock()


def run_times(name):
    """How long a light drawing runs at each speed: the streamed drawing as air_run.py times it (from the
    plan's joints; the run re-solves them, within a degree or so), plus getting there, the two shutter
    pauses and the way home. Cached next to the plan."""
    cache = os.path.join(PLANS, name + "_times.json")
    if os.path.exists(cache):
        return json.load(open(cache))
    with _times_lock:
        if os.path.exists(cache):
            return json.load(open(cache))
        f = os.path.join(PLANS, name + "_checked.json")
        d = json.load(open(f if os.path.exists(f) else os.path.join(PLANS, name + ".json")))
        wps = [w for w in d["waypoints"] if w["ok"]]
        Q = np.array([w.get("arm_ik") or w["joints"] for w in wps], float)
        tips = np.array([w["tip"] for w in wps], float)
        L = np.linalg.norm(np.diff(tips, axis=0), axis=1)
        est = armstream.estimate(Q, L, [0.010 * k for k in FASTS])
        out = dict(name=name, length_mm=round(float(L.sum()) * 1000),
                   times={str(k): dict(s=round(est[0.010 * k][0], 1), mm_s=round(est[0.010 * k][1] * 1000, 1)) for k in FASTS},
                   extra_s=45)          # down to the start, back out and Home: about 45 s, plus the shutter pauses
        json.dump(out, open(cache, "w"))
        return out


def source_file(name):
    """A simulator path by name ("" = the built-in model). Only names this server made."""
    if not name:
        return None
    if not re.fullmatch(r"(sim|att)-[0-9-]+", name) or not os.path.exists(os.path.join(IMPORTS, name + ".json")):
        raise ValueError("no simulator path called %r" % name)
    return os.path.join(IMPORTS, name + ".json")


def save_import(d):
    pts = d.get("points")
    if not isinstance(pts, list) or not 20 <= len(pts) <= 200000:
        raise ValueError("expected 20 to 200000 points")
    A = np.asarray(pts, float)
    if A.ndim != 2 or A.shape[1] != 4 or not np.isfinite(A).all():
        raise ValueError("each point must be t, x, y, z")
    mm = float(d.get("mm_per_90", 0))
    if not 10 <= mm <= 1000:
        raise ValueError("mm_per_90 must be 10-1000")
    os.makedirs(IMPORTS, exist_ok=True)
    name = time.strftime("sim-%Y%m%d-%H%M%S")
    keep = dict(format="homeostat-path/1", seed=d.get("seed"), preset=str(d.get("preset", ""))[:60],
                t_end=float(A[-1, 0]), view={k: float(v) for k, v in (d.get("view") or {}).items() if k in ("xw", "yw")},
                mm_per_90=mm, received=time.strftime("%Y-%m-%d %H:%M"), points=A.round(4).tolist())
    json.dump(keep, open(os.path.join(IMPORTS, name + ".json"), "w"))
    return name


def make_attractor(body):
    kind = str(body.get("kind", ""))
    if kind not in attractors.ATTRACTORS:
        raise ValueError("no attractor called %r" % kind)
    dur = float(body.get("duration") or attractors.ATTRACTORS[kind][6])
    if not 0 < dur <= 20 * attractors.ATTRACTORS[kind][6]:
        raise ValueError("run it for 0 to %g" % (20 * attractors.ATTRACTORS[kind][6]))
    d = attractors.export(kind, dur, None, int(body.get("seed", 0)))
    d["received"] = time.strftime("%Y-%m-%d %H:%M")
    os.makedirs(IMPORTS, exist_ok=True)
    name = time.strftime("att-%Y%m%d-%H%M%S")
    json.dump(d, open(os.path.join(IMPORTS, name + ".json"), "w"))
    return name


def list_imports():
    out = []
    if os.path.isdir(IMPORTS):
        for f in sorted(os.listdir(IMPORTS), reverse=True):
            if re.fullmatch(r"(sim|att)-[0-9-]+\.json", f):
                d = json.load(open(os.path.join(IMPORTS, f)))
                out.append(dict(name=f[:-5], seed=d.get("seed"), preset=d.get("preset"), t_end=d.get("t_end"),
                                view=d.get("view"), mm_per_90=d.get("mm_per_90"), points=len(d["points"]), received=d.get("received"),
                                attractor=d.get("attractor"), params=d.get("params")))
    return out


def placed(p):
    B, T = ec.base_shape(p["preset"], p["seconds"], p["scale"], p["from_t"], path_file=p.get("path_file"))
    W = plan.place_from_base(B, (p["x"], p["y"]), p["table"], p["lift"], p["yaw"], p["roll"], p["pitch"])
    return W, T


def reach(table, light=False):
    key = (round(table, 3), bool(light))
    with _reach_lock:
        if key not in _reach:
            _reach[key] = ec.reach_map(key[0], key[1])
        return _reach[key]


def load_sweep():
    """The working sweep the painting rig allows (armster ~/kinova/layout.json 'sector'), if reachable."""
    try:
        if ON_ARM:
            sec = json.load(open(LAYOUT))["sector"]
        else:
            out = subprocess.run(["ssh", "-o", "ConnectTimeout=6", "-o", "BatchMode=yes", PI, "cat ~/kinova/layout.json"],
                                 capture_output=True, text=True, timeout=15)
            sec = json.loads(out.stdout)["sector"]
        _sweep.update(state="ok", sweep=[float(sec["from"]), float(sec["to"])])
    except Exception as e:
        _sweep.update(state="unavailable (%s)" % str(e).splitlines()[0][:60] if str(e) else "unavailable", sweep=None)


def log(line):
    job["log"].append(time.strftime("%H:%M:%S ") + line)


def run(cmd, timeout=900):
    log("$ " + " ".join(cmd))
    r = subprocess.run(cmd, cwd=HERE, capture_output=True, text=True, timeout=timeout)
    for line in (r.stdout + r.stderr).strip().splitlines()[-12:]:
        log("  " + line)
    if r.returncode:
        raise RuntimeError("%s exited %d" % (cmd[0], r.returncode))
    return r.stdout


def plan_job(p, check):
    try:
        os.makedirs(PLANS, exist_ok=True)
        name = time.strftime("layout-%Y%m%d-%H%M%S")
        out = os.path.join("plans", name + ".json")
        args = [sys.executable, "plan.py"] + (["--path-file", os.path.relpath(p["path_file"], HERE)] if p.get("path_file") else []) + [
                "--preset", str(p["preset"]), "--seconds", str(p["seconds"]),
                "--scale", str(p["scale"]), "--from-t", str(p["from_t"]), "--x", str(p["x"]), "--y", str(p["y"]),
                "--table", str(p["table"]), "--lift", str(p["lift"]), "--yaw", str(p["yaw"]),
                "--roll", str(p["roll"]), "--pitch", str(p["pitch"]), "--out", out] + (["--light"] if p.get("light") else [])
        if stream(args[:1] + ["-u"] + args[1:] + ["--progress"], {}):
            raise RuntimeError("plan.py failed")
        final = out
        check_summary = None
        if check and ON_ARM:
            log("checking on the arm (no motion)")
            run([sys.executable, "arm_sim.py", out], 900)
            final = os.path.join("plans", name + "_checked.json")
        elif check:
            log("deploying the scripts to %s and checking on the arm (no motion)" % PI)
            run(["scp", "-q"] + DEPLOY + [out, "%s:%s/" % (PI, PI_DIR)], 120)
            run(["ssh", PI, "cd ~/%s && ~/kinova-py310/bin/python arm_sim.py %s.json" % (PI_DIR, name)], 900)
            final = os.path.join("plans", name + "_checked.json")
            run(["scp", "-q", "%s:%s/%s_checked.json" % (PI, PI_DIR, name), final], 120)
        run([sys.executable, "make_viewer.py", final, os.path.join("plans", name + ".html")])
        d = json.load(open(os.path.join(HERE, final)))
        c = d.get("arm_check")
        if c:
            check_summary = dict(ik_refused=len(c["ik_refused"]), clashes=len(c["clashes"]),
                                 traj_errors=len(c["traj_errors"]), worst_dq=round(c["worst_dq"], 2))
        job["result"] = dict(name=name, viewer="/plans/%s.html" % name, printable_mm=d["printable_mm"],
                             total_mm=d["total_mm"], fails=d["fails"], points=len(d["waypoints"]), check=check_summary,
                             params=dict(p, source=p.get("source", "")), checked=bool(c))
        json.dump(job["result"], open(LAST_RESULT, "w"))     # so a restart of the editor doesn't lose it
        job["state"] = "done"
        log("done: %.0f of %.0f mm printable" % (d["printable_mm"], d["total_mm"]))
    except Exception as e:
        log("FAILED: %s" % e)
        job["state"] = "failed"


def stream(cmd, env_extra, cwd=HERE):
    """Run a script, its output going to the job log line by line. Returns the exit code."""
    log("$ " + " ".join(os.path.basename(c) if c == sys.executable else c for c in cmd))
    p = subprocess.Popen(cmd, cwd=cwd, env=dict(os.environ, **env_extra), stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True, bufsize=1)
    _proc[0] = p
    try:
        for line in p.stdout:
            log("  " + line.rstrip())
        return p.wait()
    finally:
        _proc[0] = None


GO_FILE = os.path.join(PLANS, ".continue")   # the Continue button: a light drawing waiting at its start goes on


def airrun_job(name, raise_mm, fast=1.0, light_pin=0, hold_s=5.0):
    try:
        path = os.path.join("plans", name + "_checked.json")
        env = {"AIR_RAISE_MM": "%g" % raise_mm, "AIR_FAST": "%g" % fast,
               "AIR_LIGHT_PIN": "%d" % light_pin, "AIR_LIGHT_HOLD_S": "%g" % hold_s, "AIR_GO_FILE": GO_FILE}
        if os.path.exists(GO_FILE):
            os.remove(GO_FILE)            # a Continue left over from an earlier run must not start this one
        log("asking the arm whether it can run this, raised %g mm, %s (no motion)" % (raise_mm, "print speed" if fast == 1 else "%g x faster" % fast))
        if stream([sys.executable, "-u", "air_run.py", path], env):
            if any("no IK solution" in l for l in job["log"][-4:]):
                # too high for the arm somewhere: find the most raise that does work, without moving
                for r in [v for v in (10, 5, 0) if v < raise_mm]:
                    log("trying %g mm (no motion)" % r)
                    c = subprocess.run([sys.executable, "air_run.py", path], cwd=HERE, capture_output=True, text=True,
                                       env=dict(os.environ, AIR_RAISE_MM=str(r), AIR_FAST="%g" % fast), timeout=600)
                    if c.returncode == 0:
                        raise RuntimeError("raised %g mm the arm can't reach every point, but it can at %g mm: "
                                           "set the raise to %g and run it again. Nothing moved." % (raise_mm, r, r))
                raise RuntimeError("the arm can't reach every point of this plan even at 0 mm raise. Nothing moved.")
            raise RuntimeError("the arm won't run it like this (see the log). Nothing moved.")
        if job["stop"]:
            raise RuntimeError("stopped before moving")
        log("MOVING THE ARM")
        rc = stream([sys.executable, "-u", "air_run.py", path], dict(env, KINOVA_CONFIRM="yes"))
        job["run"] = dict(name=name, raise_mm=raise_mm, fast=fast, ok=rc == 0 and not job["stop"], stopped=job["stop"])
        if job["stop"]:
            raise RuntimeError("stopped part way. Park the arm at Home before the next run.")
        if rc:
            raise RuntimeError("air_run.py exited %d" % rc)
        job["state"] = "done"; log("done")
    except Exception as e:
        log("FAILED: %s" % e); job["state"] = "failed"


def park_job():
    try:
        last = json.load(open(LAST_RUN)) if os.path.exists(LAST_RUN) else {}
        args = [sys.executable, "-u", "safe_park.py"]
        if last.get("name") and os.path.exists(os.path.join(PLANS, last["name"] + "_checked.json")):
            args.append(os.path.join("plans", last["name"] + "_checked.json"))
            log("keeping clear of %s (air-run %g mm up)" % (last["name"], last.get("raise_mm", 0)))
        else:
            log("no plan known: backing out and rising anyway, then Home")
        env = {"PARK_RAISE_MM": "%g" % last.get("raise_mm", 0)}
        if stream(args, env):
            if any("TimeoutError" in l for l in job["log"][-6:]):
                raise RuntimeError("the arm didn't answer; nothing moved. If it stays silent, support it and power-cycle it")
            raise RuntimeError("no safe way out found from here; nothing moved (see the log)")
        if job["stop"]:
            raise RuntimeError("stopped before moving")
        rc = stream(args, dict(env, KINOVA_CONFIRM="yes"))
        if job["stop"]:
            raise RuntimeError("stopped part way")
        if rc:
            raise RuntimeError("safe_park.py exited %d" % rc)
        job["state"] = "done"; log("parked at Home")
    except Exception as e:
        log("FAILED: %s" % e); job["state"] = "failed"


def stop_arm():
    job["stop"] = True
    log("STOP pressed")
    p = _proc[0]
    if p and p.poll() is None:
        p.send_signal(signal.SIGINT)          # air_run.py sends Stop itself on Ctrl-C
    # and always tell the arm directly: goto_pose.py does not stop the arm on Ctrl-C
    r = subprocess.run([sys.executable, "arm_stop.py"], cwd=HERE, capture_output=True, text=True, timeout=30)
    log("  " + (r.stdout + r.stderr).strip().splitlines()[-1] if (r.stdout + r.stderr).strip() else "  arm_stop.py ran")


def start_job(kind, target, args=()):
    with _job_lock:
        if job["state"] == "running":
            return False
        keep = job["result"]
        job.update(state="running", kind=kind, log=[], stop=False, run=None, started=time.time(),
                   result=None if kind == "plan" else keep)
    threading.Thread(target=target, args=args, daemon=True).start()
    return True


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def send(self, code, body, ctype="application/json"):
        b = body if isinstance(body, bytes) else json.dumps(body).encode()
        self.send_response(code); self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(b))); self.send_header("Cache-Control", "no-store")
        self.end_headers(); self.wfile.write(b)

    def do_GET(self):
        u = urlparse(self.path); q = {k: v[0] for k, v in parse_qs(u.query).items()}
        try:
            if u.path in ("/", "/editor.html"):
                return self.send(200, open(os.path.join(HERE, "editor.html"), "rb").read(), "text/html; charset=utf-8")
            if u.path in ("/sim", "/sim/"):
                return self.send(200, open(os.path.join(HERE, "homeostat.html"), "rb").read(), "text/html; charset=utf-8")
            if u.path == "/api/runtimes":
                name = q.get("name", "")
                if not re.fullmatch(r"layout-[0-9-]+", name):
                    return self.send(400, dict(error="no such plan"))
                return self.send(200, run_times(name))
            if u.path == "/api/attractors":
                return self.send(200, [dict(kind=k, label=a[0], duration=a[6], about=a[7]) for k, a in attractors.ATTRACTORS.items()])
            if u.path == "/api/imports":
                return self.send(200, list_imports())
            if u.path == "/api/shape":
                p = nums(q)
                B, T = ec.base_shape(p["preset"], p["seconds"], p["scale"], p["from_t"], path_file=source_file(q.get("source", "")))
                return self.send(200, dict(base=B.round(5).tolist(), t=T.round(3).tolist(), scale=p["scale"]))
            if u.path == "/api/reach":
                return self.send(200, reach(float(q.get("table", 0.16)), q.get("light") == "1"))
            if u.path == "/api/sweep":
                if _sweep["state"] not in ("ok", "loading"):    # armster was slow or away: try again
                    _sweep["state"] = "loading"
                    threading.Thread(target=load_sweep, daemon=True).start()
                return self.send(200, _sweep)
            if u.path == "/api/armref":     # the arm standing at Home, as drawn for scale (base, joints, flange, tip)
                F = gen3.frames(HOME, plan.TOOL)
                return self.send(200, dict(points=[[0, 0, 0]] + [f[:3, 3].round(4).tolist() for f in F]))
            if u.path == "/api/table":
                return self.send(200, table_state())
            if u.path == "/api/live":
                return self.send(200, live_state())
            if u.path == "/api/job":
                return self.send(200, dict(job, on_arm=ON_ARM, now=time.time(), version=VERSION))
            if u.path == "/api/version":
                return self.send(200, dict(VERSION, label=version.label(VERSION)))
            m = re.fullmatch(r"/plans/(layout-[0-9-]+(?:_checked)?\.(html|json))", u.path)
            if m and os.path.exists(os.path.join(PLANS, m.group(1))):
                ct = "text/html; charset=utf-8" if m.group(2) == "html" else "application/json"
                return self.send(200, open(os.path.join(PLANS, m.group(1)), "rb").read(), ct)
            self.send(404, {"error": "not found"})
        except Exception as e:
            traceback.print_exc(); self.send(400, {"error": str(e)})

    def do_POST(self):
        u = urlparse(self.path)
        try:
            body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
            if u.path == "/api/stop":
                if not ON_ARM:
                    return self.send(400, {"error": "Stop works on armster only"})
                stop_arm()
                return self.send(200, {"stopped": True})
            if u.path == "/api/continue":
                if not (job["state"] == "running" and job["kind"] == "airrun"):
                    return self.send(409, {"error": "nothing is waiting"})
                open(GO_FILE, "w").close()
                return self.send(200, {"ok": True})
            if u.path in ("/api/airrun", "/api/park"):
                if not ON_ARM:
                    return self.send(400, {"error": "the arm can only be moved from the editor on armster"})
                if u.path == "/api/park":
                    ok = start_job("park", park_job)
                else:
                    name = str(body.get("name", ""))
                    raise_mm = float(body.get("raise_mm", 20))
                    if not re.fullmatch(r"layout-[0-9-]+", name) or not os.path.exists(os.path.join(PLANS, name + "_checked.json")):
                        return self.send(400, {"error": "no checked plan called %r" % name})
                    if not (0 <= raise_mm <= 100):
                        return self.send(400, {"error": "raise must be 0-100 mm"})
                    fast = float(body.get("fast", 1))
                    if not (1 <= fast <= 8):
                        return self.send(400, {"error": "speed must be 1-8 x"})
                    light_pin, hold_s = int(body.get("light_pin", 0)), float(body.get("hold_s", 5))
                    if not (0 <= light_pin <= 4 and 0 <= hold_s <= 60):
                        return self.send(400, {"error": "light pin must be 0-4 and the shutter pause 0-60 s"})
                    c = json.load(open(os.path.join(PLANS, name + "_checked.json"))).get("arm_check") or {}
                    if c.get("ik_refused") or c.get("clashes") or c.get("traj_errors"):
                        return self.send(400, {"error": "that plan did not pass the arm check"})
                    ok = start_job("airrun", airrun_job, (name, raise_mm, fast, light_pin, hold_s))
                    if ok:
                        json.dump(dict(name=name, raise_mm=raise_mm), open(LAST_RUN, "w"))
                return self.send(200 if ok else 409, {"started": ok} if ok else {"error": "something is already running"})
            if u.path == "/api/import":
                return self.send(200, {"name": save_import(body)})
            if u.path == "/api/attractor":
                return self.send(200, {"name": make_attractor(body)})
            if u.path == "/api/table/capture":       # reads the arm only: you put the tip there by hand
                if not ON_ARM:
                    return self.send(400, {"error": "measuring the table works from the editor on armster"})
                if job["state"] == "running" and job["kind"] != "plan":
                    return self.send(409, {"error": "the arm is being moved by the editor; wait for it"})
                q = read_arm_joints()
                tip = gen3.fk(q, plan.TOOL)[:3, 3]
                _touches.append(dict(tip=[round(float(v), 4) for v in tip], joints=[round(v, 2) for v in q], at=time.strftime("%H:%M:%S")))
                return self.send(200, table_state())
            if u.path == "/api/table/clear":
                _touches.clear()
                return self.send(200, table_state())
            if u.path == "/api/table/save":
                z = float(body.get("z"))
                if not -0.3 <= z <= 0.6:          # the table can be below the arm's base mount
                    return self.send(400, {"error": "table height must be -0.3 to 0.6 m in the arm's frame"})
                json.dump(dict(z=round(z, 4), touches=len(_touches), when=time.strftime("%Y-%m-%d %H:%M")), open(TABLE, "w"))
                return self.send(200, table_state())
            p = nums(body)
            p["light"] = bool(body.get("light"))
            p["source"] = str(body.get("source", "") or "")
            p["path_file"] = source_file(p["source"])
            if u.path == "/api/quick":
                W, T = placed(p)
                sweep = _sweep["sweep"] if body.get("use_sweep") else None
                _quick_gen[0] += 1; mine = _quick_gen[0]
                try:
                    res = ec.quick_check(W, p["table"], sweep, cancel=lambda: _quick_gen[0] != mine, light=p["light"])
                except ec.Cancelled:
                    return self.send(409, {"error": "superseded by a newer check"})
                res["W"] = W.round(5).tolist()
                return self.send(200, res)
            if u.path == "/api/plan":
                if not start_job("plan", plan_job, (p, bool(body.get("check")))):
                    return self.send(409, {"error": "something is already running"})
                return self.send(200, {"started": True})
            self.send(404, {"error": "not found"})
        except Exception as e:
            traceback.print_exc(); self.send(400, {"error": str(e)})


def listen_address():
    if HOST != "tailscale":
        return HOST
    # Tailscale may still be coming up at boot: fail, and let systemd restart us until it is there
    out = subprocess.run(["tailscale", "ip", "-4"], capture_output=True, text=True, timeout=10).stdout.split()
    if not out:
        sys.exit("No Tailscale address yet.")
    return out[0]


if __name__ == "__main__":
    host = listen_address()
    try:
        job["result"] = json.load(open(LAST_RESULT))
    except Exception:
        pass
    threading.Thread(target=load_sweep, daemon=True).start()
    def warm():      # the measured table height (else the default), for printing and for light
        z = json.load(open(TABLE))["z"] if os.path.exists(TABLE) else 0.16
        reach(z); reach(z, True)
    threading.Thread(target=warm, daemon=True).start()
    print("layout editor on http://%s:%d%s  (Ctrl-C to stop)" % (host, PORT, ", on the arm" if ON_ARM else ""), flush=True)
    ThreadingHTTPServer((host, PORT), H).serve_forever()
