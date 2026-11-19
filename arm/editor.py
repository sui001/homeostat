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
import gen3, plan, editor_core as ec

HERE = os.path.dirname(os.path.abspath(__file__))
PLANS = os.path.join(HERE, "plans")
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
_quick_gen = [0]          # a newer quick check makes older ones stop early
_job_lock = threading.Lock()


def nums(d):
    p = {k: float(d[k]) for k in NUM if k in d}
    for k, v in p.items():
        if not math.isfinite(v):
            raise ValueError("%s is not a number" % k)
    p["preset"] = int(p.get("preset", 0))
    if not 10 <= p.get("scale", 200) <= 1000:
        raise ValueError("scale must be 10-1000 mm per 90 deg")
    return p


def placed(p):
    B, T = ec.base_shape(p["preset"], p["seconds"], p["scale"], p["from_t"])
    W = plan.place_from_base(B, (p["x"], p["y"]), p["table"], p["lift"], p["yaw"], p["roll"], p["pitch"])
    return W, T


def reach(table):
    key = round(table, 3)
    with _reach_lock:
        if key not in _reach:
            _reach[key] = ec.reach_map(key)
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
        args = [sys.executable, "plan.py", "--preset", str(p["preset"]), "--seconds", str(p["seconds"]),
                "--scale", str(p["scale"]), "--from-t", str(p["from_t"]), "--x", str(p["x"]), "--y", str(p["y"]),
                "--table", str(p["table"]), "--lift", str(p["lift"]), "--yaw", str(p["yaw"]),
                "--roll", str(p["roll"]), "--pitch", str(p["pitch"]), "--out", out]
        run(args)
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
                             params=p, checked=bool(c))
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


def airrun_job(name, raise_mm):
    try:
        path = os.path.join("plans", name + "_checked.json")
        env = {"AIR_RAISE_MM": "%g" % raise_mm}
        log("asking the arm whether it can run this, raised %g mm (no motion)" % raise_mm)
        if stream([sys.executable, "-u", "air_run.py", path], env):
            raise RuntimeError("the arm can't run it like this; nothing moved")
        if job["stop"]:
            raise RuntimeError("stopped before moving")
        log("MOVING THE ARM")
        rc = stream([sys.executable, "-u", "air_run.py", path], dict(env, KINOVA_CONFIRM="yes"))
        job["run"] = dict(name=name, raise_mm=raise_mm, ok=rc == 0 and not job["stop"], stopped=job["stop"])
        if job["stop"]:
            raise RuntimeError("stopped part way. Park the arm at Home before the next run.")
        if rc:
            raise RuntimeError("air_run.py exited %d" % rc)
        job["state"] = "done"; log("done")
    except Exception as e:
        log("FAILED: %s" % e); job["state"] = "failed"


def park_job():
    try:
        rc = stream([sys.executable, "-u", os.path.join(KINOVA, "goto_pose.py")],
                    {"KINOVA_TARGET": "Home", "KINOVA_CONFIRM": "yes"}, cwd=KINOVA)
        if job["stop"]:
            raise RuntimeError("stopped part way")
        if rc:
            raise RuntimeError("goto_pose.py exited %d" % rc)
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
        job.update(state="running", kind=kind, log=[], stop=False, run=None,
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
            if u.path == "/api/shape":
                p = nums(q)
                B, T = ec.base_shape(p["preset"], p["seconds"], p["scale"], p["from_t"])
                return self.send(200, dict(base=B.round(5).tolist(), t=T.round(3).tolist(), scale=p["scale"]))
            if u.path == "/api/reach":
                return self.send(200, reach(float(q.get("table", 0.16))))
            if u.path == "/api/sweep":
                if _sweep["state"] not in ("ok", "loading"):    # armster was slow or away: try again
                    _sweep["state"] = "loading"
                    threading.Thread(target=load_sweep, daemon=True).start()
                return self.send(200, _sweep)
            if u.path == "/api/armref":     # the arm standing at Home, as drawn for scale (base, joints, flange, tip)
                F = gen3.frames(HOME, plan.TOOL)
                return self.send(200, dict(points=[[0, 0, 0]] + [f[:3, 3].round(4).tolist() for f in F]))
            if u.path == "/api/job":
                return self.send(200, dict(job, on_arm=ON_ARM))
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
                    c = json.load(open(os.path.join(PLANS, name + "_checked.json"))).get("arm_check") or {}
                    if c.get("ik_refused") or c.get("clashes") or c.get("traj_errors"):
                        return self.send(400, {"error": "that plan did not pass the arm check"})
                    ok = start_job("airrun", airrun_job, (name, raise_mm))
                return self.send(200 if ok else 409, {"started": ok} if ok else {"error": "something is already running"})
            p = nums(body)
            if u.path == "/api/quick":
                W, T = placed(p)
                sweep = _sweep["sweep"] if body.get("use_sweep") else None
                _quick_gen[0] += 1; mine = _quick_gen[0]
                try:
                    res = ec.quick_check(W, p["table"], sweep, cancel=lambda: _quick_gen[0] != mine)
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
    threading.Thread(target=load_sweep, daemon=True).start()
    threading.Thread(target=reach, args=(0.16,), daemon=True).start()   # warm the default table height
    print("layout editor on http://%s:%d%s  (Ctrl-C to stop)" % (host, PORT, ", on the arm" if ON_ARM else ""), flush=True)
    ThreadingHTTPServer((host, PORT), H).serve_forever()
