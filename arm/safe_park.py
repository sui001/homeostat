#!/usr/bin/env python3
"""Park the arm at Home without running into a print: back out along the nozzle, rise above the print,
then a joint move to Home, every pose checked first against the strand printed so far (exit_path.py).

    ~/kinova-py310/bin/python safe_park.py [plan_checked.json]                 # check only
    KINOVA_CONFIRM=yes ~/kinova-py310/bin/python safe_park.py [plan_checked.json]

With a plan, the print is taken to have got as far as the strand point nearest the tip now (a stopped
run). PARK_RAISE_MM: the plan was air-run this much above its height (the strand is moved up to match).
Without a plan it still backs out and rises before going Home, against no strand.
Refuses, moving nothing, if the way out would come too close to the print or the table.
"""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.expanduser("~/kinova"))
import kenv; kenv.load()
import armlock, waypoint
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Session_pb2
import gen3, plan, exit_path, armmove

PLAN = sys.argv[1] if len(sys.argv) > 1 else None
RAISE = float(os.environ.get("PARK_RAISE_MM", 0)) / 1000
ARMED = os.environ.get("KINOVA_CONFIRM") == "yes"

W, table, LIGHT = np.zeros((0, 3)), 0.16, False
if PLAN:
    d = json.load(open(PLAN))
    W, table, LIGHT = np.array(d["strand"]) + [0, 0, RAISE], d["params"]["table"], bool(d["params"].get("light"))

t = TCPTransport(); r = RouterClient(t, RouterClient.basicErrorCallback); t.connect("192.168.1.10", 10000)
info = Session_pb2.CreateSessionInfo(); info.username = os.environ["KINOVA_USER"]; info.password = os.environ["KINOVA_PASS"]
info.session_inactivity_timeout = 60000; info.connection_inactivity_timeout = 2000
sm = SessionManager(r); sm.CreateSession(info)
base = BaseClient(r); cyc = BaseCyclicClient(r)
lock = live = None
try:
    q = [a.position for a in cyc.RefreshFeedback().actuators]
    if max(abs(x) for x in waypoint.travel(q, exit_path.HOME)) < 1.0:
        print("Already at Home."); sys.exit(0)
    tip = gen3.fk(q, plan.TOOL)[:3, 3]
    upto = exit_path.nearest_point(W, tip) if len(W) else None
    if len(W):
        print("print taken as far as point %d of %d (nearest the tip, %.0f mm away)"
              % (upto, len(W) - 1, 1000 * np.linalg.norm(W[upto] - tip)))
    ex = exit_path.plan_exit(q, W, table, upto, light=LIGHT)
    if not ex["ok"]:
        sys.exit("No safe way out from here: %s. Nothing moved: move the arm clear by hand." % ex["reason"])
    print("way out: back out %.0f mm along the nozzle, up to z %.2f m, then Home; closest %.1f mm (%s)"
          % (exit_path.BACK_M * 1000, ex["rise_to"], ex["worst_mm"], ex["what"]), flush=True)
    if not ARMED:
        print("Check only. Set KINOVA_CONFIRM=yes to move."); sys.exit(0)
    lock = armlock.take("homeostat safe park")
    live = armmove.Live(cyc, "parking safely", plan=PLAN).start()
    armmove.run_exit(base, cyc, ex, "safe park")
    print("parked at Home")
except KeyboardInterrupt:
    base.Stop(); print("\nCtrl-C: sent Stop().")
    sys.exit(130)
finally:
    if live: live.stop()
    if lock: lock.release()
    sm.CloseSession(); r.SetActivationStatus(False)
