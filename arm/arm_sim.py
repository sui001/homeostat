#!/usr/bin/env python3
"""Check a homeostat print plan against the real arm, without moving it.

Only ever calls ComputeInverseKinematics, ComputeForwardKinematics, ValidateWaypointList and
reads feedback. There is no Execute / Reach / Send call anywhere in this file.

For every planned point it
  1. asks the arm's own IK for the pose (seeded with the planned joints),
  2. re-runs the hot end + arm-link clearance check on the arm's joint solution,
  3. then validates the whole thing as Cartesian waypoint trajectories, in chunks,
     at the speed the print would run.

    ~/kinova-py310/bin/python arm_sim.py plan.json [speed_mm_s]
Writes plan_checked.json next to the plan.
"""
import os, sys, json, math
import numpy as np
sys.path.insert(0, os.path.expanduser("~/kinova")); import kenv; kenv.load()
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.ControlConfigClientRpc import ControlConfigClient
from kortex_api.autogen.messages import Base_pb2, Session_pb2
import gen3, plan

PLAN = sys.argv[1] if len(sys.argv) > 1 else "plan.json"
SPEED = float(sys.argv[2]) / 1000 if len(sys.argv) > 2 else 0.010   # m/s along the strand
CHUNK = 40

d = json.load(open(PLAN))
if d["params"].get("long_nozzle"): plan.HOTEND[:] = plan.HOTEND_LONG
W = np.array(d["strand"]); wps = d["waypoints"]; table = d["params"]["table"]
s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(W, axis=0), axis=1))]
S = np.arange(0, s[-1] + 1e-9, 0.001); dense = np.c_[[np.interp(S, s, W[:, k]) for k in range(3)]].T
if d['params']['lift'] > 0: dense, S = plan.with_post(W, dense, S, table)

t = TCPTransport(); r = RouterClient(t, RouterClient.basicErrorCallback); t.connect("192.168.1.10", 10000)
info = Session_pb2.CreateSessionInfo(); info.username = os.environ["KINOVA_USER"]; info.password = os.environ["KINOVA_PASS"]
info.session_inactivity_timeout = 60000; info.connection_inactivity_timeout = 2000
sm = SessionManager(r); sm.CreateSession(info)
try:
    base = BaseClient(r); cyc = BaseCyclicClient(r)
    tt = ControlConfigClient(r).GetToolConfiguration().tool_transform
    if abs(tt.z - 0.2) > 1e-3 or abs(tt.x) + abs(tt.y) > 1e-3:
        sys.exit("Tool transform on the arm is %s, the plan assumes z = 0.200 m. Stopping." % [tt.x, tt.y, tt.z])
    cur = [a.position for a in cyc.RefreshFeedback().actuators]
    print("arm reachable; current joints", [round(a, 1) for a in cur])

    ik_bad, clash, worst_dq, worst_tip = [], [], 0.0, 0.0
    worst_hot, worst_arm = 1e9, 1e9
    for w in wps:
        if not w["ok"]:
            continue
        ikd = Base_pb2.IKData()
        p = ikd.cartesian_pose; p.x, p.y, p.z = w["tip"]; p.theta_x, p.theta_y, p.theta_z = w["euler"]
        for q in w["joints"]:
            ikd.guess.joint_angles.add().value = q
        try:
            sol = base.ComputeInverseKinematics(ikd)
        except Exception as e:
            ik_bad.append((w["i"], str(e).splitlines()[0][:80])); w["arm_ik"] = None; continue
        qa = np.array([j.value for j in sol.joint_angles])
        dq = np.abs(((qa - np.array(w["joints"]) + 180) % 360) - 180).max()
        worst_dq = max(worst_dq, dq); w["arm_ik"] = qa.round(3).tolist()
        # the arm's solution, put back through our model, must land on the tip and clear the strand
        F = gen3.frames(qa, plan.TOOL); tip_err = np.linalg.norm(F[-1][:3, 3] - w["tip"])
        worst_tip = max(worst_tip, tip_err)
        m = S <= s[w["i"]] + 1e-9
        # the hot end goes where it is commanded (the arm does its own FK), so check it at the planned pose;
        # the links come from the arm's joint answer, through our model (~3 mm, inside the 15 mm link margin)
        hc = plan.tool_clearance(gen3.from_euler(*w["euler"]), np.array(w["tip"]), dense[m], s[w["i"]] - S[m], table)
        ac = plan.arm_clearance(F[:-1], dense[m], table)
        worst_hot = min(worst_hot, min(hc)); worst_arm = min(worst_arm, min(ac))
        if min(hc) < -1e-4 or min(ac) < -1e-4:   # 0.1 mm: pose angles are stored to 0.001 deg
            clash.append((w["i"], plan.HOTEND[int(np.argmin(hc))][0] if min(hc) < 0 else plan.ARM[int(np.argmin(ac))][0],
                          round(min(hc) * 1000, 1), round(min(ac) * 1000, 1)))
    ok = [w for w in wps if w["ok"] and w.get("arm_ik") is not None]
    print("IK: %d/%d points solved by the arm; %d refused" % (len(ok), sum(w["ok"] for w in wps), len(ik_bad)))
    print("    arm's joints vs planned joints: worst %.2f deg; arm solution tip vs plan %.1f mm (model error)" % (worst_dq, worst_tip * 1000))
    print("clearance on the arm's own solutions: hot end %.1f mm, arm links %.1f mm (after margins); %d clashes"
          % (worst_hot * 1000, worst_arm * 1000, len(clash)))
    for c in clash[:10]: print("   clash at point", c)
    for b in ik_bad[:10]: print("   IK refused", b)

    # trajectory validation, as the arm would run it: Cartesian waypoints at print speed
    traj_err = []
    for k in range(0, len(ok), CHUNK - 1):
        part = ok[k:k + CHUNK]
        if len(part) < 2: break
        wl = Base_pb2.WaypointList(); wl.use_optimal_blending = False
        for w in part:
            wp = wl.waypoints.add(); wp.name = "p%d" % w["i"]
            cw = wp.cartesian_waypoint
            cw.pose.x, cw.pose.y, cw.pose.z = w["tip"]; cw.pose.theta_x, cw.pose.theta_y, cw.pose.theta_z = w["euler"]
            cw.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
            cw.maximum_linear_velocity = SPEED; cw.maximum_angular_velocity = 15.0   # this arm refuses anything over 20 deg/s here
            cw.blending_radius = 0.0
        rep = base.ValidateWaypointList(wl)
        for e in rep.trajectory_error_report.trajectory_error_elements:
            traj_err.append((part[0]["i"], Base_pb2.TrajectoryErrorType.Name(e.error_type), round(e.error_value, 3),
                             round(e.min_value, 3), round(e.max_value, 3), e.index))
    print("ValidateWaypointList (Cartesian, %.0f mm/s): %d chunks, %d errors" % (SPEED * 1000, math.ceil(len(ok) / (CHUNK - 1)), len(traj_err)))
    for e in traj_err[:12]: print("   ", e)
    d["arm_check"] = dict(ik_refused=ik_bad, clashes=clash, traj_errors=traj_err, worst_dq=worst_dq,
                          worst_hot_mm=worst_hot * 1000, worst_arm_mm=worst_arm * 1000)
    json.dump(d, open(PLAN.replace(".json", "_checked.json"), "w"))
finally:
    sm.CloseSession(); r.SetActivationStatus(False)
