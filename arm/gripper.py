#!/usr/bin/env python3
"""Open the gripper, or close it slowly. The editor's Open gripper / Slow close buttons run this.

    KINOVA_CONFIRM=yes ~/kinova-py310/bin/python gripper.py open  [speed]
    KINOVA_CONFIRM=yes ~/kinova-py310/bin/python gripper.py close [speed]

Speed is a fraction of the gripper's top finger speed (0.01-1): open defaults to 0.3, close to 0.05.
It runs in speed mode, so a close stops by itself when the fingers meet something (the gripper's own
object detection) and then keeps squeezing with the gripper's grip force, whatever the object's size.
Closing slowly softens the hit, not the squeeze that follows. Without KINOVA_CONFIRM it only reports
where the fingers are. Ctrl-C (the editor's Stop) stops the fingers where they are.
"""
import os, sys, time
sys.path.insert(0, os.path.expanduser("~/kinova"))
import kenv; kenv.load()
import armlock
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Session_pb2, Base_pb2
import version; print("software version %s" % version.label(), flush=True)

ACTION = sys.argv[1] if len(sys.argv) > 1 else ""
if ACTION not in ("open", "close"):
    sys.exit("usage: gripper.py open|close [speed 0.01-1]")
SPEED = float(sys.argv[2]) if len(sys.argv) > 2 else (0.3 if ACTION == "open" else 0.05)
if not 0.01 <= SPEED <= 1:
    sys.exit("speed must be 0.01-1")
ARMED = os.environ.get("KINOVA_CONFIRM") == "yes"
TIMEOUT_S = 60


def command(base, v):
    c = Base_pb2.GripperCommand(); c.mode = Base_pb2.GRIPPER_SPEED
    f = c.gripper.finger.add(); f.finger_identifier = 1; f.value = v     # positive opens, negative closes
    base.SendGripperCommand(c)


def measured(base, mode):
    rq = Base_pb2.GripperRequest(); rq.mode = mode
    m = base.GetMeasuredGripperMovement(rq).finger
    return m[0].value if len(m) else None


t = TCPTransport(); r = RouterClient(t, RouterClient.basicErrorCallback); t.connect("192.168.1.10", 10000)
info = Session_pb2.CreateSessionInfo(); info.username = os.environ["KINOVA_USER"]; info.password = os.environ["KINOVA_PASS"]
info.session_inactivity_timeout = 60000; info.connection_inactivity_timeout = 2000
sm = SessionManager(r); sm.CreateSession(info)
base, lock = BaseClient(r), None
try:
    p0 = measured(base, Base_pb2.GRIPPER_POSITION)
    print("fingers %.0f%% closed" % (100 * p0))
    if not ARMED:
        print("Check only. Set KINOVA_CONFIRM=yes to move the fingers."); sys.exit(0)
    lock = armlock.take("homeostat gripper %s" % ACTION)
    print("MOVING THE FINGERS: %s at %.0f%% speed" % ("opening" if ACTION == "open" else "closing", 100 * SPEED), flush=True)
    command(base, SPEED if ACTION == "open" else -SPEED)
    t0, moved, last = time.time(), False, None
    while time.time() - t0 < TIMEOUT_S:
        time.sleep(0.1)
        v = measured(base, Base_pb2.GRIPPER_SPEED)
        moved = moved or (v is not None and abs(v) > 0.001)
        if moved and v is not None and abs(v) <= 0.001 or (not moved and time.time() - t0 > 1.5):
            break                    # stopped: at the end of travel, or on something
        p = measured(base, Base_pb2.GRIPPER_POSITION)
        if last is None or abs(p - last) >= 0.1:
            print("  %.0f%% closed" % (100 * p), flush=True); last = p
    else:
        command(base, 0.0); print("still moving after %d s: stopped the fingers" % TIMEOUT_S)
    p = measured(base, Base_pb2.GRIPPER_POSITION)
    print("%s: fingers stopped %.0f%% closed%s" % (ACTION, 100 * p, "" if ACTION == "open" else
          " (well short of fully closed = holding something, with the gripper's grip force)"))
except KeyboardInterrupt:
    try: command(base, 0.0)
    except Exception: pass
    print("\nCtrl-C: fingers stopped where they are."); sys.exit(130)
finally:
    if lock: lock.release()
    sm.CloseSession(); r.SetActivationStatus(False)
