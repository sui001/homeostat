"""Read-only: dump arm FK for random joint sets + the tool config, as JSON. Commands no motion."""
import os, sys, json, numpy as np
sys.path.insert(0, os.path.expanduser("~/kinova")); import kenv; kenv.load()
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.ControlConfigClientRpc import ControlConfigClient
from kortex_api.autogen.messages import Base_pb2, Session_pb2
t=TCPTransport(); r=RouterClient(t, RouterClient.basicErrorCallback); t.connect("192.168.1.10",10000)
i=Session_pb2.CreateSessionInfo(); i.username=os.environ["KINOVA_USER"]; i.password=os.environ["KINOVA_PASS"]
i.session_inactivity_timeout=60000; i.connection_inactivity_timeout=2000
s=SessionManager(r); s.CreateSession(i)
try:
    b=BaseClient(r); tc=ControlConfigClient(r).GetToolConfiguration().tool_transform
    out={"tool":[tc.x,tc.y,tc.z,tc.theta_x,tc.theta_y,tc.theta_z],"samples":[]}
    rng=np.random.default_rng(7)
    for k in range(40):
        q=[rng.uniform(-170,170), rng.uniform(-110,110), rng.uniform(-140,140), rng.uniform(-170,170), rng.uniform(-115,115), rng.uniform(-170,170)]
        if k<6: q=[0]*6; q[k]=30.0
        ja=Base_pb2.JointAngles()
        for n,v in enumerate(q):
            a=ja.joint_angles.add(); a.joint_identifier=n; a.value=v
        p=b.ComputeForwardKinematics(ja)
        out["samples"].append([q,[p.x,p.y,p.z,p.theta_x,p.theta_y,p.theta_z]])
    print(json.dumps(out))
finally:
    s.CloseSession(); r.SetActivationStatus(False)
