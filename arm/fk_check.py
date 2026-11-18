"""Read-only: ask the arm for FK at random joint sets and compare with gen3.py. Commands no motion."""
import os, sys, numpy as np
sys.path.insert(0, os.path.expanduser("~/kinova")); import kenv; kenv.load()
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, Session_pb2
import gen3
t=TCPTransport(); r=RouterClient(t, RouterClient.basicErrorCallback); t.connect("192.168.1.10",10000)
i=Session_pb2.CreateSessionInfo(); i.username=os.environ["KINOVA_USER"]; i.password=os.environ["KINOVA_PASS"]
i.session_inactivity_timeout=60000; i.connection_inactivity_timeout=2000
s=SessionManager(r); s.CreateSession(i)
try:
    b=BaseClient(r); c=BaseCyclicClient(r)
    fb=c.RefreshFeedback()
    cur=[a.position for a in fb.actuators]
    print("current joints", [round(a,2) for a in cur])
    print("tool pose (arm)", [round(v,4) for v in (fb.base.tool_pose_x, fb.base.tool_pose_y, fb.base.tool_pose_z, fb.base.tool_pose_theta_x, fb.base.tool_pose_theta_y, fb.base.tool_pose_theta_z)])
    try:
        tc = b.GetControlParameters() if hasattr(b,'GetControlParameters') else None
    except Exception as e: tc=None
    rng=np.random.default_rng(1); worst=0
    for k in range(8):
        q = cur if k==0 else [rng.uniform(-170,170), rng.uniform(-100,100), rng.uniform(-130,130), rng.uniform(-170,170), rng.uniform(-110,110), rng.uniform(-170,170)]
        ja=Base_pb2.JointAngles()
        for n,v in enumerate(q):
            a=ja.joint_angles.add(); a.joint_identifier=n; a.value=v
        p=b.ComputeForwardKinematics(ja)
        T=gen3.fk(q); e=gen3.euler_xyz(T[:3,:3])
        armR=gen3.from_euler(p.theta_x,p.theta_y,p.theta_z)
        dp=np.linalg.norm(T[:3,3]-[p.x,p.y,p.z]); dr=np.degrees(np.arccos(np.clip((np.trace(armR.T@T[:3,:3])-1)/2,-1,1)))
        worst=max(worst,dp)
        Ta=np.eye(4); Ta[:3,:3]=armR; Ta[:3,3]=[p.x,p.y,p.z]
        X=np.linalg.inv(T)@Ta; print("  flange->armtool", np.round(X[:3,3],4).tolist(), np.round(gen3.euler_xyz(X[:3,:3]),2).tolist())
        print(k, "arm", [round(v,4) for v in (p.x,p.y,p.z,p.theta_x,p.theta_y,p.theta_z)], "mine", np.round(np.r_[T[:3,3],e],4).tolist(), "dpos %.1f mm drot %.2f deg"%(dp*1000,dr))
    print("worst position error %.2f mm"%(worst*1000))
    from kortex_api.autogen.client_stubs.ControlConfigClientRpc import ControlConfigClient
    cc=ControlConfigClient(r); tc=cc.GetToolConfiguration(); print("TOOL CONFIG", tc)
finally:
    s.CloseSession(); r.SetActivationStatus(False)
