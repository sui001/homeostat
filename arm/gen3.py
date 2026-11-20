"""Kinova Gen3 6-DOF kinematics from the URDF joint origins (ros_kortex gen3 6dof).
Checked against the arm's own ComputeForwardKinematics by fk_check.py."""
import numpy as np

ARM_TCP = 0.200   # the arm's own tool transform (z); the painting rig relies on it, so it is left alone
NOZZLE = 0.230    # flange face to nozzle tip along the tool axis, measured 2026-11-18 (hot end held in the gripper)


def arm_fk_check(base, q, tcp, euler, pos_tol=0.005, ang_tol=2.0):
    """The arm's own forward kinematics for joints q (its 0.200 m tool): (ok, mm off, deg off) against the
    pose we want it at. Its IK gives up on some poses it can be in (tool pointing above horizontal, high
    up), so where it refuses, our model's joints are used if the arm itself confirms where they put it."""
    from kortex_api.autogen.messages import Base_pb2
    ja = Base_pb2.JointAngles()
    for k, v in enumerate(q):
        a = ja.joint_angles.add(); a.joint_identifier = k; a.value = float(v)
    p = base.ComputeForwardKinematics(ja)
    d_mm = 1000 * float(np.linalg.norm(np.array([p.x, p.y, p.z]) - np.asarray(tcp, float)))
    Ra, Rw = from_euler(p.theta_x, p.theta_y, p.theta_z), from_euler(*euler)
    d_deg = float(np.degrees(np.arccos(np.clip((np.trace(Ra.T @ Rw) - 1) / 2, -1, 1))))
    return d_mm <= pos_tol * 1000 and d_deg <= ang_tol, d_mm, d_deg


def arm_tcp(tip, euler, nozzle=NOZZLE):
    """Pose to command the arm (whose TCP is ARM_TCP from the flange) so the nozzle tip lands on `tip`:
    same orientation, (nozzle - ARM_TCP) back up the tool axis."""
    return np.asarray(tip, float) - (nozzle - ARM_TCP) * from_euler(*euler)[:, 2]


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                     [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                     [-sp, cp*sr, cp*cr]])

def _T(xyz, rpy):
    T = np.eye(4); T[:3, :3] = _rpy(*rpy); T[:3, 3] = xyz; return T

PI = np.pi
# URDF origins refined by a least-squares fit to this arm's ComputeForwardKinematics
# (40 samples, held-out error 2.8 mm mean / 5.7 mm max, 0.6 deg).
JOINTS = [
          _T([np.float64(-0.000248), np.float64(-0.000346), np.float64(0.154895)], [np.float64(3.137694), np.float64(0.000976), np.float64(-0.000759)]),
          _T([np.float64(0.000446), np.float64(0.005577), np.float64(-0.126857)], [np.float64(1.570691), np.float64(0.000669), np.float64(0.000311)]),
          _T([np.float64(0.001016), np.float64(-0.410924), np.float64(-0.000188)], [np.float64(3.139112), np.float64(0.001292), np.float64(-0.002285)]),
          _T([np.float64(-0.000249), np.float64(0.208458), np.float64(-0.006129)], [np.float64(1.580624), np.float64(-0.003094), np.float64(0.001223)]),
          _T([np.float64(0.000609), np.float64(4.6e-05), np.float64(-0.105744)], [np.float64(-1.573933), np.float64(0.002136), np.float64(0.003288)]),
          _T([np.float64(-0.000295), np.float64(0.106689), np.float64(-0.000142)], [np.float64(1.571291), np.float64(0.004867), np.float64(0.002706)]),]
Q_OFFSET = [np.float64(0.2239), np.float64(-0.0265), np.float64(0.1065), np.float64(0.2231), np.float64(0.0999), np.float64(-0.3287)]   # deg added to each joint before FK
EE = _T([0, 0, -0.061525], [PI, 0, PI])   # flange frame as the arm reports it
# Joint limits (deg) for the 6-DOF: J2 +-128.9, J3 +-147.8, J5 +-120.3, others continuous
LIM = [None, 128.9, 147.8, None, 120.3, None]

def _rz(q):
    c, s = np.cos(q), np.sin(q); R = np.eye(4); R[0, 0] = c; R[0, 1] = -s; R[1, 0] = s; R[1, 1] = c; return R

def frames(q_deg, tool=None):
    """Frame of each joint after its rotation (6), then the flange; then tool if given."""
    T = np.eye(4); out = []
    for J, q in zip(JOINTS, np.radians(np.asarray(q_deg, float) + Q_OFFSET)):
        T = T @ J @ _rz(q); out.append(T.copy())
    T = T @ EE; out.append(T.copy())
    if tool is not None: out.append(T @ tool)
    return out

def fk(q_deg, tool=None):
    return frames(q_deg, tool)[-1]

def euler_xyz(R):
    """Kinova theta_x/y/z (deg): R = Rz(tz) Ry(ty) Rx(tx)."""
    ty = np.arcsin(-np.clip(R[2, 0], -1, 1)); tx = np.arctan2(R[2, 1], R[2, 2]); tz = np.arctan2(R[1, 0], R[0, 0])
    return np.degrees([tx, ty, tz])

def from_euler(tx, ty, tz):
    return _rpy(*np.radians([tx, ty, tz]))
