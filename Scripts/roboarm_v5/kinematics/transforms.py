"""
Rigid-body helpers. Lengths are mm, angles are radians, rotations are 3x3,
poses are 4x4 homogeneous matrices. numpy only.

RPY follows URDF: fixed-axis roll (X), pitch (Y), yaw (Z), so
R = Rz(yaw) @ Ry(pitch) @ Rx(roll).
"""

import math
import numpy as np


def rot(axis, angle):
    """Rotation about an arbitrary unit axis (Rodrigues)."""
    x, y, z = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return np.array([
        [c + x * x * C,     x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C,     y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def rotx(a): return rot((1, 0, 0), a)
def roty(a): return rot((0, 1, 0), a)
def rotz(a): return rot((0, 0, 1), a)


def pose(R=None, p=None):
    """4x4 from a rotation and a translation (either may be omitted)."""
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if p is not None:
        T[:3, 3] = p
    return T


def trans(x=0.0, y=0.0, z=0.0):
    return pose(p=(x, y, z))


def rpy_to_R(roll, pitch, yaw):
    return rotz(yaw) @ roty(pitch) @ rotx(roll)


def R_to_rpy(R):
    """Inverse of rpy_to_R. At pitch = ±90° roll is folded into yaw."""
    sy = -R[2, 0]
    if abs(sy) >= 1.0 - 1e-9:
        pitch = math.copysign(math.pi / 2, sy)
        roll = 0.0
        yaw = math.atan2(-R[0, 1], R[1, 1])
    else:
        pitch = math.asin(sy)
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def xyzrpy_to_pose(xyz, rpy):
    return pose(rpy_to_R(*rpy), xyz)


def rotvec(R):
    """
    Axis-angle vector (axis * angle) of R — the log map. Robust near 0 and
    near pi, which matters because IK error vectors pass through both.
    """
    cos_a = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    angle = math.acos(cos_a)
    if angle < 1e-9:
        # First-order: skew part of R
        return 0.5 * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if math.pi - angle < 1e-6:
        # Near pi the skew part vanishes; recover the axis from the symmetric part.
        M = (R + np.eye(3)) / 2.0
        i = int(np.argmax(np.diag(M)))
        axis = M[:, i] / math.sqrt(max(M[i, i], 1e-12))
        return axis * angle
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return w * (angle / (2.0 * math.sin(angle)))


def angle_between(R1, R2):
    """Geodesic angle (rad) between two rotations."""
    return float(np.linalg.norm(rotvec(R1 @ R2.T)))


def R_to_quat(R):
    """Unit quaternion (w, x, y, z)."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    return q / np.linalg.norm(q)


def quat_to_R(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def slerp(R0, R1, t):
    """Spherical interpolation between two rotations, t in [0, 1]."""
    q0, q1 = R_to_quat(R0), R_to_quat(R1)
    d = float(np.dot(q0, q1))
    if d < 0.0:                       # take the short way round
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + t * (q1 - q0)
    else:
        th = math.acos(d)
        q = (math.sin((1 - t) * th) * q0 + math.sin(t * th) * q1) / math.sin(th)
    return quat_to_R(q)
