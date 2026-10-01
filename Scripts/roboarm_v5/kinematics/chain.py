"""
Forward kinematics and the geometric Jacobian for any serial revolute chain.

Built once from a RobotModel; every call takes joint angles in radians, in
kinematic convention (see kinematics/mapping.py for motor <-> kinematic).
"""

import numpy as np

from .transforms import rot, pose, trans, rotx, roty, rotz, xyzrpy_to_pose


def ops_to_pose(ops):
    T = np.eye(4)
    for op in ops:
        k = op[0]
        if k == "tx":   M = trans(x=op[1])
        elif k == "ty": M = trans(y=op[1])
        elif k == "tz": M = trans(z=op[1])
        elif k == "rx": M = pose(rotx(op[1]))
        elif k == "ry": M = pose(roty(op[1]))
        elif k == "rz": M = pose(rotz(op[1]))
        elif k == "xyzrpy": M = xyzrpy_to_pose(op[1], op[2])
        else:
            raise ValueError("unknown chain op {!r}".format(k))
        T = T @ M
    return T


class Chain:
    def __init__(self, model):
        self.model = model
        self.n = model.dof
        self.base = ops_to_pose(model.base)
        self.tool = ops_to_pose(model.tool)
        self.pre = [ops_to_pose(s.pre) for s in model.segments]
        self.post = [ops_to_pose(s.post) for s in model.segments]
        self.axes = [np.asarray(s.axis, float) / np.linalg.norm(s.axis)
                     for s in model.segments]
        lims = np.radians(np.array([j.limits_deg for j in model.joints], float))
        self.lo, self.hi = lims[:, 0], lims[:, 1]

    def frames(self, q):
        """
        Walk the chain. Returns (joint_origins, joint_axes, T_tcp), all in the
        world frame; origins/axes are where each joint sits *before* it turns.
        """
        T = self.base.copy()
        origins = np.empty((self.n, 3))
        axes = np.empty((self.n, 3))
        for i in range(self.n):
            T = T @ self.pre[i]
            origins[i] = T[:3, 3]
            axes[i] = T[:3, :3] @ self.axes[i]
            T = T @ pose(rot(self.axes[i], q[i])) @ self.post[i]
        return origins, axes, T @ self.tool

    def fk(self, q):
        """TCP pose (4x4) for joint angles q (rad)."""
        return self.frames(q)[2]

    def flange(self, q):
        """Pose without the tool offset."""
        return self.fk(q) @ np.linalg.inv(self.tool)

    def jacobian(self, q):
        """
        6xN geometric Jacobian at the TCP: rows 0-2 linear (mm/rad),
        rows 3-5 angular (rad/rad). Closed form, no finite differences.
        """
        origins, axes, T = self.frames(q)
        p = T[:3, 3]
        J = np.empty((6, self.n))
        J[:3] = np.cross(axes, p - origins).T
        J[3:] = axes.T
        return J, T
