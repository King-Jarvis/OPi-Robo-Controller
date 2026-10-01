"""
The kinematics everyone else talks to — degrees and mm in, degrees and mm out.

numpy costs seconds to import on an H618, so it is loaded on a worker thread:
the HMI paints and the E-STOP arms immediately, and anything that needs the
solver checks `ready` or degrades gracefully (V4 :684-741 did the same).
This module itself must not import numpy at the top.
"""

import math
import threading
import time
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class TcpPose:
    xyz: List[float]                      # mm
    rpy_deg: List[float]                  # URDF roll/pitch/yaw, degrees
    T: object                             # 4x4 numpy array


class Kinematics:
    def __init__(self, model, log, background=True):
        self.model = model
        self.log = log
        self.ready = threading.Event()
        self.error = None
        self.chain = None
        self._lock = threading.Lock()
        self._np = self._ik = self._tf = None
        if background:
            threading.Thread(target=self._load, name="kin-load", daemon=True).start()
        else:
            self._load()

    def _load(self):
        t0 = time.monotonic()
        try:
            import numpy
            from . import chain, ik, transforms
            self._np, self._ik, self._tf = numpy, ik, transforms
            self.chain = chain.Chain(self.model)
            self.ready.set()
            self.log("Solver ready ({:.1f}s)".format(time.monotonic() - t0))
        except Exception as e:
            self.error = str(e)
            self.log("Solver unavailable: {}".format(e), alarm=True)

    def set_model(self, model):
        self.model = model
        if not self.ready.is_set():
            return
        from .chain import Chain
        with self._lock:
            self.chain = Chain(model)
        self.log("Kinematic chain rebuilt for {}".format(model.name))

    # ── conversions ──────────────────────────────────────────────────────
    def pose(self, xyz, rpy_deg):
        return self._tf.xyzrpy_to_pose(xyz, [math.radians(a) for a in rpy_deg])

    def describe(self, T):
        rpy = self._tf.R_to_rpy(T[:3, :3])
        return TcpPose([float(v) for v in T[:3, 3]],
                       [math.degrees(a) for a in rpy], T)

    # ── forward / inverse ────────────────────────────────────────────────
    def fk(self, angles_deg) -> Optional[TcpPose]:
        """TCP pose, or None while the solver is still loading."""
        if not self.ready.is_set():
            return None
        with self._lock:
            T = self.chain.fk(self._np.radians(angles_deg))
        return self.describe(T)

    def ik(self, T, seed_deg=None, mode="auto", quick=False):
        """
        Solve for TCP pose T (4x4). quick=True tries only the seed — for
        jogging, where a slow failure would stall the pendant.
        Returns a kinematics.ik.Solution (use .deg), or None if not loaded.
        """
        if not self.ready.is_set():
            return None
        seed = None if seed_deg is None else self._np.radians(seed_deg)
        with self._lock:
            sol = self._ik.solve(self.chain, T, seed, mode=mode,
                                 extra_seeds=0 if quick else 10)
        return sol

    def ik_xyz(self, xyz, rpy_deg=None, seed_deg=None, mode="auto", quick=False):
        """
        Solve for a position and (optionally) orientation. With rpy_deg None
        the orientation at the seed pose is held (or ignored in position mode).
        """
        if not self.ready.is_set():
            return None
        if rpy_deg is None:
            cur = self.fk(seed_deg if seed_deg is not None else self.model.mid_pose())
            T = cur.T.copy()
            T[:3, 3] = xyz
        else:
            T = self.pose(xyz, rpy_deg)
        return self.ik(T, seed_deg, mode, quick)
