"""
Cartesian motion: jogging in the world or tool frame, and straight-line moves.

V4 passed the J4-J6 *joint* angles in as Rx/Ry/Rz (Euler) targets and built
the tool frame from J5/J6 alone. Here every jog starts from the real TCP pose
(FK of where the arm is) and is applied as a proper rigid motion:

  world frame   translate along world axes, rotate about world axes through
                the TCP:   T' = (Δp + p, ΔR·R)
  tool frame    translate/rotate about the tool's own axes:   T' = T·ΔT

numpy is imported lazily (see kinematics/service.py).
"""

import math
import time


class Cartesian:
    SEG_MM = 5.0                          # straight-line waypoint spacing
    SEG_DEG = 2.0
    MAX_WAYPOINT_JUMP_DEG = 20.0          # per 5 mm / 2° waypoint
    JOG_CHAIN_S = 0.25                    # jogs closer than this build on each other
    JOG_MAX_LEAD_MM = 25.0                # ...but never get further than this ahead
    JOG_MAX_LEAD_DEG = 8.0

    def __init__(self, bank, mover, kin, log):
        self.bank = bank
        self.mover = mover
        self.kin = kin
        self.log = log
        self._last_fail = 0.0
        self._chain = None                # (commanded TCP pose, its joint solution)
        self._chain_t = 0.0

    def _ready(self):
        if not self.bank.all_homed:
            return False
        return self.kin.ready.is_set()

    def _say_once(self, msg):
        # jogging calls this ~10x/s; don't flood the log with one complaint
        now = time.monotonic()
        if now - self._last_fail > 2.0:
            self.log(msg)
            self._last_fail = now

    def jog_target(self, q_deg, d_mm=(0, 0, 0), d_deg=(0, 0, 0), frame="world"):
        """The TCP pose a jog from q_deg would aim for."""
        return self._apply(self.kin.fk(q_deg).T, d_mm, d_deg, frame)

    @staticmethod
    def _apply(T, d_mm, d_deg, frame):
        import numpy as np
        from ..kinematics.transforms import rpy_to_R
        T = T.copy()
        dR = rpy_to_R(*[math.radians(a) for a in d_deg])
        d = np.asarray(d_mm, float)
        if frame == "tool":
            T[:3, 3] += T[:3, :3] @ d
            T[:3, :3] = T[:3, :3] @ dR
        else:
            T[:3, 3] += d
            T[:3, :3] = dR @ T[:3, :3]
        return T

    def jog(self, d_mm=(0, 0, 0), d_deg=(0, 0, 0), frame="world", wait=False):
        """
        One jog increment. Returns True if a move was started.

        While jogs arrive back to back (a held button or stick), each one is
        applied to the previous jog's *commanded* pose, not to wherever the
        arm happens to be mid-move. Otherwise every increment inherits the
        last one's path error and a held Z jog wanders off in X/Y.
        """
        if not self._ready():
            return False
        now = time.monotonic()
        if self._chain is not None and now - self._chain_t < self.JOG_CHAIN_S:
            base_T, q = self._chain
            # don't let the commanded pose run away from a slow arm
            import numpy as np
            from ..kinematics.transforms import angle_between
            actual = self.kin.fk(self.bank.angles()).T
            lead_mm = float(np.linalg.norm(base_T[:3, 3] - actual[:3, 3]))
            lead_deg = math.degrees(angle_between(base_T[:3, :3], actual[:3, :3]))
            if lead_mm > self.JOG_MAX_LEAD_MM or lead_deg > self.JOG_MAX_LEAD_DEG:
                return False
        else:
            q = self.bank.angles()
            base_T = self.kin.fk(q).T
        T = self._apply(base_T, d_mm, d_deg, frame)
        sol = self.kin.ik(T, q, quick=True)
        if sol is None or not sol.ok:
            self._chain = None
            self._say_once("Jog: {}".format(sol.reason if sol else "solver loading"))
            return False
        self._chain, self._chain_t = (T, sol.deg), now
        self.mover.move_all(sol.deg, wait=wait)
        return True

    def linear(self, target, speed=None):
        """
        Straight line from here to `target` — joint angles (list, degrees) or
        a 4x4 TCP pose. Position is interpolated linearly, orientation by
        slerp, each waypoint solved from the previous one. An unreachable
        waypoint stops the move (V4 silently switched to joint interpolation
        halfway along, so the path was no longer the one asked for).
        """
        if not self._ready():
            self.log("Linear move needs a homed arm and a loaded solver", alarm=True)
            return False
        import numpy as np
        from ..kinematics.transforms import slerp, angle_between

        q = self.bank.angles()
        T0 = self.kin.fk(q).T
        final_q = None
        if isinstance(target, (list, tuple)):
            final_q = list(target)
            T1 = self.kin.fk(final_q).T
        else:
            T1 = np.asarray(target, float)
        dist = float(np.linalg.norm(T1[:3, 3] - T0[:3, 3]))
        turn = math.degrees(angle_between(T0[:3, :3], T1[:3, :3]))
        n = max(1, int(math.ceil(max(dist / self.SEG_MM, turn / self.SEG_DEG))))

        for k in range(1, n + 1):
            if self.bank.estop_active:
                return False
            t = k / n
            Tk = np.eye(4)
            Tk[:3, 3] = T0[:3, 3] + (T1[:3, 3] - T0[:3, 3]) * t
            Tk[:3, :3] = slerp(T0[:3, :3], T1[:3, :3], t)
            if k == n and final_q is not None:
                way = final_q
            else:
                sol = self.kin.ik(Tk, q, quick=True)
                if sol is None or not sol.ok:
                    sol = self.kin.ik(Tk, q)          # retry with every seed
                why = None
                if sol is None or not sol.ok:
                    why = sol.reason if sol else "solver loading"
                else:
                    # a far-seed answer may be a different arm configuration
                    # (wrist flipped); jumping to it is not a straight line
                    jump = max(abs(a - b) for a, b in zip(sol.deg, q))
                    if jump > self.MAX_WAYPOINT_JUMP_DEG:
                        why = "configuration change ({:.0f}° joint jump)".format(jump)
                if why:
                    self.log("Linear move stopped at {:.0f}%: {}".format(
                        100 * (k - 1) / n, why), alarm=True)
                    return False
                way = sol.deg
            if not self.mover.move_all(way, wait=True, speed=speed):
                if self.bank.estop_active:
                    return False
            q = way
        return True
