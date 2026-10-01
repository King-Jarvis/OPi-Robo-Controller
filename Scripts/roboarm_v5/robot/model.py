"""
The robot description: the one place that says what this arm is.

Everything here is plain Python — no numpy — so the HMI and the E-STOP can
come up before the (slow, on an H618) numpy import finishes. The kinematic
chain is kept as a list of elementary operations; kinematics/chain.py turns
it into matrices once numpy is available.

Elementary ops (lengths mm, angles rad):
    ("tx", v) ("ty", v) ("tz", v)        translate along a local axis
    ("rx", a) ("ry", a) ("rz", a)        rotate about a local axis
    ("xyzrpy", (x, y, z), (r, p, y))     URDF-style origin
A Segment is   pre-ops · rotate(axis, q) · post-ops.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

Op = tuple


@dataclass
class Segment:
    """One revolute joint and the rigid geometry around it."""
    pre: List[Op]
    axis: Tuple[float, float, float]
    post: List[Op] = field(default_factory=list)


@dataclass
class Home:
    """
    How a joint finds its datum.

    switch     "min" or "max": which end of travel the limit switch sits at.
               None: no switch — the joint is zeroed where it stands.
    datum_deg  kinematic angle at which the switch fires (or the angle the
               joint is assumed to be at when zeroed in place).
    park_deg   kinematic angle to move to once the datum is found.
    """
    switch: Optional[str] = None
    datum_deg: float = 0.0
    park_deg: float = 0.0


@dataclass
class Joint:
    name: str
    step_pin: int
    dir_pin: int
    limit_pin: Optional[int]
    steps_per_deg: float
    max_freq: float                       # step rate (Hz) at 100% speed
    limits_deg: Tuple[float, float]       # kinematic degrees — the only limits
    home: Home = field(default_factory=Home)
    invert_dir: bool = False              # dir pin high drives the angle negative
    speed: int = 30                       # default jog speed, % of max_freq

    @property
    def span_deg(self):
        return self.limits_deg[1] - self.limits_deg[0]

    @property
    def mid_deg(self):
        return 0.5 * (self.limits_deg[0] + self.limits_deg[1])


@dataclass
class Board:
    gpiochip: int = 0
    estop_pin: Optional[int] = None
    fan_pin: Optional[int] = None
    fan_pwm_freq: int = 1000
    limit_triggered: int = 1              # NC switches to ground + pull-up
    estop_triggered: int = 1


@dataclass
class RobotModel:
    name: str
    joints: List[Joint]
    segments: List[Segment]               # one per joint, same order
    base: List[Op] = field(default_factory=list)
    tool: List[Op] = field(default_factory=list)
    board: Board = field(default_factory=Board)
    source: str = ""                      # file this came from
    kin_type: str = "dh"                  # "dh" | "urdf"
    # IK acceptance thresholds
    ik_pos_tol_mm: float = 0.5
    ik_ori_tol_deg: float = 0.5

    @property
    def dof(self):
        return len(self.joints)

    @property
    def names(self):
        return [j.name for j in self.joints]

    def joint(self, name):
        for j in self.joints:
            if j.name == name:
                return j
        raise KeyError(name)

    def index(self, name):
        return self.names.index(name)

    def park_pose(self):
        return [j.home.park_deg for j in self.joints]

    def mid_pose(self):
        return [j.mid_deg for j in self.joints]

    def within_limits(self, angles_deg, eps=1e-6):
        return all(j.limits_deg[0] - eps <= a <= j.limits_deg[1] + eps
                   for j, a in zip(self.joints, angles_deg))

    def clamp(self, angles_deg):
        return [min(max(a, j.limits_deg[0]), j.limits_deg[1])
                for j, a in zip(self.joints, angles_deg)]
