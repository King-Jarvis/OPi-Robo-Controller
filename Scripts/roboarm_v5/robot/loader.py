"""
Robot file (JSON) -> RobotModel, with validation that says what is wrong.

Keys starting with "_" are ignored anywhere, so they can be used as notes.
See robots/roboarm_opi.json for a complete, commented example.
"""

import json
import math
import os

from .model import RobotModel, Joint, Home, Board
from .dh import dh_segments
from .urdf import parse_urdf

MAX_DOF = 7


class RobotFileError(ValueError):
    pass


def _strip_notes(o):
    if isinstance(o, dict):
        return {k: _strip_notes(v) for k, v in o.items() if not k.startswith("_")}
    if isinstance(o, list):
        return [_strip_notes(v) for v in o]
    return o


def _frame_ops(d, what):
    """{"xyz": [mm], "rpy_deg": [deg]} -> [("xyzrpy", ...)]"""
    if not d:
        return []
    try:
        xyz = tuple(float(v) for v in d.get("xyz", (0, 0, 0)))
        rpy = tuple(math.radians(float(v)) for v in d.get("rpy_deg", (0, 0, 0)))
    except (TypeError, ValueError) as e:
        raise RobotFileError("{}: {}".format(what, e))
    if len(xyz) != 3 or len(rpy) != 3:
        raise RobotFileError("{}: xyz and rpy_deg need 3 values each".format(what))
    return [("xyzrpy", xyz, rpy)]


def _joint(d, i, urdf_limits=None):
    where = "joints[{}]".format(i)
    try:
        name = str(d.get("name", "J{}".format(i + 1)))
        where = "joint {}".format(name)
        lim = d.get("limits_deg", urdf_limits)
        if lim is None:
            raise RobotFileError("{}: limits_deg is required".format(where))
        lo, hi = float(lim[0]), float(lim[1])
        if not lo < hi:
            raise RobotFileError("{}: limits_deg must be [low, high]".format(where))
        h = d.get("home", {}) or {}
        switch = h.get("switch")
        if switch not in (None, "min", "max"):
            raise RobotFileError("{}: home.switch must be \"min\", \"max\" or null"
                                 .format(where))
        default_datum = {"min": lo, "max": hi, None: 0.0}[switch]
        home = Home(switch=switch,
                    datum_deg=float(h.get("datum_deg", default_datum)),
                    park_deg=float(h.get("park_deg", 0.5 * (lo + hi) if switch else
                                         h.get("datum_deg", 0.0))))
        limit_pin = d.get("limit_pin")
        if switch and limit_pin is None:
            raise RobotFileError("{}: home.switch is set but there is no limit_pin"
                                 .format(where))
        j = Joint(name=name,
                  step_pin=int(d["step_pin"]), dir_pin=int(d["dir_pin"]),
                  limit_pin=None if limit_pin is None else int(limit_pin),
                  steps_per_deg=float(d["steps_per_deg"]),
                  max_freq=float(d.get("max_freq", 2000)),
                  limits_deg=(lo, hi), home=home,
                  invert_dir=bool(d.get("invert_dir", False)),
                  speed=int(d.get("speed", 30)))
    except KeyError as e:
        raise RobotFileError("{}: missing {}".format(where, e))
    except (TypeError, ValueError) as e:
        if isinstance(e, RobotFileError):
            raise
        raise RobotFileError("{}: {}".format(where, e))
    if j.steps_per_deg <= 0 or j.max_freq <= 0:
        raise RobotFileError("{}: steps_per_deg and max_freq must be positive".format(where))
    if not lo - 1e-6 <= home.park_deg <= hi + 1e-6:
        raise RobotFileError("{}: park_deg {} is outside limits".format(where, home.park_deg))
    return j


def load_robot_dict(d, base_dir="."):
    d = _strip_notes(d)
    kin = d.get("kinematics")
    if not isinstance(kin, dict):
        raise RobotFileError("missing \"kinematics\" section")
    jdefs = d.get("joints")
    if not isinstance(jdefs, list) or not jdefs:
        raise RobotFileError("missing \"joints\" list")

    ktype = kin.get("type", "dh").lower()
    tool = []
    urdf_limits = [None] * len(jdefs)
    name = d.get("name")
    if ktype == "dh":
        rows = kin.get("dh")
        if not isinstance(rows, list):
            raise RobotFileError("kinematics.dh must be a list of rows")
        try:
            segments = dh_segments(rows, kin.get("convention", "standard"))
        except ValueError as e:
            raise RobotFileError(str(e))
    elif ktype == "urdf":
        f = kin.get("file")
        if not f:
            raise RobotFileError("kinematics.file is required for type urdf")
        path = f if os.path.isabs(f) else os.path.join(base_dir, f)
        try:
            segments, tool, ujoints, uname = parse_urdf(path, kin.get("base"), kin.get("tip"))
        except (OSError, ValueError) as e:
            raise RobotFileError("URDF {}: {}".format(f, e))
        urdf_limits = [u.limits_deg for u in ujoints]
        name = name or uname
    else:
        raise RobotFileError("kinematics.type must be \"dh\" or \"urdf\"")

    if len(segments) != len(jdefs):
        raise RobotFileError("kinematics describes {} joints but \"joints\" lists {}"
                             .format(len(segments), len(jdefs)))
    if not 1 <= len(jdefs) <= MAX_DOF:
        raise RobotFileError("{} joints: 1 to {} are supported".format(len(jdefs), MAX_DOF))

    joints = [_joint(jd, i, urdf_limits[i] if i < len(urdf_limits) else None)
              for i, jd in enumerate(jdefs)]
    names = [j.name for j in joints]
    if len(set(names)) != len(names):
        raise RobotFileError("joint names must be unique: {}".format(names))

    b = d.get("board", {}) or {}
    board = Board(gpiochip=int(b.get("gpiochip", 0)),
                  estop_pin=b.get("estop_pin"), fan_pin=b.get("fan_pin"),
                  fan_pwm_freq=int(b.get("fan_pwm_freq", 1000)),
                  limit_triggered=int(b.get("limit_triggered", 1)),
                  estop_triggered=int(b.get("estop_triggered", 1)))
    pins = []
    for j in joints:
        pins += [j.step_pin, j.dir_pin] + ([j.limit_pin] if j.limit_pin is not None else [])
    pins += [p for p in (board.estop_pin, board.fan_pin) if p is not None]
    dup = sorted({p for p in pins if pins.count(p) > 1})
    if dup:
        raise RobotFileError("GPIO used twice: {}".format(dup))

    ik = d.get("ik", {}) or {}
    return RobotModel(
        name=name or "robot", joints=joints, segments=segments,
        base=_frame_ops(d.get("base"), "base"),
        tool=tool + _frame_ops(d.get("tool"), "tool"),
        board=board, kin_type=ktype,
        ik_pos_tol_mm=float(ik.get("pos_tol_mm", 0.5)),
        ik_ori_tol_deg=float(ik.get("ori_tol_deg", 0.5)))


def save_joint_tuning(model, path=None):
    """
    Write per-joint tuning edited on the HMI back into the robot file,
    leaving everything else in it (geometry, notes, layout of keys) alone.
    """
    path = path or model.source
    with open(path) as f:
        d = json.load(f)
    by_name = {j.name: j for j in model.joints}
    for i, jd in enumerate(d.get("joints", [])):
        j = by_name.get(jd.get("name", "J{}".format(i + 1)))
        if j is None:
            continue
        jd["steps_per_deg"] = j.steps_per_deg
        jd["max_freq"] = j.max_freq
        jd["limits_deg"] = list(j.limits_deg)
        jd["invert_dir"] = j.invert_dir
        jd["speed"] = j.speed
        jd.setdefault("home", {})["park_deg"] = j.home.park_deg
        jd["home"]["datum_deg"] = j.home.datum_deg
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(d, f, indent=2)
    os.replace(tmp, path)


def load_robot(path):
    try:
        with open(path) as f:
            d = json.load(f)
    except json.JSONDecodeError as e:
        raise RobotFileError("{}: line {} col {}: {}".format(path, e.lineno, e.colno, e.msg))
    m = load_robot_dict(d, base_dir=os.path.dirname(os.path.abspath(path)))
    m.source = os.path.abspath(path)
    return m
