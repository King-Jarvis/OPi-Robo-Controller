"""
URDF -> chain segments, with the standard library's XML parser only.

For people who would rather export the arm from CAD (Fusion 360, SolidWorks,
Onshape all have URDF exporters) than work out a DH table by hand.

Supported: revolute and continuous joints along one unbranched path from
`base` to `tip`. Fixed joints are folded into the next segment (or into the
flange if they come last). Prismatic, planar and floating joints are refused.
URDF lengths are metres; they are converted to mm here.
"""

import math
import xml.etree.ElementTree as ET

from .model import Segment

M_TO_MM = 1000.0


def _floats(s, n, default=0.0):
    if s is None:
        return tuple([default] * n)
    v = [float(x) for x in s.split()]
    if len(v) != n:
        raise ValueError("expected {} numbers, got {!r}".format(n, s))
    return tuple(v)


def _origin_op(joint_el):
    o = joint_el.find("origin")
    xyz = _floats(o.get("xyz") if o is not None else None, 3)
    rpy = _floats(o.get("rpy") if o is not None else None, 3)
    return ("xyzrpy", tuple(c * M_TO_MM for c in xyz), rpy)


class UrdfJoint:
    def __init__(self, name, limits_deg):
        self.name = name
        self.limits_deg = limits_deg      # None if the URDF gives none


def parse_urdf(path_or_text, base=None, tip=None):
    """
    Returns (segments, tail_ops, joints, robot_name).
    tail_ops are the fixed transforms after the last moving joint (flange).
    """
    if path_or_text.lstrip().startswith("<"):
        root = ET.fromstring(path_or_text)
    else:
        root = ET.parse(path_or_text).getroot()
    if root.tag != "robot":
        raise ValueError("not a URDF: root element is <{}>".format(root.tag))

    by_parent = {}
    children = set()
    links = {l.get("name") for l in root.findall("link")}
    for j in root.findall("joint"):
        p = j.find("parent").get("link")
        c = j.find("child").get("link")
        by_parent.setdefault(p, []).append(j)
        children.add(c)

    if base is None:
        roots = sorted(links - children)
        if len(roots) != 1:
            raise ValueError("URDF has {} root links {}; set 'base'".format(len(roots), roots))
        base = roots[0]
    if base not in links:
        raise ValueError("base link {!r} not in URDF".format(base))

    segments, joints = [], []
    pending = []                          # fixed ops waiting for the next joint
    link = base
    while link != tip:
        nxt = by_parent.get(link, [])
        if not nxt:
            if tip is None:
                break
            raise ValueError("tip link {!r} not reachable from {!r}".format(tip, base))
        if len(nxt) > 1:
            if tip is None:
                raise ValueError("URDF branches at {!r}; set 'tip'".format(link))
            nxt = [j for j in nxt if _reaches(by_parent, j.find("child").get("link"), tip)]
            if len(nxt) != 1:
                raise ValueError("cannot find a unique path to {!r} at {!r}".format(tip, link))
        j = nxt[0]
        jtype = j.get("type")
        op = _origin_op(j)
        if jtype == "fixed":
            pending.append(op)
        elif jtype in ("revolute", "continuous"):
            ax = j.find("axis")
            axis = _floats(ax.get("xyz") if ax is not None else None, 3)
            if ax is None:
                axis = (1.0, 0.0, 0.0)    # URDF default
            lim = None
            le = j.find("limit")
            if jtype == "revolute" and le is not None and le.get("lower") is not None:
                lim = (math.degrees(float(le.get("lower"))),
                       math.degrees(float(le.get("upper"))))
            segments.append(Segment(pre=pending + [op], axis=axis, post=[]))
            joints.append(UrdfJoint(j.get("name"), lim))
            pending = []
        else:
            raise ValueError("joint {!r}: {} joints are not supported"
                             .format(j.get("name"), jtype))
        link = j.find("child").get("link")

    if not segments:
        raise ValueError("no revolute joints between {!r} and {!r}".format(base, tip or link))
    return segments, pending, joints, root.get("name", "")


def _reaches(by_parent, link, tip):
    if link == tip:
        return True
    return any(_reaches(by_parent, j.find("child").get("link"), tip)
               for j in by_parent.get(link, []))
