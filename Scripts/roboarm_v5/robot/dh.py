"""
Denavit-Hartenberg table -> chain segments.

Each row: {"a": mm, "alpha": deg, "d": mm, "theta_offset": deg}
theta_offset is added to the joint angle, so the kinematic zero of a joint can
sit wherever the robot's natural "zero pose" is.

standard (distal) DH, per joint i:   Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)
modified (proximal, Craig) DH:       Rx(alpha_{i-1}) Tx(a_{i-1}) Rz(theta_i) Tz(d_i)
    — in the modified table each row's a/alpha are the ones *before* the joint.
"""

import math

from .model import Segment

Z = (0.0, 0.0, 1.0)


def _row(r, i):
    try:
        return (float(r.get("a", 0.0)), math.radians(float(r.get("alpha", 0.0))),
                float(r.get("d", 0.0)), math.radians(float(r.get("theta_offset", 0.0))))
    except (TypeError, ValueError) as e:
        raise ValueError("DH row {}: {}".format(i + 1, e))


def dh_segments(rows, convention="standard"):
    convention = (convention or "standard").lower()
    if convention not in ("standard", "modified"):
        raise ValueError("DH convention must be 'standard' or 'modified', not {!r}"
                         .format(convention))
    segs = []
    for i, r in enumerate(rows):
        a, alpha, d, off = _row(r, i)
        if convention == "standard":
            segs.append(Segment(pre=[("rz", off)], axis=Z,
                                post=[("tz", d), ("tx", a), ("rx", alpha)]))
        else:
            # Rz and Tz commute, so Tz(d) can sit with the pre-ops.
            segs.append(Segment(pre=[("rx", alpha), ("tx", a), ("rz", off), ("tz", d)],
                                axis=Z, post=[]))
    return segs
