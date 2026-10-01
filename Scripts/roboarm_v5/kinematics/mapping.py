"""
Motor steps  <->  kinematic angle.

The motor layer counts signed pulses: +1 for every pulse sent with the dir
pin high, -1 with it low, zeroed at the joint's datum (its limit switch, or
wherever it stood when zeroed in place). The kinematics only ever sees the
joint angle in kinematic degrees. This module is the single bridge between
the two — V4 had no such bridge, so its IK angles and motor angles differed
by each joint's home offset.

    kin_deg = datum_deg + sign * pulses / steps_per_deg
    sign    = -1 if invert_dir else +1

No numpy here: the motor and safety layers use it before numpy has loaded.
"""


def sign(joint):
    return -1.0 if joint.invert_dir else 1.0


def pulses_to_deg(joint, pulses):
    return joint.home.datum_deg + sign(joint) * pulses / joint.steps_per_deg


def deg_to_pulses(joint, deg):
    return sign(joint) * (deg - joint.home.datum_deg) * joint.steps_per_deg


def dir_level(joint, toward_positive):
    """dir pin level that moves the joint toward +angle (True) or -angle."""
    return int(bool(toward_positive) != bool(joint.invert_dir))


def level_is_positive(joint, level):
    """True if driving with this dir pin level increases the joint angle."""
    return bool(level) != bool(joint.invert_dir)


def homing_toward_positive(joint):
    """Direction (in angle terms) that drives a joint onto its switch."""
    if joint.home.switch is None:
        return None
    return joint.home.switch == "max"
