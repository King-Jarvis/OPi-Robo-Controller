"""
Kinematics tests — no hardware, no ikpy. Run from Scripts/:
    python3 -m unittest discover -s tests -v
"""

import math
import os
import sys
import time
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
ROBOTS = os.path.join(os.path.dirname(HERE), "robots")

from roboarm_v5.robot.loader import load_robot, load_robot_dict, RobotFileError
from roboarm_v5.robot.model import RobotModel, Joint, Segment
from roboarm_v5.kinematics.chain import Chain
from roboarm_v5.kinematics import ik, mapping
from roboarm_v5.kinematics.transforms import (
    rpy_to_R, R_to_rpy, rotvec, rot, slerp, angle_between, xyzrpy_to_pose)


def robot(name):
    return load_robot(os.path.join(ROBOTS, name))


def random_q(chain, rng, margin=0.02):
    span = chain.hi - chain.lo
    return rng.uniform(chain.lo + margin * span, chain.hi - margin * span)


class TestTransforms(unittest.TestCase):
    def test_rpy_roundtrip(self):
        rng = np.random.default_rng(1)
        for _ in range(200):
            rpy = rng.uniform([-3, -1.5, -3], [3, 1.5, 3])
            R = rpy_to_R(*rpy)
            self.assertLess(angle_between(R, rpy_to_R(*R_to_rpy(R))), 1e-9)

    def test_rotvec_including_near_pi(self):
        for ang in (0.0, 1e-7, 0.3, 2.0, math.pi - 1e-8, math.pi):
            axis = np.array([1.0, -2.0, 0.5]) / np.linalg.norm([1.0, -2.0, 0.5])
            w = rotvec(rot(axis, ang))
            self.assertAlmostEqual(np.linalg.norm(w), ang, places=6)
            if ang > 1e-3:
                self.assertLess(angle_between(rot(w, np.linalg.norm(w)), rot(axis, ang)), 1e-6)

    def test_slerp_endpoints_and_midpoint(self):
        R0, R1 = rpy_to_R(0, 0, 0), rpy_to_R(0, 0, 1.0)
        self.assertLess(angle_between(slerp(R0, R1, 0), R0), 1e-9)
        self.assertLess(angle_between(slerp(R0, R1, 1), R1), 1e-9)
        self.assertLess(angle_between(slerp(R0, R1, 0.5), rpy_to_R(0, 0, 0.5)), 1e-9)


class TestLoader(unittest.TestCase):
    def test_all_robot_files_load(self):
        for f in os.listdir(ROBOTS):
            if f.endswith(".json"):
                m = robot(f)
                self.assertEqual(len(m.segments), m.dof, f)

    def test_errors_are_specific(self):
        base = {"kinematics": {"type": "dh", "dh": [{"a": 10}]},
                "joints": [{"name": "A", "step_pin": 1, "dir_pin": 2,
                            "steps_per_deg": 1, "limits_deg": [-90, 90]}]}
        load_robot_dict(base)
        bad = dict(base, joints=[dict(base["joints"][0], limits_deg=[90, -90])])
        with self.assertRaisesRegex(RobotFileError, "joint A: limits_deg"):
            load_robot_dict(bad)
        bad = dict(base, joints=base["joints"] * 2)
        with self.assertRaisesRegex(RobotFileError, "describes 1 joints"):
            load_robot_dict(bad)
        bad = dict(base, joints=[dict(base["joints"][0], home={"switch": "max"})])
        with self.assertRaisesRegex(RobotFileError, "no limit_pin"):
            load_robot_dict(bad)
        bad = dict(base, joints=[dict(base["joints"][0], dir_pin=1)])
        with self.assertRaisesRegex(RobotFileError, "GPIO used twice"):
            load_robot_dict(bad)


class TestForwardKinematics(unittest.TestCase):
    """Hand-calculated poses for robots/roboarm_opi.json."""

    @classmethod
    def setUpClass(cls):
        cls.c = Chain(robot("roboarm_opi.json"))

    def tcp(self, *deg):
        T = self.c.fk(np.radians(deg))
        return T[:3, 3], T[:3, 2]

    def check(self, deg, pos, zdir):
        p, z = self.tcp(*deg)
        np.testing.assert_allclose(p, pos, atol=1e-6)
        np.testing.assert_allclose(z, zdir, atol=1e-9)

    def test_zero_pose_straight_up(self):
        # 136 + 250 + 235.25 + 11.5 + 7.5 = 640.25 up, elbow offset 26 forward
        self.check([0, 0, 0, 0, 0, 0], [26, 0, 640.25], [0, 0, 1])

    def test_elbow_90(self):
        # forearm horizontal: offset now points down, 254.25 of forearm+tool forward
        self.check([0, 0, 90, 0, 0, 0], [254.25, 0, 360], [1, 0, 0])

    def test_shoulder_90(self):
        self.check([0, 90, 0, 0, 0, 0], [504.25, 0, 110], [1, 0, 0])

    def test_wrist_90(self):
        self.check([0, 0, 0, 0, 90, 0], [26 + 19, 0, 621.25], [1, 0, 0])

    def test_base_90(self):
        self.check([90, 0, 0, 0, 0, 0], [0, 26, 640.25], [0, 0, 1])

    def test_forearm_roll_spins_wrist_about_forearm(self):
        # J4 is a roll about the forearm: with J5 bent, J4=90 swings the tool
        # from +X to +Y without moving the wrist centre. (V4 got this axis wrong.)
        self.check([0, 0, 0, 90, 90, 0], [26, 19, 621.25], [0, 1, 0])

    def test_dh_and_urdf_agree(self):
        u = Chain(robot("roboarm_opi_urdf.json"))
        rng = np.random.default_rng(2)
        for _ in range(200):
            q = random_q(self.c, rng)
            np.testing.assert_allclose(self.c.fk(q), u.fk(q), atol=1e-6)

    def test_jacobian_matches_finite_difference(self):
        rng = np.random.default_rng(3)
        for _ in range(20):
            q = random_q(self.c, rng)
            J, T = self.c.jacobian(q)
            h = 1e-6
            for i in range(6):
                dq = np.zeros(6); dq[i] = h
                T2 = self.c.fk(q + dq)
                np.testing.assert_allclose((T2[:3, 3] - T[:3, 3]) / h, J[:3, i], atol=1e-3)
                np.testing.assert_allclose(rotvec(T2[:3, :3] @ T[:3, :3].T) / h, J[3:, i],
                                           atol=1e-5)


class TestInverseKinematics(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.m = robot("roboarm_opi.json")
        cls.c = Chain(cls.m)

    def roundtrip(self, chain, n, mode, seed_from, tol_mm=0.1, tol_deg=0.1, rng_seed=10):
        rng = np.random.default_rng(rng_seed)
        fails, worst_ms = [], 0.0
        for k in range(n):
            q = random_q(chain, rng)
            T = chain.fk(q)
            if seed_from == "near":
                seed = np.clip(q + rng.normal(0, 0.15, chain.n), chain.lo, chain.hi)
            else:
                seed = None
            t0 = time.perf_counter()
            s = ik.solve(chain, T, seed, mode=mode, pos_tol_mm=tol_mm, ori_tol_deg=tol_deg)
            worst_ms = max(worst_ms, 1000 * (time.perf_counter() - t0))
            if not s.ok:
                fails.append((k, s.reason))
                continue
            self.assertTrue(np.all(s.q >= chain.lo - 1e-9) and np.all(s.q <= chain.hi + 1e-9))
            T2 = chain.fk(s.q)
            self.assertLess(np.linalg.norm(T2[:3, 3] - T[:3, 3]), tol_mm + 1e-9)
            if mode == "pose":
                self.assertLess(math.degrees(angle_between(T2[:3, :3], T[:3, :3])), tol_deg + 1e-9)
        return fails, worst_ms

    def test_pose_roundtrip_from_nearby_seed(self):
        # the normal case: target is close to where the arm already is
        fails, ms = self.roundtrip(self.c, 500, "pose", "near")
        self.assertEqual(fails, [])

    def test_pose_roundtrip_from_mid_range(self):
        # the hard case: no useful seed at all
        fails, ms = self.roundtrip(self.c, 200, "pose", "mid")
        self.assertLessEqual(len(fails), 2, fails)

    def test_axis_mode(self):
        fails, _ = self.roundtrip(self.c, 200, "axis", "near")
        self.assertEqual(fails, [])

    def test_prefers_solution_near_current_pose(self):
        q = np.radians([10, 20, 60, 30, 40, -20])
        s = ik.solve(self.c, self.c.fk(q), q + 0.05)
        self.assertTrue(s.ok)
        np.testing.assert_allclose(s.q, q, atol=1e-2)

    def test_unreachable_reports_why(self):
        T = xyzrpy_to_pose([2000, 0, 300], [0, 0, 0])
        s = ik.solve(self.c, T, np.zeros(6), extra_seeds=2)
        self.assertFalse(s.ok)
        self.assertIn("unreachable", s.reason)

    def test_four_dof_position_mode(self):
        c = Chain(robot("example_4dof.json"))
        self.assertEqual(ik.auto_mode(c.n), "position")
        fails, _ = self.roundtrip(c, 200, "auto", "near")
        self.assertEqual(fails, [])

    def test_seven_dof_redundant(self):
        # bolt an extra roll joint onto the flange: 7 joints, 6 constraints
        m = self.m
        segs = m.segments + [Segment(pre=[("tz", 30.0)], axis=(0, 0, 1))]
        joints = m.joints + [Joint("J7", 1, 2, None, 1.0, 1000, (-170, 170))]
        m7 = RobotModel("seven", joints, segs, tool=m.tool)
        c = Chain(m7)
        fails, _ = self.roundtrip(c, 100, "pose", "near")
        self.assertEqual(fails, [])


class TestMapping(unittest.TestCase):
    def test_roundtrip_and_sign(self):
        m = robot("roboarm_opi.json")
        for j in m.joints:
            for inv in (False, True):
                j.invert_dir = inv
                for deg in (j.limits_deg[0], 0.0, j.limits_deg[1]):
                    self.assertAlmostEqual(
                        mapping.pulses_to_deg(j, mapping.deg_to_pulses(j, deg)), deg)
                # one pulse with the "positive" dir level must increase the angle
                lvl = mapping.dir_level(j, True)
                p = 1.0 if lvl == 1 else -1.0
                self.assertGreater(mapping.pulses_to_deg(j, p), mapping.pulses_to_deg(j, 0))

    def test_datum_is_where_the_switch_is(self):
        j1 = robot("roboarm_opi.json").joint("J1")
        self.assertEqual(mapping.pulses_to_deg(j1, 0), 175.0)
        self.assertTrue(mapping.homing_toward_positive(j1))
        # park (0 deg) is reached by backing *away* from the switch
        self.assertLess(mapping.deg_to_pulses(j1, 0.0), 0)


if __name__ == "__main__":
    unittest.main()
