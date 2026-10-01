"""
AGV packets, programs (save/load, V4 import, playback in the simulator),
config migration, and robot-file save-back.
    python3 -m unittest tests.test_io -v
"""

import json
import os
import shutil
import sys
import tempfile
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
ROBOTS = os.path.join(os.path.dirname(HERE), "robots")

from roboarm_v5.io import agv
from roboarm_v5.io.config import Config
from roboarm_v5.io.programs import Program, ProgramError
from roboarm_v5.log import Log
from roboarm_v5.robot.loader import load_robot, save_joint_tuning
from roboarm_v5.runtime import Runtime


class TestAGV(unittest.TestCase):
    def test_packet_layout_matches_v4(self):
        p = agv.pack(0.5, -1.0)
        self.assertEqual(len(p), 8)
        self.assertEqual(p[:3], b"EVA")
        self.assertEqual(p[3:7], bytes.fromhex("1388d8f0"))   # +5000, -10000
        self.assertEqual(p[7], agv.crc8(p[:7]))
        self.assertEqual(agv.pack(9, -9), agv.pack(1, -1))     # clamped

    def test_crc8_known_value(self):
        self.assertEqual(agv.crc8(b"123456789"), 0xF4)          # CRC-8/SMBUS check value


class TempDir(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp)


class TestPrograms(TempDir):
    def test_save_load_roundtrip_and_robot_guard(self):
        m = load_robot(os.path.join(ROBOTS, "roboarm_opi.json"))
        p = Program(m, Log())
        p.add_point([0, 0, 70, 0, 0, 0])
        p.add_point([10, -5, 60, 20, 30, 40], move_type="L")
        f = os.path.join(self.tmp, "p.json")
        self.assertTrue(p.save(f))
        q = Program(m, Log())
        q.load(f)
        self.assertEqual(q.points, p.points)
        self.assertEqual(q.problems(), [])
        small = load_robot(os.path.join(ROBOTS, "example_4dof.json"))
        with self.assertRaisesRegex(ProgramError, "6-joint"):
            Program(small, Log()).load(f)

    def test_v4_program_is_converted_through_the_step_model(self):
        m = load_robot(os.path.join(ROBOTS, "roboarm_opi.json"))
        # V4 measured from each switch: J2 (switch at min, -75°) 75° from it = 0°
        v4 = {"name": "OLD", "points": [{"index": 1, "label": "P[1]", "move_type": "J",
                                         "speed_pct": 50, "cnt": 0,
                                         "angles": [-175, 75, 70, 150, -75, 0],
                                         "tcp": [0, 0, 0], "comment": ""}]}
        f = os.path.join(self.tmp, "v4.json")
        json.dump(v4, open(f, "w"))
        log = Log()
        p = Program(m, log)
        p.load(f)
        np.testing.assert_allclose(p.points[0]["angles"], [0, 0, 70, 0, 0, 0], atol=1e-6)
        self.assertTrue(p.modified)
        self.assertTrue(any("CHECK EVERY POINT" in a for a in log.alarms))

    def test_out_of_limit_points_block_playback(self):
        m = load_robot(os.path.join(ROBOTS, "roboarm_opi.json"))
        p = Program(m, Log())
        p.add_point([0, 0, 170, 0, 0, 0])
        self.assertIn("outside joint limits", p.problems()[0])


class TestPlaybackSim(unittest.TestCase):
    def test_plays_joint_and_linear_points(self):
        rt = Runtime(os.path.join(ROBOTS, "roboarm_opi.json"), sim=True,
                     sim_start=[0, 0, 70, 0, 30, 0], kin_background=False, agv=False).start()
        try:
            rt.assume_homed_at([0, 0, 70, 0, 30, 0])
            rt.program.add_point([10, 5, 60, 0, 40, 0], speed_pct=100)
            rt.program.add_point([-10, 10, 50, 10, 30, -20], move_type="L", speed_pct=100)
            rt.playback.run(rt.program)
            np.testing.assert_allclose(rt.gpio.true_deg(), [-10, 10, 50, 10, 30, -20],
                                       atol=0.5)
        finally:
            rt.shutdown()


class TestConfig(TempDir):
    def test_v4_migration_copies_fan_and_agv_and_reports_the_rest(self):
        v4 = os.path.join(self.tmp, "config.json")
        json.dump({"fan": {"auto": False, "idle": 10, "t_lo": 40, "t_hi": 70},
                   "agv": {"ip": "10.0.0.9", "port": 6000},
                   "links": {"L1": 140}}, open(v4, "w"))
        log = Log()
        c = Config(log, path=os.path.join(self.tmp, "config_v5.json"), v4_path=v4).load()
        self.assertEqual(c["fan"]["idle"], 10)
        self.assertEqual(c["agv"]["ip"], "10.0.0.9")
        self.assertTrue(os.path.exists(os.path.join(self.tmp, "config_v5.json")))
        self.assertTrue(any("belong in the robot file" in s for s in log.system))
        self.assertEqual(json.load(open(v4))["links"], {"L1": 140})     # V4 untouched


class TestRobotSave(TempDir):
    def test_tuning_written_back_without_touching_geometry_or_notes(self):
        src = os.path.join(ROBOTS, "roboarm_opi.json")
        f = os.path.join(self.tmp, "r.json")
        shutil.copy(src, f)
        m = load_robot(f)
        m.joint("J3").steps_per_deg = 14.0
        m.joint("J3").limits_deg = (0.0, 130.0)
        save_joint_tuning(m)
        d = json.load(open(f))
        self.assertEqual(d["kinematics"], json.load(open(src))["kinematics"])
        self.assertIn("_verify", d)
        self.assertEqual(load_robot(f).joint("J3").limits_deg, (0.0, 130.0))


if __name__ == "__main__":
    unittest.main()
