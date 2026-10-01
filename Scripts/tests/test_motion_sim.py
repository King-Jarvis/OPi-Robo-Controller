"""
Motor, safety, homing and motion tests against the simulated board (SimGPIO).
Real threads and real time, so these take a few seconds.
    python3 -m unittest tests.test_motion_sim -v
"""

import os
import sys
import threading
import time
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
ROBOTS = os.path.join(os.path.dirname(HERE), "robots")

from roboarm_v5.robot.loader import load_robot_dict, load_robot
from roboarm_v5.runtime import Runtime
from roboarm_v5.log import Log

# Two fast joints: A homes to a switch at its max end, B to a switch at its
# min end and has its dir pin wired backwards.
TOY = {
    "name": "toy",
    "kinematics": {"type": "dh", "dh": [{"a": 100}, {"a": 100}]},
    "joints": [
        {"name": "A", "step_pin": 1, "dir_pin": 2, "limit_pin": 3,
         "steps_per_deg": 10, "max_freq": 4000, "limits_deg": [-30, 30],
         "home": {"switch": "max", "park_deg": 0}},
        {"name": "B", "step_pin": 4, "dir_pin": 5, "limit_pin": 6, "invert_dir": True,
         "steps_per_deg": 10, "max_freq": 4000, "limits_deg": [-20, 40],
         "home": {"switch": "min", "park_deg": 10}},
    ],
    "board": {"estop_pin": 9, "fan_pin": 10},
}


def wait_for(cond, timeout=5.0):
    end = time.monotonic() + timeout
    while not cond():
        if time.monotonic() > end:
            return False
        time.sleep(0.005)
    return True


class SimCase(unittest.TestCase):
    model_dict = TOY
    start = None

    def setUp(self):
        model = (load_robot_dict(self.model_dict) if isinstance(self.model_dict, dict)
                 else load_robot(self.model_dict))
        self.rt = Runtime(model, sim=True, sim_start=self.start,
                          log=Log(echo=bool(os.environ.get("V5_ECHO"))),
                          kin_background=False, agv=False).start()
        self.sim = self.rt.gpio

    def tearDown(self):
        self.rt.shutdown()


class TestHoming(SimCase):
    start = [-12.0, 25.0]                 # true pose at power-on, unknown to software

    def test_homes_both_switch_sides_and_parks(self):
        self.assertTrue(self.rt.homing.run())
        true = self.sim.true_deg()
        for m, t, park in zip(self.rt.bank, true, [0.0, 10.0]):
            self.assertTrue(m.homed)
            self.assertAlmostEqual(t, park, delta=0.3, msg=m.name)
            self.assertAlmostEqual(m.angle, t, delta=0.3, msg=m.name)

    def test_homes_when_starting_on_the_switch(self):
        self.rt.shutdown()
        self.setUpWith([30.5, -20.5])
        self.assertTrue(self.rt.homing.run())
        np.testing.assert_allclose(self.sim.true_deg(), [0.0, 10.0], atol=0.3)

    def setUpWith(self, start):
        self.start = start
        self.setUp()

    def test_estop_aborts_homing(self):
        t = threading.Thread(target=self.rt.homing.run)
        t.start()
        time.sleep(0.1)
        self.sim.estop_open = True
        t.join(5)
        self.assertFalse(t.is_alive())
        self.assertFalse(self.rt.bank.all_homed)
        self.assertFalse(self.rt.bank.any_running)


class TestSafety(SimCase):
    start = [0.0, 10.0]

    def test_unhomed_jog_stops_at_switch_and_can_back_off(self):
        a = self.rt.bank["A"]
        self.assertIsNone(a.start(True))
        self.assertTrue(wait_for(lambda: not a.running, 3))
        self.assertTrue(a.at_limit)
        self.assertEqual(a.start(True), "limit")      # further into the switch: refused
        self.assertIsNone(a.start(False))              # away from it: allowed
        self.assertTrue(wait_for(lambda: not a.at_limit, 2))
        a.stop()

    def test_soft_limit_on_homed_joint(self):
        self.rt.assume_homed_at([0.0, 10.0])
        b = self.rt.bank["B"]
        b.start(True)
        self.assertTrue(wait_for(lambda: not b.running, 3))
        self.assertAlmostEqual(self.sim.true_deg()[1], 40.0, delta=1.0)
        self.assertEqual(b.start(True), "hi")

    def test_estop_latches_until_reset(self):
        self.rt.assume_homed_at([0.0, 10.0])
        a = self.rt.bank["A"]
        a.start(True, freq=200)
        time.sleep(0.05)
        self.sim.estop_open = True
        self.assertTrue(wait_for(lambda: not a.running, 0.1))
        self.assertEqual(a.start(False), "estop")
        self.assertFalse(self.rt.safety.reset())       # loop still open
        self.sim.estop_open = False
        time.sleep(0.05)
        self.assertTrue(self.rt.bank.estop_active)      # still latched
        self.assertTrue(self.rt.safety.reset())
        self.assertIsNone(a.start(False, freq=200))
        a.stop()


class TestMoves(SimCase):
    start = [0.0, 10.0]

    def test_move_to_matches_true_position(self):
        self.rt.assume_homed_at([0.0, 10.0])
        self.assertTrue(self.rt.mover.move_to(0, -25.0))
        self.assertTrue(self.rt.mover.move_to(1, 33.0))
        np.testing.assert_allclose(self.sim.true_deg(), [-25.0, 33.0], atol=0.2)

    def test_coordinated_move_arrives_together(self):
        self.rt.assume_homed_at([0.0, 10.0])
        done = {}
        bank = self.rt.bank

        def watch():
            t0 = time.monotonic()
            while len(done) < 2 and time.monotonic() - t0 < 5:
                for m in bank:
                    if m.name not in done and not m.running and time.monotonic() - t0 > 0.05:
                        done[m.name] = time.monotonic() - t0
                time.sleep(0.002)
        w = threading.Thread(target=watch)
        w.start()
        self.assertTrue(self.rt.mover.move_all([20.0, -15.0], speed=20))
        w.join()
        self.assertLess(abs(done["A"] - done["B"]), 0.08, done)
        np.testing.assert_allclose(self.sim.true_deg(), [20.0, -15.0], atol=0.2)


class TestCartesian(SimCase):
    model_dict = os.path.join(ROBOTS, "roboarm_opi.json")
    start = [0, 0, 70, 0, 30, 0]

    def setUp(self):
        super().setUp()
        self.rt.assume_homed_at(self.start)
        self.rt.bank.set_all_speeds(100)

    def true_tcp(self):
        return self.rt.kin.fk(self.sim.true_deg())

    def test_world_z_jog_keeps_x_y(self):
        p0 = self.true_tcp()
        for _ in range(4):
            self.assertTrue(self.rt.cart.jog(d_mm=(0, 0, 5), wait=True))
        p1 = self.true_tcp()
        np.testing.assert_allclose(p1.xyz[:2], p0.xyz[:2], atol=0.5)
        self.assertAlmostEqual(p1.xyz[2] - p0.xyz[2], 20.0, delta=0.5)
        np.testing.assert_allclose(p1.rpy_deg, p0.rpy_deg, atol=0.5)

    def test_held_jog_does_not_drift(self):
        # a held HMI button at the default 30% speed: an 8 mm jog every 90 ms,
        # each pre-empting the last before it has arrived
        self.rt.bank.set_all_speeds(30)
        p0 = self.true_tcp()
        for _ in range(10):
            self.rt.cart.jog(d_mm=(0, 0, 8))
            time.sleep(0.09)
        self.assertTrue(wait_for(lambda: not self.rt.bank.any_running, 5))
        time.sleep(0.05)
        p1 = self.true_tcp()
        np.testing.assert_allclose(p1.xyz[:2], p0.xyz[:2], atol=0.5)
        self.assertGreater(p1.xyz[2] - p0.xyz[2], 20.0)
        np.testing.assert_allclose(p1.rpy_deg, p0.rpy_deg, atol=0.5)

    def test_tool_frame_jog_moves_along_tool_axis(self):
        p0 = self.true_tcp()
        self.assertTrue(self.rt.cart.jog(d_mm=(0, 0, 10), frame="tool", wait=True))
        p1 = self.true_tcp()
        moved = np.array(p1.xyz) - np.array(p0.xyz)
        np.testing.assert_allclose(moved, 10 * p0.T[:3, 2], atol=0.5)

    def test_linear_move_stays_on_the_line(self):
        p0 = self.true_tcp()
        target = self.rt.kin.pose([p0.xyz[0] + 60, p0.xyz[1] - 80, p0.xyz[2] - 40],
                                  p0.rpy_deg)
        samples, stop = [], threading.Event()

        def sample():
            while not stop.is_set():
                samples.append(self.true_tcp().xyz)
                time.sleep(0.003)
        s = threading.Thread(target=sample)
        s.start()
        ok = self.rt.cart.linear(target, speed=40)
        stop.set(); s.join()
        self.assertTrue(ok)
        a, b = np.array(p0.xyz), target[:3, 3]
        u = (b - a) / np.linalg.norm(b - a)
        dev = max(np.linalg.norm((np.array(p) - a) - np.dot(np.array(p) - a, u) * u)
                  for p in samples)
        self.assertLess(dev, 0.5)
        np.testing.assert_allclose(self.true_tcp().xyz, b, atol=0.6)


if __name__ == "__main__":
    unittest.main()
