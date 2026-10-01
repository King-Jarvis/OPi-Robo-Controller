"""
Teach-and-replay programs — V4 :1084-1205.

A point stores joint angles (kinematic degrees, one per joint of the robot it
was taught on) and the TCP pose at that moment. Files carry the robot name and
joint count, so a program taught on one arm won't run on another.

V4 program files (no "format" key) stored step-count angles measured from
each switch; they are converted on load through the same formula the motor
layer uses, and flagged so the operator checks them before running.
"""

import json
import os
import threading
import time

from ..kinematics import mapping

FORMAT = "roboarm-v5-program"


class ProgramError(ValueError):
    pass


class Program:
    def __init__(self, model, log):
        self.model = model
        self.log = log
        self.name = "PROG1"
        self.points = []
        self.variables = {}
        self.filepath = None
        self.modified = False

    def _reindex(self):
        for i, p in enumerate(self.points):
            p["index"] = i + 1
            p["label"] = "P[{}]".format(i + 1)

    def add_point(self, angles, tcp=None, move_type="J", speed_pct=100, cnt=0, comment=""):
        """tcp: kinematics TcpPose or None (solver not loaded yet)."""
        idx = len(self.points) + 1
        pt = {"index": idx, "label": "P[{}]".format(idx), "move_type": move_type,
              "speed_pct": speed_pct, "cnt": cnt,
              "angles": [round(a, 4) for a in angles],
              "tcp": tcp.xyz if tcp else None,
              "rpy": tcp.rpy_deg if tcp else None,
              "comment": comment}
        self.points.append(pt)
        self.modified = True
        self.log("Recorded {} ({})".format(pt["label"], move_type))
        return pt

    def delete_point(self, idx):
        if 0 <= idx < len(self.points):
            self.points.pop(idx)
            self._reindex()
            self.modified = True
            self.log("Point deleted")

    def move_point(self, idx, delta):
        j = idx + delta
        if 0 <= idx < len(self.points) and 0 <= j < len(self.points):
            self.points[idx], self.points[j] = self.points[j], self.points[idx]
            self._reindex()
            self.modified = True
            return True
        return False

    def problems(self):
        """Reasons this program can't run on the current robot (empty = fine)."""
        out = []
        for p in self.points:
            a = p.get("angles", [])
            if len(a) != self.model.dof:
                out.append("{}: {} angles for a {}-joint robot".format(
                    p["label"], len(a), self.model.dof))
            elif not self.model.within_limits(a):
                out.append("{}: outside joint limits".format(p["label"]))
        return out

    def save(self, filepath=None):
        if filepath:
            self.filepath = filepath
        if not self.filepath:
            return False
        try:
            tmp = self.filepath + ".tmp"
            with open(tmp, "w") as f:
                json.dump({"format": FORMAT, "robot": self.model.name,
                           "dof": self.model.dof, "joints": self.model.names,
                           "name": self.name, "points": self.points,
                           "variables": self.variables}, f, indent=2)
            os.replace(tmp, self.filepath)
            self.modified = False
            self.log("Saved " + os.path.basename(self.filepath))
            return True
        except Exception as e:
            self.log("Save failed: {}".format(e), alarm=True)
            return False

    def load(self, filepath):
        with open(filepath) as f:
            d = json.load(f)
        if d.get("format") == FORMAT:
            if d.get("dof") != self.model.dof:
                raise ProgramError("program is for a {}-joint robot ({}), this is {}-joint"
                                   .format(d.get("dof"), d.get("robot"), self.model.dof))
            if d.get("robot") != self.model.name:
                self.log("Program was taught on '{}', running on '{}'".format(
                    d.get("robot"), self.model.name), alarm=True)
            points, converted = d.get("points", []), False
        else:
            points, converted = self._from_v4(d.get("points", [])), True
        self.name = d.get("name", "PROG")
        self.points = points
        self.variables = d.get("variables", {})
        # a converted V4 file is not saved in V5 form yet
        self.filepath = None if converted else filepath
        self.modified = converted
        self._reindex()
        self.log("Loaded " + os.path.basename(filepath))

    def _from_v4(self, points):
        if self.model.dof != 6:
            raise ProgramError("V4 programs are 6-joint; this robot has {}".format(
                self.model.dof))
        for p in points:
            # V4 angle = pulses since the switch / steps_per_deg
            p["angles"] = [round(mapping.pulses_to_deg(j, a * j.steps_per_deg), 4)
                           for j, a in zip(self.model.joints, p.get("angles", []))]
            p["tcp"], p["rpy"] = None, None
        self.log("V4 program converted — CHECK EVERY POINT before running it", alarm=True)
        return points


class Playback:
    def __init__(self, bank, mover, cart, log):
        self.bank = bank
        self.mover = mover
        self.cart = cart
        self.log = log
        self.running = False
        self.step_mode = False

    def run(self, program, speed_override=None, forward=True):
        self.running = True
        try:
            pts = program.points if forward else list(reversed(program.points))
            self.log("Playback start — {} points, {}".format(
                len(pts), "single step" if self.step_mode else "continuous"))
            for i, pt in enumerate(pts):
                if not self.running or self.bank.estop_active:
                    break
                spd = speed_override or pt["speed_pct"]
                self.log("→ {}  {}  {}%".format(pt["label"], pt["move_type"], spd))
                if pt["move_type"] == "L":
                    ok = self.cart.linear(list(pt["angles"]), speed=spd)
                else:
                    ok = self.mover.move_all(list(pt["angles"]), wait=True, speed=spd)
                if not ok and self.running:
                    self.log("Playback stopped at {}".format(pt["label"]), alarm=True)
                    break
                if self.step_mode:
                    break
                cnt = pt.get("cnt", 0)
                time.sleep((100 - cnt) / 1000.0 if (cnt > 0 and i < len(pts) - 1) else 0.1)
            else:
                self.log("Playback complete")
        finally:
            self.running = False
            self.step_mode = False

    def start(self, program, speed_override=None, forward=True, step=False):
        if self.running:
            return False
        if not self.bank.all_homed:
            self.log("Home the arm before running a program", alarm=True)
            return False
        bad = program.problems()
        if bad:
            for b in bad[:5]:
                self.log("Program: " + b, alarm=True)
            return False
        self.step_mode = step
        threading.Thread(target=self.run, args=(program, speed_override, forward),
                         name="playback", daemon=True).start()
        return True

    def stop(self):
        self.running = False
        self.mover.cancel_all()
        self.bank.stop_all("Playback stopped")
