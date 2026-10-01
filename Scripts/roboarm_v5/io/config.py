"""
Machine settings — things about *this* Pi, not about the arm's geometry.

    ~/.roboarm/config_v5.json
        robot   path of the robot file last used
        fan     auto / idle % / t_lo / t_hi
        agv     ip / port
        speeds  per-joint jog speed %

The arm itself (pins, gearing, limits, geometry) lives in its robot file.
V4's ~/.roboarm/config.json is left untouched so V4 keeps working; on first
run its fan and AGV settings are copied over, and any motor tuning or link
lengths it holds are logged so they can be moved into the robot file by hand.
"""

import json
import os

V5_PATH = os.path.expanduser("~/.roboarm/config_v5.json")
V4_PATH = os.path.expanduser("~/.roboarm/config.json")

DEFAULTS = {
    "robot": None,
    "fan": {"auto": True, "idle": 2, "t_lo": 45.0, "t_hi": 72.0},
    "agv": {"ip": "192.168.4.1", "port": 5005},
    "speeds": {},
}


def _merge(base, over):
    out = dict(base)
    for k, v in over.items():
        out[k] = _merge(base[k], v) if isinstance(v, dict) and isinstance(base.get(k), dict) else v
    return out


class Config:
    def __init__(self, log, path=V5_PATH, v4_path=V4_PATH):
        self.log = log
        self.path = path
        self.v4_path = v4_path
        self.data = json.loads(json.dumps(DEFAULTS))

    def load(self):
        if os.path.exists(self.path):
            try:
                with open(self.path) as f:
                    self.data = _merge(self.data, json.load(f))
            except Exception as e:
                self.log("Config load failed: {}".format(e), alarm=True)
        elif os.path.exists(self.v4_path):
            self._migrate_v4()
        return self

    def _migrate_v4(self):
        try:
            with open(self.v4_path) as f:
                v4 = json.load(f)
        except Exception as e:
            self.log("V4 config unreadable: {}".format(e), alarm=True)
            return
        if "fan" in v4:
            self.data["fan"] = _merge(self.data["fan"], v4["fan"])
        if "agv" in v4:
            self.data["agv"] = _merge(self.data["agv"], v4["agv"])
        self.log("Imported fan/AGV settings from V4 config")
        moved = [k for k in ("steps_per_deg", "target_freq", "limits_ik",
                             "limits_phys", "home_target", "links") if k in v4]
        if moved:
            self.log("V4 config also has {} — those belong in the robot file now; "
                     "V4 values: {}".format(", ".join(moved),
                                            json.dumps({k: v4[k] for k in moved})))
        self.save()

    def save(self):
        try:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            tmp = self.path + ".tmp"
            with open(tmp, "w") as f:
                json.dump(self.data, f, indent=2)
            os.replace(tmp, self.path)          # atomic — no half-written config
            return True
        except Exception as e:
            self.log("Config save failed: {}".format(e), alarm=True)
            return False

    def __getitem__(self, k):
        return self.data[k]

    def __setitem__(self, k, v):
        self.data[k] = v
