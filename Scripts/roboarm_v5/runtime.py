"""
Everything the arm needs at run time, wired together once. The UI, the
pendant and the tests all hold one Runtime instead of reaching for globals
(V4 kept ~30 module globals that every layer read and wrote).
"""

from .log import Log
from .robot.loader import load_robot
from .hw.gpio import open_backend
from .hw.motors import MotorBank
from .hw.safety import Safety
from .hw.fan import Fan
from .kinematics.service import Kinematics
from .motion.moves import Mover
from .motion.homing import Homing
from .motion.cartesian import Cartesian
from .io.agv import AGVLink
from .io.programs import Program, Playback
from .io.pendant import Pendant


class Runtime:
    def __init__(self, model, sim=False, sim_start=None, estop_latching=True,
                 log=None, kin_background=True, config=None, agv=True):
        if isinstance(model, str):
            model = load_robot(model)
        self.model = model
        self.sim = sim
        self.log = log or Log()
        self.config = config
        self.gpio = open_backend(model, sim=sim,
                                 **({"start_deg": sim_start} if sim else {}))
        self.bank = MotorBank(model, self.gpio, self.log)
        self.safety = Safety(self.bank, latching=estop_latching)
        self.fan = Fan(self.gpio, model.board)
        self.kin = Kinematics(model, self.log, background=kin_background)
        self.mover = Mover(self.bank, self.log)
        self.homing = Homing(self.bank, self.mover, self.log)
        self.cart = Cartesian(self.bank, self.mover, self.kin, self.log)
        ag = config["agv"] if config else {}
        self.agv = AGVLink(ag.get("ip", "192.168.4.1"), int(ag.get("port", 5005)), start=agv)
        self.program = Program(model, self.log)
        self.playback = Playback(self.bank, self.mover, self.cart, self.log)
        self.pendant = Pendant(self.bank, self.cart, self.agv, self.log)
        if config:
            f = config["fan"]
            self.fan.auto, self.fan.idle = bool(f["auto"]), int(f["idle"])
            self.fan.t_lo, self.fan.t_hi = float(f["t_lo"]), float(f["t_hi"])
            for m in self.bank:
                if m.name in config["speeds"]:
                    m.speed = int(config["speeds"][m.name])
        self._started = False

    def start(self):
        self.bank.claim()
        self.safety.claim()
        self.fan.claim()
        self.safety.start()
        self._started = True
        self.log("{} — {} joints{}".format(self.model.name, self.model.dof,
                                           "  (SIMULATED)" if self.sim else ""))
        return self

    def save_config(self):
        if not self.config:
            return False
        c = self.config
        c["robot"] = self.model.source or c["robot"]
        c["fan"] = {"auto": self.fan.auto, "idle": self.fan.idle,
                    "t_lo": self.fan.t_lo, "t_hi": self.fan.t_hi}
        c["agv"] = {"ip": self.agv.ip, "port": self.agv.port}
        c["speeds"] = {m.name: m.speed for m in self.bank}
        return c.save()

    def new_program(self):
        self.program = Program(self.model, self.log)
        return self.program

    def record_point(self, **kw):
        q = self.bank.angles()
        return self.program.add_point(q, self.kin.fk(q), **kw)

    def shutdown(self):
        self.playback.running = False
        self.agv.close()
        self.mover.cancel_all()
        self.bank.stop_all(None)
        if self._started:
            self.safety.stop()
            self.fan.off_hard()
        self.gpio.close()

    # convenience for tests / bench work
    def assume_homed_at(self, angles_deg):
        """Declare the arm homed at these angles without moving it."""
        for m, a in zip(self.bank, angles_deg):
            m.set_angle(a)
            m.homed = True
