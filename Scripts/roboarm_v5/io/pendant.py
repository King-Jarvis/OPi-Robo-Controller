"""
PS4 pendant and the control mode it shares with the HMI — V4 :1207-1362.
Button map unchanged from V3/V4:

    ✕  stop all / stop AGV        ○  toggle AGV mode
    □  cycle joint / stop AGV     △  cycle JOINT → WORLD → TOOL
    L1/R1 speed trim (joint) or fine Z (cartesian)
    L3/R3 coarse Z (cartesian)    D-pad joint select + speed
    AGV: left stick Y drives, right stick X turns.

Joint selection wraps over however many joints the robot has. In WORLD/TOOL
mode all stick inputs of one poll are combined into a single jog (V4 issued
one jog per axis, each pre-empting the last).
"""

import os
import threading
import time

DEADZONE = 0.25
CART_DEADZONE = 0.30
CART_MAJOR_MM = 8.0
CART_MINOR_MM = 2.0
CART_ROT_DEG = 3.0
MODE_CYCLE = ["JOINT", "WORLD", "TOOL"]


class Pendant:
    def __init__(self, bank, cart, agv, log):
        self.bank = bank
        self.cart = cart
        self.agv = agv
        self.log = log
        self.mode = "JOINT"               # JOINT | WORLD | TOOL | AGV
        self.last_arm_mode = "JOINT"
        self.selected = 0
        self.connected = False
        self._thread = None

    # ── shared with the HMI ──────────────────────────────────────────────
    @property
    def selected_motor(self):
        return self.bank[self.selected]

    def select(self, index=None, delta=0):
        n = len(self.bank)
        self.selected = (self.selected + delta) % n if index is None else index % n
        self.bank.stop_all("Selected " + self.selected_motor.name, alarm=False)

    def set_mode(self, mode):
        if mode == "AGV":
            self.enter_agv()
            return
        if self.mode == "AGV":
            self.agv.stop()
            self.agv.enabled = False
        self.mode = self.last_arm_mode = mode
        self.bank.stop_all("Mode " + mode, alarm=False)

    def enter_agv(self):
        if self.mode != "AGV":
            self.last_arm_mode = self.mode
        self.bank.stop_all("Entering AGV mode", alarm=False)
        self.agv.stop()
        self.agv.enabled = True
        self.mode = "AGV"
        self.log("AGV mode — left stick drives, right stick turns")

    def toggle_agv(self):
        if self.mode == "AGV":
            self.agv.stop()
            self.agv.enabled = False
            self.mode = self.last_arm_mode
            self.log("AGV off → " + self.mode)
        else:
            self.enter_agv()

    def stop_everything(self):
        self.agv.stop()
        self.bank.stop_all("Pendant stop", alarm=False)

    # ── PS4 thread ───────────────────────────────────────────────────────
    def start(self):
        if self._thread is None or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._loop, name="pendant", daemon=True)
            self._thread.start()
            self.log("Pendant listener started")

    def _loop(self):
        # SDL otherwise installs its own SIGINT/SIGTERM handlers and turns them
        # into joystick-queue events nobody reads — `kill` would then fail to
        # stop the controller (V4 had this). Must be set before pygame.init().
        os.environ.setdefault("SDL_NO_SIGNAL_HANDLERS", "1")
        try:
            import pygame
        except ImportError:
            self.log("pygame not installed — no pendant", alarm=True)
            return
        pygame.init()
        pygame.joystick.init()
        js = None
        prev = dict(cross=False, circle=False, sq=False, tri=False, l3=False, r3=False)
        prev_dpad = (0, 0)
        active = set()

        while True:
            try:
                if js is None:
                    pygame.joystick.quit()
                    pygame.joystick.init()
                    if pygame.joystick.get_count() > 0:
                        js = pygame.joystick.Joystick(0)
                        js.init()
                        self.connected = True
                        self.log("Pendant: " + js.get_name())
                    else:
                        self.connected = False
                        time.sleep(1)
                        continue

                pygame.event.pump()
                nb, na = js.get_numbuttons(), js.get_numaxes()
                btn = lambda i: js.get_button(i) if nb > i else False
                ly, lx = js.get_axis(1), js.get_axis(0)
                rx = js.get_axis(3) if na > 3 else 0.0
                ry = js.get_axis(4) if na > 4 else 0.0
                cross, circle, sq, tri = btn(0), btn(1), btn(2), btn(3)
                l1, r1, l3, r3 = btn(4), btn(5), btn(7), btn(8)
                dpad_x, dpad_y = js.get_hat(0) if js.get_numhats() > 0 else (0, 0)
                edge = lambda k, v: v and not prev[k]

                if edge("circle", circle):
                    self.toggle_agv()
                if edge("tri", tri) and self.mode != "AGV":
                    self.set_mode(MODE_CYCLE[(MODE_CYCLE.index(self.mode) + 1) % len(MODE_CYCLE)])
                if edge("sq", sq):
                    if self.mode == "AGV":
                        self.agv.stop(); self.log("AGV stop")
                    else:
                        self.select(delta=1)
                if edge("cross", cross):
                    self.stop_everything()
                prev.update(circle=circle, tri=tri, sq=sq, cross=cross)

                if self.mode == "AGV":
                    self.agv.set(-ly if abs(ly) > DEADZONE else 0.0,
                                 rx if abs(rx) > DEADZONE else 0.0)
                elif self.mode == "JOINT":
                    active = self._joint(ly, l1, r1, dpad_x, dpad_y, prev_dpad, active)
                else:
                    self._cartesian(ly, lx, rx, ry, l1, r1, l3, r3, prev,
                                    dpad_x, dpad_y, prev_dpad)
                    active = set()

                prev["l3"], prev["r3"] = l3, r3
                prev_dpad = (dpad_x, dpad_y)
                time.sleep(0.02 if self.mode == "AGV" else 0.05)

            except Exception as e:
                self.connected = False
                js = None
                self.agv.stop()
                self.log("Pendant error: {}".format(e), alarm=True)
                time.sleep(1)

    def _joint(self, ly, l1, r1, dpad_x, dpad_y, prev_dpad, active):
        if dpad_x != prev_dpad[0] and dpad_x != 0:
            self.select(delta=dpad_x)
        m = self.selected_motor
        if dpad_y != prev_dpad[1] and dpad_y != 0:
            m.set_speed(m.speed + 5 * dpad_y)
            self.log("{} {}%".format(m.name, m.speed))
        if l1: m.set_speed(m.speed - 1)
        if r1: m.set_speed(m.speed + 1)

        new = set()
        if abs(ly) > DEADZONE:
            positive = ly < 0                 # stick up = +angle
            if not m.running or m.moving_positive != positive:
                m.start(positive)
            new.add(m.name)
        for name in active - new:
            self.bank[name].stop()
        return new

    def _cartesian(self, ly, lx, rx, ry, l1, r1, l3, r3, prev, dpad_x, dpad_y, prev_dpad):
        if dpad_y != prev_dpad[1] and dpad_y != 0:
            for m in self.bank:
                m.set_speed(m.speed + 5 * dpad_y)
            self.log("All axes {}%".format(self.bank[0].speed))
        if not self.bank.all_homed:
            return
        sgn = lambda v: (1 if v > 0 else -1) if abs(v) > CART_DEADZONE else 0
        dx = -sgn(ly) * CART_MAJOR_MM
        dy = -sgn(ry) * CART_MAJOR_MM
        dz = 0.0
        if l3 and not prev["l3"]: dz += CART_MAJOR_MM
        if r3 and not prev["r3"]: dz -= CART_MAJOR_MM
        if l1: dz -= CART_MINOR_MM
        if r1: dz += CART_MINOR_MM
        drx = sgn(lx) * CART_ROT_DEG
        dry = -sgn(rx) * CART_ROT_DEG
        drz = dpad_x * CART_ROT_DEG if dpad_x != prev_dpad[0] else 0.0
        if any((dx, dy, dz, drx, dry, drz)):
            self.cart.jog((dx, dy, dz), (drx, dry, drz),
                          frame="tool" if self.mode == "TOOL" else "world")
