#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROBOARM  V4   —   6-axis arm + AGV pendant   —   Orange Pi Zero 2W

A rework of roboarm_V3.py. Hardware map, wiring, AGV wire protocol and the
PS4 button map are unchanged; the arm will behave the same on the bench.

WHAT CHANGED
────────────
UI  — rebuilt around the Japanese design pillars:
      Kanso    (簡素) the main screen carries only what you need while the arm
                      is moving. IK targets, AGV, programs and config moved
                      behind five named doors on the command bar.
      Ma       (間)   a real spacing scale (4/8/12/16/24/32). Emptiness is
                      load-bearing; nothing touches anything else.
      Shibui   (渋い) a sumi-ink palette. Desaturated warm neutrals, one quiet
                      accent (asagi), and saturated colour reserved for state
                      that matters — vermillion means danger and nothing else.
      Seijaku  (静寂) the idle screen is grey and still. Colour is an event.
                      No bevels, no arcade buttons, no blinking.
      Fukinsei (不均整) deliberate asymmetry — a wide data field against a
                      narrow rail, weight to the left, the clock adrift right.
      Yūgen    (幽玄) progressive disclosure. Depth is suggested, not dumped.

FIXES
─────
· Step accounting was wrong.  V3 incremented position by ±1 per 10 ms poll
  regardless of the pulse train, so a joint stepping at 3 kHz was counted at
  100 Hz — angle_deg was off by ~30x. V4 integrates commanded frequency over
  real elapsed time, flushed on every frequency/direction/run-state change.
· Absolute moves slept for a precomputed duration and hoped. They now track
  the step model, so a speed change made mid-move is honoured, a short move
  is logged rather than silently swallowed, and arrival lands on the target
  instead of a poll boundary. (The pulse train lives in lgpio either way — a
  stalled mover thread still overshoots; nothing in software can fix that.)
· move_all_to_angles is coordinated — per-joint frequency is scaled so every
  joint arrives together instead of finishing at random times.
· inverse_kinematics backed the tool length out along world Z even when the
  tool was not pointing along world Z. Now uses the current tool vector.
· Soft limits on jog: a homed joint refuses to be driven past its physical
  range instead of relying on the limit switch to catch it.
· E-STOP latches until explicitly reset (ESTOP_LATCHING, top of file).
· Fan curve: V3 pinned the fan to 80% above 48 °C, which is idle temperature
  for an H618. Replaced with a real curve + hysteresis.
· Routine stops (mode change, joint select) no longer file themselves as
  faults, so the fault log is worth reading.

ORANGE PI ZERO 2W
─────────────────
Quad A53, no GPU-accelerated Tk, modest RAM. V4 is built for that:
· numpy/ikpy are imported on a background thread, so the HMI and the E-STOP
  are live in well under a second instead of after a ~4 s import stall.
· Widgets are never reconfigured with a value they already hold (see cfg()).
  V3 reconfigured ~120 widgets 10x/second whether or not anything moved.
· The log pane appends only new lines. V3 cleared and refilled a Text widget
  ten times a second.
· Three update tiers: 100 ms motion, 250 ms log, 1 s system telemetry.
· Forward kinematics is cached and only re-solved when a joint actually moves.
· Layout density is chosen from the real panel height at start-up, so a 7"
  1024x600 touchscreen gets the whole HMI rather than a clipped one.

Config lives in ~/.roboarm/config.json — motor tuning and link lengths now
survive a restart.

Esc quits.
"""

import os, json, math, time, socket, struct, threading, subprocess
import tkinter as tk
import tkinter.font as tkfont
from tkinter import filedialog, messagebox

import lgpio
import pygame

# ═════════════════════════════════════════════════════════════════════════
#  HARDWARE  —  identical to V3
# ═════════════════════════════════════════════════════════════════════════
GPIOCHIP      = 0
FAN_PIN       = 267
ESTOP_PIN     = 266
DEBOUNCE_TIME = 0.05
FAN_PWM_FREQ  = 1000

# Limit switches and the E-STOP loop are wired normally-closed to ground with
# an internal pull-up: 0 = healthy, 1 = switch opened / loop broken. Fail-safe,
# so a cut wire reads as a fault. Do not invert this without rewiring.
LIMIT_TRIGGERED = 1
ESTOP_TRIGGERED = 1

# Latch the E-STOP until an operator presses RESET, rather than clearing it the
# instant the button pops back out. Set False for V3's momentary behaviour.
ESTOP_LATCHING = True

CONFIG_PATH = os.path.expanduser("~/.roboarm/config.json")

# ═════════════════════════════════════════════════════════════════════════
#  KINEMATICS  /  MOTORS
# ═════════════════════════════════════════════════════════════════════════
L1, L2, L3, L4, L5, TCP = 136.0, 250.0, 26.0, 235.25, 11.5, 7.5

STEPS_PER_DEG = {"J1": 61.111, "J2": 35.455, "J3": 13.333,
                 "J4": 22.222, "J5": 4.444,  "J6": 2.222}

JOINT_LIMITS_IK   = {"J1": (-175.0, 175.0), "J2": (-90.0, 90.0), "J3": (0.0, 140.0),
                     "J4": (-150.0, 150.0), "J5": (-150.0, 150.0), "J6": (-180.0, 180.0)}
JOINT_LIMITS_PHYS = {"J1": (0.0, 350.0), "J2": (0.0, 150.0), "J3": (0.0, 140.0),
                     "J4": (0.0, 300.0), "J5": (0.0, 150.0), "J6": (0.0, 360.0)}

# Direction each joint drives to find its switch. None = no switch fitted.
HOME_DIR    = {"J1": 1, "J2": 0, "J3": 0, "J4": 0, "J5": 1, "J6": None}
# Angle the joint moves to once the switch is found. J1/J4/J5 are range centres.
HOME_TARGET = {"J1": 175.0, "J2": 75.0, "J3": 70.0, "J4": 150.0, "J5": 75.0, "J6": 0.0}

JOINT_ORDER = ["J1", "J2", "J3", "J4", "J5", "J6"]

motors = {
    "J1": {"step": 226, "dir": 227, "target_freq": 3000, "limit": 260},
    "J2": {"step": 228, "dir": 229, "target_freq": 4000, "limit": 261},
    "J3": {"step": 230, "dir": 231, "target_freq": 5000, "limit": 262},
    "J4": {"step": 232, "dir": 233, "target_freq": 4000, "limit": 263},
    "J5": {"step": 256, "dir": 257, "target_freq": 8000, "limit": 264},
    "J6": {"step": 258, "dir": 259, "target_freq": 8000, "limit": 265},
}
for _m in motors.values():
    _m.update(running=False, direction=1, at_limit=False, homed=False,
              speed=30, angle_deg=0.0, position=0,
              # step model: _pulses is a float integral of commanded frequency
              # over elapsed time, flushed whenever freq/dir/running changes.
              _pulses=0.0, _freq=0.0, _t0=time.monotonic(), soft_block=None)

# ═════════════════════════════════════════════════════════════════════════
#  DESIGN TOKENS
# ═════════════════════════════════════════════════════════════════════════
class Ink:
    """Sumi-ink palette. Grounds are warm neutrals; saturation is rationed."""
    void     = "#0B0C0E"   # deepest ground — readouts, wells
    ground   = "#121417"   # page
    raised   = "#181B1F"   # header / footer
    panel    = "#1E2226"   # cards, rows
    sel      = "#232A2E"   # selected row
    hover    = "#262C31"
    line     = "#282D33"   # hairline
    line_lit = "#3B434B"

    text     = "#EDEAE3"   # 白練 shironeri — primary
    text_2   = "#A5A39B"   # 生成 kinari    — secondary
    text_3   = "#6E7378"   # 鈍色 nibi      — labels
    text_4   = "#464B51"   # disabled / dormant

    asagi    = "#7FB3AE"   # 浅葱 — selection, the working accent
    ai       = "#5B84B1"   # 藍   — information, AGV
    wakatake = "#8AAE79"   # 若竹 — ready, ok
    yamabuki = "#D2A24C"   # 山吹 — caution
    shu      = "#BE4E3A"   # 朱   — danger (the only saturated red)
    beni     = "#D9604A"   # 紅   — danger, lit

# Ma — one spacing scale, used everywhere, no ad-hoc numbers.
SP1, SP2, SP3, SP4, SP5, SP6 = 4, 8, 12, 16, 24, 32

class Type:
    """Resolved after the root window exists — families vary by image."""
    mono = sans = cjk = None
    micro = label = body = body_b = data = data_lg = hero = title = rail = None

    @classmethod
    def resolve(cls, root):
        fams = set(tkfont.families(root))
        def pick(cands, fallback):
            for c in cands:
                if c in fams:
                    return c
            return fallback
        cls.mono = pick(["JetBrains Mono", "IBM Plex Mono", "Roboto Mono",
                         "DejaVu Sans Mono", "Liberation Mono", "Noto Sans Mono"],
                        "Courier")
        cls.sans = pick(["Inter", "IBM Plex Sans", "Noto Sans", "DejaVu Sans",
                         "Liberation Sans"], "Helvetica")
        # Only show kanji if something on this image can actually draw them,
        # otherwise the accents render as tofu and the screen looks broken.
        cls.cjk = pick(["Noto Sans CJK JP", "Noto Serif CJK JP", "Noto Sans JP",
                        "Source Han Sans JP", "IPAGothic", "TakaoGothic",
                        "WenQuanYi Zen Hei", "Droid Sans Fallback"], None)

        cls.micro   = (cls.sans, 7)
        cls.label   = (cls.sans, 8)
        cls.body    = (cls.sans, 10)
        cls.body_b  = (cls.sans, 10, "bold")
        cls.title   = (cls.sans, 12)
        cls.data    = (cls.mono, 13)
        cls.data_lg = (cls.mono, 17)
        cls.hero    = (cls.mono, 40)
        cls.rail    = (cls.cjk, 13) if cls.cjk else (cls.sans, 9)

    @classmethod
    def jp(cls, kanji, romaji):
        """Kanji when the image can render it, romaji when it can't."""
        return kanji if cls.cjk else romaji

class Metrics:
    """
    Layout density, chosen from the actual panel height at start-up. A 7"
    1024x600 touchscreen cannot carry desktop spacing and still show the log,
    so below 700px everything tightens by one step. Ma is preserved in
    proportion — the scale shrinks, the rhythm does not change.
    """
    compact = False
    row_h = 42
    rail_w = 252
    sec_pady = (SP4, SP2)
    log_lines = 5
    travel_w = 110
    slider_w = 132
    state_w = 10
    jog_padx = SP5
    rule_pady = SP5          # air around the rail's dividing rules
    stop_pady = SP4
    hero_font = None
    tcp_font = None

    @classmethod
    def resolve(cls, root):
        # Fullscreen means window height == screen height.
        cls.compact = root.winfo_screenheight() < 700
        if cls.compact:
            cls.row_h, cls.rail_w = 36, 224
            cls.sec_pady = (SP3, SP1)
            cls.log_lines = 3
            cls.travel_w, cls.slider_w, cls.state_w = 84, 104, 9
            cls.jog_padx = SP4
            cls.rule_pady, cls.stop_pady = SP2, SP3
            cls.hero_font = (Type.mono, 30)
            cls.tcp_font = Type.data
        else:
            cls.hero_font = Type.hero
            cls.tcp_font = Type.data_lg


# ═════════════════════════════════════════════════════════════════════════
#  STATE
# ═════════════════════════════════════════════════════════════════════════
h = lgpio.gpiochip_open(GPIOCHIP)

estop_active   = False
estop_pin_raw  = False
fan_speed      = 2
fan_auto       = True
FAN_IDLE, FAN_T_LO, FAN_T_HI = 2, 45.0, 72.0   # H618 throttles around 85 °C

system_log, alarm_log = [], []
log_seq = 0                     # bumped on every entry so the UI can tail
homing_active = False
all_homed     = False
start_time    = time.time()

control_mode   = "JOINT"        # JOINT | WORLD | TOOL | AGV
selected_joint = 0
ps4_connected  = False
ps4_thread     = None
DEADZONE       = 0.25
MODE_CYCLE     = ["JOINT", "WORLD", "TOOL"]
_last_arm_mode = "JOINT"

_log_lock = threading.Lock()


def log(msg, alarm=False):
    global log_seq
    entry = "{}  {}".format(time.strftime("%H:%M:%S"), msg)
    with _log_lock:
        system_log.append(entry)
        if len(system_log) > 400:
            del system_log[:100]
        if alarm:
            alarm_log.append(entry)
            if len(alarm_log) > 200:
                del alarm_log[:50]
        log_seq += 1


# ── GPIO claim ───────────────────────────────────────────────────────────
for _name, _m in motors.items():
    lgpio.gpio_claim_output(h, _m["step"], 0)
    lgpio.gpio_claim_output(h, _m["dir"], 0)
    lgpio.gpio_claim_input(h, _m["limit"], lgpio.SET_PULL_UP)
lgpio.gpio_claim_input(h, ESTOP_PIN, lgpio.SET_PULL_UP)
lgpio.gpio_claim_output(h, FAN_PIN, 0)
lgpio.tx_pwm(h, FAN_PIN, FAN_PWM_FREQ, FAN_IDLE)


# ═════════════════════════════════════════════════════════════════════════
#  CONFIG PERSISTENCE
# ═════════════════════════════════════════════════════════════════════════
def save_config():
    data = {
        "steps_per_deg": STEPS_PER_DEG,
        "target_freq":   {n: motors[n]["target_freq"] for n in JOINT_ORDER},
        "limits_ik":     {n: list(JOINT_LIMITS_IK[n]) for n in JOINT_ORDER},
        "limits_phys":   {n: list(JOINT_LIMITS_PHYS[n]) for n in JOINT_ORDER},
        "home_target":   HOME_TARGET,
        "links":         {"L1": L1, "L2": L2, "L3": L3, "L4": L4, "L5": L5, "TCP": TCP},
        "fan":           {"auto": fan_auto, "idle": FAN_IDLE,
                          "t_lo": FAN_T_LO, "t_hi": FAN_T_HI},
        "agv":           {"ip": ESP32_IP, "port": ESP32_PORT},
    }
    try:
        os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
        tmp = CONFIG_PATH + ".tmp"
        with open(tmp, "w") as f:
            json.dump(data, f, indent=2)
        os.replace(tmp, CONFIG_PATH)          # atomic — no half-written config
        log("Config saved to " + CONFIG_PATH)
        return True
    except Exception as e:
        log("Config save failed: {}".format(e), alarm=True)
        return False


def load_config():
    global L1, L2, L3, L4, L5, TCP, fan_auto, FAN_IDLE, FAN_T_LO, FAN_T_HI
    global ESP32_IP, ESP32_PORT
    if not os.path.exists(CONFIG_PATH):
        return
    try:
        with open(CONFIG_PATH) as f:
            d = json.load(f)
        STEPS_PER_DEG.update(d.get("steps_per_deg", {}))
        for n, v in d.get("target_freq", {}).items():
            if n in motors:
                motors[n]["target_freq"] = int(v)
        for n, v in d.get("limits_ik", {}).items():
            JOINT_LIMITS_IK[n] = (float(v[0]), float(v[1]))
        for n, v in d.get("limits_phys", {}).items():
            JOINT_LIMITS_PHYS[n] = (float(v[0]), float(v[1]))
        HOME_TARGET.update({k: float(v) for k, v in d.get("home_target", {}).items()})
        lk = d.get("links", {})
        L1 = float(lk.get("L1", L1)); L2 = float(lk.get("L2", L2))
        L3 = float(lk.get("L3", L3)); L4 = float(lk.get("L4", L4))
        L5 = float(lk.get("L5", L5)); TCP = float(lk.get("TCP", TCP))
        fn = d.get("fan", {})
        fan_auto = bool(fn.get("auto", fan_auto))
        FAN_IDLE = int(fn.get("idle", FAN_IDLE))
        FAN_T_LO = float(fn.get("t_lo", FAN_T_LO))
        FAN_T_HI = float(fn.get("t_hi", FAN_T_HI))
        ag = d.get("agv", {})
        ESP32_IP = ag.get("ip", ESP32_IP)
        ESP32_PORT = int(ag.get("port", ESP32_PORT))
        log("Config loaded from " + CONFIG_PATH)
    except Exception as e:
        log("Config load failed: {}".format(e), alarm=True)


# ═════════════════════════════════════════════════════════════════════════
#  AGV LINK  —  wire protocol identical to V3
# ═════════════════════════════════════════════════════════════════════════
ESP32_IP    = "192.168.4.1"     # ESP32 hotspot gateway
ESP32_PORT  = 5005
AGV_SEND_HZ = 50


class AGVLink:
    """
    8-byte binary packets to the ESP32 at 50 Hz while in AGV mode:
        b'EVA' + int16be(t*10000) + int16be(r*10000) + CRC8(poly 0x07)
        t = translate  -1 reverse .. +1 forward   (left stick Y)
        r = rotate     -1 left    .. +1 right     (right stick X)
    /status is polled twice a second for telemetry.
    """

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.translate = 0.0
        self.rotate = 0.0
        self.running = True
        self.status_json = {}
        self.online = False
        threading.Thread(target=self._send_loop, daemon=True).start()
        threading.Thread(target=self._status_loop, daemon=True).start()

    @staticmethod
    def _crc8(data):
        crc = 0
        for b in data:
            crc ^= b
            for _ in range(8):
                crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
        return crc

    @staticmethod
    def _pack(t, r):
        ti = int(max(-1.0, min(1.0, t)) * 10000)
        ri = int(max(-1.0, min(1.0, r)) * 10000)
        header = b'EVA' + struct.pack('>hh', ti, ri)
        return header + bytes([AGVLink._crc8(header)])

    def _send_loop(self):
        interval = 1.0 / AGV_SEND_HZ
        while self.running:
            if control_mode == "AGV":
                try:
                    self.sock.sendto(self._pack(self.translate, self.rotate),
                                     (ESP32_IP, ESP32_PORT))
                except Exception:
                    pass
            time.sleep(interval)

    def _status_loop(self):
        import urllib.request
        while self.running:
            try:
                with urllib.request.urlopen(
                        "http://{}/status".format(ESP32_IP), timeout=0.4) as r:
                    self.status_json = json.loads(r.read().decode())
                    self.online = True
            except Exception:
                self.online = False
            time.sleep(0.5)

    def set(self, translate, rotate):
        self.translate = max(-1.0, min(1.0, translate))
        self.rotate = max(-1.0, min(1.0, rotate))

    def stop(self):
        self.translate = 0.0
        self.rotate = 0.0


agv = AGVLink()


# ═════════════════════════════════════════════════════════════════════════
#  MOTOR CORE
#
#  The step model. lgpio generates the pulse train in hardware, so nothing
#  counts real edges — position is the integral of commanded frequency over
#  elapsed time. That integral is only correct if it is flushed *before*
#  frequency, direction or run state changes, which is what _flush does.
#  Every mutation below goes through it.
# ═════════════════════════════════════════════════════════════════════════
def _flush(m, now=None):
    now = time.monotonic() if now is None else now
    if m["running"] and m["_freq"] > 0.0:
        dt = now - m["_t0"]
        if dt > 0.0:
            m["_pulses"] += m["_freq"] * dt * (1 if m["direction"] == 1 else -1)
    m["_t0"] = now
    m["position"] = int(m["_pulses"])
    m["angle_deg"] = m["_pulses"] / STEPS_PER_DEG_OF(m)


def STEPS_PER_DEG_OF(m):
    return STEPS_PER_DEG[m["_name"]]


for _n in JOINT_ORDER:
    motors[_n]["_name"] = _n


def deg_to_steps(name, deg):  return deg * STEPS_PER_DEG[name]
def steps_to_deg(name, steps): return steps / STEPS_PER_DEG[name]


def set_fan(pct):
    global fan_speed
    pct = max(0, min(100, int(pct)))
    fan_speed = pct
    if pct == 0:
        lgpio.tx_pwm(h, FAN_PIN, 1, 0)
        time.sleep(0.02)
        lgpio.gpio_write(h, FAN_PIN, 0)
    else:
        lgpio.tx_pwm(h, FAN_PIN, FAN_PWM_FREQ, pct)


def fan_off_hard():
    lgpio.tx_pwm(h, FAN_PIN, 1, 0); time.sleep(0.05)
    lgpio.gpio_write(h, FAN_PIN, 0); time.sleep(0.05)


def stop_motor(name):
    m = motors[name]
    _flush(m)
    lgpio.tx_pwm(h, m["step"], 1, 0)
    lgpio.gpio_claim_output(h, m["step"], 0)
    m["running"] = False
    m["_freq"] = 0.0
    _flush(m)


def stop_all(reason="STOP ALL", alarm=True):
    for name in motors:
        stop_motor(name)
    log(reason, alarm=alarm)


def soft_limit_block(name, direction):
    """
    True when driving `direction` would take a homed joint outside its physical
    range. Unhomed joints are unconstrained — position is unknown, so the limit
    switch is the only authority. Returns None when the move is allowed.
    """
    m = motors[name]
    if not m["homed"]:
        return None
    lo, hi = JOINT_LIMITS_PHYS[name]
    a = m["angle_deg"]
    if direction == 1 and a >= hi:
        return "hi"
    if direction == 0 and a <= lo:
        return "lo"
    return None


def start_motor(name, direction, freq_override=None):
    m = motors[name]
    if estop_active:
        return
    if m["at_limit"] and m["direction"] == direction:
        return
    block = soft_limit_block(name, direction)
    m["soft_block"] = block
    if block:
        return

    stop_motor(name)                  # flushes the step integral first
    time.sleep(0.02)
    m["direction"] = direction
    lgpio.gpio_write(h, m["dir"], direction)
    time.sleep(0.01)
    freq = freq_override or max(10, int(m["target_freq"] * m["speed"] / 100))
    m["running"] = True
    m["_freq"] = float(freq)
    m["_t0"] = time.monotonic()
    lgpio.tx_pwm(h, m["step"], freq, 50)


def set_motor_speed(name, pct):
    m = motors[name]
    m["speed"] = max(1, min(100, int(pct)))
    if m["running"]:
        freq = max(10, int(m["target_freq"] * m["speed"] / 100))
        _flush(m)                      # bank the pulses at the old rate first
        m["_freq"] = float(freq)
        lgpio.tx_pwm(h, m["step"], freq, 50)


def set_all_speeds(pct):
    for name in JOINT_ORDER:
        set_motor_speed(name, pct)


def zero_positions(mark_homed=False):
    for name in JOINT_ORDER:
        m = motors[name]
        m["_pulses"] = 0.0
        m["position"] = 0
        m["angle_deg"] = 0.0
        m["_t0"] = time.monotonic()
        if mark_homed:
            m["homed"] = True


def estop_reset():
    """Clear a latched E-STOP. Refuses while the loop is still open."""
    global estop_active
    if estop_pin_raw:
        log("E-STOP reset refused — loop still open", alarm=True)
        return False
    estop_active = False
    log("E-STOP reset — drives re-enabled")
    return True


# ── fan curve ────────────────────────────────────────────────────────────
def get_cpu_temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return float(f.read()) / 1000.0
    except Exception:
        return 0.0


_fan_target = FAN_IDLE


def fan_curve(temp):
    """Idle below t_lo, linear to 100% at t_hi. H618 idles near 45 °C."""
    if temp <= FAN_T_LO:
        return FAN_IDLE
    if temp >= FAN_T_HI:
        return 100
    span = max(1.0, FAN_T_HI - FAN_T_LO)
    return int(FAN_IDLE + (100 - FAN_IDLE) * (temp - FAN_T_LO) / span)


def fan_tick(temp):
    """Called once a second. 4% hysteresis so the fan doesn't hunt."""
    global _fan_target
    if not fan_auto:
        return
    want = fan_curve(temp)
    if abs(want - _fan_target) >= 4 or (want == 100 and _fan_target != 100):
        _fan_target = want
        set_fan(want)


# ═════════════════════════════════════════════════════════════════════════
#  SAFETY LOOP
# ═════════════════════════════════════════════════════════════════════════
def safety_loop():
    global estop_active, estop_pin_raw
    last_trigger = {n: 0.0 for n in motors}
    last_estop = 0.0
    while True:
        try:
            now = time.monotonic()

            # ── E-STOP ────────────────────────────────────────────────
            raw = (lgpio.gpio_read(h, ESTOP_PIN) == ESTOP_TRIGGERED)
            if raw:
                if now - last_estop > DEBOUNCE_TIME:
                    if not estop_active:
                        estop_active = True
                        stop_all("E-STOP TRIGGERED")
                    last_estop = now
                if not estop_pin_raw:
                    estop_pin_raw = True
            else:
                if estop_pin_raw:
                    estop_pin_raw = False
                    log("E-STOP loop closed" +
                        ("  —  press RESET to re-enable" if ESTOP_LATCHING else ""))
                if estop_active and not ESTOP_LATCHING:
                    estop_active = False

            # ── limits + step integration ─────────────────────────────
            for name, m in motors.items():
                triggered = (lgpio.gpio_read(h, m["limit"]) == LIMIT_TRIGGERED)
                if triggered:
                    if now - last_trigger[name] > DEBOUNCE_TIME:
                        if not m["at_limit"]:
                            m["at_limit"] = True
                            if not homing_active:
                                stop_motor(name)
                                log("{} limit switch".format(name), alarm=True)
                        last_trigger[name] = now
                else:
                    if m["at_limit"]:
                        log("{} limit cleared".format(name))
                    m["at_limit"] = False

                if m["running"]:
                    _flush(m, now)

                # A soft limit is a property of where the joint IS, not of
                # whether it happens to be moving — evaluating it only while
                # running latches it on forever once the joint stops.
                if m["homed"]:
                    lo, hi = JOINT_LIMITS_PHYS[name]
                    blk = ("hi" if m["angle_deg"] >= hi else
                           "lo" if m["angle_deg"] <= lo else None)
                else:
                    blk = None
                if blk and m["running"] and \
                        ((blk == "hi" and m["direction"] == 1) or
                         (blk == "lo" and m["direction"] == 0)):
                    stop_motor(name)
                    log("{} soft limit ({})".format(name, blk), alarm=True)
                m["soft_block"] = blk

            time.sleep(0.01)
        except Exception as e:
            log("Safety loop fault: {}".format(e), alarm=True)
            time.sleep(0.2)


threading.Thread(target=safety_loop, daemon=True).start()


# ═════════════════════════════════════════════════════════════════════════
#  KINEMATICS
#
#  numpy + ikpy cost several seconds to import on an H618. They are loaded on
#  a worker thread so the HMI paints and the E-STOP arms immediately; anything
#  that needs the solver waits on _kin_ready or degrades gracefully.
# ═════════════════════════════════════════════════════════════════════════
_kin_ready = threading.Event()
_kin_error = None
_np = None
_ikchain = None
_iklink = None
_arm_chain = None
_kin_lock = threading.Lock()


def _build_chain():
    L = _iklink
    return _ikchain.Chain([
        L.OriginLink(),
        L.URDFLink(name='J1', origin_translation=[0, 0, L1],
                   origin_orientation=[0, 0, 0], rotation=[0, 0, 1]),
        L.URDFLink(name='J2', origin_translation=[0, 0, 0],
                   origin_orientation=[0, -_np.pi / 2, 0], rotation=[0, 1, 0]),
        L.URDFLink(name='J3', origin_translation=[L2, 0, 0],
                   origin_orientation=[0, 0, 0], rotation=[0, 1, 0]),
        L.URDFLink(name='J4', origin_translation=[L3, 0, 0],
                   origin_orientation=[0, _np.pi / 2, 0], rotation=[1, 0, 0]),
        L.URDFLink(name='J5', origin_translation=[0, 0, L4],
                   origin_orientation=[0, -_np.pi / 2, 0], rotation=[0, 1, 0]),
        L.URDFLink(name='J6', origin_translation=[0, 0, 0],
                   origin_orientation=[0, _np.pi / 2, 0], rotation=[0, 0, 1]),
    ])


def _kin_load():
    global _np, _ikchain, _iklink, _arm_chain, _kin_error
    t0 = time.monotonic()
    try:
        import numpy
        from ikpy import chain as ikchain, link as iklink
        _np = numpy
        _ikchain, _iklink = ikchain, iklink
        _arm_chain = _build_chain()
        _kin_ready.set()
        log("Solver ready ({:.1f}s)".format(time.monotonic() - t0))
    except Exception as e:
        _kin_error = str(e)
        log("Solver unavailable: {}".format(e), alarm=True)


threading.Thread(target=_kin_load, daemon=True).start()


def rebuild_chain():
    global _arm_chain
    if not _kin_ready.is_set():
        return
    with _kin_lock:
        _arm_chain = _build_chain()
    log("IK chain rebuilt for new link lengths")


def forward_kinematics(angles_deg):
    """TCP position in mm, or None while the solver is still loading."""
    if not _kin_ready.is_set():
        return None
    with _kin_lock:
        rads = [0.0] + [math.radians(a) for a in angles_deg]
        fk = _arm_chain.forward_kinematics(rads)
    pos = fk[:3, 3] + fk[:3, 2] * (L5 + TCP)
    return [float(pos[0]), float(pos[1]), float(pos[2])]


def _tool_axis(angles_deg):
    """Unit vector the tool currently points along, in world frame."""
    with _kin_lock:
        rads = [0.0] + [math.radians(a) for a in angles_deg]
        fk = _arm_chain.forward_kinematics(rads)
    return fk[:3, 2]


def inverse_kinematics(tx, ty, tz, rx=0.0, ry=0.0, rz=0.0):
    if not _kin_ready.is_set():
        log("IK: solver still loading")
        return None
    try:
        tool_len = L5 + TCP
        target = _np.array([tx, ty, tz], dtype=float)
        current_deg = [motors[n]['angle_deg'] for n in JOINT_ORDER]
        current = [0.0] + [math.radians(a) for a in current_deg]
        has_orientation = not (rx == 0.0 and ry == 0.0 and rz == 0.0)

        if has_orientation:
            crx, srx = math.cos(math.radians(rx)), math.sin(math.radians(rx))
            cry, sry = math.cos(math.radians(ry)), math.sin(math.radians(ry))
            crz, srz = math.cos(math.radians(rz)), math.sin(math.radians(rz))
            Rx = _np.array([[1, 0, 0], [0, crx, -srx], [0, srx, crx]])
            Ry = _np.array([[cry, 0, sry], [0, 1, 0], [-sry, 0, cry]])
            Rz = _np.array([[crz, -srz, 0], [srz, crz, 0], [0, 0, 1]])
            R = Rz @ Ry @ Rx
            j6_pos = target - R[:, 2] * tool_len
            with _kin_lock:
                sol = _arm_chain.inverse_kinematics(
                    target_position=j6_pos, target_orientation=R,
                    orientation_mode='Z', initial_position=current, max_iter=300)
        else:
            # Position only. V3 backed the tool length out along world Z, which
            # is only right when the tool happens to point straight up. Use the
            # tool's actual current direction instead.
            axis = _tool_axis(current_deg)
            j6_pos = target - _np.asarray(axis, dtype=float) * tool_len
            with _kin_lock:
                sol = _arm_chain.inverse_kinematics(
                    target_position=j6_pos, initial_position=current, max_iter=300)

        angles_out = [math.degrees(float(a)) for a in sol[1:7]]

        with _kin_lock:
            fk_check = _arm_chain.forward_kinematics(sol)
        error = float(_np.linalg.norm(fk_check[:3, 3] - j6_pos))
        if error > 8.0:
            log('IK: converged {:.1f}mm off target — rejected'.format(error))
            return None

        for i, name in enumerate(JOINT_ORDER):
            lo, hi = JOINT_LIMITS_IK[name]
            if not (lo <= angles_out[i] <= hi):
                log('IK: {} would be {:+.1f}deg, outside [{:.0f},{:.0f}]'.format(
                    name, angles_out[i], lo, hi))
                return None
        return angles_out
    except Exception as e:
        log('IK error: {}'.format(e), alarm=True)
        return None


# ═════════════════════════════════════════════════════════════════════════
#  ABSOLUTE / COORDINATED MOVES
# ═════════════════════════════════════════════════════════════════════════
_move_cancel = {n: threading.Event() for n in JOINT_ORDER}


def move_to_angle(name, target_deg, freq=None):
    """
    Drive one joint to an absolute angle.

    V3 slept for a precomputed duration and hoped. This tracks the step model
    each pass instead, so the move honours a speed change made while it is in
    flight, times out with a logged shortfall rather than silently under-
    travelling, and converges on the target instead of a poll boundary.
    Cancellable: starting another move on the same joint pre-empts this one.
    """
    m = motors[name]
    if not m["homed"] or estop_active:
        return
    lo, hi = JOINT_LIMITS_PHYS[name]
    target_deg = max(lo, min(hi, target_deg))
    target_pulses = deg_to_steps(name, target_deg)

    _move_cancel[name].set()
    time.sleep(0.015)                 # let any previous mover fall out
    _move_cancel[name].clear()
    stop_motor(name)                  # flushes the step integral first

    delta = target_pulses - m["_pulses"]
    if abs(delta) < 2:
        return

    direction = 1 if delta > 0 else 0
    if soft_limit_block(name, direction):
        return
    if freq is None:
        freq = max(10, int(m["target_freq"] * m["speed"] / 100))
    freq = max(10, int(freq))
    # Generous ceiling: nominal duration plus 50% plus a second of slack.
    deadline = time.monotonic() + abs(delta) / freq * 1.5 + 1.0

    time.sleep(0.015)
    m["direction"] = direction
    lgpio.gpio_write(h, m["dir"], direction)
    time.sleep(0.008)
    m["running"] = True
    m["_freq"] = float(freq)
    m["_t0"] = time.monotonic()
    lgpio.tx_pwm(h, m["step"], freq, 50)

    while True:
        if estop_active or m["at_limit"] or _move_cancel[name].is_set():
            break
        _flush(m)
        remaining = target_pulses - m["_pulses"]
        if (direction == 1 and remaining <= 0) or (direction == 0 and remaining >= 0):
            break
        if time.monotonic() > deadline:
            log("{} move timed out {:.1f}deg short".format(
                name, steps_to_deg(name, abs(remaining))), alarm=True)
            break
        # Sleep the shorter of a coarse slice and exactly the time left, so the
        # final wake-up lands on arrival rather than a poll boundary.
        time.sleep(min(0.004, max(0.0002, abs(remaining) / max(1.0, m["_freq"]))))

    stop_motor(name)


def move_all_to_angles(angles, wait=True):
    """
    Coordinated move: every joint's frequency is scaled so they all arrive at
    the same instant. V3 started each joint at its own speed and they finished
    whenever, which made multi-axis moves swing through unintended poses.
    """
    plan = []
    longest = 0.0
    for i, name in enumerate(JOINT_ORDER):
        m = motors[name]
        _flush(m)
        lo, hi = JOINT_LIMITS_PHYS[name]
        tgt = max(lo, min(hi, angles[i]))
        pulses = abs(deg_to_steps(name, tgt) - m["_pulses"])
        fmax = max(10.0, m["target_freq"] * m["speed"] / 100.0)
        duration = pulses / fmax if pulses > 2 else 0.0
        longest = max(longest, duration)
        plan.append((name, tgt, pulses, fmax))

    threads = []
    for name, tgt, pulses, fmax in plan:
        if pulses <= 2:
            continue
        # Slow every joint down to the pace of the slowest one.
        freq = pulses / longest if longest > 0 else fmax
        freq = max(10.0, min(fmax, freq))
        t = threading.Thread(target=move_to_angle, args=(name, tgt, freq), daemon=True)
        threads.append(t)
        t.start()
    if wait:
        for t in threads:
            t.join()


def linear_move(target_angles, steps=20):
    """
    Straight line in tool space when the solver is up: interpolate the TCP and
    solve each waypoint. Falls back to interpolating joint angles otherwise.
    """
    start_angles = [motors[n]["angle_deg"] for n in JOINT_ORDER]
    p0 = forward_kinematics(start_angles)
    p1 = forward_kinematics(target_angles)

    if p0 is None or p1 is None:
        for s in range(1, steps + 1):
            if estop_active:
                return
            t = s / steps
            move_all_to_angles(
                [start_angles[i] + (target_angles[i] - start_angles[i]) * t
                 for i in range(6)], wait=True)
        return

    for s in range(1, steps + 1):
        if estop_active:
            return
        t = s / steps
        way = [p0[k] + (p1[k] - p0[k]) * t for k in range(3)]
        sol = inverse_kinematics(*way)
        if sol is None:
            # Unreachable waypoint — finish on joint interpolation rather than
            # abandoning the arm halfway along the path.
            sol = [start_angles[i] + (target_angles[i] - start_angles[i]) * t
                   for i in range(6)]
        move_all_to_angles(sol, wait=True)


# ═════════════════════════════════════════════════════════════════════════
#  HOMING
# ═════════════════════════════════════════════════════════════════════════
def home_joint(name):
    m = motors[name]
    hdir = HOME_DIR[name]

    if hdir is None:
        m["_pulses"] = 0.0
        m["position"] = 0
        m["angle_deg"] = 0.0
        m["homed"] = True
        log("{} has no switch — zeroed in place".format(name))
        return True

    target_deg = HOME_TARGET[name]
    log("{} seeking switch, then {:.1f}deg".format(name, target_deg))

    home_freq = max(10, int(m["target_freq"] * 0.2))
    stop_motor(name)
    time.sleep(0.02)
    m["direction"] = hdir
    lgpio.gpio_write(h, m["dir"], hdir)
    time.sleep(0.01)
    m["running"] = True
    m["_freq"] = float(home_freq)
    m["_t0"] = time.monotonic()
    lgpio.tx_pwm(h, m["step"], home_freq, 50)

    timeout = time.time() + 30.0
    while not m["at_limit"]:
        if estop_active:
            stop_motor(name); log("{} homing aborted".format(name), alarm=True); return False
        if time.time() > timeout:
            stop_motor(name); log("{} homing timed out".format(name), alarm=True); return False
        time.sleep(0.01)

    stop_motor(name)
    m["_pulses"] = 0.0
    m["position"] = 0
    m["angle_deg"] = 0.0
    log("{} datum found".format(name))
    time.sleep(0.15)

    m["homed"] = True
    saved = m["speed"]
    m["speed"] = 20
    move_to_angle(name, target_deg)
    m["speed"] = saved
    if estop_active:
        return False
    log("{} home {:+.1f}deg".format(name, m["angle_deg"]))
    return True


def run_homing():
    global homing_active, all_homed
    homing_active = True
    all_homed = False
    log("HOMING SEQUENCE START")
    for name in JOINT_ORDER:
        if estop_active:
            log("Homing aborted by E-STOP", alarm=True)
            homing_active = False
            return
        if not home_joint(name):
            homing_active = False
            return
        time.sleep(0.3)
    homing_active = False
    all_homed = True
    log("HOMING COMPLETE  —  " + "  ".join(
        "{}:{:+.1f}".format(n, motors[n]["angle_deg"]) for n in JOINT_ORDER))


def start_homing():
    if homing_active:
        return
    if estop_active:
        log("Cannot home while E-STOP is active", alarm=True)
        return
    threading.Thread(target=run_homing, daemon=True).start()


# ═════════════════════════════════════════════════════════════════════════
#  CARTESIAN JOG
# ═════════════════════════════════════════════════════════════════════════
CART_SCALE_MAJOR = 8.0
CART_SCALE_MINOR = 2.0
CART_SCALE_ROT   = 3.0


def _cart_jog(dx, dy, dz, drx=0.0, dry=0.0, drz=0.0, tool_frame=False):
    if not all_homed or not _kin_ready.is_set():
        return
    angles = [motors[n]["angle_deg"] for n in JOINT_ORDER]
    tcp = forward_kinematics(angles)
    if tcp is None:
        return
    if tool_frame:
        j6 = math.radians(angles[5]); j5 = math.radians(angles[4])
        wx = dx * math.cos(j6) - dy * math.sin(j6)
        wy = dx * math.sin(j6) + dy * math.cos(j6)
        wz = dz * math.cos(j5) - dx * math.sin(j5)
        dx, dy, dz = wx, wy, wz
    sol = inverse_kinematics(tcp[0] + dx, tcp[1] + dy, tcp[2] + dz,
                             angles[3] + drx, angles[4] + dry, angles[5] + drz)
    if sol is None:
        return
    move_all_to_angles(sol, wait=False)


def cartesian_jog(axis, direction):
    tool = (control_mode == "TOOL")
    if control_mode not in ("WORLD", "TOOL"):
        return
    s  = CART_SCALE_MAJOR * direction
    sf = CART_SCALE_MINOR * direction
    sr = CART_SCALE_ROT * direction
    if   axis == "X":  _cart_jog(s, 0, 0, tool_frame=tool)
    elif axis == "Y":  _cart_jog(0, s, 0, tool_frame=tool)
    elif axis == "Z":  _cart_jog(0, 0, s, tool_frame=tool)
    elif axis == "Zf": _cart_jog(0, 0, sf, tool_frame=tool)
    elif axis == "Rx": _cart_jog(0, 0, 0, drx=sr)
    elif axis == "Ry": _cart_jog(0, 0, 0, dry=sr)
    elif axis == "Rz": _cart_jog(0, 0, 0, drz=sr)


# ═════════════════════════════════════════════════════════════════════════
#  PROGRAM  /  POINTS
# ═════════════════════════════════════════════════════════════════════════
class Program:
    def __init__(self):
        self.name = "PROG1"
        self.points = []
        self.variables = {}
        self.filepath = None
        self.modified = False

    def _reindex(self):
        for i, p in enumerate(self.points):
            p["index"] = i + 1
            p["label"] = "P[{}]".format(i + 1)

    def add_point(self, move_type="J", speed_pct=100, cnt=0, comment=""):
        angles = [motors[n]["angle_deg"] for n in JOINT_ORDER]
        tcp = forward_kinematics(angles) or [0.0, 0.0, 0.0]
        idx = len(self.points) + 1
        pt = {"index": idx, "label": "P[{}]".format(idx), "move_type": move_type,
              "speed_pct": speed_pct, "cnt": cnt, "angles": list(angles),
              "tcp": list(tcp), "comment": comment}
        self.points.append(pt)
        self.modified = True
        log("Recorded P[{}] ({})".format(idx, move_type))
        return pt

    def delete_point(self, idx):
        if 0 <= idx < len(self.points):
            self.points.pop(idx)
            self._reindex()
            self.modified = True
            log("Point deleted")

    def move_point(self, idx, delta):
        j = idx + delta
        if 0 <= idx < len(self.points) and 0 <= j < len(self.points):
            self.points[idx], self.points[j] = self.points[j], self.points[idx]
            self._reindex()
            self.modified = True
            return True
        return False

    def save(self, filepath=None):
        if filepath:
            self.filepath = filepath
        if not self.filepath:
            return False
        try:
            with open(self.filepath, "w") as f:
                json.dump({"name": self.name, "points": self.points,
                           "variables": self.variables}, f, indent=2)
            self.modified = False
            log("Saved " + os.path.basename(self.filepath))
            return True
        except Exception as e:
            log("Save failed: {}".format(e), alarm=True)
            return False

    def load(self, filepath):
        with open(filepath) as f:
            d = json.load(f)
        self.name = d.get("name", "PROG")
        self.points = d.get("points", [])
        self.variables = d.get("variables", {})
        self.filepath = filepath
        self.modified = False
        self._reindex()
        log("Loaded " + os.path.basename(filepath))


current_program = Program()

playback_running = False
playback_step_mode = False


def run_playback(program, speed_override=None, forward=True):
    global playback_running
    playback_running = True
    pts = program.points if forward else list(reversed(program.points))
    log("Playback start — {} points, {}".format(
        len(pts), "single step" if playback_step_mode else "continuous"))
    for i, pt in enumerate(pts):
        if not playback_running or estop_active:
            break
        spd = speed_override or pt["speed_pct"]
        set_all_speeds(spd)
        log("→ {}  {}  {}%".format(pt["label"], pt["move_type"], spd))
        if pt["move_type"] == "L":
            linear_move(list(pt["angles"]), steps=30)
        else:
            move_all_to_angles(list(pt["angles"]), wait=True)
        if playback_step_mode:
            playback_running = False
            break
        cnt = pt["cnt"]
        time.sleep((100 - cnt) / 1000.0 if (cnt > 0 and i < len(pts) - 1) else 0.1)
    if playback_running:
        log("Playback complete")
    playback_running = False


def start_playback(program, speed_override=None, forward=True):
    if playback_running:
        return
    if not all_homed:
        log("Home the arm before running a program", alarm=True)
        return
    threading.Thread(target=run_playback,
                     args=(program, speed_override, forward), daemon=True).start()


def stop_playback():
    global playback_running
    playback_running = False
    stop_all("Playback stopped")


def step_playback(program, speed_override=None):
    global playback_step_mode
    playback_step_mode = True
    start_playback(program, speed_override)


# ═════════════════════════════════════════════════════════════════════════
#  PS4 PENDANT   —   button map unchanged from V3
#
#    ✕  stop all / stop AGV        ○  toggle AGV mode
#    □  cycle joint / stop AGV     △  cycle JOINT→WORLD→TOOL
#    L1/R1 speed trim (joint) or fine Z (cartesian)
#    L3/R3 coarse Z (cartesian)    D-pad joint select + speed
#  AGV: left stick Y drives, right stick X turns.
# ═════════════════════════════════════════════════════════════════════════
def get_joint_name():
    return JOINT_ORDER[selected_joint]


def ps4_loop():
    global ps4_connected, control_mode, selected_joint, _last_arm_mode
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
                    ps4_connected = True
                    log("Pendant: " + js.get_name())
                else:
                    ps4_connected = False
                    time.sleep(1)
                    continue

            pygame.event.pump()
            nb, na = js.get_numbuttons(), js.get_numaxes()
            ly = js.get_axis(1); lx = js.get_axis(0)
            rx = js.get_axis(3) if na > 3 else 0.0
            ry = js.get_axis(4) if na > 4 else 0.0
            l1 = js.get_button(4) if nb > 4 else False
            r1 = js.get_button(5) if nb > 5 else False
            cross  = js.get_button(0)
            circle = js.get_button(1) if nb > 1 else False
            sq     = js.get_button(2) if nb > 2 else False
            tri    = js.get_button(3) if nb > 3 else False
            l3     = js.get_button(7) if nb > 7 else False
            r3     = js.get_button(8) if nb > 8 else False
            dpad_x, dpad_y = js.get_hat(0) if js.get_numhats() > 0 else (0, 0)

            if circle and not prev["circle"]:
                if control_mode == "AGV":
                    agv.stop()
                    control_mode = _last_arm_mode
                    log("AGV off → " + control_mode)
                else:
                    _last_arm_mode = control_mode
                    stop_all("Entering AGV mode", alarm=False)
                    agv.stop()
                    control_mode = "AGV"
                    log("AGV mode — left stick drives, right stick turns")
            prev["circle"] = circle

            if tri and not prev["tri"] and control_mode != "AGV":
                control_mode = MODE_CYCLE[(MODE_CYCLE.index(control_mode) + 1)
                                          % len(MODE_CYCLE)]
                _last_arm_mode = control_mode
                stop_all("Mode " + control_mode, alarm=False)
            prev["tri"] = tri

            if sq and not prev["sq"]:
                if control_mode == "AGV":
                    agv.stop(); log("AGV stop")
                else:
                    selected_joint = (selected_joint + 1) % len(JOINT_ORDER)
                    stop_all("Selected " + get_joint_name(), alarm=False)
            prev["sq"] = sq

            if cross and not prev["cross"]:
                agv.stop()
                stop_all("Pendant stop", alarm=False)
            prev["cross"] = cross

            # ── AGV ───────────────────────────────────────────────────
            if control_mode == "AGV":
                agv.set(-ly if abs(ly) > DEADZONE else 0.0,
                        rx if abs(rx) > DEADZONE else 0.0)
                prev["l3"], prev["r3"] = l3, r3
                prev_dpad = (dpad_x, dpad_y)
                time.sleep(0.02)
                continue

            # ── JOINT ─────────────────────────────────────────────────
            if control_mode == "JOINT":
                if dpad_x != prev_dpad[0] and dpad_x != 0:
                    selected_joint = (selected_joint + dpad_x) % len(JOINT_ORDER)
                    stop_all("Selected " + get_joint_name(), alarm=False)
                jname = get_joint_name()
                if dpad_y != prev_dpad[1] and dpad_y != 0:
                    set_motor_speed(jname, motors[jname]["speed"] + 5 * dpad_y)
                    log("{} {}%".format(jname, motors[jname]["speed"]))
                if l1: set_motor_speed(jname, motors[jname]["speed"] - 1)
                if r1: set_motor_speed(jname, motors[jname]["speed"] + 1)

                new = set()
                if abs(ly) > DEADZONE:
                    d = 0 if ly > 0 else 1
                    m = motors[jname]
                    if not m["running"] or m["direction"] != d:
                        start_motor(jname, d)
                    new.add(jname)
                for j in active - new:
                    stop_motor(j)
                active = new

            # ── WORLD / TOOL ──────────────────────────────────────────
            else:
                if dpad_y != prev_dpad[1] and dpad_y != 0:
                    for n in JOINT_ORDER:
                        set_motor_speed(n, motors[n]["speed"] + 5 * dpad_y)
                    log("All axes {}%".format(motors["J1"]["speed"]))
                if all_homed:
                    CD = 0.30
                    if abs(ly) > CD: cartesian_jog("X", -1 if ly > 0 else 1)
                    if abs(ry) > CD: cartesian_jog("Y", -1 if ry > 0 else 1)
                    if l3 and not prev["l3"]: cartesian_jog("Z", 1)
                    if r3 and not prev["r3"]: cartesian_jog("Z", -1)
                    if l1: cartesian_jog("Zf", -1)
                    if r1: cartesian_jog("Zf", 1)
                    if abs(lx) > CD: cartesian_jog("Rx", 1 if lx > 0 else -1)
                    if abs(rx) > CD: cartesian_jog("Ry", -1 if rx > 0 else 1)
                    if dpad_x != prev_dpad[0] and dpad_x != 0:
                        cartesian_jog("Rz", dpad_x)
                active = set()

            prev["l3"], prev["r3"] = l3, r3
            prev_dpad = (dpad_x, dpad_y)
            time.sleep(0.05)

        except Exception as e:
            ps4_connected = False
            js = None
            agv.stop()
            log("Pendant error: {}".format(e), alarm=True)
            time.sleep(1)


def start_ps4():
    global ps4_thread
    if ps4_thread is None or not ps4_thread.is_alive():
        ps4_thread = threading.Thread(target=ps4_loop, daemon=True)
        ps4_thread.start()
        log("Pendant listener started")


# ═════════════════════════════════════════════════════════════════════════
#  SYSTEM TELEMETRY
# ═════════════════════════════════════════════════════════════════════════
def get_ip():
    try:
        r = subprocess.check_output(["hostname", "-I"], timeout=2).decode().strip()
        return r.split()[0] if r else "—"
    except Exception:
        return "—"


def get_memory():
    try:
        with open("/proc/meminfo") as f:
            lines = f.readlines()
        total = int([l for l in lines if "MemTotal" in l][0].split()[1])
        avail = int([l for l in lines if "MemAvailable" in l][0].split()[1])
        return (total - avail) // 1024, total // 1024
    except Exception:
        return 0, 0


def get_uptime():
    try:
        with open("/proc/uptime") as f:
            s = float(f.read().split()[0])
        return "{}h {:02d}m".format(int(s // 3600), int((s % 3600) // 60))
    except Exception:
        return "—"


def get_load():
    try:
        return "{:.2f}".format(os.getloadavg()[0])
    except Exception:
        return "—"


def get_disk():
    try:
        p = subprocess.check_output(["df", "-h", "/"], timeout=2
                                    ).decode().strip().split("\n")[1].split()
        return p[2], p[4]
    except Exception:
        return "—", "—"


# ═════════════════════════════════════════════════════════════════════════
#  WIDGET TOOLKIT
#
#  Everything here is flat: no bevels, no relief, hairlines instead of
#  borders. Colour is applied only to carry state. cfg() is the cheap-redraw
#  gate that keeps this affordable on an H618.
# ═════════════════════════════════════════════════════════════════════════
def cfg(widget, **kw):
    """config() the widget only with values that actually changed."""
    cache = getattr(widget, "_ink_cache", None)
    if cache is None:
        cache = widget._ink_cache = {}
    delta = {k: v for k, v in kw.items() if cache.get(k, _MISS) != v}
    if delta:
        cache.update(delta)
        widget.config(**delta)


_MISS = object()


def track(s):
    """Letterspaced caps. Tk has no letter-spacing, so space the glyphs."""
    return " ".join(s.upper())


def hairline(parent, color=None, pad=0, side=None, **pk):
    f = tk.Frame(parent, bg=color or Ink.line, height=1)
    if side:
        f.pack(side=side, fill="x", padx=pad, **pk)
    else:
        f.pack(fill="x", padx=pad, **pk)
    return f


def vrule(parent, color=None, **pk):
    f = tk.Frame(parent, bg=color or Ink.line, width=1)
    f.pack(fill="y", **pk)
    return f


def text(parent, s="", font=None, fg=None, bg=None, **kw):
    return tk.Label(parent, text=s, font=font or Type.body,
                    fg=fg or Ink.text_2, bg=bg or Ink.ground,
                    bd=0, highlightthickness=0, **kw)


class InkButton(tk.Frame):
    """
    Flat pressable. Hover exists for a mouse, press feedback for a finger.
    variant: quiet | solid | primary | danger
    Set hold=True for jog buttons — press/release callbacks instead of click.
    """

    VARIANTS = {
        "quiet":   (Ink.ground, Ink.text_2, Ink.hover, Ink.text,  Ink.line_lit),
        "solid":   (Ink.panel,  Ink.text_2, Ink.hover, Ink.text,  Ink.asagi),
        "primary": (Ink.panel,  Ink.asagi,  Ink.hover, Ink.asagi, Ink.asagi),
        "danger":  (Ink.panel,  Ink.beni,   Ink.hover, Ink.beni,  Ink.shu),
    }

    def __init__(self, parent, label, command=None, variant="solid",
                 font=None, padx=SP4, pady=SP2, width=None, sub=None,
                 on_press=None, on_release=None, **kw):
        bg, fg, hbg, hfg, pbg = self.VARIANTS.get(variant, self.VARIANTS["solid"])
        super().__init__(parent, bg=bg, bd=0, highlightthickness=0, **kw)
        self._c = (bg, fg, hbg, hfg, pbg)
        self._command = command
        self._on_press = on_press
        self._on_release = on_release
        self._enabled = True
        self._down = False

        self.lbl = tk.Label(self, text=label, font=font or Type.label, fg=fg, bg=bg,
                            bd=0, highlightthickness=0, padx=padx, pady=pady)
        if width:
            self.lbl.config(width=width)
        self.lbl.pack(fill="both", expand=True)
        self.sub = None
        if sub:
            self.sub = tk.Label(self, text=sub, font=Type.micro, fg=Ink.text_4,
                                bg=bg, bd=0, pady=0)
            self.sub.pack(fill="x", pady=(0, SP1))

        for w in self._parts():
            w.bind("<Enter>", self._enter)
            w.bind("<Leave>", self._leave)
            w.bind("<ButtonPress-1>", self._press)
            w.bind("<ButtonRelease-1>", self._release)

    def _parts(self):
        return [w for w in (self, self.lbl, self.sub) if w is not None]

    def _paint(self, bg, fg):
        for w in self._parts():
            cfg(w, bg=bg)
        cfg(self.lbl, fg=fg)
        if self.sub is not None:
            cfg(self.sub, fg=Ink.text_4)

    def _enter(self, _e=None):
        if self._enabled and not self._down:
            self._paint(self._c[2], self._c[3])

    def _leave(self, _e=None):
        self._down = False
        self._paint(self._c[0], self._c[1] if self._enabled else Ink.text_4)

    def _press(self, _e=None):
        if not self._enabled:
            return
        self._down = True
        self._paint(self._c[4], Ink.void)
        if self._on_press:
            self._on_press()

    def _release(self, _e=None):
        if not self._enabled:
            return
        was = self._down
        self._down = False
        self._paint(self._c[2], self._c[3])
        if self._on_release:
            self._on_release()
        if was and self._command:
            self._command()

    def set_label(self, s):
        cfg(self.lbl, text=s)

    def set_base_bg(self, bg):
        """Re-ground the button, e.g. to follow a row that became selected.
        Leaves a button that is currently held down alone."""
        if bg == self._c[0]:
            return
        self._c = (bg,) + self._c[1:]
        if not self._down:
            self._paint(bg, self._c[1] if self._enabled else Ink.text_4)

    def set_colors(self, bg=None, fg=None, hover_bg=None, hover_fg=None, press=None):
        c = list(self._c)
        for i, v in enumerate((bg, fg, hover_bg, hover_fg, press)):
            if v is not None:
                c[i] = v
        self._c = tuple(c)
        self._leave()

    def set_enabled(self, on):
        if on == self._enabled:
            return
        self._enabled = on
        self._paint(self._c[0], self._c[1] if on else Ink.text_4)


class InkSlider(tk.Canvas):
    """Hairline track, filled lead-in, small handle. Drag or tap anywhere."""

    def __init__(self, parent, from_=1, to=100, value=30, width=180, height=26,
                 command=None, accent=None, bg=None):
        bg = bg or Ink.panel
        super().__init__(parent, width=width, height=height, bg=bg,
                         highlightthickness=0, bd=0, cursor="hand2")
        self._from, self._to = from_, to
        self._px_w, self._px_h = width, height   # _w is tkinter's; do not reuse
        self._pad = 9
        self._accent = accent or Ink.asagi
        self._command = command
        self._value = value

        y = height // 2
        self._track = self.create_line(self._pad, y, width - self._pad, y,
                                       fill=Ink.line_lit, width=1)
        self._fill = self.create_line(self._pad, y, self._pad, y,
                                      fill=self._accent, width=2)
        self._knob = self.create_oval(0, 0, 0, 0, fill=self._accent, outline="")
        self.bind("<ButtonPress-1>", self._drag)
        self.bind("<B1-Motion>", self._drag)
        self.bind("<ButtonRelease-1>", self._drag)
        self._redraw()

    def _x_for(self, v):
        span = max(1e-9, self._to - self._from)
        t = (v - self._from) / span
        return self._pad + t * (self._px_w - 2 * self._pad)

    def _redraw(self):
        x, y = self._x_for(self._value), self._px_h // 2
        self.coords(self._fill, self._pad, y, x, y)
        self.coords(self._knob, x - 5, y - 5, x + 5, y + 5)

    def set(self, v, notify=False):
        v = max(self._from, min(self._to, v))
        if v == self._value:
            return
        self._value = v
        self._redraw()
        if notify and self._command:
            self._command(v)

    def get(self):
        return self._value

    def _drag(self, e):
        span = self._px_w - 2 * self._pad
        t = max(0.0, min(1.0, (e.x - self._pad) / max(1, span)))
        v = int(round(self._from + t * (self._to - self._from)))
        if v != self._value:
            self._value = v
            self._redraw()
            if self._command:
                self._command(v)


class TravelBar(tk.Canvas):
    """Where a joint sits inside its physical range. Read-only, very quiet."""

    def __init__(self, parent, width=104, height=20, bg=None):
        bg = bg or Ink.panel
        super().__init__(parent, width=width, height=height, bg=bg,
                         highlightthickness=0, bd=0)
        self._px_w, self._px_h = width, height   # _w is tkinter's; do not reuse
        y = height // 2
        self._track = self.create_line(2, y, width - 2, y, fill=Ink.line_lit, width=1)
        self._mark = self.create_rectangle(0, 0, 0, 0, fill=Ink.text_3, outline="")
        self._pos = None

    def set(self, value, lo, hi, color=None):
        span = max(1e-9, hi - lo)
        t = max(0.0, min(1.0, (value - lo) / span))
        x = 2 + t * (self._px_w - 4)
        key = (round(x, 1), color)
        if key == self._pos:
            return
        self._pos = key
        y = self._px_h // 2
        self.coords(self._mark, x - 1, y - 6, x + 1, y + 6)
        self.itemconfig(self._mark, fill=color or Ink.text_3)

    def blank(self):
        if self._pos == "blank":
            return
        self._pos = "blank"
        self.coords(self._mark, 0, 0, 0, 0)


class Dot(tk.Canvas):
    """A 7px state dot. Dormant is nearly invisible — silence is the default."""

    def __init__(self, parent, bg=None, size=9):
        bg = bg or Ink.raised
        super().__init__(parent, width=size, height=size, bg=bg,
                         highlightthickness=0, bd=0)
        self._o = self.create_oval(1, 1, size - 1, size - 1,
                                   fill=Ink.text_4, outline="")
        self._last = None

    def set(self, on, color, off=None):
        c = color if on else (off or Ink.text_4)
        if c != self._last:
            self._last = c
            self.itemconfig(self._o, fill=c)


def well(parent, width=10, font=None, fg=None, anchor="e", pady=SP1):
    """A recessed numeric readout. Deepest ground so the number floats."""
    f = tk.Frame(parent, bg=Ink.void, bd=0, highlightthickness=0)
    l = tk.Label(f, text="", font=font or Type.data, fg=fg or Ink.text,
                 bg=Ink.void, width=width, anchor=anchor, padx=SP2, pady=pady)
    l.pack()
    return f, l


def section(parent, title, kanji=None, pady=(SP5, SP2), bg=None):
    """Section mark: tracked caps, a kanji whisper, then air. Ma before Kanso."""
    bg = bg or Ink.ground
    f = tk.Frame(parent, bg=bg)
    f.pack(fill="x", pady=pady)
    tk.Label(f, text=track(title), font=Type.label, fg=Ink.text_3, bg=bg,
             anchor="w").pack(side="left", padx=(SP4, 0))
    if kanji and Type.cjk:
        tk.Label(f, text=kanji, font=(Type.cjk, 9), fg=Ink.text_4, bg=bg
                 ).pack(side="left", padx=SP3)
    return f


# ═════════════════════════════════════════════════════════════════════════
#  OVERLAY BASE
#  Yūgen — depth is behind a door, not spread across the main screen.
# ═════════════════════════════════════════════════════════════════════════
class Overlay:
    TITLE = ""
    KANJI = None
    GEOM = "900x620+60+40"
    RESIZE = (False, False)

    def __init__(self, root):
        self.root = root
        self.win = None
        self.body = None

    def toggle(self):
        if self.win and self.win.winfo_exists():
            self.close()
        else:
            self.open()

    def open(self):
        if self.win and self.win.winfo_exists():
            self.win.lift()
            return
        self.win = tk.Toplevel(self.root)
        self.win.title(self.TITLE)
        self.win.configure(bg=Ink.ground)
        self.win.geometry(self.GEOM)
        self.win.resizable(*self.RESIZE)
        self.win.protocol("WM_DELETE_WINDOW", self.close)
        self.win.bind("<Escape>", lambda e: self.close())
        self._header()
        self.body = tk.Frame(self.win, bg=Ink.ground)
        self.body.pack(fill="both", expand=True)
        self.build(self.body)

    def close(self):
        self.on_close()
        if self.win and self.win.winfo_exists():
            self.win.destroy()
        self.win = None

    def alive(self):
        return bool(self.win and self.win.winfo_exists())

    def on_close(self):
        pass

    def build(self, body):
        raise NotImplementedError

    def _header(self):
        bar = tk.Frame(self.win, bg=Ink.raised, height=52)
        bar.pack(fill="x")
        bar.pack_propagate(False)
        tk.Label(bar, text=track(self.TITLE), font=Type.title, fg=Ink.text,
                 bg=Ink.raised).pack(side="left", padx=(SP5, 0))
        if self.KANJI and Type.cjk:
            tk.Label(bar, text=self.KANJI, font=(Type.cjk, 12), fg=Ink.text_4,
                     bg=Ink.raised).pack(side="left", padx=SP4)
        InkButton(bar, "CLOSE", command=self.close, variant="quiet",
                  padx=SP4).pack(side="right", padx=SP4, pady=SP3)
        hairline(self.win)


# ═════════════════════════════════════════════════════════════════════════
#  ON-SCREEN KEYBOARD
# ═════════════════════════════════════════════════════════════════════════
class OSK(Overlay):
    TITLE = "keyboard"
    KANJI = "鍵盤"
    GEOM = "840x310+70+400"

    ROWS = [
        list("`1234567890-="), ["BKSP"],
        list("qwertyuiop[]\\"),
        list("asdfghjkl;'"), ["ENTER"],
        ["SHIFT"] + list("zxcvbnm,./"),
        ["SPACE", "CLEAR", "◀", "▶"],
    ]
    SHIFTED = {"`": "~", "1": "!", "2": "@", "3": "#", "4": "$", "5": "%",
               "6": "^", "7": "&", "8": "*", "9": "(", "0": ")", "-": "_",
               "=": "+", "[": "{", "]": "}", "\\": "|", ";": ":", "'": '"',
               ",": "<", ".": ">", "/": "?"}

    def __init__(self, root):
        super().__init__(root)
        self.target = None
        self.shift = False

    def attach(self, entry):
        self.target = entry

    def build(self, body):
        self.keys_frame = tk.Frame(body, bg=Ink.ground)
        self.keys_frame.pack(expand=True, pady=SP4)
        self._draw()

    def _draw(self):
        for w in self.keys_frame.winfo_children():
            w.destroy()
        # Flatten into visual rows, keeping the wide keys with their row.
        layout = [self.ROWS[0] + self.ROWS[1], self.ROWS[2],
                  self.ROWS[3] + self.ROWS[4], self.ROWS[5], self.ROWS[6]]
        for row in layout:
            rf = tk.Frame(self.keys_frame, bg=Ink.ground)
            rf.pack(pady=SP1)
            for key in row:
                disp = key
                if self.shift:
                    disp = key.upper() if key.isalpha() else self.SHIFTED.get(key, key)
                wide = {"BKSP": 6, "ENTER": 6, "SHIFT": 6, "CLEAR": 6, "SPACE": 22}
                w = wide.get(key, 3)
                variant = "quiet" if key in wide else "solid"
                InkButton(rf, disp, variant=variant, font=Type.body, width=w,
                          padx=SP2, pady=SP2,
                          command=lambda k=key, d=disp: self._press(k, d)
                          ).pack(side="left", padx=2)

    def _press(self, key, disp):
        t = self.target
        if key == "SHIFT":
            self.shift = not self.shift
            self._draw()
            return
        if t is None:
            return
        try:
            if key == "BKSP":
                p = t.index("insert")
                if p > 0:
                    t.delete(p - 1, p)
            elif key == "CLEAR":
                t.delete(0, "end")
            elif key == "ENTER":
                t.event_generate("<Return>")
            elif key == "SPACE":
                t.insert("insert", " ")
            elif key == "◀":
                t.icursor(max(0, t.index("insert") - 1))
            elif key == "▶":
                t.icursor(t.index("insert") + 1)
            else:
                t.insert("insert", disp)
        except Exception:
            pass
        if self.alive():
            self.win.lift()


def bind_osk(entry, osk):
    entry.bind("<FocusIn>", lambda e, w=entry: osk.attach(w))
    return entry


def field_entry(parent, value="", width=10, font=None, fg=None, osk=None):
    """Flat entry: no relief, a hairline underline instead of a sunken box."""
    wrap = tk.Frame(parent, bg=Ink.void)
    e = tk.Entry(wrap, font=font or Type.data, width=width,
                 bg=Ink.void, fg=fg or Ink.text, insertbackground=Ink.asagi,
                 relief="flat", bd=0, highlightthickness=0, justify="right")
    e.insert(0, str(value))
    e.pack(padx=SP2, pady=SP2)
    tk.Frame(wrap, bg=Ink.line_lit, height=1).pack(fill="x")
    if osk:
        bind_osk(e, osk)
    return wrap, e


# ═════════════════════════════════════════════════════════════════════════
#  AGV PENDANT
# ═════════════════════════════════════════════════════════════════════════
class AGVPanel(Overlay):
    TITLE = "agv pendant"
    KANJI = "搬送車"
    GEOM = "440x600+780+30"

    JOY, RAD, NUB = 300, 122, 30

    def __init__(self, root):
        super().__init__(root)
        self._canvas = self._nub = None
        self._dragging = False
        self._last_send = 0.0
        self._cx = self._cy = self.JOY // 2
        self.telem = {}

    def on_close(self):
        agv.stop()

    def build(self, body):
        st = tk.Frame(body, bg=Ink.ground)
        st.pack(fill="x", pady=(SP4, 0))
        tk.Label(st, text=track("link"), font=Type.label, fg=Ink.text_3,
                 bg=Ink.ground).pack(side="left", padx=(SP5, SP3))
        self.link_dot = Dot(st, bg=Ink.ground)
        self.link_dot.pack(side="left")
        self.link_lbl = text(st, "—", Type.body, Ink.text_3)
        self.link_lbl.pack(side="left", padx=SP2)
        self.src_lbl = text(st, ESP32_IP, Type.body, Ink.text_4)
        self.src_lbl.pack(side="right", padx=SP5)

        self.mode_lbl = text(body, "standby", Type.body, Ink.text_3)
        self.mode_lbl.pack(pady=(SP4, SP3))

        mid = tk.Frame(body, bg=Ink.ground)
        mid.pack()
        c = tk.Canvas(mid, width=self.JOY, height=self.JOY, bg=Ink.void,
                      highlightthickness=0, bd=0, cursor="hand2")
        c.pack()
        self._canvas = c
        cx = cy = self.JOY // 2
        R = self.RAD
        c.create_oval(cx - R, cy - R, cx + R, cy + R, outline=Ink.line_lit, width=1)
        c.create_oval(cx - R // 2, cy - R // 2, cx + R // 2, cy + R // 2,
                      outline=Ink.line, width=1)
        c.create_line(cx - R, cy, cx + R, cy, fill=Ink.line)
        c.create_line(cx, cy - R, cx, cy + R, fill=Ink.line)
        for txt, x, y in [("▲", cx, cy - R - 14), ("▼", cx, cy + R + 14),
                          ("◀", cx - R - 14, cy), ("▶", cx + R + 14, cy)]:
            c.create_text(x, y, text=txt, fill=Ink.text_4, font=Type.micro)
        self._nub = c.create_oval(cx - self.NUB, cy - self.NUB,
                                  cx + self.NUB, cy + self.NUB,
                                  fill=Ink.ai, outline="")
        c.bind("<ButtonPress-1>", self._joy_press)
        c.bind("<B1-Motion>", self._joy_move)
        c.bind("<ButtonRelease-1>", self._joy_release)
        for seq in ("<Button-4>", "<Button-5>", "<MouseWheel>"):
            c.bind(seq, lambda e: "break")

        InkButton(body, "STOP AGV", command=self._stop, variant="danger",
                  font=Type.body_b, pady=SP3).pack(fill="x", padx=SP5, pady=SP5)

        hairline(body)
        strip = tk.Frame(body, bg=Ink.ground)
        strip.pack(fill="x", side="bottom", pady=SP4)
        for key, label in [("t", "throttle"), ("r", "steering"),
                           ("left", "l-motor"), ("right", "r-motor")]:
            cell = tk.Frame(strip, bg=Ink.ground)
            cell.pack(side="left", expand=True)
            v = tk.Label(cell, text="—", font=Type.data, fg=Ink.text_3, bg=Ink.ground)
            v.pack()
            tk.Label(cell, text=track(label), font=Type.micro, fg=Ink.text_4,
                     bg=Ink.ground).pack()
            self.telem[key] = v
        self._tick()

    # ── joystick ──────────────────────────────────────────────────────
    def _joy_press(self, e):
        self._dragging = True
        self._last_send = 0.0
        self._joy_move(e)

    def _joy_move(self, e):
        if not self._dragging:
            return
        dx, dy = e.x - self._cx, e.y - self._cy
        d = math.hypot(dx, dy)
        if d > self.RAD:
            dx, dy = dx / d * self.RAD, dy / d * self.RAD
        self._place_nub(dx, dy)
        now = time.time()
        if now - self._last_send < 0.02:
            return
        self._last_send = now
        t = -(dy / self.RAD)
        r = dx / self.RAD
        PDEAD = 0.08
        if control_mode == "AGV":
            agv.set(t if abs(t) > PDEAD else 0.0, r if abs(r) > PDEAD else 0.0)

    def _joy_release(self, _e):
        self._dragging = False
        self._place_nub(0, 0)
        agv.stop()

    def _place_nub(self, dx, dy):
        n, cx, cy = self.NUB, self._cx, self._cy
        self._canvas.coords(self._nub, cx + dx - n, cy + dy - n,
                            cx + dx + n, cy + dy + n)

    def _stop(self):
        agv.stop()
        log("AGV stop")

    def _tick(self):
        if not self.alive():
            return
        st = agv.status_json
        t, r = agv.translate, agv.rotate
        is_agv = (control_mode == "AGV")

        if not self._dragging:
            dx, dy = r * self.RAD, -t * self.RAD
            d = math.hypot(dx, dy)
            if d > self.RAD:
                dx, dy = dx / d * self.RAD, dy / d * self.RAD
            self._place_nub(dx, dy)
        self._canvas.itemconfig(self._nub, fill=Ink.ai if is_agv else Ink.line_lit)

        cfg(self.mode_lbl,
            text="active  —  joystick or pendant" if is_agv
                 else "standby  —  ○ on the pendant to activate",
            fg=Ink.ai if is_agv else Ink.text_3)
        self.link_dot.set(agv.online, Ink.wakatake)
        cfg(self.link_lbl, text="online" if agv.online else "offline",
            fg=Ink.text_2 if agv.online else Ink.text_4)
        cfg(self.src_lbl, text="{}  {}".format(ESP32_IP, st.get("source", "—")))
        cfg(self.telem["t"], text="{:+.2f}".format(t),
            fg=Ink.ai if abs(t) > 0.05 else Ink.text_3)
        cfg(self.telem["r"], text="{:+.2f}".format(r),
            fg=Ink.ai if abs(r) > 0.05 else Ink.text_3)
        cfg(self.telem["left"], text=str(st.get("left", "—")),
            fg=Ink.ai if "left" in st else Ink.text_3)
        cfg(self.telem["right"], text=str(st.get("right", "—")),
            fg=Ink.ai if "right" in st else Ink.text_3)
        self.win.after(120, self._tick)


# ═════════════════════════════════════════════════════════════════════════
#  TARGET  —  cartesian goal + inverse kinematics
# ═════════════════════════════════════════════════════════════════════════
class TargetPanel(Overlay):
    TITLE = "target"
    KANJI = "目標"
    GEOM = "620x520+180+60"

    def __init__(self, root, osk):
        super().__init__(root)
        self.osk = osk
        self.e = {}
        self.solution = None

    def build(self, body):
        section(body, "current tool centre point", "現在位置", pady=(SP5, SP3))
        cur = tk.Frame(body, bg=Ink.ground)
        cur.pack(fill="x", padx=SP5)
        self.cur = {}
        for ax in ("X", "Y", "Z"):
            cell = tk.Frame(cur, bg=Ink.ground)
            cell.pack(side="left", expand=True, fill="x")
            tk.Label(cell, text=track(ax), font=Type.label, fg=Ink.text_3,
                     bg=Ink.ground).pack(anchor="w")
            l = tk.Label(cell, text="—", font=Type.data_lg, fg=Ink.text,
                         bg=Ink.ground, anchor="w")
            l.pack(anchor="w")
            self.cur[ax] = l
        tk.Label(cur, text="mm", font=Type.label, fg=Ink.text_4,
                 bg=Ink.ground).pack(side="left", padx=SP3)

        section(body, "goal position", "位置")
        r1 = tk.Frame(body, bg=Ink.ground)
        r1.pack(fill="x", padx=SP5)
        for ax in ("X", "Y", "Z"):
            cell = tk.Frame(r1, bg=Ink.ground)
            cell.pack(side="left", padx=(0, SP4))
            tk.Label(cell, text=track(ax), font=Type.label, fg=Ink.asagi,
                     bg=Ink.ground).pack(anchor="w", pady=(0, SP1))
            wrap, e = field_entry(cell, "0.0", width=9, osk=self.osk)
            wrap.pack()
            self.e[ax] = e
        InkButton(r1, "← FROM\nCURRENT", command=self._from_current, variant="quiet",
                  font=Type.micro).pack(side="left", padx=SP3, pady=(SP4, 0))

        section(body, "goal orientation", "姿勢")
        r2 = tk.Frame(body, bg=Ink.ground)
        r2.pack(fill="x", padx=SP5)
        for ax in ("Rx", "Ry", "Rz"):
            cell = tk.Frame(r2, bg=Ink.ground)
            cell.pack(side="left", padx=(0, SP4))
            tk.Label(cell, text=track(ax), font=Type.label, fg=Ink.yamabuki,
                     bg=Ink.ground).pack(anchor="w", pady=(0, SP1))
            wrap, e = field_entry(cell, "0.0", width=9, fg=Ink.text, osk=self.osk)
            wrap.pack()
            self.e[ax] = e
        tk.Label(r2, text="deg   ·   all zero solves position only",
                 font=Type.micro, fg=Ink.text_4, bg=Ink.ground
                 ).pack(side="left", padx=SP3, pady=(SP4, 0))

        section(body, "solution", "解")
        self.result = tk.Label(body, text="—", font=Type.data, fg=Ink.text_3,
                               bg=Ink.ground, anchor="w", justify="left")
        self.result.pack(fill="x", padx=SP5, pady=SP2)

        acts = tk.Frame(body, bg=Ink.ground)
        acts.pack(fill="x", padx=SP5, pady=SP5, side="bottom")
        InkButton(acts, "SOLVE", command=self.solve, variant="primary",
                  font=Type.body_b, padx=SP5, pady=SP3).pack(side="left")
        self.btn_move = InkButton(acts, "MOVE TO SOLUTION", command=self.move,
                                  variant="solid", font=Type.body_b,
                                  padx=SP5, pady=SP3)
        self.btn_move.pack(side="left", padx=SP3)
        self.btn_move.set_enabled(False)
        InkButton(acts, "SET CURRENT AS HOME", command=self._set_home,
                  variant="quiet", padx=SP4, pady=SP3).pack(side="right")
        self._tick()

    def _from_current(self):
        tcp = forward_kinematics([motors[n]["angle_deg"] for n in JOINT_ORDER])
        if not tcp:
            return
        for i, ax in enumerate(("X", "Y", "Z")):
            self.e[ax].delete(0, "end")
            self.e[ax].insert(0, "{:.2f}".format(tcp[i]))

    def _set_home(self):
        zero_positions(mark_homed=True)
        log("Current pose accepted as home — switches not used")

    def solve(self):
        try:
            v = {k: float(self.e[k].get()) for k in ("X", "Y", "Z", "Rx", "Ry", "Rz")}
        except ValueError:
            cfg(self.result, text="invalid input", fg=Ink.shu)
            return
        sol = inverse_kinematics(v["X"], v["Y"], v["Z"], v["Rx"], v["Ry"], v["Rz"])
        if sol is None:
            self.solution = None
            self.btn_move.set_enabled(False)
            cfg(self.result, text="no solution — unreachable or outside joint limits",
                fg=Ink.shu)
        else:
            self.solution = sol
            self.btn_move.set_enabled(all_homed)
            cfg(self.result, fg=Ink.wakatake, text="   ".join(
                "{} {:+7.2f}".format(JOINT_ORDER[i], sol[i]) for i in range(6)))
            log("IK solved")

    def move(self):
        if self.solution is None:
            return
        if not all_homed:
            log("Home the arm before moving to a target", alarm=True)
            return
        threading.Thread(target=move_all_to_angles,
                         args=(self.solution, False), daemon=True).start()
        log("Moving to solved target")

    def _tick(self):
        if not self.alive():
            return
        tcp = forward_kinematics([motors[n]["angle_deg"] for n in JOINT_ORDER])
        for i, ax in enumerate(("X", "Y", "Z")):
            cfg(self.cur[ax], text="{:+.2f}".format(tcp[i]) if tcp else "—")
        self.win.after(250, self._tick)


# ═════════════════════════════════════════════════════════════════════════
#  PROGRAM EDITOR
# ═════════════════════════════════════════════════════════════════════════
class ProgramEditor(Overlay):
    TITLE = "program"
    KANJI = "教示"
    GEOM = "1000x640+30+30"
    RESIZE = (True, True)

    def __init__(self, root, osk):
        super().__init__(root)
        self.osk = osk
        self.program = current_program
        self.sel = None
        self.speed = 50
        self._sig = None

    def build(self, body):
        tools = tk.Frame(body, bg=Ink.ground)
        tools.pack(fill="x", padx=SP4, pady=SP3)
        for label, cmd, variant in [
                ("NEW", self._new, "quiet"), ("OPEN", self._load, "quiet"),
                ("SAVE", self._save, "quiet"), ("SAVE AS", self._save_as, "quiet"),
                ("RECORD", self._record, "primary"), ("EDIT", self._edit, "solid"),
                ("UP", lambda: self._shift(-1), "solid"),
                ("DOWN", lambda: self._shift(1), "solid"),
                ("DELETE", self._delete, "danger"),
                ("REGISTERS", self._variables, "quiet")]:
            InkButton(tools, label, command=cmd, variant=variant,
                      padx=SP3).pack(side="left", padx=2)

        hairline(body)
        split = tk.Frame(body, bg=Ink.ground)
        split.pack(fill="both", expand=True)

        left = tk.Frame(split, bg=Ink.ground)
        left.pack(side="left", fill="both", expand=True, padx=(SP4, SP3), pady=SP3)
        tk.Label(left, text=self._header_line(), font=(Type.mono, 9),
                 fg=Ink.text_4, bg=Ink.ground, anchor="w"
                 ).pack(fill="x", padx=SP2)
        hairline(left, pady=(SP1, 0))
        lw = tk.Frame(left, bg=Ink.void)
        lw.pack(fill="both", expand=True)
        self.listbox = tk.Listbox(
            lw, bg=Ink.void, fg=Ink.text_2, font=(Type.mono, 9),
            selectbackground=Ink.sel, selectforeground=Ink.asagi,
            activestyle="none", relief="flat", bd=0, highlightthickness=0)
        sb = tk.Scrollbar(lw, orient="vertical", command=self.listbox.yview,
                          bg=Ink.panel, troughcolor=Ink.void, bd=0,
                          highlightthickness=0, relief="flat", width=10)
        self.listbox.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.listbox.pack(fill="both", expand=True, padx=SP2, pady=SP2)
        self.listbox.bind("<<ListboxSelect>>", self._on_select)
        self.listbox.bind("<Double-Button-1>", lambda e: self._edit())

        right = tk.Frame(split, bg=Ink.ground, width=270)
        right.pack(side="right", fill="y")
        right.pack_propagate(False)

        section(right, "playback", "再生", pady=(SP4, SP3))
        row = tk.Frame(right, bg=Ink.ground)
        row.pack(fill="x", padx=SP4)
        InkButton(row, "RUN", variant="primary", padx=SP4, pady=SP3,
                  command=lambda: start_playback(self.program, self.speed, True)
                  ).pack(side="left")
        InkButton(row, "REVERSE", variant="solid", padx=SP3, pady=SP3,
                  command=lambda: start_playback(self.program, self.speed, False)
                  ).pack(side="left", padx=SP2)
        InkButton(row, "STOP", variant="danger", padx=SP3, pady=SP3,
                  command=stop_playback).pack(side="right")
        row2 = tk.Frame(right, bg=Ink.ground)
        row2.pack(fill="x", padx=SP4, pady=SP2)
        InkButton(row2, "SINGLE STEP", variant="quiet", padx=SP3,
                  command=lambda: step_playback(self.program, self.speed)
                  ).pack(side="left")
        self.step_btn = InkButton(row2, "STEP MODE  OFF", variant="quiet",
                                  padx=SP3, command=self._toggle_step)
        self.step_btn.pack(side="right")

        sf = tk.Frame(right, bg=Ink.ground)
        sf.pack(fill="x", padx=SP4, pady=(SP4, 0))
        tk.Label(sf, text=track("speed"), font=Type.label, fg=Ink.text_3,
                 bg=Ink.ground).pack(side="left")
        self.spd_lbl = tk.Label(sf, text="50%", font=Type.data, fg=Ink.text,
                                bg=Ink.ground)
        self.spd_lbl.pack(side="right")
        InkSlider(right, 1, 100, 50, width=238, bg=Ink.ground,
                  command=self._set_speed).pack(padx=SP4, pady=SP2)

        section(right, "point", "点")
        self.detail = tk.Label(right, text="—", font=(Type.mono, 9),
                               fg=Ink.text_2, bg=Ink.ground, justify="left",
                               anchor="nw")
        self.detail.pack(fill="both", expand=True, padx=SP4, pady=SP2)

        self.status = tk.Label(body, text="", font=Type.label, fg=Ink.text_4,
                               bg=Ink.raised, anchor="w", padx=SP4, pady=SP2)
        self.status.pack(fill="x", side="bottom")
        self.refresh(force=True)

    def _set_speed(self, v):
        self.speed = v
        cfg(self.spd_lbl, text="{}%".format(v))

    def _toggle_step(self):
        global playback_step_mode
        playback_step_mode = not playback_step_mode
        self.step_btn.set_label("STEP MODE  " + ("ON" if playback_step_mode else "OFF"))

    # ── list ──────────────────────────────────────────────────────────
    ROW_FMT = "{:>3}  {}  {:<7} {}  {:>4} {:>3}  {}"

    @classmethod
    def _line(cls, pt):
        return cls.ROW_FMT.format(
            pt["index"], pt["move_type"], pt["label"],
            " ".join("{:+7.1f}".format(a) for a in pt["angles"]),
            "{}%".format(pt["speed_pct"]), pt["cnt"], pt["comment"])

    @classmethod
    def _header_line(cls):
        return cls.ROW_FMT.format(
            "NO", "T", "POINT",
            " ".join("{:>7}".format(n) for n in JOINT_ORDER),
            "SPD", "CNT", "NOTE")

    def refresh(self, force=False):
        if not self.alive():
            return
        sig = (len(self.program.points), self.program.modified,
               self.program.name, self.sel,
               tuple(id(p) for p in self.program.points))
        if sig == self._sig and not force:
            return
        self._sig = sig
        self.listbox.delete(0, "end")
        for pt in self.program.points:
            self.listbox.insert("end", self._line(pt))
        if self.sel is not None and self.sel < len(self.program.points):
            self.listbox.selection_set(self.sel)
            self.listbox.see(self.sel)
        cfg(self.status, text="{}   ·   {} points   ·   {}{}".format(
            self.program.name, len(self.program.points),
            self.program.filepath or "unsaved",
            "   ·   modified" if self.program.modified else ""),
            fg=Ink.yamabuki if self.program.modified else Ink.text_4)

    def _on_select(self, _e=None):
        s = self.listbox.curselection()
        if not s:
            return
        self.sel = s[0]
        self._detail(self.program.points[self.sel])

    def _detail(self, pt):
        tcp = pt.get("tcp", [0, 0, 0])
        lines = ["{}   {}".format(pt["label"],
                                  "joint" if pt["move_type"] == "J" else "linear"),
                 "speed {}%   cnt {}".format(pt["speed_pct"], pt["cnt"]),
                 pt["comment"] or "—", ""]
        lines += ["{}  {:+9.3f}".format(JOINT_ORDER[i], pt["angles"][i])
                  for i in range(6)]
        lines += ["", "X   {:+9.3f}".format(tcp[0]),
                  "Y   {:+9.3f}".format(tcp[1]),
                  "Z   {:+9.3f}".format(tcp[2])]
        cfg(self.detail, text="\n".join(lines))

    # ── actions ───────────────────────────────────────────────────────
    def _record(self):
        pt = self.program.add_point(move_type="J")
        self.sel = len(self.program.points) - 1
        self.refresh(force=True)
        self._detail(pt)

    def _delete(self):
        if self.sel is None:
            return
        if messagebox.askyesno("Delete", "Delete P[{}]?".format(self.sel + 1),
                               parent=self.win):
            self.program.delete_point(self.sel)
            self.sel = max(0, self.sel - 1) if self.program.points else None
            self.refresh(force=True)

    def _shift(self, delta):
        if self.sel is None:
            return
        if self.program.move_point(self.sel, delta):
            self.sel += delta
            self.refresh(force=True)

    def _new(self):
        global current_program
        if self.program.modified and not messagebox.askyesno(
                "Unsaved", "Discard changes?", parent=self.win):
            return
        current_program = Program()
        self.program = current_program
        self.sel = None
        self.refresh(force=True)

    def _save(self):
        if not self.program.filepath:
            self._save_as()
        else:
            self.program.save()
            self.refresh(force=True)

    def _save_as(self):
        fp = filedialog.asksaveasfilename(
            parent=self.win, title="Save program", defaultextension=".json",
            filetypes=[("Robot program", "*.json"), ("All files", "*.*")],
            initialdir=os.path.expanduser("~"))
        if fp:
            self.program.name = os.path.splitext(os.path.basename(fp))[0]
            self.program.save(fp)
            self.refresh(force=True)

    def _load(self):
        global current_program
        if self.program.modified and not messagebox.askyesno(
                "Unsaved", "Discard changes?", parent=self.win):
            return
        fp = filedialog.askopenfilename(
            parent=self.win, title="Open program",
            filetypes=[("Robot program", "*.json"), ("All files", "*.*")],
            initialdir=os.path.expanduser("~"))
        if fp:
            current_program = Program()
            current_program.load(fp)
            self.program = current_program
            self.sel = None
            self.refresh(force=True)

    def _edit(self):
        if self.sel is None:
            return
        self._point_editor(self.program.points[self.sel])

    def _point_editor(self, pt):
        w = tk.Toplevel(self.win)
        w.title("Edit " + pt["label"])
        w.configure(bg=Ink.ground)
        w.geometry("480x560+220+60")
        w.resizable(False, False)
        w.bind("<Escape>", lambda e: w.destroy())
        head = tk.Frame(w, bg=Ink.raised, height=48)
        head.pack(fill="x"); head.pack_propagate(False)
        tk.Label(head, text=track("edit " + pt["label"]), font=Type.title,
                 fg=Ink.text, bg=Ink.raised).pack(side="left", padx=SP5)
        hairline(w)

        f = tk.Frame(w, bg=Ink.ground)
        f.pack(fill="both", expand=True, padx=SP5, pady=SP4)
        es = {}

        def field(label, value, row, fg=None):
            tk.Label(f, text=track(label), font=Type.label, fg=Ink.text_3,
                     bg=Ink.ground, anchor="w").grid(row=row, column=0,
                                                     sticky="w", pady=SP1)
            wrap, e = field_entry(f, value, width=12, fg=fg, osk=self.osk)
            wrap.grid(row=row, column=1, sticky="e", pady=SP1)
            es[label] = e

        mt = tk.StringVar(value=pt["move_type"])
        tk.Label(f, text=track("move"), font=Type.label, fg=Ink.text_3,
                 bg=Ink.ground).grid(row=0, column=0, sticky="w", pady=SP1)
        mf = tk.Frame(f, bg=Ink.ground)
        mf.grid(row=0, column=1, sticky="e")
        for val, lab in (("J", "joint"), ("L", "linear")):
            tk.Radiobutton(mf, text=lab, variable=mt, value=val, font=Type.body,
                           fg=Ink.text_2, bg=Ink.ground, selectcolor=Ink.void,
                           activebackground=Ink.ground, activeforeground=Ink.text,
                           bd=0, highlightthickness=0).pack(side="left")
        field("speed %", pt["speed_pct"], 1)
        field("cnt", pt["cnt"], 2)
        field("note", pt["comment"], 3)
        tk.Label(f, text=track("joint angles"), font=Type.label, fg=Ink.asagi,
                 bg=Ink.ground).grid(row=4, column=0, sticky="w", pady=(SP4, SP1))
        for i, n in enumerate(JOINT_ORDER):
            field(n, "{:.4f}".format(pt["angles"][i]), 5 + i)
        f.grid_columnconfigure(0, weight=1)

        def apply_changes():
            try:
                pt["move_type"] = mt.get()
                pt["speed_pct"] = int(es["speed %"].get())
                pt["cnt"] = int(es["cnt"].get())
                pt["comment"] = es["note"].get()
                for i, n in enumerate(JOINT_ORDER):
                    pt["angles"][i] = float(es[n].get())
                pt["tcp"] = forward_kinematics(pt["angles"]) or pt.get("tcp", [0, 0, 0])
                self.program.modified = True
                self.refresh(force=True)
                self._detail(pt)
                log("{} edited".format(pt["label"]))
                w.destroy()
            except Exception as ex:
                messagebox.showerror("Invalid", str(ex), parent=w)

        def from_current():
            for n in JOINT_ORDER:
                es[n].delete(0, "end")
                es[n].insert(0, "{:.4f}".format(motors[n]["angle_deg"]))

        def go_here():
            if not all_homed:
                messagebox.showwarning("Not homed", "Home the arm first.", parent=w)
                return
            threading.Thread(target=move_all_to_angles,
                             args=(pt["angles"], False), daemon=True).start()

        bar = tk.Frame(w, bg=Ink.ground)
        bar.pack(fill="x", padx=SP5, pady=SP4)
        InkButton(bar, "APPLY", command=apply_changes, variant="primary",
                  padx=SP5, pady=SP3).pack(side="left")
        InkButton(bar, "TEACH FROM CURRENT", command=from_current, variant="quiet",
                  padx=SP3, pady=SP3).pack(side="left", padx=SP2)
        InkButton(bar, "MOVE HERE", command=go_here, variant="solid",
                  padx=SP3, pady=SP3).pack(side="left")
        InkButton(bar, "CANCEL", command=w.destroy, variant="quiet",
                  padx=SP3, pady=SP3).pack(side="right")

    def _variables(self):
        w = tk.Toplevel(self.win)
        w.title("Registers")
        w.configure(bg=Ink.ground)
        w.geometry("560x480+240+90")
        w.bind("<Escape>", lambda e: w.destroy())
        head = tk.Frame(w, bg=Ink.raised, height=48)
        head.pack(fill="x"); head.pack_propagate(False)
        tk.Label(head, text=track("registers"), font=Type.title, fg=Ink.text,
                 bg=Ink.raised).pack(side="left", padx=SP5)
        hairline(w)

        lw = tk.Frame(w, bg=Ink.void)
        lw.pack(fill="both", expand=True, padx=SP4, pady=SP4)
        lb = tk.Listbox(lw, bg=Ink.void, fg=Ink.text_2, font=(Type.mono, 9),
                        selectbackground=Ink.sel, selectforeground=Ink.asagi,
                        activestyle="none", relief="flat", bd=0,
                        highlightthickness=0)
        lb.pack(fill="both", expand=True, padx=SP2, pady=SP2)

        def refresh_vars():
            lb.delete(0, "end")
            for k, v in self.program.variables.items():
                val = v.get("value", "") if isinstance(v, dict) else v
                cmt = v.get("comment", "") if isinstance(v, dict) else ""
                if isinstance(val, dict):
                    val = "pose"
                lb.insert("end", "  {:<18} {:<20} {}".format(k, str(val)[:20], cmt))

        refresh_vars()
        af = tk.Frame(w, bg=Ink.ground)
        af.pack(fill="x", padx=SP4)
        fields = {}
        for label, wdt in (("name", 12), ("value", 12), ("note", 16)):
            cell = tk.Frame(af, bg=Ink.ground)
            cell.pack(side="left", padx=(0, SP3))
            tk.Label(cell, text=track(label), font=Type.micro, fg=Ink.text_4,
                     bg=Ink.ground).pack(anchor="w")
            wrap, e = field_entry(cell, "", width=wdt, osk=self.osk)
            wrap.pack()
            fields[label] = e

        def add_var():
            n = fields["name"].get().strip()
            if not n:
                return
            self.program.variables[n] = {"value": fields["value"].get().strip(),
                                         "comment": fields["note"].get().strip()}
            self.program.modified = True
            refresh_vars()

        def del_var():
            s = lb.curselection()
            keys = list(self.program.variables.keys())
            if s and s[0] < len(keys):
                del self.program.variables[keys[s[0]]]
                self.program.modified = True
                refresh_vars()

        def store_pose():
            n = fields["name"].get().strip() or "PR[{}]".format(
                len(self.program.variables) + 1)
            angles = [motors[j]["angle_deg"] for j in JOINT_ORDER]
            self.program.variables[n] = {
                "value": {"angles": angles,
                          "tcp": forward_kinematics(angles) or [0, 0, 0]},
                "comment": fields["note"].get().strip() or "position register"}
            self.program.modified = True
            refresh_vars()
            log("Position register {} stored".format(n))

        bar = tk.Frame(w, bg=Ink.ground)
        bar.pack(fill="x", padx=SP4, pady=SP4)
        InkButton(bar, "ADD / UPDATE", command=add_var, variant="primary",
                  padx=SP4, pady=SP3).pack(side="left")
        InkButton(bar, "STORE CURRENT POSE", command=store_pose, variant="solid",
                  padx=SP3, pady=SP3).pack(side="left", padx=SP2)
        InkButton(bar, "DELETE", command=del_var, variant="danger",
                  padx=SP3, pady=SP3).pack(side="right")


# ═════════════════════════════════════════════════════════════════════════
#  SYSTEM  —  telemetry, tuning, faults
#  Custom tab strip rather than ttk.Notebook, which cannot be themed to match.
# ═════════════════════════════════════════════════════════════════════════
class SystemMenu(Overlay):
    TITLE = "system"
    KANJI = "設定"
    GEOM = "900x640+60+30"
    RESIZE = (True, True)

    TABS = [("status", "状態"), ("motors", "軸"), ("geometry", "寸法"),
            ("faults", "警報"), ("network", "通信")]

    def __init__(self, root, osk):
        super().__init__(root)
        self.osk = osk
        self.tab = "status"
        self.tab_btns = {}
        self.pages = {}
        self._alarm_len = -1

    def build(self, body):
        strip = tk.Frame(body, bg=Ink.ground)
        strip.pack(fill="x", padx=SP4, pady=SP3)
        for name, kanji in self.TABS:
            b = InkButton(strip, track(name), variant="quiet", padx=SP4, pady=SP2,
                          sub=kanji if Type.cjk else None,
                          command=lambda n=name: self.show(n))
            b.pack(side="left", padx=(0, SP2))
            self.tab_btns[name] = b
        hairline(body)

        self.holder = tk.Frame(body, bg=Ink.ground)
        self.holder.pack(fill="both", expand=True)
        for name, _ in self.TABS:
            p = tk.Frame(self.holder, bg=Ink.ground)
            self.pages[name] = p
            getattr(self, "_page_" + name)(p)
        self.show("status")
        self._tick()

    def show(self, name):
        self.tab = name
        for p in self.pages.values():
            p.pack_forget()
        self.pages[name].pack(fill="both", expand=True)
        for n, b in self.tab_btns.items():
            b.set_colors(bg=Ink.ground, fg=Ink.asagi if n == name else Ink.text_3,
                         hover_bg=Ink.hover, hover_fg=Ink.text, press=Ink.asagi)

    # ── status ────────────────────────────────────────────────────────
    def _page_status(self, p):
        self.si = {}
        grid = tk.Frame(p, bg=Ink.ground)
        grid.pack(fill="both", expand=True, padx=SP5, pady=SP4)
        rows = ["cpu temp", "fan", "load", "memory", "disk", "uptime",
                "session", "ip address", "solver", "homed", "e-stop",
                "axes moving", "faults logged"]
        for i, label in enumerate(rows):
            r = tk.Frame(grid, bg=Ink.ground)
            r.pack(fill="x")
            tk.Label(r, text=track(label), font=Type.label, fg=Ink.text_3,
                     bg=Ink.ground, width=22, anchor="w").pack(side="left", pady=SP2)
            v = tk.Label(r, text="—", font=Type.data, fg=Ink.text,
                         bg=Ink.ground, anchor="w")
            v.pack(side="left")
            self.si[label] = v
            if i < len(rows) - 1:
                hairline(grid, color=Ink.line)

    def _refresh_status(self):
        um, tm = get_memory()
        du, dp = get_disk()
        sess = int(time.time() - start_time)
        temp = get_cpu_temp()
        solver = ("ready" if _kin_ready.is_set()
                  else ("failed" if _kin_error else "loading…"))
        vals = {
            "cpu temp": ("{:.1f} °C".format(temp),
                         Ink.shu if temp > FAN_T_HI else
                         Ink.yamabuki if temp > FAN_T_LO else Ink.text),
            "fan": ("{}%{}".format(fan_speed, "  auto" if fan_auto else "  manual"), Ink.text),
            "load": (get_load(), Ink.text),
            "memory": ("{} / {} MB".format(um, tm), Ink.text),
            "disk": ("{}  ({})".format(du, dp), Ink.text),
            "uptime": (get_uptime(), Ink.text),
            "session": ("{}h {:02d}m".format(sess // 3600, (sess % 3600) // 60), Ink.text),
            "ip address": (get_ip(), Ink.text),
            "solver": (solver, Ink.wakatake if _kin_ready.is_set()
                       else Ink.shu if _kin_error else Ink.yamabuki),
            "homed": ("yes" if all_homed else "no",
                      Ink.wakatake if all_homed else Ink.yamabuki),
            "e-stop": ("ACTIVE" if estop_active else "clear",
                       Ink.shu if estop_active else Ink.wakatake),
            "axes moving": (str(sum(1 for m in motors.values() if m["running"])), Ink.text),
            "faults logged": (str(len(alarm_log)),
                              Ink.yamabuki if alarm_log else Ink.text_3),
        }
        for k, (v, c) in vals.items():
            if k in self.si:
                cfg(self.si[k], text=v, fg=c)

    # ── motors ────────────────────────────────────────────────────────
    def _page_motors(self, p):
        wrap = tk.Frame(p, bg=Ink.ground)
        wrap.pack(fill="both", expand=True)
        c = tk.Canvas(wrap, bg=Ink.ground, highlightthickness=0, bd=0)
        sb = tk.Scrollbar(wrap, orient="vertical", command=c.yview, bd=0,
                          bg=Ink.panel, troughcolor=Ink.ground, width=10,
                          relief="flat", highlightthickness=0)
        inner = tk.Frame(c, bg=Ink.ground)
        inner.bind("<Configure>", lambda e: c.configure(scrollregion=c.bbox("all")))
        c.create_window((0, 0), window=inner, anchor="nw")
        c.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        c.pack(side="left", fill="both", expand=True)

        self.mc = {}
        FIELDS = [("steps/deg", 8), ("max hz", 8), ("ik lo", 7), ("ik hi", 7),
                  ("phys lo", 7), ("phys hi", 7), ("home at", 7)]
        for name in JOINT_ORDER:
            m = motors[name]
            lo_i, hi_i = JOINT_LIMITS_IK[name]
            lo_p, hi_p = JOINT_LIMITS_PHYS[name]
            block = tk.Frame(inner, bg=Ink.ground)
            block.pack(fill="x", padx=SP5, pady=(SP4, 0))
            tk.Label(block, text=name, font=Type.data_lg, fg=Ink.asagi,
                     bg=Ink.ground, width=4, anchor="w").pack(side="left")
            es = {}
            vals = [STEPS_PER_DEG[name], m["target_freq"], lo_i, hi_i,
                    lo_p, hi_p, HOME_TARGET[name]]
            for (fl, fw), fv in zip(FIELDS, vals):
                cell = tk.Frame(block, bg=Ink.ground)
                cell.pack(side="left", padx=SP2)
                tk.Label(cell, text=track(fl), font=Type.micro, fg=Ink.text_4,
                         bg=Ink.ground).pack(anchor="w")
                wrap2, e = field_entry(cell, fv, width=fw, font=(Type.mono, 10),
                                 osk=self.osk)
                wrap2.pack()
                es[fl] = e
            InkButton(block, "APPLY", variant="primary", padx=SP3,
                      command=lambda n=name, en=es: self._apply_motor(n, en)
                      ).pack(side="left", padx=SP3)
            self.mc[name] = es
            hairline(inner, pad=SP5, pady=(SP3, 0))

        foot = tk.Frame(inner, bg=Ink.ground)
        foot.pack(fill="x", padx=SP5, pady=SP5)
        InkButton(foot, "SAVE ALL TO DISK", variant="solid", padx=SP5, pady=SP3,
                  command=save_config).pack(side="left")
        tk.Label(foot, text=CONFIG_PATH, font=Type.micro, fg=Ink.text_4,
                 bg=Ink.ground).pack(side="left", padx=SP3)

    def _apply_motor(self, name, es):
        try:
            STEPS_PER_DEG[name] = float(es["steps/deg"].get())
            motors[name]["target_freq"] = int(es["max hz"].get())
            JOINT_LIMITS_IK[name] = (float(es["ik lo"].get()), float(es["ik hi"].get()))
            JOINT_LIMITS_PHYS[name] = (float(es["phys lo"].get()),
                                       float(es["phys hi"].get()))
            HOME_TARGET[name] = float(es["home at"].get())
            log("{} tuning applied".format(name))
        except Exception as e:
            log("{} tuning rejected: {}".format(name, e), alarm=True)

    # ── geometry ──────────────────────────────────────────────────────
    def _page_geometry(self, p):
        section(p, "link lengths", "リンク長", pady=(SP5, SP3))
        f = tk.Frame(p, bg=Ink.ground)
        f.pack(fill="x", padx=SP5)
        self.kin = {}
        for label, val, note in [("L1", L1, "base to shoulder"),
                                 ("L2", L2, "upper arm"),
                                 ("L3", L3, "elbow offset"),
                                 ("L4", L4, "forearm"),
                                 ("L5", L5, "wrist to flange"),
                                 ("TCP", TCP, "flange to tool point")]:
            r = tk.Frame(f, bg=Ink.ground)
            r.pack(fill="x", pady=SP1)
            tk.Label(r, text=label, font=Type.data, fg=Ink.text, bg=Ink.ground,
                     width=5, anchor="w").pack(side="left")
            tk.Label(r, text=note, font=Type.label, fg=Ink.text_4, bg=Ink.ground,
                     width=24, anchor="w").pack(side="left")
            wrap, e = field_entry(r, val, width=10, osk=self.osk)
            wrap.pack(side="left")
            tk.Label(r, text="mm", font=Type.label, fg=Ink.text_4,
                     bg=Ink.ground).pack(side="left", padx=SP2)
            self.kin[label] = e

        bar = tk.Frame(p, bg=Ink.ground)
        bar.pack(fill="x", padx=SP5, pady=SP5)
        InkButton(bar, "APPLY", command=self._apply_links, variant="primary",
                  padx=SP5, pady=SP3).pack(side="left")
        InkButton(bar, "SAVE TO DISK", command=save_config, variant="solid",
                  padx=SP4, pady=SP3).pack(side="left", padx=SP3)

        section(p, "cooling", "冷却")
        cf = tk.Frame(p, bg=Ink.ground)
        cf.pack(fill="x", padx=SP5)
        self.fan_auto_btn = InkButton(
            cf, "AUTO  " + ("ON" if fan_auto else "OFF"), variant="solid",
            padx=SP4, pady=SP2, command=self._toggle_fan_auto)
        self.fan_auto_btn.pack(side="left")
        tk.Label(cf, text="idle {}%  ·  ramps {:.0f}→{:.0f} °C".format(
            FAN_IDLE, FAN_T_LO, FAN_T_HI), font=Type.label, fg=Ink.text_4,
            bg=Ink.ground).pack(side="left", padx=SP4)

    def _toggle_fan_auto(self):
        global fan_auto
        fan_auto = not fan_auto
        self.fan_auto_btn.set_label("AUTO  " + ("ON" if fan_auto else "OFF"))
        log("Fan control: " + ("automatic" if fan_auto else "manual"))

    def _apply_links(self):
        global L1, L2, L3, L4, L5, TCP
        try:
            L1 = float(self.kin["L1"].get()); L2 = float(self.kin["L2"].get())
            L3 = float(self.kin["L3"].get()); L4 = float(self.kin["L4"].get())
            L5 = float(self.kin["L5"].get()); TCP = float(self.kin["TCP"].get())
            rebuild_chain()
            log("Link lengths applied")
        except Exception as e:
            log("Link lengths rejected: {}".format(e), alarm=True)

    # ── faults ────────────────────────────────────────────────────────
    def _page_faults(self, p):
        bar = tk.Frame(p, bg=Ink.ground)
        bar.pack(fill="x", padx=SP4, pady=SP3)
        InkButton(bar, "CLEAR HISTORY", variant="quiet", padx=SP4,
                  command=lambda: (alarm_log.clear(), self._refresh_faults(True))
                  ).pack(side="right")
        w = tk.Frame(p, bg=Ink.void)
        w.pack(fill="both", expand=True, padx=SP4, pady=(0, SP4))
        self.fault_text = tk.Text(w, bg=Ink.void, fg=Ink.beni, font=(Type.mono, 9),
                                  relief="flat", bd=0, highlightthickness=0,
                                  state="disabled", wrap="none", padx=SP3, pady=SP3)
        sb = tk.Scrollbar(w, orient="vertical", command=self.fault_text.yview,
                          bd=0, bg=Ink.panel, troughcolor=Ink.void, width=10,
                          relief="flat", highlightthickness=0)
        self.fault_text.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.fault_text.pack(fill="both", expand=True)

    def _refresh_faults(self, force=False):
        if len(alarm_log) == self._alarm_len and not force:
            return
        self._alarm_len = len(alarm_log)
        self.fault_text.config(state="normal")
        self.fault_text.delete("1.0", "end")
        self.fault_text.insert("end", "\n".join(alarm_log) if alarm_log
                               else "no faults recorded")
        self.fault_text.config(state="disabled")
        self.fault_text.see("end")

    # ── network ───────────────────────────────────────────────────────
    def _page_network(self, p):
        bar = tk.Frame(p, bg=Ink.ground)
        bar.pack(fill="x", padx=SP4, pady=SP3)
        tk.Label(bar, text=track("agv endpoint"), font=Type.label, fg=Ink.text_3,
                 bg=Ink.ground).pack(side="left")
        wrap, self.agv_ip_e = field_entry(bar, ESP32_IP, width=16, osk=self.osk)
        wrap.pack(side="left", padx=SP3)
        InkButton(bar, "APPLY", variant="primary", padx=SP4,
                  command=self._apply_net).pack(side="left")
        InkButton(bar, "REFRESH", variant="quiet", padx=SP4,
                  command=lambda: self._refresh_net(True)).pack(side="right")
        w = tk.Frame(p, bg=Ink.void)
        w.pack(fill="both", expand=True, padx=SP4, pady=(0, SP4))
        self.net_text = tk.Text(w, bg=Ink.void, fg=Ink.text_2, font=(Type.mono, 9),
                                relief="flat", bd=0, highlightthickness=0,
                                state="disabled", wrap="none", padx=SP3, pady=SP3)
        sb = tk.Scrollbar(w, orient="vertical", command=self.net_text.yview,
                          bd=0, bg=Ink.panel, troughcolor=Ink.void, width=10,
                          relief="flat", highlightthickness=0)
        self.net_text.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.net_text.pack(fill="both", expand=True)
        self._net_loaded = False

    def _apply_net(self):
        global ESP32_IP
        v = self.agv_ip_e.get().strip()
        if v:
            ESP32_IP = v
            log("AGV endpoint set to " + ESP32_IP)

    def _refresh_net(self, force=False):
        if self._net_loaded and not force:
            return
        self._net_loaded = True

        def worker():
            try:
                addr = subprocess.check_output(["ip", "-br", "addr"], timeout=3).decode()
            except Exception:
                addr = "ip addr unavailable"
            try:
                route = subprocess.check_output(["ip", "route"], timeout=3).decode()
            except Exception:
                route = "ip route unavailable"
            out = "INTERFACES\n{}\nROUTES\n{}".format(addr, route)
            if self.alive():
                self.win.after(0, lambda: self._set_net(out))

        threading.Thread(target=worker, daemon=True).start()

    def _set_net(self, s):
        if not self.alive():
            return
        self.net_text.config(state="normal")
        self.net_text.delete("1.0", "end")
        self.net_text.insert("end", s)
        self.net_text.config(state="disabled")

    def _tick(self):
        if not self.alive():
            return
        if self.tab == "status":
            self._refresh_status()
        elif self.tab == "faults":
            self._refresh_faults()
        elif self.tab == "network":
            self._refresh_net()
        self.win.after(1000, self._tick)


# ═════════════════════════════════════════════════════════════════════════
#  MAIN SCREEN
#
#  Fukinsei — a wide data field on the left against a narrow rail on the
#  right. Weight sits left; the clock drifts to the far edge. Nothing is
#  centred for the sake of being centred.
# ═════════════════════════════════════════════════════════════════════════
class RoboArm:
    CART_LABELS = {"WORLD": ["X", "Y", "Z", "Rx", "Ry", "Rz"],
                   "TOOL":  ["Xt", "Yt", "Zt", "Rxt", "Ryt", "Rzt"]}
    CART_AXES = ["X", "Y", "Z", "Rx", "Ry", "Rz"]

    def __init__(self, root):
        self.root = root
        root.title("ROBOARM V4")
        root.configure(bg=Ink.ground)
        root.attributes("-fullscreen", True)

        self.osk = OSK(root)
        self.system = SystemMenu(root, self.osk)
        self.program = ProgramEditor(root, self.osk)
        self.target = TargetPanel(root, self.osk)
        self.agv_panel = AGVPanel(root)

        self.jw = {}
        self._cart_hold = None
        self._cart_after = None
        self._log_seen = 0
        self._fk_key = None
        self._fk = None
        self._tick_n = 0

        self._header()
        hairline(root)
        # Fixed chrome top and bottom first. An expanding body packed before
        # them takes the whole window and pushes the footer off the edge.
        self._command_bar()
        hairline(root, side="bottom")
        body = tk.Frame(root, bg=Ink.ground)
        body.pack(fill="both", expand=True)
        self._rail(body)
        vrule(body, side="right")
        self._field(body)

        self._bind_keys()
        start_ps4()
        log("ROBOARM V4 ready")
        log("Home the arm before cartesian modes or program playback")
        self._tick()
        self._tick_log()
        self._tick_slow()

    # ── header ────────────────────────────────────────────────────────
    def _header(self):
        bar = tk.Frame(self.root, bg=Ink.raised, height=64)
        bar.pack(fill="x")
        bar.pack_propagate(False)

        left = tk.Frame(bar, bg=Ink.raised)
        left.pack(side="left", padx=(SP6, 0))
        tk.Label(left, text=track("roboarm"), font=(Type.sans, 14),
                 fg=Ink.text, bg=Ink.raised).pack(side="left")
        if Type.cjk:
            tk.Label(left, text="六軸", font=(Type.cjk, 13), fg=Ink.text_4,
                     bg=Ink.raised).pack(side="left", padx=SP4)

        self.mode_chip = tk.Label(bar, text=track("joint"), font=Type.label,
                                  fg=Ink.void, bg=Ink.asagi, padx=SP4, pady=SP1)
        self.mode_chip.pack(side="left", padx=SP6)
        self.sel_chip = tk.Label(bar, text="J1", font=Type.data, fg=Ink.text_3,
                                 bg=Ink.raised)
        self.sel_chip.pack(side="left")

        self.clock = tk.Label(bar, text="", font=(Type.mono, 18), fg=Ink.text_2,
                              bg=Ink.raised)
        self.clock.pack(side="right", padx=SP6)

        pill = tk.Frame(bar, bg=Ink.raised)
        pill.pack(side="right", padx=SP4)
        self.state_dot = Dot(pill, bg=Ink.raised, size=10)
        self.state_dot.pack(side="left", padx=(0, SP2))
        self.state_lbl = tk.Label(pill, text=track("not homed"), font=Type.label,
                                  fg=Ink.yamabuki, bg=Ink.raised)
        self.state_lbl.pack(side="left")

        self.solver_lbl = tk.Label(bar, text=track("solver loading"),
                                   font=Type.micro, fg=Ink.text_4, bg=Ink.raised)
        self.solver_lbl.pack(side="right", padx=SP4)

        pen = tk.Frame(bar, bg=Ink.raised)
        pen.pack(side="right", padx=SP4)
        self.pen_dot = Dot(pen, bg=Ink.raised)
        self.pen_dot.pack(side="left", padx=(0, SP2))
        self.pen_lbl = tk.Label(pen, text=track("pendant"), font=Type.micro,
                                fg=Ink.text_4, bg=Ink.raised)
        self.pen_lbl.pack(side="left")

    # ── left field ────────────────────────────────────────────────────
    def _field(self, parent):
        f = tk.Frame(parent, bg=Ink.ground)
        f.pack(side="left", fill="both", expand=True)

        self.axis_hdr = section(f, "axes", "軸", pady=Metrics.sec_pady, bg=Ink.ground)
        self.axis_hint = tk.Label(self.axis_hdr, text="", font=Type.micro,
                                  fg=Ink.text_4, bg=Ink.ground)
        self.axis_hint.pack(side="right", padx=SP5)

        rows = tk.Frame(f, bg=Ink.ground)
        rows.pack(fill="x", padx=SP4)
        for i, name in enumerate(JOINT_ORDER):
            self._axis_row(rows, name, i)

        section(f, "tool centre point", "先端位置", pady=Metrics.sec_pady, bg=Ink.ground)
        tcp = tk.Frame(f, bg=Ink.ground)
        tcp.pack(fill="x", padx=SP5)
        self.tcp_lbls = {}
        for ax in ("X", "Y", "Z"):
            cell = tk.Frame(tcp, bg=Ink.ground)
            cell.pack(side="left", padx=(0, SP6))
            tk.Label(cell, text=track(ax), font=Type.label, fg=Ink.text_3,
                     bg=Ink.ground).pack(anchor="w")
            l = tk.Label(cell, text="—", font=Metrics.tcp_font, fg=Ink.text,
                         bg=Ink.ground, anchor="w", width=10)
            l.pack(anchor="w")
            self.tcp_lbls[ax] = l
        tk.Label(tcp, text="mm", font=Type.label, fg=Ink.text_4,
                 bg=Ink.ground).pack(side="left", pady=(SP4, 0))

        section(f, "log", "記録", pady=Metrics.sec_pady, bg=Ink.ground)
        lw = tk.Frame(f, bg=Ink.void)
        lw.pack(fill="both", expand=True, padx=SP4, pady=(0, SP4))
        self.log_text = tk.Text(lw, bg=Ink.void, fg=Ink.text_3,
                                font=(Type.mono, 9), relief="flat", bd=0,
                                highlightthickness=0, state="disabled",
                                wrap="none", padx=SP3, pady=SP2,
                                height=Metrics.log_lines)
        self.log_text.pack(fill="both", expand=True)
        self.log_text.tag_configure("alarm", foreground=Ink.beni)
        self.log_text.tag_configure("plain", foreground=Ink.text_3)

    def _axis_row(self, parent, name, idx):
        row = tk.Frame(parent, bg=Ink.panel, height=Metrics.row_h)
        row.pack(fill="x", pady=1)
        row.pack_propagate(False)

        mark = tk.Frame(row, bg=Ink.panel, width=3)
        mark.pack(side="left", fill="y")

        ax = tk.Label(row, text=name, font=Type.data_lg, fg=Ink.text_2,
                      bg=Ink.panel, width=4, cursor="hand2")
        ax.pack(side="left", padx=(SP4, SP2))

        ang = tk.Label(row, text="—", font=Type.data_lg, fg=Ink.text,
                       bg=Ink.panel, width=9, anchor="e")
        ang.pack(side="left")
        unit = tk.Label(row, text="°", font=Type.label, fg=Ink.text_3,
                        bg=Ink.panel, width=3, anchor="w")
        unit.pack(side="left")

        travel = TravelBar(row, width=Metrics.travel_w, bg=Ink.panel)
        travel.pack(side="left", padx=SP4)

        state = tk.Label(row, text="", font=Type.label, fg=Ink.text_4,
                         bg=Ink.panel, width=Metrics.state_w, anchor="w")
        state.pack(side="left", padx=SP2)

        # jog on the right edge — thumbs reach there on a panel-mounted screen
        plus = InkButton(row, "+", variant="solid", font=(Type.sans, 15),
                         padx=Metrics.jog_padx, pady=SP2,
                         on_press=lambda n=name, i=idx: self._jog(n, 1, i),
                         on_release=lambda n=name, i=idx: self._jog_stop(n, i))
        plus.pack(side="right", padx=(SP2, SP4))
        minus = InkButton(row, "−", variant="solid", font=(Type.sans, 15),
                          padx=Metrics.jog_padx, pady=SP2,
                          on_press=lambda n=name, i=idx: self._jog(n, 0, i),
                          on_release=lambda n=name, i=idx: self._jog_stop(n, i))
        minus.pack(side="right")

        spd_lbl = tk.Label(row, text="30%", font=Type.body, fg=Ink.text_3,
                           bg=Ink.panel, width=5, anchor="e")
        spd_lbl.pack(side="right", padx=SP2)
        slider = InkSlider(row, 1, 100, 30, width=Metrics.slider_w, bg=Ink.panel,
                           command=lambda v, n=name: self._set_speed(n, v))
        slider.pack(side="right")

        for w in (row, ax, ang, unit, state):
            w.bind("<Button-1>", lambda e, n=name: self._select(n))

        self.jw[name] = dict(row=row, mark=mark, ax=ax, ang=ang, unit=unit,
                             travel=travel, state=state, spd_lbl=spd_lbl,
                             slider=slider, plus=plus, minus=minus, idx=idx)

    # ── right rail ────────────────────────────────────────────────────
    def _rail(self, parent):
        r = tk.Frame(parent, bg=Ink.ground, width=Metrics.rail_w)
        r.pack(side="right", fill="y")
        r.pack_propagate(False)

        # E-STOP is claimed first so it can never be squeezed off a short
        # panel by the readouts above it. Safety outranks layout.
        stop_zone = tk.Frame(r, bg=Ink.ground)
        stop_zone.pack(side="bottom", fill="x", pady=Metrics.stop_pady)
        self.estop_btn = InkButton(stop_zone, track("stop"), variant="danger",
                                   font=(Type.sans, 16), pady=Metrics.stop_pady,
                                   command=self._estop_button)
        self.estop_btn.pack(fill="x", padx=SP4)
        self.estop_note = tk.Label(stop_zone, text="", font=Type.micro,
                                   fg=Ink.text_4, bg=Ink.ground)
        self.estop_note.pack(pady=(SP1, 0))

        pad = tk.Frame(r, bg=Ink.ground)
        pad.pack(fill="x", pady=(Metrics.rule_pady, 0))
        self.hero = tk.Label(pad, text="J1", font=Metrics.hero_font,
                             fg=Ink.asagi, bg=Ink.ground)
        self.hero.pack()
        self.hero_sub = tk.Label(pad, text="", font=Type.rail, fg=Ink.text_4,
                                 bg=Ink.ground)
        self.hero_sub.pack(pady=(0, Metrics.rule_pady))

        self.rail_rows = {}
        for key, label in (("angle", "angle"), ("speed", "speed"), ("state", "state")):
            row = tk.Frame(r, bg=Ink.ground)
            row.pack(fill="x", padx=SP5, pady=SP1)
            tk.Label(row, text=track(label), font=Type.label, fg=Ink.text_3,
                     bg=Ink.ground).pack(side="left")
            v = tk.Label(row, text="—", font=Type.data, fg=Ink.text, bg=Ink.ground)
            v.pack(side="right")
            self.rail_rows[key] = v

        hairline(r, pad=SP5, pady=Metrics.rule_pady)

        ov = tk.Frame(r, bg=Ink.ground)
        ov.pack(fill="x", padx=SP5)
        tk.Label(ov, text=track("override"), font=Type.label, fg=Ink.text_3,
                 bg=Ink.ground).pack(side="left")
        self.ovr_lbl = tk.Label(ov, text="30%", font=Type.data, fg=Ink.text,
                                bg=Ink.ground)
        self.ovr_lbl.pack(side="right")
        self.ovr_slider = InkSlider(r, 1, 100, 30, width=Metrics.rail_w - 2 * SP5,
                                    bg=Ink.ground, command=self._set_override)
        self.ovr_slider.pack(padx=SP5, pady=(SP1, Metrics.rule_pady))

        fn = tk.Frame(r, bg=Ink.ground)
        fn.pack(fill="x", padx=SP5)
        tk.Label(fn, text=track("cooling"), font=Type.label, fg=Ink.text_3,
                 bg=Ink.ground).pack(side="left")
        self.fan_lbl = tk.Label(fn, text="2%", font=Type.data, fg=Ink.text,
                                bg=Ink.ground)
        self.fan_lbl.pack(side="right")
        self.fan_slider = InkSlider(r, 0, 100, FAN_IDLE,
                                    width=Metrics.rail_w - 2 * SP5,
                                    bg=Ink.ground, command=self._set_fan_manual)
        self.fan_slider.pack(padx=SP5, pady=(SP1, SP2))
        self.temp_lbl = tk.Label(r, text="—", font=Type.label, fg=Ink.text_4,
                                 bg=Ink.ground)
        self.temp_lbl.pack(padx=SP5, anchor="w")

        hairline(r, pad=SP5, pady=Metrics.rule_pady)

    # ── command bar ───────────────────────────────────────────────────
    def _command_bar(self):
        bar = tk.Frame(self.root, bg=Ink.raised, height=60)
        bar.pack(fill="x", side="bottom")
        bar.pack_propagate(False)
        items = [("home", "原点", start_homing),
                 ("mode", "座標", self._cycle_mode),
                 ("target", "目標", self.target.toggle),
                 ("program", "教示", self.program.toggle),
                 ("agv", "搬送", self.agv_panel.toggle),
                 ("system", "設定", self.system.toggle)]
        for i, (label, kanji, cmd) in enumerate(items):
            b = InkButton(bar, track(label), command=cmd, variant="quiet",
                          font=Type.body, padx=SP5, pady=SP2,
                          sub="F{}  {}".format(i + 1, kanji) if Type.cjk
                              else "F{}".format(i + 1))
            b.pack(side="left", fill="y", padx=(SP4 if i == 0 else SP2, 0))
        InkButton(bar, track("exit"), command=self.quit_app, variant="quiet",
                  font=Type.body, padx=SP5, pady=SP2, sub="esc"
                  ).pack(side="right", padx=SP4)
        InkButton(bar, track("keyboard"), command=self.osk.toggle, variant="quiet",
                  font=Type.body, padx=SP4, pady=SP2, sub="on-screen"
                  ).pack(side="right", padx=SP2)

    def _bind_keys(self):
        r = self.root
        r.bind("<Escape>", lambda e: self.quit_app())
        r.bind("<F1>", lambda e: start_homing())
        r.bind("<F2>", lambda e: self._cycle_mode())
        r.bind("<F3>", lambda e: self.target.toggle())
        r.bind("<F4>", lambda e: self.program.toggle())
        r.bind("<F5>", lambda e: self.agv_panel.toggle())
        r.bind("<F6>", lambda e: self.system.toggle())
        r.bind("<space>", lambda e: stop_all("Keyboard stop", alarm=False))

    # ── actions ───────────────────────────────────────────────────────
    def _select(self, name):
        global selected_joint
        selected_joint = JOINT_ORDER.index(name)

    def _set_speed(self, name, v):
        set_motor_speed(name, v)
        cfg(self.jw[name]["spd_lbl"], text="{}%".format(v))

    def _set_override(self, v):
        set_all_speeds(v)
        cfg(self.ovr_lbl, text="{}%".format(v))
        for n in JOINT_ORDER:
            self.jw[n]["slider"].set(v)
            cfg(self.jw[n]["spd_lbl"], text="{}%".format(v))

    def _set_fan_manual(self, v):
        global fan_auto
        if fan_auto:
            fan_auto = False
            log("Fan control: manual")
        set_fan(v)
        cfg(self.fan_lbl, text="{}%".format(v))

    def _jog(self, name, direction, idx):
        self._select(name)
        if control_mode == "JOINT":
            start_motor(name, direction)
        elif control_mode in ("WORLD", "TOOL"):
            axis = self.CART_AXES[idx]
            self._cart_hold = (axis, 1 if direction == 1 else -1)
            self._cart_repeat()

    def _cart_repeat(self):
        if not self._cart_hold:
            return
        axis, sign = self._cart_hold
        cartesian_jog(axis, sign)
        self._cart_after = self.root.after(90, self._cart_repeat)

    def _jog_stop(self, name, idx):
        if control_mode == "JOINT":
            stop_motor(name)
        else:
            self._cart_hold = None
            if self._cart_after:
                self.root.after_cancel(self._cart_after)
                self._cart_after = None

    def _cycle_mode(self):
        global control_mode, _last_arm_mode
        order = ["JOINT", "WORLD", "TOOL", "AGV"]
        control_mode = order[(order.index(control_mode) + 1) % len(order)]
        if control_mode == "AGV":
            stop_all("Entering AGV mode", alarm=False)
        else:
            _last_arm_mode = control_mode
            if control_mode in ("WORLD", "TOOL") and not all_homed:
                log("Cartesian modes need a homed arm", alarm=True)
        log("Mode " + control_mode)

    def _estop_button(self):
        if estop_active and ESTOP_LATCHING:
            estop_reset()
        else:
            stop_all("Screen stop")

    def quit_app(self):
        try:
            agv.stop()
            agv.running = False
            stop_all("Shutting down")
            fan_off_hard()
            lgpio.gpiochip_close(h)
        except Exception:
            pass
        self.root.destroy()

    # ── update: motion tier, 100 ms ───────────────────────────────────
    def _tick(self):
        self._tick_n += 1
        mode = control_mode
        jname = get_joint_name()

        # header
        chip_bg = {"JOINT": Ink.asagi, "WORLD": Ink.ai,
                   "TOOL": Ink.wakatake, "AGV": Ink.ai}[mode]
        cfg(self.mode_chip, text=track(mode.lower()), bg=chip_bg)
        cfg(self.sel_chip, text=jname if mode != "AGV" else "—")

        if estop_active:
            cfg(self.state_lbl, text=track("e-stop"), fg=Ink.beni)
            self.state_dot.set(True, Ink.shu)
        elif homing_active:
            cfg(self.state_lbl, text=track("homing"), fg=Ink.yamabuki)
            self.state_dot.set(True, Ink.yamabuki)
        elif playback_running:
            cfg(self.state_lbl, text=track("running"), fg=Ink.asagi)
            self.state_dot.set(True, Ink.asagi)
        elif all_homed:
            cfg(self.state_lbl, text=track("ready"), fg=Ink.wakatake)
            self.state_dot.set(True, Ink.wakatake)
        else:
            cfg(self.state_lbl, text=track("not homed"), fg=Ink.yamabuki)
            self.state_dot.set(False, Ink.text_4)

        # e-stop control reads RESET only while latched
        if estop_active and ESTOP_LATCHING:
            self.estop_btn.set_label(track("reset"))
            cfg(self.estop_note,
                text="loop open" if estop_pin_raw else "release, then reset",
                fg=Ink.beni)
        else:
            self.estop_btn.set_label(track("stop"))
            cfg(self.estop_note, text="all axes", fg=Ink.text_4)

        cfg(self.axis_hint, text={
            "JOINT": "jog drives the selected joint",
            "WORLD": "jog moves the tool in world axes",
            "TOOL":  "jog moves the tool in its own frame",
            "AGV":   "arm jog disabled while driving",
        }[mode])

        # axis rows
        labels = self.CART_LABELS.get(mode)
        cart = labels is not None
        for i, name in enumerate(JOINT_ORDER):
            w = self.jw[name]
            m = motors[name]
            selected = (i == selected_joint) and mode != "AGV"
            bg = Ink.sel if selected else Ink.panel
            for k in ("row", "ax", "ang", "unit", "state", "spd_lbl"):
                cfg(w[k], bg=bg)
            cfg(w["mark"], bg=Ink.asagi if selected else bg)
            cfg(w["travel"], bg=bg)
            cfg(w["slider"], bg=bg)
            w["plus"].set_base_bg(bg)
            w["minus"].set_base_bg(bg)

            cfg(w["ax"], text=labels[i] if cart else name,
                fg=Ink.asagi if selected else Ink.text_2)
            cfg(w["ang"], text="{:+.2f}".format(m["angle_deg"]),
                fg=Ink.text_4 if mode == "AGV" else Ink.text)
            cfg(w["unit"], text="mm" if cart and i < 3 else "°")
            cfg(w["spd_lbl"], text="{}%".format(m["speed"]))
            if w["slider"].get() != m["speed"]:
                w["slider"].set(m["speed"])

            if estop_active:
                st, c = "e-stop", Ink.beni
            elif m["at_limit"]:
                st, c = "limit", Ink.yamabuki
            elif m["soft_block"]:
                st, c = "soft limit", Ink.yamabuki
            elif m["running"]:
                st, c = ("moving +" if m["direction"] == 1 else "moving −"), Ink.asagi
            elif not m["homed"]:
                st, c = "not homed", Ink.text_4
            else:
                st, c = "idle", Ink.text_4
            cfg(w["state"], text=st, fg=c)

            if cart or mode == "AGV":
                w["travel"].blank()
            else:
                lo, hi = JOINT_LIMITS_PHYS[name]
                w["travel"].set(m["angle_deg"], lo, hi,
                                Ink.asagi if selected else Ink.text_3)

        # rail
        m = motors[jname]
        cfg(self.hero, text=jname if mode != "AGV" else "AGV",
            fg=Ink.ai if mode == "AGV" else Ink.asagi)
        cfg(self.hero_sub, text=Type.jp("軸", "axis") if mode != "AGV"
            else Type.jp("搬送車", "vehicle"))
        cfg(self.rail_rows["angle"], text="{:+.2f}°".format(m["angle_deg"]))
        cfg(self.rail_rows["speed"], text="{}%".format(m["speed"]))
        if estop_active:
            cfg(self.rail_rows["state"], text="e-stop", fg=Ink.beni)
        elif mode == "AGV":
            cfg(self.rail_rows["state"],
                text="{:+.2f} / {:+.2f}".format(agv.translate, agv.rotate),
                fg=Ink.ai)
        elif m["running"]:
            cfg(self.rail_rows["state"], text="moving", fg=Ink.asagi)
        elif m["at_limit"] or m["soft_block"]:
            cfg(self.rail_rows["state"], text="at limit", fg=Ink.yamabuki)
        else:
            cfg(self.rail_rows["state"], text="idle", fg=Ink.text_3)

        self.pen_dot.set(ps4_connected, Ink.wakatake)
        cfg(self.pen_lbl, text=track("pendant"),
            fg=Ink.text_3 if ps4_connected else Ink.text_4)

        # tool centre point — FK is expensive, so only when a joint moved
        if self._tick_n % 3 == 0:
            key = tuple(round(motors[n]["angle_deg"], 2) for n in JOINT_ORDER)
            if key != self._fk_key:
                self._fk_key = key
                self._fk = forward_kinematics(list(key))
            for i, ax in enumerate(("X", "Y", "Z")):
                cfg(self.tcp_lbls[ax],
                    text="{:+.2f}".format(self._fk[i]) if self._fk else "—",
                    fg=Ink.text if self._fk else Ink.text_4)

        if self.program.alive():
            self.program.refresh()

        self.root.after(100, self._tick)

    # ── update: log tier, 300 ms, append only ─────────────────────────
    def _tick_log(self):
        if log_seq != self._log_seen:
            with _log_lock:
                new = log_seq - self._log_seen
                self._log_seen = log_seq
                # A burst larger than the ring means older lines are already
                # gone; take whatever survived rather than a fixed guess.
                take = min(new, len(system_log))
                lines = system_log[-take:] if take > 0 else []
                alarms = set(alarm_log[-40:])
            self.log_text.config(state="normal")
            for line in lines:
                self.log_text.insert("end", line + "\n",
                                     "alarm" if line in alarms else "plain")
            # keep the widget bounded — this runs for hours
            excess = int(self.log_text.index("end-1c").split(".")[0]) - 200
            if excess > 0:
                self.log_text.delete("1.0", "{}.0".format(excess + 1))
            self.log_text.config(state="disabled")
            self.log_text.see("end")
        self.root.after(300, self._tick_log)

    # ── update: system tier, 1 s ──────────────────────────────────────
    def _tick_slow(self):
        cfg(self.clock, text=time.strftime("%H:%M:%S"))

        temp = get_cpu_temp()
        fan_tick(temp)
        cfg(self.temp_lbl, text="cpu {:.1f} °C   ·   {}".format(
            temp, "auto" if fan_auto else "manual"),
            fg=Ink.shu if temp > FAN_T_HI else
               Ink.yamabuki if temp > FAN_T_LO + 15 else Ink.text_4)
        cfg(self.fan_lbl, text="{}%".format(fan_speed))
        if fan_auto and self.fan_slider.get() != fan_speed:
            self.fan_slider.set(fan_speed)

        if _kin_ready.is_set():
            cfg(self.solver_lbl, text=track("solver ready"), fg=Ink.text_4)
        elif _kin_error:
            cfg(self.solver_lbl, text=track("solver failed"), fg=Ink.shu)
        else:
            cfg(self.solver_lbl, text=track("solver loading"), fg=Ink.yamabuki)

        self.root.after(1000, self._tick_slow)


# ═════════════════════════════════════════════════════════════════════════
#  ENTRY
# ═════════════════════════════════════════════════════════════════════════
def main():
    root = tk.Tk()
    Type.resolve(root)
    Metrics.resolve(root)
    load_config()
    app = RoboArm(root)
    root.protocol("WM_DELETE_WINDOW", app.quit_app)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.quit_app()


if __name__ == "__main__":
    main()
