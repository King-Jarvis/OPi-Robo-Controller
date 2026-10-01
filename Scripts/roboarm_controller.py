import lgpio
import time
import threading
import tkinter as tk
import pygame

# ─────────────────────────────────────────────
# GPIO SETUP
# ─────────────────────────────────────────────
GPIOCHIP      = 0
FAN_PIN       = 267
ESTOP_PIN     = 266
DEBOUNCE_TIME = 0.05
FAN_PWM_FREQ  = 1000

motors = {
    "J1": {"step": 226, "dir": 227, "target_freq": 500,  "limit": 260, "running": False, "direction": 1, "at_limit": False, "position": 0, "speed": 50},
    "J2": {"step": 228, "dir": 229, "target_freq": 2000, "limit": 261, "running": False, "direction": 1, "at_limit": False, "position": 0, "speed": 50},
    "J3": {"step": 230, "dir": 231, "target_freq": 4000, "limit": 262, "running": False, "direction": 1, "at_limit": False, "position": 0, "speed": 50},
    "J4": {"step": 232, "dir": 233, "target_freq": 500,  "limit": 263, "running": False, "direction": 1, "at_limit": False, "position": 0, "speed": 50},
    "J5": {"step": 256, "dir": 257, "target_freq": 4000, "limit": 264, "running": False, "direction": 1, "at_limit": False, "position": 0, "speed": 50},
    "J6": {"step": 258, "dir": 259, "target_freq": 4000, "limit": 265, "running": False, "direction": 1, "at_limit": False, "position": 0, "speed": 50},
}

JOINT_ORDER = ["J1", "J2", "J3", "J4", "J5", "J6"]

h = lgpio.gpiochip_open(GPIOCHIP)
estop_active = False
fan_speed    = 0
system_log   = []

def log(msg):
    ts = time.strftime("%H:%M:%S")
    entry = "[{}] {}".format(ts, msg)
    system_log.append(entry)
    if len(system_log) > 50:
        system_log.pop(0)

for name, m in motors.items():
    lgpio.gpio_claim_output(h, m["step"], 0)
    lgpio.gpio_claim_output(h, m["dir"],  0)
    lgpio.gpio_claim_input(h,  m["limit"], lgpio.SET_PULL_UP)

lgpio.gpio_claim_input(h,  ESTOP_PIN, lgpio.SET_PULL_UP)
lgpio.gpio_claim_output(h, FAN_PIN, 0)

# Guarantee fan starts OFF
lgpio.tx_pwm(h, FAN_PIN, 1, 0)
lgpio.gpio_write(h, FAN_PIN, 0)

# ─────────────────────────────────────────────
# MOTOR CONTROL
# ─────────────────────────────────────────────
def set_fan(speed_percent):
    global fan_speed
    speed_percent = max(0, min(100, speed_percent))
    fan_speed = speed_percent
    # Always cancel any running PWM first before reconfiguring the pin
    lgpio.tx_pwm(h, FAN_PIN, 1, 0)
    time.sleep(0.02)
    if speed_percent == 0:
        lgpio.gpio_write(h, FAN_PIN, 0)
    else:
        lgpio.tx_pwm(h, FAN_PIN, FAN_PWM_FREQ, speed_percent)

def fan_off_hard():
    lgpio.tx_pwm(h, FAN_PIN, 1, 0)
    time.sleep(0.05)
    lgpio.gpio_write(h, FAN_PIN, 0)
    time.sleep(0.05)

def stop_motor(name):
    m = motors[name]
    lgpio.tx_pwm(h, m["step"], 1, 0)
    lgpio.gpio_claim_output(h, m["step"], 0)
    m["running"] = False

def stop_all():
    for name in motors:
        stop_motor(name)
    log("STOP ALL")

def start_motor(name, direction):
    m = motors[name]
    if estop_active:
        log("E-STOP active")
        return
    if m["at_limit"] and m["direction"] == direction:
        log("{} at limit".format(name))
        return
    lgpio.tx_pwm(h, m["step"], 1, 0)
    lgpio.gpio_claim_output(h, m["step"], 0)
    time.sleep(0.02)
    m["direction"] = direction
    m["running"]   = True
    lgpio.gpio_write(h, m["dir"], direction)
    time.sleep(0.01)
    freq = max(10, int(m["target_freq"] * m["speed"] / 100))
    lgpio.tx_pwm(h, m["step"], freq, 50)

def set_motor_speed(name, speed_percent):
    m = motors[name]
    m["speed"] = max(1, min(100, int(speed_percent)))
    if m["running"]:
        freq = max(10, int(m["target_freq"] * m["speed"] / 100))
        lgpio.tx_pwm(h, m["step"], freq, 50)

# ─────────────────────────────────────────────
# LIMIT + ESTOP MONITOR
# ─────────────────────────────────────────────
def monitor_limits():
    global estop_active
    last_trigger = {name: 0 for name in motors}
    last_estop   = 0
    while True:
        try:
            if lgpio.gpio_read(h, ESTOP_PIN) == 1:
                now = time.time()
                if now - last_estop > DEBOUNCE_TIME:
                    if not estop_active:
                        estop_active = True
                        stop_all()
                        log("!! E-STOP TRIGGERED !!")
                    last_estop = now
            else:
                if estop_active:
                    log("E-STOP released")
                estop_active = False

            for name, m in motors.items():
                state = lgpio.gpio_read(h, m["limit"])
                if state == 1:
                    now = time.time()
                    if now - last_trigger[name] > DEBOUNCE_TIME:
                        if not m["at_limit"]:
                            m["at_limit"] = True
                            stop_motor(name)
                            log("{} limit switch hit".format(name))
                        last_trigger[name] = now
                else:
                    if m["at_limit"]:
                        log("{} limit cleared".format(name))
                    m["at_limit"] = False
                if m["running"]:
                    m["position"] += 1 if m["direction"] == 1 else -1
            time.sleep(0.01)
        except Exception:
            break

monitor_thread = threading.Thread(target=monitor_limits, daemon=True)
monitor_thread.start()

# ─────────────────────────────────────────────
# WORLD MODE STUB
# ─────────────────────────────────────────────
LINK_LENGTHS = {"L1": 0, "L2": 0, "L3": 0, "L4": 0, "L5": 0}

def world_mode_move(dx, dy, dz):
    pass

# ─────────────────────────────────────────────
# PS4 CONTROLLER
# D-pad left/right  = cycle selected joint
# D-pad up/down     = speed +5/-5 on selected joint
# Left stick Y      = jog selected joint
# L1/R1             = fine speed trim
# Cross             = stop all
# Triangle          = toggle joint/world mode
# ─────────────────────────────────────────────
control_mode   = "JOINT"
selected_joint = 0
ps4_connected  = False
ps4_thread     = None
DEADZONE       = 0.25

def get_joint_name():
    return JOINT_ORDER[selected_joint]

def ps4_loop():
    global ps4_connected, control_mode, selected_joint
    pygame.init()
    pygame.joystick.init()

    joystick      = None
    prev_tri      = False
    prev_cross    = False
    prev_dpad     = (0, 0)
    active_joints = set()

    while True:
        try:
            if joystick is None:
                pygame.joystick.quit()
                pygame.joystick.init()
                if pygame.joystick.get_count() > 0:
                    joystick = pygame.joystick.Joystick(0)
                    joystick.init()
                    ps4_connected = True
                    log("Controller connected: " + joystick.get_name())
                else:
                    ps4_connected = False
                    time.sleep(1)
                    continue

            pygame.event.pump()

            ly    = joystick.get_axis(1)
            l1    = joystick.get_button(4)
            r1    = joystick.get_button(5)
            cross = joystick.get_button(0)
            tri   = joystick.get_button(3)
            hat   = joystick.get_hat(0) if joystick.get_numhats() > 0 else (0, 0)
            dx    = hat[0]
            dy    = hat[1]

            if tri and not prev_tri:
                control_mode = "WORLD" if control_mode == "JOINT" else "JOINT"
                log("Mode: " + control_mode)
            prev_tri = tri

            if cross and not prev_cross:
                stop_all()
            prev_cross = cross

            jname = get_joint_name()

            if dx != prev_dpad[0]:
                if dx == 1:
                    selected_joint = (selected_joint + 1) % len(JOINT_ORDER)
                    stop_all()
                    log("Sel: " + get_joint_name())
                elif dx == -1:
                    selected_joint = (selected_joint - 1) % len(JOINT_ORDER)
                    stop_all()
                    log("Sel: " + get_joint_name())

            if dy != prev_dpad[1]:
                jname = get_joint_name()
                if dy == 1:
                    set_motor_speed(jname, min(100, motors[jname]["speed"] + 5))
                    log("{} spd {}%".format(jname, motors[jname]["speed"]))
                elif dy == -1:
                    set_motor_speed(jname, max(1, motors[jname]["speed"] - 5))
                    log("{} spd {}%".format(jname, motors[jname]["speed"]))

            prev_dpad = (dx, dy)

            jname = get_joint_name()
            if l1:
                set_motor_speed(jname, max(1, motors[jname]["speed"] - 1))
            if r1:
                set_motor_speed(jname, min(100, motors[jname]["speed"] + 1))

            if control_mode == "JOINT":
                new_joints = set()
                if abs(ly) > DEADZONE:
                    direction = 0 if ly > 0 else 1
                    if not motors[jname]["running"] or motors[jname]["direction"] != direction:
                        start_motor(jname, direction)
                    new_joints.add(jname)
                for j in active_joints - new_joints:
                    stop_motor(j)
                active_joints = new_joints
            else:
                pass

            time.sleep(0.05)

        except Exception:
            ps4_connected = False
            joystick = None
            log("Controller disconnected")
            time.sleep(1)

def start_ps4():
    global ps4_thread
    if ps4_thread is None or not ps4_thread.is_alive():
        ps4_thread = threading.Thread(target=ps4_loop, daemon=True)
        ps4_thread.start()
        log("PS4 thread started")

# ─────────────────────────────────────────────
# KUKA-STYLE THEME
# ─────────────────────────────────────────────
BG          = "#1c1c1c"
BG2         = "#242424"
BG3         = "#2c2c2c"
PANEL       = "#303030"
PANEL_SEL   = "#3a3010"
BORDER      = "#484848"
ORANGE      = "#ff6a00"
ORANGE_LT   = "#ff8c33"
ORANGE_DK   = "#7a3200"
GREEN       = "#00b050"
GREEN_DK    = "#004d22"
RED         = "#cc2200"
RED_LT      = "#ff3300"
RED_DK      = "#4d0d00"
YELLOW      = "#e6b800"
WHITE       = "#e8e8e8"
GREY        = "#888888"
DARKGREY    = "#3c3c3c"
SCRN_BG     = "#0a0c0a"
SCRN_GREEN  = "#00dd44"

FT       = ("Courier New", 9)
FT_SM    = ("Courier New", 8)
FT_MED   = ("Courier New", 10, "bold")
FT_LG    = ("Courier New", 13, "bold")
FT_TINY  = ("Courier New", 7)
FT_TITLE = ("Courier New", 12, "bold")

def led(parent, bg_col, size=11):
    c = tk.Canvas(parent, width=size, height=size, bg=bg_col, highlightthickness=0)
    o = c.create_oval(1, 1, size-1, size-1, fill="#333", outline="#555")
    return c, o

def set_led(c, o, on, col_on, col_off="#333"):
    c.itemconfig(o, fill=col_on if on else col_off)

# ─────────────────────────────────────────────
# GUI
# ─────────────────────────────────────────────
class Pendant:
    def __init__(self, root):
        self.root = root
        self.root.title("ARM CONTROL PENDANT  v1.0")
        self.root.configure(bg=BG)
        self.root.geometry("1100x700")
        self.root.resizable(False, False)

        self._build_titlebar()
        self._build_statusbar()
        self._build_body()
        self._build_softkeys()

        log("System ready")
        self.update_loop()

    def _build_titlebar(self):
        bar = tk.Frame(self.root, bg="#111111", height=40)
        bar.pack(fill="x")
        bar.pack_propagate(False)

        badge = tk.Frame(bar, bg=ORANGE, width=120)
        badge.pack(side="left", fill="y")
        badge.pack_propagate(False)
        tk.Label(badge, text="ARM PENDANT",
                 font=("Courier New", 9, "bold"),
                 fg="white", bg=ORANGE).pack(expand=True)

        tk.Label(bar, text="AXIS CONTROL SYSTEM  |  T1 MANUAL MODE",
                 font=FT_TITLE, fg=WHITE, bg="#111111").pack(side="left", padx=14)

        right = tk.Frame(bar, bg="#111111")
        right.pack(side="right", padx=10)

        self.time_lbl = tk.Label(right, text="00:00:00",
                                 font=("Courier New", 14, "bold"),
                                 fg=ORANGE, bg="#111111")
        self.time_lbl.pack(side="right", padx=6)

        tk.Label(right, text="SYS TIME",
                 font=FT_TINY, fg=GREY, bg="#111111").pack(side="right")

    def _build_statusbar(self):
        bar = tk.Frame(self.root, bg="#181818", height=30)
        bar.pack(fill="x")
        bar.pack_propagate(False)

        self.status_leds = {}
        items = [
            ("DRIVES",  True,  GREEN,  GREEN_DK),
            ("MOTORS",  True,  GREEN,  GREEN_DK),
            ("POWER",   True,  GREEN,  GREEN_DK),
            ("LIMITS",  False, YELLOW, "#555200"),
            ("E-STOP",  False, RED,    RED_DK),
            ("CTRL",    False, ORANGE, ORANGE_DK),
        ]
        for label, default, con, coff in items:
            f = tk.Frame(bar, bg="#181818")
            f.pack(side="left", padx=6, pady=4)
            lc, lo = led(f, "#181818", 11)
            lc.pack(side="left", padx=2)
            set_led(lc, lo, default, con, coff)
            tk.Label(f, text=label, font=FT_SM,
                     fg=GREY, bg="#181818").pack(side="left")
            self.status_leds[label] = (lc, lo, con, coff)

        right = tk.Frame(bar, bg="#181818")
        right.pack(side="right", padx=8)

        self.ovr_badge = tk.Label(right, text=" OVR: 50% ",
                                  font=FT_SM, fg="black", bg=GREY)
        self.ovr_badge.pack(side="right", padx=3)

        self.mode_badge = tk.Label(right, text=" JOINT ",
                                   font=FT_MED, fg="black", bg=ORANGE)
        self.mode_badge.pack(side="right", padx=3)

        self.sel_badge = tk.Label(right, text=" SEL: J1 ",
                                  font=FT_MED, fg=ORANGE, bg="#181818")
        self.sel_badge.pack(side="right", padx=6)

    def _build_body(self):
        body = tk.Frame(self.root, bg=BG)
        body.pack(fill="both", expand=True, padx=4, pady=3)
        self._build_axis_panel(body)
        self._build_right_panel(body)

    def _build_axis_panel(self, parent):
        frame = tk.Frame(parent, bg=BG2)
        frame.pack(side="left", fill="both", expand=True, padx=(0, 3))

        hdr = tk.Frame(frame, bg="#181818", height=22)
        hdr.pack(fill="x")
        hdr.pack_propagate(False)
        for text, w in [("SEL", 3), ("AXIS", 5), ("ACTUAL POS", 11),
                         ("STATUS", 9), ("LIM", 4), ("DIR", 5),
                         ("SPEED", 19), ("JOG -/+", 13)]:
            tk.Label(hdr, text=text, font=FT_TINY, fg=GREY, bg="#181818",
                     width=w, anchor="w").pack(side="left", padx=4)

        self.jw = {}
        for name in JOINT_ORDER:
            self._make_axis_row(frame, name)

        # Cartesian display
        cart = tk.Frame(frame, bg="#181818", pady=4)
        cart.pack(fill="x", pady=(4, 0))

        tk.Label(cart, text="  CARTESIAN  /  BASE FRAME",
                 font=FT_SM, fg=GREY, bg="#181818").pack(anchor="w", padx=6)

        crow = tk.Frame(cart, bg="#181818")
        crow.pack(fill="x", padx=6, pady=2)

        self.coord_lbls = {}
        for axis in ["X", "Y", "Z", "A", "B", "C"]:
            f = tk.Frame(crow, bg=SCRN_BG, relief="sunken", bd=1)
            f.pack(side="left", padx=3)
            tk.Label(f, text=axis, font=FT_SM, fg=GREY,
                     bg=SCRN_BG, width=2).pack(side="left", padx=3)
            lbl = tk.Label(f, text="  0.000",
                           font=("Courier New", 9, "bold"),
                           fg=SCRN_GREEN, bg=SCRN_BG, width=8, anchor="e")
            lbl.pack(side="left", padx=2)
            self.coord_lbls[axis] = lbl

        tk.Label(cart,
                 text="  [ WORLD MODE: implement IK in world_mode_move() to enable ]",
                 font=FT_TINY, fg="#555", bg="#181818").pack(anchor="w", padx=6, pady=(0, 3))

        # System log
        log_frame = tk.Frame(frame, bg=SCRN_BG, pady=2)
        log_frame.pack(fill="both", expand=True, padx=4, pady=(3, 0))

        tk.Label(log_frame, text=" SYSTEM LOG",
                 font=FT_TINY, fg=GREY, bg=SCRN_BG).pack(anchor="w")

        self.log_text = tk.Text(log_frame, height=5,
                                bg=SCRN_BG, fg=SCRN_GREEN,
                                font=("Courier New", 8),
                                relief="flat", state="disabled",
                                insertbackground=SCRN_GREEN)
        self.log_text.pack(fill="both", expand=True, padx=4, pady=2)

    def _make_axis_row(self, parent, name):
        row = tk.Frame(parent, bg=PANEL, pady=0, cursor="hand2")
        row.pack(fill="x", pady=2)
        row.bind("<Button-1>", lambda e, n=name: self._select(n))

        sel_lbl = tk.Label(row, text=" ", font=FT_LG,
                           fg=ORANGE, bg=PANEL, width=2)
        sel_lbl.pack(side="left", padx=2)
        sel_lbl.bind("<Button-1>", lambda e, n=name: self._select(n))

        ax_lbl = tk.Label(row, text=name, font=FT_LG,
                          fg=WHITE, bg=PANEL, width=4, cursor="hand2")
        ax_lbl.pack(side="left")
        ax_lbl.bind("<Button-1>", lambda e, n=name: self._select(n))

        pos_bg = tk.Frame(row, bg=SCRN_BG, relief="sunken", bd=1)
        pos_bg.pack(side="left", padx=5)
        pos_lbl = tk.Label(pos_bg, text="+000000",
                           font=("Courier New", 12, "bold"),
                           fg=SCRN_GREEN, bg=SCRN_BG,
                           width=9, anchor="e", padx=5, pady=2)
        pos_lbl.pack()

        stat_lbl = tk.Label(row, text="IDLE    ",
                            font=("Courier New", 9), fg=GREY,
                            bg=PANEL, width=9, anchor="w")
        stat_lbl.pack(side="left", padx=4)

        lc, lo = led(row, PANEL, 12)
        lc.pack(side="left", padx=4)
        set_led(lc, lo, False, RED, GREEN_DK)

        dir_lbl = tk.Label(row, text="FWD", font=FT_SM,
                           fg=ORANGE, bg=PANEL, width=4)
        dir_lbl.pack(side="left", padx=2)

        spd_var = tk.IntVar(value=50)
        spd_scale = tk.Scale(row, from_=1, to=100,
                             orient="horizontal", variable=spd_var,
                             length=120, bg=PANEL, fg=WHITE,
                             troughcolor=BG, highlightthickness=0,
                             bd=0, activebackground=ORANGE,
                             showvalue=False,
                             command=lambda v, n=name: set_motor_speed(n, int(v)))
        spd_scale.pack(side="left")

        spd_lbl = tk.Label(row, text=" 50%", font=FT_SM,
                           fg=ORANGE, bg=PANEL, width=5)
        spd_lbl.pack(side="left")

        jog = tk.Frame(row, bg=PANEL)
        jog.pack(side="left", padx=5)

        btn_n = tk.Button(jog, text=" - ", font=FT_MED,
                          bg="#3a1a0a", fg=WHITE, relief="raised",
                          activebackground=RED_LT, activeforeground=WHITE,
                          bd=2, padx=4, pady=3, cursor="hand2")
        btn_n.pack(side="left", padx=1)
        btn_n.bind("<ButtonPress-1>",   lambda e, n=name: self._jog(n, 0))
        btn_n.bind("<ButtonRelease-1>", lambda e, n=name: self._stop(n))

        btn_p = tk.Button(jog, text=" + ", font=FT_MED,
                          bg="#0a2a0a", fg=WHITE, relief="raised",
                          activebackground=GREEN, activeforeground="black",
                          bd=2, padx=4, pady=3, cursor="hand2")
        btn_p.pack(side="left", padx=1)
        btn_p.bind("<ButtonPress-1>",   lambda e, n=name: self._jog(n, 1))
        btn_p.bind("<ButtonRelease-1>", lambda e, n=name: self._stop(n))

        self.jw[name] = {
            "row": row, "sel": sel_lbl, "ax": ax_lbl,
            "pos": pos_lbl, "stat": stat_lbl,
            "lc": lc, "lo": lo, "dir": dir_lbl,
            "spd_var": spd_var, "spd_lbl": spd_lbl,
        }

    def _build_right_panel(self, parent):
        rp = tk.Frame(parent, bg=BG2, width=240)
        rp.pack(side="right", fill="y")
        rp.pack_propagate(False)

        tk.Button(rp, text="E - S T O P",
                  font=("Courier New", 14, "bold"),
                  bg="#aa1100", fg="white",
                  activebackground=RED_LT,
                  relief="raised", bd=5,
                  height=2, cursor="hand2",
                  command=stop_all).pack(fill="x", padx=8, pady=(10, 3))

        tk.Button(rp, text="STOP ALL AXES",
                  font=FT_MED, bg=DARKGREY, fg=YELLOW,
                  activebackground=YELLOW, activeforeground="black",
                  relief="raised", bd=2, cursor="hand2",
                  command=stop_all).pack(fill="x", padx=8, pady=2)

        self._div(rp)

        tk.Label(rp, text="ACTIVE AXIS", font=FT_SM,
                 fg=GREY, bg=BG2).pack(anchor="w", padx=10)

        self.sel_xl = tk.Label(rp, text="J1",
                               font=("Courier New", 42, "bold"),
                               fg=ORANGE, bg=BG2)
        self.sel_xl.pack()

        self.sel_spd_lbl = tk.Label(rp, text="SPEED:  50%",
                                    font=FT_MED, fg=WHITE, bg=BG2)
        self.sel_spd_lbl.pack()

        self.sel_pos_lbl = tk.Label(rp, text="POS: +000000",
                                    font=("Courier New", 10),
                                    fg=SCRN_GREEN, bg=BG2)
        self.sel_pos_lbl.pack()

        self.sel_stat_lbl = tk.Label(rp, text="STATUS: IDLE",
                                     font=FT_SM, fg=GREY, bg=BG2)
        self.sel_stat_lbl.pack()

        spd_row = tk.Frame(rp, bg=BG2)
        spd_row.pack(pady=4)

        tk.Button(spd_row, text=" SPD - ",
                  font=FT_MED, bg=DARKGREY, fg=WHITE,
                  activebackground=RED, relief="raised", bd=2,
                  cursor="hand2",
                  command=lambda: self._nudge(-5)).pack(side="left", padx=3)

        tk.Button(spd_row, text=" SPD + ",
                  font=FT_MED, bg=DARKGREY, fg=WHITE,
                  activebackground=GREEN, relief="raised", bd=2,
                  cursor="hand2",
                  command=lambda: self._nudge(5)).pack(side="left", padx=3)

        self._div(rp)

        tk.Label(rp, text="GLOBAL OVERRIDE", font=FT_SM,
                 fg=GREY, bg=BG2).pack(anchor="w", padx=10)

        self.ovr_var = tk.IntVar(value=50)
        tk.Scale(rp, from_=1, to=100, orient="horizontal",
                 variable=self.ovr_var, length=210,
                 bg=BG2, fg=WHITE, troughcolor=BG,
                 highlightthickness=0, bd=0,
                 activebackground=ORANGE, showvalue=False,
                 command=self._set_override).pack(padx=8)

        self.ovr_lbl = tk.Label(rp, text="OVR:  50%",
                                font=FT_MED, fg=ORANGE, bg=BG2)
        self.ovr_lbl.pack()

        self._div(rp)

        tk.Label(rp, text="COOLING FAN", font=FT_SM,
                 fg=GREY, bg=BG2).pack(anchor="w", padx=10)

        fan_btns = tk.Frame(rp, bg=BG2)
        fan_btns.pack(fill="x", padx=8, pady=2)

        for lbl, val in [("OFF", 0), ("25%", 25), ("50%", 50), ("75%", 75), ("100%", 100)]:
            tk.Button(fan_btns, text=lbl, font=FT_TINY,
                      bg=DARKGREY, fg=WHITE,
                      activebackground=ORANGE, activeforeground="black",
                      relief="raised", bd=1, cursor="hand2",
                      command=lambda v=val: self._set_fan(v)).pack(
                          side="left", expand=True, fill="x", padx=1)

        self.fan_var = tk.IntVar(value=0)
        tk.Scale(rp, from_=0, to=100, orient="horizontal",
                 variable=self.fan_var, length=210,
                 bg=BG2, fg=WHITE, troughcolor=BG,
                 highlightthickness=0, bd=0,
                 activebackground=ORANGE, showvalue=False,
                 command=lambda v: self._set_fan(int(v))).pack(padx=8)

        self.fan_lbl = tk.Label(rp, text="FAN:   0%",
                                font=FT_MED, fg=ORANGE, bg=BG2)
        self.fan_lbl.pack()

        self._div(rp)

        tk.Label(rp, text="PENDANT INPUT", font=FT_SM,
                 fg=GREY, bg=BG2).pack(anchor="w", padx=10)

        self.ctrl_lbl = tk.Label(rp, text="NO DEVICE",
                                 font=FT_MED, fg=RED, bg=BG2)
        self.ctrl_lbl.pack(pady=2)

        tk.Button(rp, text="CONNECT CONTROLLER",
                  font=FT_SM, bg=DARKGREY, fg=ORANGE,
                  activebackground=ORANGE, activeforeground="black",
                  relief="raised", bd=2, cursor="hand2",
                  command=start_ps4).pack(fill="x", padx=8, pady=2)

        self._div(rp)

        tk.Label(rp, text="PROGRAM MODE", font=FT_SM,
                 fg=GREY, bg=BG2).pack(anchor="w", padx=10)

        self.mode_var = tk.StringVar(value="JOINT")

        tk.Radiobutton(rp, text="Joint Mode",
                       variable=self.mode_var, value="JOINT",
                       font=FT, fg=WHITE, bg=BG2,
                       selectcolor=BG, activebackground=BG2,
                       command=self._set_mode).pack(anchor="w", padx=14)

        tk.Radiobutton(rp, text="World Mode  [stub]",
                       variable=self.mode_var, value="WORLD",
                       font=FT, fg=GREY, bg=BG2,
                       selectcolor=BG, activebackground=BG2,
                       command=self._set_mode).pack(anchor="w", padx=14)

        self._div(rp)

        tk.Label(rp, text="SYSTEM LOG", font=FT_SM,
                 fg=GREY, bg=BG2).pack(anchor="w", padx=10)

        self.msg_lbl = tk.Label(rp, text="Ready.",
                                font=FT_SM, fg=GREEN, bg=BG2,
                                wraplength=220, justify="left", anchor="w")
        self.msg_lbl.pack(anchor="w", padx=10)

    def _build_softkeys(self):
        bar = tk.Frame(self.root, bg="#111111", height=36)
        bar.pack(fill="x", side="bottom")
        bar.pack_propagate(False)

        keys = [
            ("F1  ZERO POS",  self._zero),
            ("F2  STOP ALL",  stop_all),
            ("F3  MODE",      self._toggle_mode),
            ("F4  FAN 50%",   lambda: self._set_fan(50)),
            ("F5  FAN OFF",   lambda: self._set_fan(0)),
            ("F6  OVR 100",   lambda: self._set_override(100)),
            ("F7  OVR 50",    lambda: self._set_override(50)),
            ("F8  EXIT",      self._quit),
        ]
        for label, cmd in keys:
            tk.Button(bar, text=label, font=FT_SM,
                      bg="#1e1e1e", fg=WHITE,
                      activebackground=ORANGE, activeforeground="black",
                      relief="flat", bd=0, padx=6, cursor="hand2",
                      command=cmd).pack(side="left", fill="y",
                                        expand=True, padx=1, pady=3)

    def _div(self, parent):
        tk.Frame(parent, bg="#484848", height=1).pack(fill="x", padx=8, pady=5)

    def _select(self, name):
        global selected_joint
        selected_joint = JOINT_ORDER.index(name)
        log("Selected: " + name)

    def _jog(self, name, direction):
        self._select(name)
        start_motor(name, direction)

    def _stop(self, name):
        stop_motor(name)

    def _nudge(self, delta):
        jname = get_joint_name()
        new_spd = max(1, min(100, motors[jname]["speed"] + delta))
        set_motor_speed(jname, new_spd)
        self.jw[jname]["spd_var"].set(new_spd)
        log("{} spd {}%".format(jname, new_spd))

    def _set_override(self, val):
        pct = int(val)
        self.ovr_var.set(pct)
        self.ovr_lbl.config(text="OVR: {:3d}%".format(pct))
        self.ovr_badge.config(text=" OVR: {}% ".format(pct))
        for name in JOINT_ORDER:
            set_motor_speed(name, pct)
            self.jw[name]["spd_var"].set(pct)

    def _set_fan(self, val):
        set_fan(val)
        self.fan_var.set(val)
        self.fan_lbl.config(text="FAN: {:3d}%".format(val))
        log("Fan: {}%".format(val))

    def _set_mode(self):
        global control_mode
        control_mode = self.mode_var.get()
        log("Mode: " + control_mode)

    def _toggle_mode(self):
        global control_mode
        control_mode = "WORLD" if control_mode == "JOINT" else "JOINT"
        self.mode_var.set(control_mode)
        log("Mode: " + control_mode)

    def _zero(self):
        for m in motors.values():
            m["position"] = 0
        log("All positions zeroed")

    def _quit(self):
        # Hard fan off before anything else
        fan_off_hard()
        stop_all()
        lgpio.gpiochip_close(h)
        self.root.destroy()

    def update_loop(self):
        self.time_lbl.config(text=time.strftime("%H:%M:%S"))

        lc, lo, con, coff = self.status_leds["E-STOP"]
        set_led(lc, lo, estop_active, con, coff)

        lc, lo, con, coff = self.status_leds["CTRL"]
        set_led(lc, lo, ps4_connected, con, coff)

        any_lim = any(m["at_limit"] for m in motors.values())
        lc, lo, con, coff = self.status_leds["LIMITS"]
        set_led(lc, lo, any_lim, con, coff)

        any_run = any(m["running"] for m in motors.values())
        lc, lo, con, coff = self.status_leds["MOTORS"]
        set_led(lc, lo, any_run, GREEN, GREEN_DK)

        if control_mode == "JOINT":
            self.mode_badge.config(text=" JOINT ", bg=ORANGE)
        else:
            self.mode_badge.config(text=" WORLD ", bg=YELLOW)

        self.sel_badge.config(text=" SEL: {} ".format(get_joint_name()))

        if ps4_connected:
            self.ctrl_lbl.config(text="CONTROLLER OK", fg=GREEN)
        else:
            self.ctrl_lbl.config(text="NO DEVICE", fg=RED)

        self.fan_lbl.config(text="FAN: {:3d}%".format(fan_speed))

        jname = get_joint_name()
        m = motors[jname]
        self.sel_xl.config(text=jname)
        self.sel_spd_lbl.config(text="SPEED: {:3d}%".format(m["speed"]))
        pos  = m["position"]
        sign = "+" if pos >= 0 else "-"
        self.sel_pos_lbl.config(text="POS: {}{:06d}".format(sign, abs(pos)))

        if estop_active:
            self.sel_stat_lbl.config(text="STATUS: E-STOP", fg=RED)
        elif m["at_limit"]:
            self.sel_stat_lbl.config(text="STATUS: AT LIMIT", fg=YELLOW)
        elif m["running"]:
            self.sel_stat_lbl.config(text="STATUS: RUNNING", fg=GREEN)
        else:
            self.sel_stat_lbl.config(text="STATUS: IDLE", fg=GREY)

        for name in JOINT_ORDER:
            m = motors[name]
            w = self.jw[name]
            is_sel = (JOINT_ORDER[selected_joint] == name)

            bg = PANEL_SEL if is_sel else PANEL
            w["row"].config(bg=bg)
            w["sel"].config(text=">" if is_sel else " ", bg=bg)
            w["ax"].config(bg=bg, fg=ORANGE if is_sel else WHITE)

            pos  = m["position"]
            sign = "+" if pos >= 0 else "-"
            w["pos"].config(text="{}{:06d}".format(sign, abs(pos)))

            if estop_active:
                w["stat"].config(text="E-STOP  ", fg=RED)
            elif m["at_limit"]:
                w["stat"].config(text="AT LIMIT", fg=YELLOW)
            elif m["running"]:
                w["stat"].config(text="RUNNING ", fg=GREEN)
            else:
                w["stat"].config(text="IDLE    ", fg=GREY)

            set_led(w["lc"], w["lo"], m["at_limit"], RED, GREEN_DK)
            w["dir"].config(text="FWD" if m["direction"] == 1 else "REV",
                            fg=GREEN if m["direction"] == 1 else RED)
            w["spd_lbl"].config(text="{:3d}%".format(m["speed"]))

        # System log
        self.log_text.config(state="normal")
        self.log_text.delete("1.0", "end")
        for entry in system_log[-6:]:
            self.log_text.insert("end", entry + "\n")
        self.log_text.config(state="disabled")
        self.log_text.see("end")

        # Latest log entry in sidebar
        if system_log:
            self.msg_lbl.config(text=system_log[-1])

        self.root.after(100, self.update_loop)

# ─────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────
root = tk.Tk()
app = Pendant(root)
root.protocol("WM_DELETE_WINDOW", app._quit)
root.mainloop()
