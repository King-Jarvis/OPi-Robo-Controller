import lgpio, time, threading, tkinter as tk, pygame, math, subprocess, os, json, socket
import numpy as np
import ikpy.chain, ikpy.link
from tkinter import ttk, filedialog, messagebox

# ─────────────────────────────────────────────
# GPIO
# ─────────────────────────────────────────────
GPIOCHIP=0; FAN_PIN=267; ESTOP_PIN=266; DEBOUNCE_TIME=0.05; FAN_PWM_FREQ=1000

# ─────────────────────────────────────────────
# KINEMATICS
# ─────────────────────────────────────────────
L1=136.0; L2=250.0; L3=26.0; L4=235.25; L5=11.5; TCP=7.5

STEPS_PER_DEG={"J1":61.111,"J2":35.455,"J3":13.333,"J4":22.222,"J5":4.444,"J6":2.222}

JOINT_LIMITS_IK={"J1":(-175.0,175.0),"J2":(-90.0,90.0),"J3":(0.0,140.0),
"J4":(-150.0,150.0),"J5":(-150.0,150.0),"J6":(-180.0,180.0)}
JOINT_LIMITS_PHYS={"J1":(0.0,350.0),"J2":(0.0,150.0),"J3":(0.0,140.0),
"J4":(0.0,300.0),"J5":(0.0,150.0),"J6":(0.0,360.0)}

HOME_DIR={"J1":1,"J2":0,"J3":0,"J4":0,"J5":1,"J6":None}
# HOME_TARGET: angle (deg) the joint moves TO after finding its limit switch.
# J1=175 (center of 0-350), J4=150 (center of 0-300), J5=75 (center of 0-150).
# J2/J3 are user-set via Motor Config menu (adjust for your arm geometry).
# J6 has no switch — zeroed in place.
HOME_TARGET={"J1":175.0,"J2":75.0,"J3":70.0,"J4":150.0,"J5":75.0,"J6":0.0}

# ─────────────────────────────────────────────
# MOTOR TABLE
# ─────────────────────────────────────────────
motors={
"J1":{"step":226,"dir":227,"target_freq":3000,"limit":260,"running":False,"direction":1,"at_limit":False,"position":0,"angle_deg":0.0,"speed":30,"homed":False},
"J2":{"step":228,"dir":229,"target_freq":4000,"limit":261,"running":False,"direction":1,"at_limit":False,"position":0,"angle_deg":0.0,"speed":30,"homed":False},
"J3":{"step":230,"dir":231,"target_freq":5000,"limit":262,"running":False,"direction":1,"at_limit":False,"position":0,"angle_deg":0.0,"speed":30,"homed":False},
"J4":{"step":232,"dir":233,"target_freq":4000,"limit":263,"running":False,"direction":1,"at_limit":False,"position":0,"angle_deg":0.0,"speed":30,"homed":False},
"J5":{"step":256,"dir":257,"target_freq":8000,"limit":264,"running":False,"direction":1,"at_limit":False,"position":0,"angle_deg":0.0,"speed":30,"homed":False},
"J6":{"step":258,"dir":259,"target_freq":8000,"limit":265,"running":False,"direction":1,"at_limit":False,"position":0,"angle_deg":0.0,"speed":30,"homed":False},
}
JOINT_ORDER=["J1","J2","J3","J4","J5","J6"]

h=lgpio.gpiochip_open(GPIOCHIP)
estop_active=False; fan_speed=2; system_log=[]; alarm_log=[]
homing_active=False; all_homed=False; start_time=time.time()

def log(msg,alarm=False):
    ts=time.strftime("%H:%M:%S")
    entry="[{}] {}".format(ts,msg)
    system_log.append(entry)
    if len(system_log)>200: system_log.pop(0)
    if alarm:
        alarm_log.append(entry)
        if len(alarm_log)>100: alarm_log.pop(0)

for name,m in motors.items():
    lgpio.gpio_claim_output(h,m["step"],0)
    lgpio.gpio_claim_output(h,m["dir"],0)
    lgpio.gpio_claim_input(h,m["limit"],lgpio.SET_PULL_UP)
lgpio.gpio_claim_input(h,ESTOP_PIN,lgpio.SET_PULL_UP)
lgpio.gpio_claim_output(h,FAN_PIN,0)
lgpio.tx_pwm(h,FAN_PIN,FAN_PWM_FREQ,2)   # 2% idle — keeps fan alive always
fan_speed = 2

# ─────────────────────────────────────────────
# CONTROL MODE  (declared early so AGVLink thread can reference it)
# ─────────────────────────────────────────────
control_mode    = "JOINT"   # JOINT | WORLD | TOOL | AGV
selected_joint  = 0
ps4_connected   = False
ps4_thread      = None
DEADZONE        = 0.25
MODE_CYCLE      = ["JOINT","WORLD","TOOL"]
_last_arm_mode  = "JOINT"

# ─────────────────────────────────────────────
# AGV UDP SENDER
# ─────────────────────────────────────────────
# The ESP32 hotspot default gateway. Confirm with: ping 192.168.4.1
ESP32_IP   = "192.168.4.1"
ESP32_PORT = 5005
AGV_SEND_HZ = 50   # packets per second

class AGVLink:
    """
    Sends UDP packets to the ESP32 AGV at ~50Hz when in AGV mode.
    Packet format: JSON {"t": float, "r": float}
      t = translate  -1.0 (reverse) to +1.0 (forward)   <- left stick Y
      r = rotate     -1.0 (left)    to +1.0 (right)      <- right stick X
    Also polls /status from the ESP32 web server every 500ms
    and makes it available via agv.status_json for the panel.
    """
    def __init__(self):
        self.sock        = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.translate   = 0.0
        self.rotate      = 0.0
        self.running     = True
        self.status_json = {}
        threading.Thread(target=self._send_loop, daemon=True).start()
        threading.Thread(target=self._status_loop, daemon=True).start()

    def _send_loop(self):
        # Send raw values at 50Hz. ESP32 ramp handles smoothing.
        interval = 1.0 / AGV_SEND_HZ
        while self.running:
            if control_mode == "AGV":
                try:
                    self.sock.sendto(
                        self._pack(self.translate, self.rotate),
                        (ESP32_IP, ESP32_PORT))
                except Exception:
                    pass
            time.sleep(interval)

    @staticmethod
    def _crc8(data: bytes) -> int:
        crc = 0
        for b in data:
            crc ^= b
            for _ in range(8):
                crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
        return crc

    @staticmethod
    def _pack(t: float, r: float) -> bytes:
        """Pack translate/rotate into compact 8-byte binary packet.
        Format: b'EVA' + int16(t*10000) + int16(r*10000) + CRC8
        Matches ESP32 handleUDP() binary parser."""
        import struct
        ti = int(max(-1.0, min(1.0, t)) * 10000)
        ri = int(max(-1.0, min(1.0, r)) * 10000)
        header = b'EVA' + struct.pack('>hh', ti, ri)
        return header + bytes([AGVLink._crc8(header)])

    def _status_loop(self):
        """Poll /status endpoint on the ESP32 for telemetry."""
        import urllib.request
        while self.running:
            try:
                url = "http://{}:{}/status".format(ESP32_IP, 80)
                with urllib.request.urlopen(url, timeout=0.4) as r:
                    raw = r.read().decode()
                    self.status_json = json.loads(raw)
            except Exception:
                pass
            time.sleep(0.5)

    def set(self, translate, rotate):
        self.translate = max(-1.0, min(1.0, translate))
        self.rotate    = max(-1.0, min(1.0, rotate))

    def stop(self):
        self.translate = 0.0
        self.rotate    = 0.0

agv = AGVLink()

# ─────────────────────────────────────────────
# MOTOR CONTROL
# ─────────────────────────────────────────────
def set_fan(pct):
    global fan_speed
    pct = max(0, min(100, pct)); fan_speed = pct
    if pct == 0:
        # Full off: stop PWM then drive pin low
        lgpio.tx_pwm(h, FAN_PIN, 1, 0)
        time.sleep(0.02)
        lgpio.gpio_write(h, FAN_PIN, 0)
    else:
        # Set new duty directly — no stop in between so it transitions smoothly
        lgpio.tx_pwm(h, FAN_PIN, FAN_PWM_FREQ, pct)

def fan_off_hard():
    lgpio.tx_pwm(h,FAN_PIN,1,0); time.sleep(0.05)
    lgpio.gpio_write(h,FAN_PIN,0); time.sleep(0.05)

def stop_motor(name):
    m=motors[name]
    lgpio.tx_pwm(h,m["step"],1,0)
    lgpio.gpio_claim_output(h,m["step"],0)
    m["running"]=False

def stop_all():
    for name in motors: stop_motor(name)
    log("STOP ALL",alarm=True)

def start_motor(name,direction,freq_override=None):
    m=motors[name]
    if estop_active or (m["at_limit"] and m["direction"]==direction): return
    lgpio.tx_pwm(h,m["step"],1,0)
    lgpio.gpio_claim_output(h,m["step"],0)
    time.sleep(0.02)
    m["direction"]=direction; m["running"]=True
    lgpio.gpio_write(h,m["dir"],direction); time.sleep(0.01)
    freq=freq_override if freq_override else max(10,int(m["target_freq"]*m["speed"]/100))
    lgpio.tx_pwm(h,m["step"],freq,50)

def set_motor_speed(name,pct):
    m=motors[name]; m["speed"]=max(1,min(100,int(pct)))
    if m["running"]:
        freq=max(10,int(m["target_freq"]*m["speed"]/100))
        lgpio.tx_pwm(h,m["step"],freq,50)

def deg_to_steps(name,deg): return int(deg*STEPS_PER_DEG[name])
def steps_to_deg(name,steps): return steps/STEPS_PER_DEG[name]

# ─────────────────────────────────────────────
# LIMIT MONITOR
# ─────────────────────────────────────────────
def monitor_limits():
    global estop_active
    last_trigger={name:0 for name in motors}; last_estop=0
    while True:
        try:
            if lgpio.gpio_read(h,ESTOP_PIN)==1:
                now=time.time()
                if now-last_estop>DEBOUNCE_TIME:
                    if not estop_active:
                        estop_active=True; stop_all()
                        log("!! E-STOP TRIGGERED !!",alarm=True)
                    last_estop=now
            else:
                if estop_active: log("E-STOP released")
                estop_active=False
            for name,m in motors.items():
                state=lgpio.gpio_read(h,m["limit"]); now=time.time()
                if state==1:
                    if now-last_trigger[name]>DEBOUNCE_TIME:
                        if not m["at_limit"]:
                            m["at_limit"]=True
                            if not homing_active:
                                stop_motor(name); log("{} limit hit".format(name),alarm=True)
                        last_trigger[name]=now
                else:
                    if m["at_limit"]: log("{} limit cleared".format(name))
                    m["at_limit"]=False
                if m["running"]:
                    m["position"]+=1 if m["direction"]==1 else -1
                    m["angle_deg"]=steps_to_deg(name,m["position"])
            time.sleep(0.01)
        except Exception: break

threading.Thread(target=monitor_limits,daemon=True).start()

# ─────────────────────────────────────────────
# HOMING
# ─────────────────────────────────────────────
def home_joint(name):
    m=motors[name]; hdir=HOME_DIR[name]

    # J6: no limit switch — zero in place and mark homed
    if hdir is None:
        m["position"]=0; m["angle_deg"]=0.0; m["homed"]=True
        log("{} skipped (no limit switch) — zeroed in place".format(name))
        return True

    target_deg = HOME_TARGET[name]
    log("{} homing — will move to {:.1f}deg after limit".format(name, target_deg))

    # ── Phase 1: drive toward limit switch at 20% speed ──────────────────
    home_freq=max(10,int(m["target_freq"]*0.2))
    lgpio.tx_pwm(h,m["step"],1,0); lgpio.gpio_claim_output(h,m["step"],0); time.sleep(0.02)
    m["direction"]=hdir; m["running"]=True
    lgpio.gpio_write(h,m["dir"],hdir); time.sleep(0.01)
    lgpio.tx_pwm(h,m["step"],home_freq,50)
    timeout=time.time()+30.0
    while not m["at_limit"]:
        if estop_active: stop_motor(name); log("{} aborted".format(name),alarm=True); return False
        if time.time()>timeout: stop_motor(name); log("{} TIMEOUT".format(name),alarm=True); return False
        time.sleep(0.01)

    # ── Limit found: this is position 0 for this joint ───────────────────
    stop_motor(name)
    m["position"]=0; m["angle_deg"]=0.0
    log("{} limit found — position zeroed, moving to {:.1f}deg".format(name, target_deg))
    time.sleep(0.15)

    # ── Phase 2: move_to_angle to the home target ─────────────────────────
    # Temporarily mark homed so move_to_angle will execute
    m["homed"]=True
    # Use a slower speed for the backoff move for safety
    saved_speed = m["speed"]
    m["speed"] = 20
    move_to_angle(name, target_deg)
    m["speed"] = saved_speed

    if estop_active: return False

    log("{} at home position {:.1f}deg".format(name, m["angle_deg"]))
    return True

def run_homing():
    global homing_active,all_homed
    homing_active=True; all_homed=False
    log("=== HOMING SEQUENCE STARTED ===")
    for name in JOINT_ORDER:
        if estop_active: log("Homing aborted",alarm=True); homing_active=False; return
        if not home_joint(name): homing_active=False; return
        time.sleep(0.3)
    homing_active=False; all_homed=True
    positions=" ".join("{}:{:.1f}".format(n,motors[n]["angle_deg"]) for n in JOINT_ORDER)
    log("=== HOMING COMPLETE - READY ===")
    log("Home positions: "+positions)

def start_homing(): threading.Thread(target=run_homing,daemon=True).start()

# ─────────────────────────────────────────────
# KINEMATICS  —  ikpy numerical solver
# ─────────────────────────────────────────────
def _build_chain():
    return ikpy.chain.Chain([
        ikpy.link.OriginLink(),
        ikpy.link.URDFLink(name='J1',
            origin_translation=[0,0,L1], origin_orientation=[0,0,0],
            rotation=[0,0,1]),
        ikpy.link.URDFLink(name='J2',
            origin_translation=[0,0,0],  origin_orientation=[0,-np.pi/2,0],
            rotation=[0,1,0]),
        ikpy.link.URDFLink(name='J3',
            origin_translation=[L2,0,0], origin_orientation=[0,0,0],
            rotation=[0,1,0]),
        ikpy.link.URDFLink(name='J4',
            origin_translation=[L3,0,0], origin_orientation=[0,np.pi/2,0],
            rotation=[1,0,0]),
        ikpy.link.URDFLink(name='J5',
            origin_translation=[0,0,L4], origin_orientation=[0,-np.pi/2,0],
            rotation=[0,1,0]),
        ikpy.link.URDFLink(name='J6',
            origin_translation=[0,0,0],  origin_orientation=[0,np.pi/2,0],
            rotation=[0,0,1]),
    ])

_arm_chain = _build_chain()

def _rebuild_chain():
    global _arm_chain
    _arm_chain = _build_chain()
    log('IK chain rebuilt')

def forward_kinematics(angles_deg):
    rads = [0.0] + [math.radians(a) for a in angles_deg]
    fk = _arm_chain.forward_kinematics(rads)
    tool_len = L5 + TCP
    pos = fk[:3, 3] + fk[:3, 2] * tool_len
    return [float(pos[0]), float(pos[1]), float(pos[2])]

def inverse_kinematics(tx, ty, tz, rx=0.0, ry=0.0, rz=0.0):
    try:
        tool_len = L5 + TCP
        target   = np.array([tx, ty, tz], dtype=float)

        # Warm-start from current joint angles for stability
        current = [0.0] + [math.radians(motors[n]['angle_deg']) for n in JOINT_ORDER]

        has_orientation = not (rx == 0.0 and ry == 0.0 and rz == 0.0)

        if has_orientation:
            crx=math.cos(math.radians(rx)); srx=math.sin(math.radians(rx))
            cry=math.cos(math.radians(ry)); sry=math.sin(math.radians(ry))
            crz=math.cos(math.radians(rz)); srz=math.sin(math.radians(rz))
            Rx=np.array([[1,0,0],[0,crx,-srx],[0,srx,crx]])
            Ry=np.array([[cry,0,sry],[0,1,0],[-sry,0,cry]])
            Rz=np.array([[crz,-srz,0],[srz,crz,0],[0,0,1]])
            R = Rz @ Ry @ Rx
            # Back out TCP offset to get J6 origin target
            j6_pos = target - R[:, 2] * tool_len
            sol = _arm_chain.inverse_kinematics(
                target_position=j6_pos,
                target_orientation=R,
                orientation_mode='Z',
                initial_position=current,
                max_iter=300,
            )
            verify_pos = j6_pos
        else:
            # Position-only: point TCP at target, ignore orientation
            # Back out tool length along a neutral Z direction
            j6_pos = target - np.array([0, 0, tool_len])
            sol = _arm_chain.inverse_kinematics(
                target_position=j6_pos,
                initial_position=current,
                max_iter=300,
            )
            verify_pos = j6_pos

        angles_out = [math.degrees(float(a)) for a in sol[1:7]]

        # Verify FK error < 8mm
        fk_check = _arm_chain.forward_kinematics(sol)
        error    = float(np.linalg.norm(fk_check[:3, 3] - verify_pos))
        if error > 8.0:
            log('IK: position error {:.1f}mm — no solution'.format(error))
            return None

        # Check joint limits
        for i, name in enumerate(JOINT_ORDER):
            lo, hi = JOINT_LIMITS_IK[name]
            if angles_out[i] < lo or angles_out[i] > hi:
                log('IK: {} = {:.1f}° outside limits'.format(name, angles_out[i]))
                return None

        return angles_out

    except Exception as e:
        log('IK error: {}'.format(e))
        return None

# ─────────────────────────────────────────────
# ABSOLUTE MOVE  —  non-blocking, interruptible
# Each joint has a cancel Event. Starting a new
# move immediately cancels the previous one so
# direction changes in world/tool mode are instant.
# move_all_to_angles(wait=True) used for playback
# is unaffected — those threads still join().
# ─────────────────────────────────────────────
_move_cancel = {name: threading.Event() for name in ["J1","J2","J3","J4","J5","J6"]}

def move_to_angle(name, target_deg):
    m = motors[name]
    if not m["homed"]: return
    lo, hi = JOINT_LIMITS_PHYS[name]
    target_deg = max(lo, min(hi, target_deg))
    target_steps = deg_to_steps(name, target_deg)
    delta = target_steps - m["position"]
    if abs(delta) < 2: return

    # Cancel any in-progress move on this joint immediately
    _move_cancel[name].set()
    time.sleep(0.015)           # let the old thread exit its loop
    _move_cancel[name].clear()

    direction = 1 if delta > 0 else 0
    steps_to_move = abs(delta)
    freq = max(10, int(m["target_freq"] * m["speed"] / 100))
    move_time = steps_to_move / freq

    lgpio.tx_pwm(h, m["step"], 1, 0)
    lgpio.gpio_claim_output(h, m["step"], 0)
    time.sleep(0.015)
    m["direction"] = direction
    m["running"] = True
    lgpio.gpio_write(h, m["dir"], direction)
    time.sleep(0.008)
    lgpio.tx_pwm(h, m["step"], freq, 50)

    elapsed = 0.0
    interval = 0.008
    while elapsed < move_time:
        if estop_active or m["at_limit"] or _move_cancel[name].is_set():
            break
        time.sleep(interval)
        elapsed += interval

    stop_motor(name)

def move_all_to_angles(angles,wait=True):
    threads=[]
    for i,name in enumerate(JOINT_ORDER):
        t=threading.Thread(target=move_to_angle,args=(name,angles[i]),daemon=True)
        threads.append(t); t.start()
    if wait:
        for t in threads: t.join()

# ─────────────────────────────────────────────
# CARTESIAN JOG
# ─────────────────────────────────────────────
CART_SCALE_MAJOR = 8.0
CART_SCALE_MINOR = 2.0
CART_SCALE_ROT   = 3.0

def _cart_jog(dx, dy, dz, drx=0.0, dry=0.0, drz=0.0, tool_frame=False):
    if not all_homed:
        return   # silently skip — not homed yet, no log spam
    angles = [motors[n]["angle_deg"] for n in JOINT_ORDER]
    tcp    = forward_kinematics(angles)
    if tool_frame:
        j6_rad = math.radians(angles[5])
        j5_rad = math.radians(angles[4])
        wx = dx * math.cos(j6_rad) - dy * math.sin(j6_rad)
        wy = dx * math.sin(j6_rad) + dy * math.cos(j6_rad)
        wz = dz * math.cos(j5_rad) - dx * math.sin(j5_rad)
        dx, dy, dz = wx, wy, wz
    new_x  = tcp[0] + dx
    new_y  = tcp[1] + dy
    new_z  = tcp[2] + dz
    cur_rx = angles[3]; cur_ry = angles[4]; cur_rz = angles[5]
    sol = inverse_kinematics(new_x, new_y, new_z,
                             cur_rx + drx, cur_ry + dry, cur_rz + drz)
    if sol is None: return
    for i, name in enumerate(JOINT_ORDER):
        threading.Thread(target=move_to_angle, args=(name, sol[i]), daemon=True).start()

def world_jog(axis, direction):
    s = CART_SCALE_MAJOR * direction
    sf = CART_SCALE_MINOR * direction
    sr = CART_SCALE_ROT * direction
    if   axis == "X":  _cart_jog(s,  0,  0)
    elif axis == "Y":  _cart_jog(0,  s,  0)
    elif axis == "Z":  _cart_jog(0,  0,  s)
    elif axis == "Zf": _cart_jog(0,  0,  sf)
    elif axis == "Rx": _cart_jog(0,  0,  0, drx=sr)
    elif axis == "Ry": _cart_jog(0,  0,  0, dry=sr)
    elif axis == "Rz": _cart_jog(0,  0,  0, drz=sr)

def tool_jog(axis, direction):
    s = CART_SCALE_MAJOR * direction
    sf = CART_SCALE_MINOR * direction
    sr = CART_SCALE_ROT * direction
    if   axis == "X":  _cart_jog(s,  0,  0, tool_frame=True)
    elif axis == "Y":  _cart_jog(0,  s,  0, tool_frame=True)
    elif axis == "Z":  _cart_jog(0,  0,  s, tool_frame=True)
    elif axis == "Zf": _cart_jog(0,  0,  sf, tool_frame=True)
    elif axis == "Rx": _cart_jog(0,  0,  0, drx=sr)
    elif axis == "Ry": _cart_jog(0,  0,  0, dry=sr)
    elif axis == "Rz": _cart_jog(0,  0,  0, drz=sr)

def cartesian_jog(axis, direction):
    if control_mode == "WORLD":
        world_jog(axis, direction)
    elif control_mode == "TOOL":
        tool_jog(axis, direction)

def world_mode_move(dx, dy, dz):
    _cart_jog(dx * CART_SCALE_MAJOR, dy * CART_SCALE_MAJOR, dz * CART_SCALE_MAJOR)

# ─────────────────────────────────────────────
# LINEAR MOVE
# ─────────────────────────────────────────────
def linear_move(target_angles,steps=20,speed_pct=None):
    start_angles=[motors[n]["angle_deg"] for n in JOINT_ORDER]
    for step in range(1,steps+1):
        if estop_active: return
        t=step/steps
        interp=[start_angles[i]+(target_angles[i]-start_angles[i])*t for i in range(6)]
        move_all_to_angles(interp,wait=True)

# ─────────────────────────────────────────────
# PROGRAM / POINT SYSTEM
# ─────────────────────────────────────────────
class Program:
    def __init__(self):
        self.name     = "PROG1"
        self.points   = []
        self.variables= {}
        self.filepath = None
        self.modified = False

    def add_point(self,move_type="J",speed_pct=100,cnt=0,comment=""):
        angles=[motors[n]["angle_deg"] for n in JOINT_ORDER]
        tcp=forward_kinematics(angles)
        idx=len(self.points)+1
        pt={
            "index":   idx,
            "label":   "P[{}]".format(idx),
            "move_type": move_type,
            "speed_pct": speed_pct,
            "cnt":     cnt,
            "angles":  list(angles),
            "tcp":     list(tcp),
            "comment": comment,
        }
        self.points.append(pt); self.modified=True
        log("Point P[{}] recorded ({})".format(idx,move_type))
        return pt

    def delete_point(self,idx):
        if 0<=idx<len(self.points):
            self.points.pop(idx)
            for i,p in enumerate(self.points):
                p["index"]=i+1; p["label"]="P[{}]".format(i+1)
            self.modified=True; log("Point deleted, reindexed")

    def move_point_up(self,idx):
        if idx>0 and idx<len(self.points):
            self.points[idx],self.points[idx-1]=self.points[idx-1],self.points[idx]
            for i,p in enumerate(self.points): p["index"]=i+1; p["label"]="P[{}]".format(i+1)
            self.modified=True

    def move_point_down(self,idx):
        if idx>=0 and idx<len(self.points)-1:
            self.points[idx],self.points[idx+1]=self.points[idx+1],self.points[idx]
            for i,p in enumerate(self.points): p["index"]=i+1; p["label"]="P[{}]".format(i+1)
            self.modified=True

    def save(self,filepath=None):
        if filepath: self.filepath=filepath
        if not self.filepath: return False
        data={"name":self.name,"points":self.points,"variables":self.variables}
        with open(self.filepath,"w") as f: json.dump(data,f,indent=2)
        self.modified=False; log("Program saved: {}".format(self.filepath))
        return True

    def load(self,filepath):
        with open(filepath,"r") as f: data=json.load(f)
        self.name=data.get("name","PROG"); self.points=data.get("points",[])
        self.variables=data.get("variables",{}); self.filepath=filepath
        self.modified=False; log("Program loaded: {}".format(filepath))

    def to_display_line(self,pt):
        return "  {:>3}]  {}  {}:  {}  {}%  cnt:{}  {}".format(
            pt["index"], pt["move_type"], pt["label"],
            " ".join("{:+.1f}".format(a) for a in pt["angles"]),
            pt["speed_pct"], pt["cnt"], pt["comment"])

current_program=Program()

# ─────────────────────────────────────────────
# PROGRAM PLAYBACK
# ─────────────────────────────────────────────
playback_running  = False
playback_step_mode= False
playback_thread   = None

def run_playback(program,speed_override=None,forward=True):
    global playback_running
    playback_running=True
    points=program.points if forward else list(reversed(program.points))
    log("Playback START ({} pts, {})".format(len(points),"step" if playback_step_mode else "continuous"))
    for i,pt in enumerate(points):
        if not playback_running or estop_active: break
        spd=speed_override if speed_override else pt["speed_pct"]
        for name in JOINT_ORDER: set_motor_speed(name,spd)
        target=list(pt["angles"])
        log("Moving to P[{}] type={} spd={}% cnt={}".format(pt["index"],pt["move_type"],spd,pt["cnt"]))
        if pt["move_type"]=="L":
            linear_move(target,steps=30)
        else:
            move_all_to_angles(target,wait=True)
        if playback_step_mode:
            playback_running=False; break
        cnt=pt["cnt"]
        if cnt>0 and i<len(points)-1:
            time.sleep((100-cnt)/1000.0)
        else:
            time.sleep(0.1)
    if playback_running:
        log("Playback COMPLETE")
    playback_running=False

def start_playback(program,speed_override=None,forward=True):
    global playback_thread,playback_running
    if playback_running: return
    playback_thread=threading.Thread(
        target=run_playback,args=(program,speed_override,forward),daemon=True)
    playback_thread.start()

def stop_playback():
    global playback_running
    playback_running=False
    stop_all(); log("Playback stopped")

def step_playback(program,speed_override=None):
    global playback_running,playback_step_mode
    playback_step_mode=True
    start_playback(program,speed_override)

# ─────────────────────────────────────────────
# PS4 CONTROLLER
# ─────────────────────────────────────────────
# BUTTON MAP (pygame DS4 on Linux):
#   button 0 = Cross      — stop all / stop AGV
#   button 1 = Circle     — toggle AGV mode on/off
#   button 2 = Square     — cycle joint (ARM) / stop AGV (AGV)
#   button 3 = Triangle   — cycle ARM mode (JOINT->WORLD->TOOL)
#   button 4 = L1
#   button 5 = R1
#   button 7 = L3 (stick press)
#   button 8 = R3 (stick press)
#
# AGV MODE controls:
#   Left  stick Y  = translate  (push forward = drive forward)
#   Right stick X  = rotate     (push right   = turn right)
#   Cross or Square = stop AGV
#   Circle again    = exit AGV back to last ARM mode
# ─────────────────────────────────────────────
def get_joint_name(): return JOINT_ORDER[selected_joint]

def ps4_loop():
    global ps4_connected, control_mode, selected_joint, _last_arm_mode
    pygame.init(); pygame.joystick.init()
    joystick = None
    prev_cross = prev_circle = prev_sq = prev_tri = False
    prev_l3 = prev_r3 = False
    prev_dpad = (0, 0)
    active_joints = set()
    agv_entry_guard = 0   # counts down after entering AGV mode

    while True:
        try:
            if joystick is None:
                pygame.joystick.quit(); pygame.joystick.init()
                if pygame.joystick.get_count() > 0:
                    joystick = pygame.joystick.Joystick(0); joystick.init()
                    ps4_connected = True
                    log("Controller: " + joystick.get_name())
                    nb = joystick.get_numbuttons()
                    log("Buttons detected: {}".format(nb))
                else:
                    ps4_connected = False; time.sleep(1); continue

            pygame.event.pump()

            nb = joystick.get_numbuttons()

            # ── Read axes ────────────────────────────────────
            # DS4 on Linux (via pygame):
            # 0=LS-X  1=LS-Y  2=L2(-1 rest)  3=RS-X  4=RS-Y  5=R2(-1 rest)
            na = joystick.get_numaxes()
            ly  = joystick.get_axis(1)              # left stick  Y
            lx  = joystick.get_axis(0)              # left stick  X
            rx  = joystick.get_axis(3) if na>3 else 0.0  # right stick X
            ry  = joystick.get_axis(4) if na>4 else 0.0  # right stick Y
            # L2/R2 triggers sit at -1.0 at rest — normalize to 0..1
            l2_raw = joystick.get_axis(2) if na>2 else -1.0
            r2_raw = joystick.get_axis(5) if na>5 else -1.0
            l2  = (l2_raw + 1.0) / 2.0   # 0.0 = not pressed, 1.0 = full
            r2  = (r2_raw + 1.0) / 2.0
            l1  = joystick.get_button(4)
            r1  = joystick.get_button(5)
            cross  = joystick.get_button(0)             # ✕
            circle = joystick.get_button(1) if nb>1 else False  # ○  — AGV toggle
            sq     = joystick.get_button(2) if nb>2 else False  # □  — joint cycle
            tri    = joystick.get_button(3) if nb>3 else False  # △  — mode cycle
            l3     = joystick.get_button(7) if nb>7 else False
            r3     = joystick.get_button(8) if nb>8 else False
            hat    = joystick.get_hat(0) if joystick.get_numhats() > 0 else (0, 0)
            dpad_x, dpad_y = hat

            # ── CIRCLE: toggle AGV mode ───────────────────────
            if circle and not prev_circle:
                if control_mode == "AGV":
                    agv.stop()
                    control_mode = _last_arm_mode
                    log("AGV OFF  →  " + control_mode)
                else:
                    _last_arm_mode = control_mode
                    stop_all()
                    agv.stop()           # zero before entering
                    control_mode = "AGV"
                    log("AGV MODE ON  —  Left stick=drive  Right stick=turn")
            prev_circle = circle

            # ── TRIANGLE: cycle ARM modes (not in AGV) ────────
            if tri and not prev_tri:
                if control_mode != "AGV":
                    idx = MODE_CYCLE.index(control_mode)
                    control_mode = MODE_CYCLE[(idx + 1) % len(MODE_CYCLE)]
                    _last_arm_mode = control_mode
                    stop_all()
                    log("Mode: " + control_mode)
            prev_tri = tri

            # ── SQUARE: cycle joint in ARM / stop in AGV ──────
            if sq and not prev_sq:
                if control_mode == "AGV":
                    agv.stop()   # zeros, keeps enabled so driving resumes
                    log("AGV STOP")
                else:
                    selected_joint = (selected_joint + 1) % len(JOINT_ORDER)
                    stop_all()
                    log("Sel: " + get_joint_name())
            prev_sq = sq

            # ── CROSS: stop everything ────────────────────────
            if cross and not prev_cross:
                agv.stop()   # zeros, keeps enabled
                stop_all()
                log("STOP")
            prev_cross = cross

            # ════════════════════════════════════════════════
            # AGV MODE
            # Left stick Y = translate, Right stick X = rotate
            # Values held continuously while stick is displaced.
            # send_loop fires at 50Hz independently of this loop.
            # ════════════════════════════════════════════════
            if control_mode == "AGV":
                t     = -ly if abs(ly) > DEADZONE else 0.0
                r_val =  rx if abs(rx) > DEADZONE else 0.0
                # Always call set — keeps values alive for send_loop
                agv.set(t, r_val)
                prev_l3 = l3; prev_r3 = r3; prev_dpad = (dpad_x, dpad_y)
                time.sleep(0.02)
                continue

            # ════════════════════════════════════════════════
            # JOINT MODE
            # ════════════════════════════════════════════════
            if control_mode == "JOINT":
                jname = get_joint_name()

                # D-pad left/right: cycle joint
                if dpad_x != prev_dpad[0]:
                    if dpad_x ==  1: selected_joint = (selected_joint + 1) % len(JOINT_ORDER); stop_all(); log("Sel: " + get_joint_name())
                    if dpad_x == -1: selected_joint = (selected_joint - 1) % len(JOINT_ORDER); stop_all(); log("Sel: " + get_joint_name())

                # D-pad up/down: speed
                if dpad_y != prev_dpad[1]:
                    jname = get_joint_name()
                    if dpad_y ==  1: set_motor_speed(jname, min(100, motors[jname]["speed"] + 5)); log("{} spd {}%".format(jname, motors[jname]["speed"]))
                    if dpad_y == -1: set_motor_speed(jname, max(1,   motors[jname]["speed"] - 5)); log("{} spd {}%".format(jname, motors[jname]["speed"]))

                # L1/R1: fine speed trim
                jname = get_joint_name()
                if l1: set_motor_speed(jname, max(1,   motors[jname]["speed"] - 1))
                if r1: set_motor_speed(jname, min(100, motors[jname]["speed"] + 1))

                # Left stick Y: jog selected joint
                new_joints = set()
                if abs(ly) > DEADZONE:
                    direction = 0 if ly > 0 else 1
                    if not motors[jname]["running"] or motors[jname]["direction"] != direction:
                        start_motor(jname, direction)
                    new_joints.add(jname)
                for j in active_joints - new_joints: stop_motor(j)
                active_joints = new_joints

            # ════════════════════════════════════════════════
            # WORLD / TOOL MODE
            # ════════════════════════════════════════════════
            else:
                # Speed adjust via dpad always allowed
                if dpad_y != prev_dpad[1]:
                    for name in JOINT_ORDER:
                        if dpad_y ==  1: set_motor_speed(name, min(100, motors[name]["speed"] + 5))
                        if dpad_y == -1: set_motor_speed(name, max(1,   motors[name]["speed"] - 5))
                    log("All spd {}%".format(motors["J1"]["speed"]))

                # All actual motion gated on homed — no log spam, no move attempts
                if all_homed:
                    CART_DZ = 0.30   # slightly higher deadzone for cart axes
                    if abs(ly) > CART_DZ: cartesian_jog("X", -1 if ly > 0 else 1)
                    if abs(ry) > CART_DZ: cartesian_jog("Y", -1 if ry > 0 else 1)
                    if l3 and not prev_l3:  cartesian_jog("Z",  1)
                    if r3 and not prev_r3:  cartesian_jog("Z", -1)
                    if l1: cartesian_jog("Zf", -1)
                    if r1: cartesian_jog("Zf",  1)
                    if abs(lx) > CART_DZ: cartesian_jog("Rx",  1 if lx > 0 else -1)
                    if abs(rx) > CART_DZ: cartesian_jog("Ry", -1 if rx > 0 else  1)
                    if dpad_x != prev_dpad[0]:
                        if dpad_x ==  1: cartesian_jog("Rz",  1)
                        if dpad_x == -1: cartesian_jog("Rz", -1)
                active_joints = set()

            prev_l3 = l3; prev_r3 = r3
            prev_dpad = (dpad_x, dpad_y)
            time.sleep(0.05)

        except Exception as e:
            import traceback
            ps4_connected = False; joystick = None; agv.stop()
            log("Controller error: {}".format(e), alarm=True)
            log(traceback.format_exc().split('\n')[-2], alarm=True)
            time.sleep(1)

def start_ps4():
    global ps4_thread
    if ps4_thread is None or not ps4_thread.is_alive():
        ps4_thread=threading.Thread(target=ps4_loop,daemon=True)
        ps4_thread.start(); log("PS4 thread started")

# ─────────────────────────────────────────────
# SYSTEM INFO
# ─────────────────────────────────────────────
def get_cpu_temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f: return float(f.read())/1000.0
    except: return 0.0

def get_ip():
    try:
        r=subprocess.check_output(["hostname","-I"],timeout=2).decode().strip()
        return r.split()[0] if r else "N/A"
    except: return "N/A"

def get_memory():
    try:
        with open("/proc/meminfo") as f: lines=f.readlines()
        total=int([l for l in lines if "MemTotal" in l][0].split()[1])
        avail=int([l for l in lines if "MemAvailable" in l][0].split()[1])
        return (total-avail)//1024,total//1024
    except: return 0,0

def get_uptime():
    try:
        with open("/proc/uptime") as f: secs=float(f.read().split()[0])
        return "{}h {}m {}s".format(int(secs//3600),int((secs%3600)//60),int(secs%60))
    except: return "N/A"

def get_disk():
    try:
        r=subprocess.check_output(["df","-h","/"],timeout=2).decode().strip().split("\n")
        p=r[1].split(); return p[2],p[4]
    except: return "N/A","N/A"

# ─────────────────────────────────────────────
# THEME
# ─────────────────────────────────────────────
BG="#1c1c1c"; BG2="#242424"; BG3="#2c2c2c"; PANEL="#303030"; PANEL_SEL="#3a3010"
BORDER="#484848"; ORANGE="#ff6a00"; ORANGE_DK="#7a3200"; GREEN="#00b050"; GREEN_DK="#004d22"
RED="#cc2200"; RED_LT="#ff3300"; RED_DK="#4d0d00"; YELLOW="#e6b800"
WHITE="#e8e8e8"; GREY="#888888"; DARKGREY="#3c3c3c"; SCRN_BG="#0a0c0a"; SCRN_GRN="#00dd44"
AGV_BG="#0a0c14"; AGV_BLUE="#4a9eff"; AGV_BLUE_DK="#1a3a6a"
FT=("Courier New",9); FT_SM=("Courier New",8); FT_MED=("Courier New",10,"bold")
FT_LG=("Courier New",13,"bold"); FT_TINY=("Courier New",7); FT_TTL=("Courier New",12,"bold")

def mk_led(parent,bg,size=11):
    c=tk.Canvas(parent,width=size,height=size,bg=bg,highlightthickness=0)
    o=c.create_oval(1,1,size-1,size-1,fill="#333",outline="#555"); return c,o

def sl(c,o,on,col_on,col_off="#333"): c.itemconfig(o,fill=col_on if on else col_off)

# ─────────────────────────────────────────────
# ON-SCREEN KEYBOARD
# ─────────────────────────────────────────────
class OSK:
    def __init__(self,root):
        self.root=root; self.win=None; self.target=None; self.caps=False

    def attach(self,entry): self.target=entry

    def toggle(self):
        if self.win and self.win.winfo_exists(): self.win.destroy(); self.win=None
        else: self._open()

    def _open(self):
        self.win=tk.Toplevel(self.root)
        self.win.title("Keyboard"); self.win.configure(bg=BG)
        self.win.attributes("-topmost",True)
        self.win.geometry("800x300+60+420")
        self.win.resizable(False,False)
        self.win.protocol("WM_DELETE_WINDOW",lambda:(self.win.destroy(), setattr(self,"win",None)))
        self._draw_keys()

    def _draw_keys(self):
        for w in self.win.winfo_children(): w.destroy()
        rows=[
            ["`","1","2","3","4","5","6","7","8","9","0","-","=","BKSP"],
            ["TAB","q","w","e","r","t","y","u","i","o","p","[","]","\\"],
            ["CAPS","a","s","d","f","g","h","j","k","l",";","'","ENTER"],
            ["SHIFT","z","x","c","v","b","n","m",",",".","/","SHIFT"],
            ["SPACE","SPACE","SPACE","SPACE","SPACE","SPACE","CLR","←","→"],
        ]
        shift_map={"`":"~","1":"!","2":"@","3":"#","4":"$","5":"%","6":"^","7":"&",
                   "8":"*","9":"(","0":")","−":"_","=":"+","[":"{","]":"}","\\":"|",
                   ";":":","'":'"',",":"<",".":">","/":"?"}
        for row in rows:
            rf=tk.Frame(self.win,bg=BG); rf.pack(pady=2)
            for key in row:
                disp=key
                if self.caps and key.isalpha(): disp=key.upper()
                if self.caps and key in shift_map: disp=shift_map[key]
                w=5
                if key in ("BKSP","ENTER","CAPS","SHIFT","TAB"): w=7
                if key=="SPACE": w=10
                if key in ("CLR","←","→"): w=5
                tk.Button(rf,text=disp,font=FT_SM,bg=DARKGREY,fg=WHITE,width=w,
                          activebackground=ORANGE,activeforeground="black",
                          relief="raised",bd=2,cursor="hand2",
                          command=lambda k=key,d=disp: self._press(k,d)
                          ).pack(side="left",padx=1)

    def _press(self,key,disp):
        if key=="BKSP":
            if self.target:
                cur=self.target.get(); pos=self.target.index("insert")
                if pos>0: self.target.delete(pos-1,pos)
        elif key=="CLR":
            if self.target: self.target.delete(0,"end")
        elif key=="ENTER":
            if self.target: self.target.event_generate("<Return>")
        elif key in ("CAPS","SHIFT"):
            self.caps=not self.caps; self._draw_keys()
        elif key=="TAB":
            if self.target: self.target.insert("insert","    ")
        elif key=="SPACE":
            if self.target: self.target.insert("insert"," ")
        elif key=="←":
            if self.target:
                pos=self.target.index("insert")
                if pos>0: self.target.icursor(pos-1)
        elif key=="→":
            if self.target:
                pos=self.target.index("insert")
                self.target.icursor(pos+1)
        else:
            if self.target: self.target.insert("insert",disp)
        if self.win and self.win.winfo_exists(): self.win.lift(); self.win.attributes("-topmost",True)

# ─────────────────────────────────────────────
# AGV WEB PANEL  (embedded in main window)
# ─────────────────────────────────────────────
# ─────────────────────────────────────────────
# AGV PANEL — native Tkinter canvas joystick
# No browser embedding. Sends UDP directly.
# Spin fix: agv.set() is gated by control_mode
# and zeroed on every mode entry/exit.
# ─────────────────────────────────────────────
class AGVPanel:
    def __init__(self, root):
        self.root      = root
        self.win       = None
        self._canvas   = None
        self._nub      = None
        self._dragging = False
        self._last_send = 0.0
        self._cx = self._cy = self._radius = self._nub_r = 0
        self.telem     = {}
        self.conn_pill = None
        self.mode_lbl  = None
        self.src_lbl   = None

    def toggle(self):
        if self.win and self.win.winfo_exists():
            self.win.lift()
        else:
            self._open()

    def _open(self):
        self.win = tk.Toplevel(self.root)
        self.win.title("EVA-AGV  |  Pendant Control")
        self.win.configure(bg=AGV_BG)
        self.win.geometry("460x620+820+20")
        self.win.resizable(False, False)
        self.win.protocol("WM_DELETE_WINDOW",
                          lambda: (agv.stop(),
                                   self.win.destroy(),
                                   setattr(self, "win", None)))
        self._build()
        self._poll_connection()
        self._update_telem()

    def _build(self):
        # ── Title bar
        tb = tk.Frame(self.win, bg="#0d1117", height=40)
        tb.pack(fill="x"); tb.pack_propagate(False)
        badge = tk.Frame(tb, bg=AGV_BLUE, width=110)
        badge.pack(side="left", fill="y"); badge.pack_propagate(False)
        tk.Label(badge, text="EVA-AGV",
                 font=("Courier New",10,"bold"),
                 fg="white", bg=AGV_BLUE).pack(expand=True)
        tk.Label(tb, text="LOCOMOTION CONTROL",
                 font=FT_SM, fg=GREY, bg="#0d1117").pack(side="left", padx=10)
        self.conn_pill = tk.Label(tb, text=" CONNECTING... ",
                                  font=FT_SM, fg="black", bg=YELLOW)
        self.conn_pill.pack(side="right", padx=8, pady=7)

        # ── Status strip
        ss = tk.Frame(self.win, bg="#0d1117", height=24)
        ss.pack(fill="x"); ss.pack_propagate(False)
        tk.Label(ss, text="NET: RoboControl",
                 font=FT_TINY, fg=GREY, bg="#0d1117").pack(side="left", padx=8)
        tk.Label(ss, text="  IP: {}".format(ESP32_IP),
                 font=FT_TINY, fg=AGV_BLUE, bg="#0d1117").pack(side="left")
        self.src_lbl = tk.Label(ss, text="  INPUT: ---",
                                font=FT_TINY, fg=GREY, bg="#0d1117")
        self.src_lbl.pack(side="left")
        tk.Frame(self.win, bg=BORDER, height=1).pack(fill="x")

        # ── Mode banner
        mf = tk.Frame(self.win, bg=AGV_BG, height=28)
        mf.pack(fill="x"); mf.pack_propagate(False)
        self.mode_lbl = tk.Label(mf,
            text="STANDBY  —  Press Circle (○) to activate AGV",
            font=FT_SM, fg=GREY, bg=AGV_BG)
        self.mode_lbl.pack(expand=True)
        tk.Frame(self.win, bg=BORDER, height=1).pack(fill="x")

        # ── Direction labels + canvas
        tk.Label(self.win, text="▲  FORWARD",
                 font=FT_SM, fg="#3a4a5c", bg=AGV_BG).pack(pady=(6,0))

        mid = tk.Frame(self.win, bg=AGV_BG); mid.pack()
        tk.Label(mid, text="◀  LEFT",
                 font=FT_SM, fg="#3a4a5c", bg=AGV_BG).pack(side="left", padx=6)

        JOY=300; RAD=120; NUBR=34
        self._cx = self._cy = JOY//2
        self._radius = RAD; self._nub_r = NUBR
        cx = cy = JOY//2

        c = tk.Canvas(mid, width=JOY, height=JOY,
                      bg="#0b0e14", highlightthickness=2,
                      highlightbackground=AGV_BLUE_DK, cursor="hand2")
        c.pack(side="left")
        self._canvas = c

        c.create_oval(cx-RAD, cy-RAD, cx+RAD, cy+RAD,
                      outline=AGV_BLUE_DK, width=2)
        hr = RAD//2
        c.create_oval(cx-hr, cy-hr, cx+hr, cy+hr,
                      outline="#1a2230", width=1, dash=(3,6))
        c.create_line(cx-RAD, cy, cx+RAD, cy, fill="#1a2230")
        c.create_line(cx, cy-RAD, cx, cy+RAD, fill="#1a2230")
        c.create_oval(cx-3, cy-3, cx+3, cy+3, fill="#1a2230", outline="")
        self._nub = c.create_oval(cx-NUBR, cy-NUBR, cx+NUBR, cy+NUBR,
                                  fill=AGV_BLUE, outline=AGV_BLUE_DK, width=3)

        c.bind("<ButtonPress-1>",   self._joy_press)
        c.bind("<B1-Motion>",       self._joy_move)
        c.bind("<ButtonRelease-1>", self._joy_release)
        # Prevent the main window's bind_all scroll handlers stealing events
        c.bind("<Button-4>", lambda e: "break")
        c.bind("<Button-5>", lambda e: "break")
        c.bind("<MouseWheel>", lambda e: "break")

        tk.Label(mid, text="RIGHT  ▶",
                 font=FT_SM, fg="#3a4a5c", bg=AGV_BG).pack(side="left", padx=6)

        tk.Label(self.win, text="▼  REVERSE",
                 font=FT_SM, fg="#3a4a5c", bg=AGV_BG).pack(pady=(0,6))
        tk.Frame(self.win, bg=BORDER, height=1).pack(fill="x")

        # ── Stop button
        tk.Button(self.win, text="■  STOP AGV",
                  font=("Courier New",11,"bold"),
                  bg="#3a0a0a", fg=RED_LT,
                  activebackground=RED_LT, activeforeground="white",
                  relief="raised", bd=3, cursor="hand2",
                  command=self._stop_agv).pack(fill="x", padx=12, pady=6)
        tk.Frame(self.win, bg=BORDER, height=1).pack(fill="x")

        # ── Telemetry strip
        tstrip = tk.Frame(self.win, bg="#0d1117", height=52)
        tstrip.pack(fill="x", side="bottom"); tstrip.pack_propagate(False)
        for key, label in [("mode","MODE"),("t","THROTTLE"),
                           ("r","STEERING"),("left","L-MOTOR"),("right","R-MOTOR")]:
            cell = tk.Frame(tstrip, bg="#0d1117")
            cell.pack(side="left", expand=True, fill="both")
            tk.Frame(cell, bg=BORDER, width=1).pack(side="left", fill="y")
            inner = tk.Frame(cell, bg="#0d1117"); inner.pack(expand=True)
            v = tk.Label(inner, text="--",
                         font=("Courier New",11,"bold"), fg=AGV_BLUE, bg="#0d1117")
            v.pack()
            tk.Label(inner, text=label,
                     font=FT_TINY, fg=GREY, bg="#0d1117").pack()
            self.telem[key] = v

    def _joy_press(self, event):
        self._dragging = True
        self._last_send = 0.0
        self._joy_move(event)

    def _joy_move(self, event):
        if not self._dragging: return
        dx = event.x - self._cx
        dy = event.y - self._cy
        dist = math.sqrt(dx*dx + dy*dy)
        if dist > self._radius:
            dx = dx / dist * self._radius
            dy = dy / dist * self._radius

        # Move nub visually on every event — smooth feel
        nr = self._nub_r; cx = self._cx; cy = self._cy
        self._canvas.coords(self._nub,
                            cx+dx-nr, cy+dy-nr, cx+dx+nr, cy+dy+nr)

        # Throttle agv.set() to ~50Hz so we don't flood the send loop
        now = time.time()
        if now - self._last_send < 0.02:
            return
        self._last_send = now

        # Pendant deadzone — ignore tiny wobbles near center
        PDEAD = 0.08
        t = -(dy / self._radius)
        r =   dx / self._radius
        t = t if abs(t) > PDEAD else 0.0
        r = r if abs(r) > PDEAD else 0.0

        if control_mode == "AGV":
            agv.set(t, r)

    def _joy_release(self, event):
        self._dragging = False
        nr = self._nub_r; cx = self._cx; cy = self._cy
        self._canvas.coords(self._nub, cx-nr, cy-nr, cx+nr, cy+nr)
        agv.stop()

    def _stop_agv(self):
        agv.stop()
        log("AGV STOP")

    def _update_telem(self):
        if not (self.win and self.win.winfo_exists()): return
        st    = agv.status_json
        t_val = agv.translate
        r_val = agv.rotate
        is_agv = (control_mode == "AGV")

        # Mirror PS4 values onto nub — but only when not dragging
        # (dragging owns the nub position; mirroring while dragging causes glitch)
        if self._canvas and self._nub and self._radius > 0 and not self._dragging:
            dx =  r_val * self._radius
            dy = -t_val * self._radius
            dist = math.sqrt(dx*dx + dy*dy)
            if dist > self._radius:
                dx = dx / dist * self._radius
                dy = dy / dist * self._radius
            nr = self._nub_r; cx = self._cx; cy = self._cy
            self._canvas.coords(self._nub,
                                cx+dx-nr, cy+dy-nr,
                                cx+dx+nr, cy+dy+nr)
        if is_agv:
            self.mode_lbl.config(
                text="● AGV ACTIVE  —  Joystick or PS4 to drive", fg=AGV_BLUE)
        else:
            self.mode_lbl.config(
                text="STANDBY  —  Press Circle (○) to activate AGV", fg=GREY)
        src = st.get("source","---")
        self.src_lbl.config(
            text="  INPUT: {}".format(src.upper()),
            fg=AGV_BLUE if src in ("udp","web") else GREY)
        self.telem["mode"].config(
            text="AGV" if is_agv else "ARM",
            fg=AGV_BLUE if is_agv else GREY)
        self.telem["t"].config(
            text="{:+.2f}".format(t_val),
            fg=AGV_BLUE if abs(t_val)>0.05 else GREY)
        self.telem["r"].config(
            text="{:+.2f}".format(r_val),
            fg=AGV_BLUE if abs(r_val)>0.05 else GREY)
        self.telem["left"].config(
            text=str(st.get("left","--")),
            fg=AGV_BLUE if "left" in st else GREY)
        self.telem["right"].config(
            text=str(st.get("right","--")),
            fg=AGV_BLUE if "right" in st else GREY)
        self.win.after(100, self._update_telem)

    def _poll_connection(self):
        if not (self.win and self.win.winfo_exists()): return
        try:
            import urllib.request
            urllib.request.urlopen(
                "http://{}/ping".format(ESP32_IP), timeout=0.4)
            self.conn_pill.config(text=" CONNECTED ", bg=GREEN,  fg="black")
        except Exception:
            self.conn_pill.config(text=" OFFLINE ",   bg=RED,    fg="white")
        self.win.after(1000, self._poll_connection)


class ProgramEditor:
    def __init__(self,root,osk):
        self.root=root; self.osk=osk; self.win=None
        self.program=current_program
        self.selected_idx=None
        self.playback_spd=tk.IntVar(value=50)
        self.step_mode=tk.BooleanVar(value=False)

    def toggle(self):
        if self.win and self.win.winfo_exists(): self.win.lift(); self.win.attributes("-topmost",False)
        else: self._open()

    def _open(self):
        self.win=tk.Toplevel(self.root)
        self.win.title("PROGRAM EDITOR  -  {}".format(self.program.name))
        self.win.configure(bg=BG)
        self.win.geometry("1060x680+20+20")
        self.win.resizable(True,True)
        self.win.protocol("WM_DELETE_WINDOW",lambda:(self.win.destroy(),setattr(self,"win",None)))
        self._build()
        self.refresh()

    def _build(self):
        tb=tk.Frame(self.win,bg="#111111",height=36); tb.pack(fill="x"); tb.pack_propagate(False)
        badge=tk.Frame(tb,bg=ORANGE,width=140); badge.pack(side="left",fill="y"); badge.pack_propagate(False)
        tk.Label(badge,text="PROGRAM EDITOR",font=("Courier New",8,"bold"),fg="white",bg=ORANGE).pack(expand=True)
        self.title_lbl=tk.Label(tb,text=self.program.name,font=FT_TTL,fg=ORANGE,bg="#111111")
        self.title_lbl.pack(side="left",padx=10)
        self.modified_lbl=tk.Label(tb,text="",font=FT_SM,fg=YELLOW,bg="#111111")
        self.modified_lbl.pack(side="left",padx=4)
        toolbar=tk.Frame(self.win,bg=BG3,height=34); toolbar.pack(fill="x"); toolbar.pack_propagate(False)
        btns=[
            ("NEW PROG",  self._new_program,    DARKGREY, ORANGE),
            ("LOAD",      self._load_program,    DARKGREY, ORANGE),
            ("SAVE",      self._save_program,    DARKGREY, GREEN),
            ("SAVE AS",   self._save_as,         DARKGREY, GREEN),
            ("REC POINT", self._record_point,    "#1a3a0a", GREEN),
            ("DEL POINT", self._delete_point,    "#3a0a0a", RED_LT),
            ("UP",        self._move_up,         DARKGREY, WHITE),
            ("DOWN",      self._move_down,       DARKGREY, WHITE),
            ("EDIT PT",   self._edit_point,      DARKGREY, YELLOW),
            ("VARIABLES", self._open_variables,  DARKGREY, YELLOW),
        ]
        for txt,cmd,bg,fg in btns:
            tk.Button(toolbar,text=txt,font=FT_TINY,bg=bg,fg=fg,
                      activebackground=fg,activeforeground="black",
                      relief="raised",bd=1,cursor="hand2",padx=4,
                      command=cmd).pack(side="left",fill="y",padx=1,pady=3)
        body=tk.Frame(self.win,bg=BG); body.pack(fill="both",expand=True,padx=4,pady=3)
        lf=tk.Frame(body,bg=BG2); lf.pack(side="left",fill="both",expand=True,padx=(0,3))
        hdr=tk.Frame(lf,bg="#181818",height=22); hdr.pack(fill="x"); hdr.pack_propagate(False)
        for txt,w in [("IDX",4),("TYPE",5),("POINT",7),("ANGLES (J1-J6)",40),("SPD",5),("CNT",5),("COMMENT",20)]:
            tk.Label(hdr,text=txt,font=FT_TINY,fg=GREY,bg="#181818",width=w,anchor="w").pack(side="left",padx=3)
        list_frame=tk.Frame(lf,bg=BG2); list_frame.pack(fill="both",expand=True)
        self.listbox=tk.Listbox(list_frame,bg=SCRN_BG,fg=SCRN_GRN,
                                 font=("Courier New",9),selectbackground=PANEL_SEL,
                                 selectforeground=ORANGE,activestyle="none",
                                 relief="flat",bd=0)
        lsb=ttk.Scrollbar(list_frame,orient="vertical",command=self.listbox.yview)
        self.listbox.configure(yscrollcommand=lsb.set)
        lsb.pack(side="right",fill="y"); self.listbox.pack(fill="both",expand=True)
        self.listbox.bind("<<ListboxSelect>>",self._on_select)
        self.listbox.bind("<Double-Button-1>",lambda e: self._edit_point())
        rp=tk.Frame(body,bg=BG2,width=280); rp.pack(side="right",fill="y"); rp.pack_propagate(False)
        tk.Label(rp,text="PLAYBACK",font=FT_MED,fg=ORANGE,bg=BG2).pack(anchor="w",padx=8,pady=(8,2))
        pb_btns=tk.Frame(rp,bg=BG2); pb_btns.pack(fill="x",padx=8,pady=2)
        tk.Button(pb_btns,text="▶ RUN FWD",font=FT_SM,bg="#0a2a0a",fg=GREEN,
                  activebackground=GREEN,activeforeground="black",relief="raised",bd=2,cursor="hand2",
                  command=lambda:start_playback(self.program,self.playback_spd.get(),forward=True)
                  ).pack(side="left",padx=2)
        tk.Button(pb_btns,text="◀ RUN REV",font=FT_SM,bg="#0a2a0a",fg=GREEN,
                  activebackground=GREEN,activeforeground="black",relief="raised",bd=2,cursor="hand2",
                  command=lambda:start_playback(self.program,self.playback_spd.get(),forward=False)
                  ).pack(side="left",padx=2)
        pb_btns2=tk.Frame(rp,bg=BG2); pb_btns2.pack(fill="x",padx=8,pady=2)
        tk.Button(pb_btns2,text="▶| STEP",font=FT_SM,bg=DARKGREY,fg=YELLOW,
                  activebackground=YELLOW,activeforeground="black",relief="raised",bd=2,cursor="hand2",
                  command=lambda:step_playback(self.program,self.playback_spd.get())
                  ).pack(side="left",padx=2)
        tk.Button(pb_btns2,text="■ STOP",font=FT_SM,bg="#3a0a0a",fg=RED_LT,
                  activebackground=RED_LT,activeforeground="white",relief="raised",bd=2,cursor="hand2",
                  command=stop_playback).pack(side="left",padx=2)
        step_row=tk.Frame(rp,bg=BG2); step_row.pack(fill="x",padx=8,pady=2)
        self.step_btn=tk.Button(step_row,text="STEP MODE: OFF",font=FT_SM,
                                bg=DARKGREY,fg=GREY,activebackground=YELLOW,activeforeground="black",
                                relief="raised",bd=2,cursor="hand2",command=self._toggle_step)
        self.step_btn.pack(fill="x")
        tk.Label(rp,text="PLAYBACK SPEED %",font=FT_SM,fg=GREY,bg=BG2).pack(anchor="w",padx=8,pady=(6,0))
        tk.Scale(rp,from_=1,to=100,orient="horizontal",variable=self.playback_spd,
                 length=240,bg=BG2,fg=WHITE,troughcolor=BG,highlightthickness=0,
                 bd=0,activebackground=ORANGE,showvalue=True).pack(padx=8)
        tk.Frame(rp,bg=BORDER,height=1).pack(fill="x",padx=8,pady=6)
        tk.Label(rp,text="POINT DETAIL",font=FT_MED,fg=ORANGE,bg=BG2).pack(anchor="w",padx=8,pady=(0,2))
        self.detail_text=tk.Text(rp,bg=SCRN_BG,fg=SCRN_GRN,font=("Courier New",8),
                                  relief="flat",state="disabled",height=18,wrap="word")
        self.detail_text.pack(fill="both",expand=True,padx=8,pady=2)
        tk.Frame(rp,bg=BORDER,height=1).pack(fill="x",padx=8,pady=4)
        tk.Label(rp,text="QUICK RECORD",font=FT_SM,fg=GREY,bg=BG2).pack(anchor="w",padx=8)
        qr=tk.Frame(rp,bg=BG2); qr.pack(fill="x",padx=8,pady=2)
        self.qr_type=tk.StringVar(value="J")
        tk.Radiobutton(qr,text="Joint (J)",variable=self.qr_type,value="J",
                       font=FT_SM,fg=WHITE,bg=BG2,selectcolor=BG,activebackground=BG2).pack(side="left")
        tk.Radiobutton(qr,text="Linear (L)",variable=self.qr_type,value="L",
                       font=FT_SM,fg=WHITE,bg=BG2,selectcolor=BG,activebackground=BG2).pack(side="left",padx=6)
        tk.Button(rp,text="● RECORD CURRENT POSITION",font=FT_SM,
                  bg="#1a3a0a",fg=GREEN,activebackground=GREEN,activeforeground="black",
                  relief="raised",bd=2,cursor="hand2",command=self._record_point).pack(fill="x",padx=8,pady=3)
        self.status_lbl=tk.Label(self.win,text="Ready",font=FT_SM,fg=GREY,bg="#111111",anchor="w")
        self.status_lbl.pack(fill="x",padx=8,pady=2,side="bottom")

    def _toggle_step(self):
        global playback_step_mode
        playback_step_mode=not playback_step_mode
        if playback_step_mode:
            self.step_btn.config(text="STEP MODE: ON",bg="#2a2a0a",fg=YELLOW)
        else:
            self.step_btn.config(text="STEP MODE: OFF",bg=DARKGREY,fg=GREY)

    def refresh(self):
        if not (self.win and self.win.winfo_exists()): return
        self.listbox.delete(0,"end")
        for pt in self.program.points:
            angs=" ".join("{:+.1f}".format(a) for a in pt["angles"])
            line="  {:>3}]  {}  {}:  {}  {}%  cnt:{}  {}".format(
                pt["index"],pt["move_type"],pt["label"],angs,
                pt["speed_pct"],pt["cnt"],pt["comment"])
            self.listbox.insert("end",line)
        if self.selected_idx is not None and self.selected_idx<len(self.program.points):
            self.listbox.selection_set(self.selected_idx)
            self.listbox.see(self.selected_idx)
        self.title_lbl.config(text=self.program.name)
        self.modified_lbl.config(text="[MODIFIED]" if self.program.modified else "")
        self.status_lbl.config(text="{} points  |  {}".format(
            len(self.program.points),
            self.program.filepath or "unsaved"))

    def _on_select(self,event=None):
        sel=self.listbox.curselection()
        if not sel: return
        self.selected_idx=sel[0]
        self._show_detail(self.program.points[self.selected_idx])

    def _show_detail(self,pt):
        self.detail_text.config(state="normal")
        self.detail_text.delete("1.0","end")
        tcp=pt.get("tcp",[0,0,0])
        lines=[
            "POINT:    {}".format(pt["label"]),
            "TYPE:     {}".format("Joint" if pt["move_type"]=="J" else "Linear"),
            "SPEED:    {}%".format(pt["speed_pct"]),
            "CNT:      {}".format(pt["cnt"]),
            "COMMENT:  {}".format(pt["comment"]),"",
            "JOINT ANGLES:",
            "  J1: {:+.3f} deg".format(pt["angles"][0]),
            "  J2: {:+.3f} deg".format(pt["angles"][1]),
            "  J3: {:+.3f} deg".format(pt["angles"][2]),
            "  J4: {:+.3f} deg".format(pt["angles"][3]),
            "  J5: {:+.3f} deg".format(pt["angles"][4]),
            "  J6: {:+.3f} deg".format(pt["angles"][5]),"",
            "CARTESIAN (FK):",
            "  X: {:+.3f} mm".format(tcp[0]),
            "  Y: {:+.3f} mm".format(tcp[1]),
            "  Z: {:+.3f} mm".format(tcp[2]),
        ]
        self.detail_text.insert("end","\n".join(lines))
        self.detail_text.config(state="disabled")

    def _record_point(self):
        pt=self.program.add_point(move_type=self.qr_type.get())
        self.selected_idx=len(self.program.points)-1
        self.refresh(); self._show_detail(pt)

    def _delete_point(self):
        if self.selected_idx is None: return
        if messagebox.askyesno("Delete","Delete P[{}]?".format(self.selected_idx+1),parent=self.win):
            self.program.delete_point(self.selected_idx)
            self.selected_idx=max(0,self.selected_idx-1) if self.program.points else None
            self.refresh()

    def _move_up(self):
        if self.selected_idx is None or self.selected_idx==0: return
        self.program.move_point_up(self.selected_idx)
        self.selected_idx-=1; self.refresh()

    def _move_down(self):
        if self.selected_idx is None or self.selected_idx>=len(self.program.points)-1: return
        self.program.move_point_down(self.selected_idx)
        self.selected_idx+=1; self.refresh()

    def _edit_point(self):
        if self.selected_idx is None: return
        pt=self.program.points[self.selected_idx]
        self._open_point_editor(pt)

    def _open_point_editor(self,pt):
        ew=tk.Toplevel(self.win)
        ew.title("Edit {}".format(pt["label"]))
        ew.configure(bg=BG); ew.attributes("-topmost",True)
        ew.geometry("520x580+200+80"); ew.resizable(False,False)
        tb=tk.Frame(ew,bg=ORANGE,height=30); tb.pack(fill="x"); tb.pack_propagate(False)
        tk.Label(tb,text="  EDIT POINT  -  {}".format(pt["label"]),
                 font=FT_MED,fg="white",bg=ORANGE).pack(side="left",padx=6,pady=4)
        fields=tk.Frame(ew,bg=BG2); fields.pack(fill="both",expand=True,padx=10,pady=8)
        entries={}
        def field(lbl,val,row,readonly=False):
            tk.Label(fields,text=lbl,font=FT_SM,fg=GREY,bg=BG2,width=20,anchor="w"
                     ).grid(row=row,column=0,padx=6,pady=4,sticky="w")
            e=tk.Entry(fields,font=FT_MED,width=18,bg=SCRN_BG,fg=SCRN_GRN,
                       insertbackground=SCRN_GRN,relief="sunken",bd=2)
            e.insert(0,str(val))
            if readonly: e.config(state="readonly",fg=GREY)
            e.grid(row=row,column=1,padx=6,pady=4,sticky="w")
            e.bind("<FocusIn>",lambda ev,ew2=e: self.osk.attach(ew2))
            entries[lbl]=e; return e
        tk.Label(fields,text="Move Type",font=FT_SM,fg=GREY,bg=BG2,width=20,anchor="w"
                 ).grid(row=0,column=0,padx=6,pady=4,sticky="w")
        mt_var=tk.StringVar(value=pt["move_type"])
        mf=tk.Frame(fields,bg=BG2); mf.grid(row=0,column=1,padx=6,pady=4,sticky="w")
        tk.Radiobutton(mf,text="J - Joint",variable=mt_var,value="J",
                       font=FT_SM,fg=WHITE,bg=BG2,selectcolor=BG,activebackground=BG2).pack(side="left")
        tk.Radiobutton(mf,text="L - Linear",variable=mt_var,value="L",
                       font=FT_SM,fg=WHITE,bg=BG2,selectcolor=BG,activebackground=BG2).pack(side="left",padx=8)
        field("Speed %",pt["speed_pct"],1)
        field("CNT (0-100)",pt["cnt"],2)
        field("Comment",pt["comment"],3)
        tk.Label(fields,text="─── Joint Angles ───",font=FT_SM,fg=ORANGE,bg=BG2
                 ).grid(row=4,column=0,columnspan=2,pady=(10,2))
        for i,name in enumerate(JOINT_ORDER):
            field("{} angle (deg)".format(name),"{:.4f}".format(pt["angles"][i]),5+i)
        bf=tk.Frame(ew,bg=BG2); bf.pack(fill="x",padx=10,pady=8)
        def apply_changes():
            try:
                pt["move_type"]=mt_var.get(); pt["speed_pct"]=int(entries["Speed %"].get())
                pt["cnt"]=int(entries["CNT (0-100)"].get()); pt["comment"]=entries["Comment"].get()
                for i,name in enumerate(JOINT_ORDER):
                    pt["angles"][i]=float(entries["{} angle (deg)".format(name)].get())
                pt["tcp"]=forward_kinematics(pt["angles"])
                self.program.modified=True; self.refresh(); self._show_detail(pt)
                log("P[{}] edited".format(pt["index"])); ew.destroy()
            except Exception as ex:
                messagebox.showerror("Error","Invalid value: {}".format(ex),parent=ew)
        def move_robot_here():
            if not all_homed:
                messagebox.showwarning("Not Homed","Home the robot first.",parent=ew); return
            move_all_to_angles(pt["angles"],wait=False)
            log("Moving to P[{}] for verification".format(pt["index"]))
        def update_from_current():
            angles=[motors[n]["angle_deg"] for n in JOINT_ORDER]
            for i,name in enumerate(JOINT_ORDER):
                e=entries["{} angle (deg)".format(name)]
                e.config(state="normal"); e.delete(0,"end")
                e.insert(0,"{:.4f}".format(angles[i]))
            log("P[{}] angles updated from current position".format(pt["index"]))
        tk.Button(bf,text="APPLY",font=FT_MED,bg="#0a2a0a",fg=GREEN,
                  activebackground=GREEN,activeforeground="black",
                  relief="raised",bd=2,cursor="hand2",command=apply_changes).pack(side="left",padx=4)
        tk.Button(bf,text="MOVE HERE",font=FT_MED,bg=DARKGREY,fg=ORANGE,
                  activebackground=ORANGE,activeforeground="black",
                  relief="raised",bd=2,cursor="hand2",command=move_robot_here).pack(side="left",padx=4)
        tk.Button(bf,text="UPDATE FROM CURRENT POS",font=FT_SM,bg=DARKGREY,fg=YELLOW,
                  activebackground=YELLOW,activeforeground="black",
                  relief="raised",bd=2,cursor="hand2",command=update_from_current).pack(side="left",padx=4)
        tk.Button(bf,text="CANCEL",font=FT_SM,bg=DARKGREY,fg=GREY,
                  activebackground=GREY,activeforeground="black",
                  relief="raised",bd=2,cursor="hand2",command=ew.destroy).pack(side="right",padx=4)

    def _open_variables(self):
        vw=tk.Toplevel(self.win)
        vw.title("Variables  -  {}".format(self.program.name))
        vw.configure(bg=BG); vw.attributes("-topmost",True)
        vw.geometry("500x500+220+100"); vw.resizable(True,True)
        tb=tk.Frame(vw,bg=ORANGE,height=30); tb.pack(fill="x"); tb.pack_propagate(False)
        tk.Label(tb,text="  VARIABLE REGISTERS",font=FT_MED,fg="white",bg=ORANGE).pack(side="left",padx=6,pady=4)
        lf=tk.Frame(vw,bg=BG2); lf.pack(fill="both",expand=True,padx=6,pady=6)
        hdr=tk.Frame(lf,bg="#181818",height=22); hdr.pack(fill="x"); hdr.pack_propagate(False)
        for txt,w in [("NAME",20),("VALUE",20),("COMMENT",30)]:
            tk.Label(hdr,text=txt,font=FT_TINY,fg=GREY,bg="#181818",width=w,anchor="w").pack(side="left",padx=4)
        self.var_listbox=tk.Listbox(lf,bg=SCRN_BG,fg=SCRN_GRN,font=("Courier New",9),
                                     selectbackground=PANEL_SEL,selectforeground=ORANGE,
                                     activestyle="none",relief="flat",bd=0)
        vsb=ttk.Scrollbar(lf,orient="vertical",command=self.var_listbox.yview)
        self.var_listbox.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right",fill="y"); self.var_listbox.pack(fill="both",expand=True)
        def refresh_vars():
            self.var_listbox.delete(0,"end")
            for k,v in self.program.variables.items():
                val=v.get("value","") if isinstance(v,dict) else v
                cmt=v.get("comment","") if isinstance(v,dict) else ""
                self.var_listbox.insert("end","  {:20s}  {:20s}  {}".format(k,str(val),cmt))
        refresh_vars()
        af=tk.Frame(vw,bg=BG3,pady=6); af.pack(fill="x",padx=6)
        tk.Label(af,text="NAME",font=FT_SM,fg=GREY,bg=BG3).grid(row=0,column=0,padx=6)
        tk.Label(af,text="VALUE",font=FT_SM,fg=GREY,bg=BG3).grid(row=0,column=1,padx=6)
        tk.Label(af,text="COMMENT",font=FT_SM,fg=GREY,bg=BG3).grid(row=0,column=2,padx=6)
        name_e=tk.Entry(af,font=FT_SM,width=16,bg=SCRN_BG,fg=SCRN_GRN,insertbackground=SCRN_GRN,relief="sunken",bd=1)
        val_e =tk.Entry(af,font=FT_SM,width=16,bg=SCRN_BG,fg=SCRN_GRN,insertbackground=SCRN_GRN,relief="sunken",bd=1)
        cmt_e =tk.Entry(af,font=FT_SM,width=20,bg=SCRN_BG,fg=SCRN_GRN,insertbackground=SCRN_GRN,relief="sunken",bd=1)
        name_e.grid(row=1,column=0,padx=6,pady=2); val_e.grid(row=1,column=1,padx=6,pady=2); cmt_e.grid(row=1,column=2,padx=6,pady=2)
        for e in (name_e,val_e,cmt_e):
            e.bind("<FocusIn>",lambda ev,ew2=e: self.osk.attach(ew2))
        def add_var():
            n=name_e.get().strip(); v=val_e.get().strip(); c=cmt_e.get().strip()
            if not n: return
            self.program.variables[n]={"value":v,"comment":c}
            self.program.modified=True; refresh_vars()
            log("Variable {} = {}".format(n,v))
        def del_var():
            sel=self.var_listbox.curselection()
            if not sel: return
            keys=list(self.program.variables.keys())
            if sel[0]<len(keys):
                del self.program.variables[keys[sel[0]]]
                self.program.modified=True; refresh_vars()
        bf=tk.Frame(vw,bg=BG2); bf.pack(fill="x",padx=6,pady=4)
        tk.Button(bf,text="ADD / UPDATE",font=FT_SM,bg="#0a2a0a",fg=GREEN,
                  activebackground=GREEN,activeforeground="black",
                  relief="raised",bd=2,cursor="hand2",command=add_var).pack(side="left",padx=4)
        tk.Button(bf,text="DELETE",font=FT_SM,bg="#3a0a0a",fg=RED_LT,
                  activebackground=RED_LT,activeforeground="white",
                  relief="raised",bd=2,cursor="hand2",command=del_var).pack(side="left",padx=4)
        tk.Button(bf,text="STORE CURRENT POS AS VAR",font=FT_SM,bg=DARKGREY,fg=YELLOW,
                  activebackground=YELLOW,activeforeground="black",
                  relief="raised",bd=2,cursor="hand2",
                  command=lambda: self._store_pos_var(name_e,cmt_e,refresh_vars)).pack(side="left",padx=4)

    def _store_pos_var(self,name_e,cmt_e,refresh_fn):
        n=name_e.get().strip()
        if not n: n="PR[{}]".format(len(self.program.variables)+1)
        angles=[motors[j]["angle_deg"] for j in JOINT_ORDER]
        tcp=forward_kinematics(angles)
        self.program.variables[n]={
            "value": {"angles":angles,"tcp":list(tcp)},
            "comment": cmt_e.get().strip() or "Position register"
        }
        self.program.modified=True; refresh_fn()
        log("Position register {} stored".format(n))

    def _new_program(self):
        global current_program
        if self.program.modified:
            if not messagebox.askyesno("Unsaved Changes","Discard changes and create new program?",parent=self.win): return
        current_program=Program(); self.program=current_program
        self.selected_idx=None; self.refresh()

    def _save_program(self):
        if not self.program.filepath: self._save_as()
        else: self.program.save(); self.refresh()

    def _save_as(self):
        fp=filedialog.asksaveasfilename(
            parent=self.win,title="Save Program",defaultextension=".json",
            filetypes=[("Robot Program","*.json"),("All Files","*.*")],
            initialdir=os.path.expanduser("~"))
        if fp:
            self.program.name=os.path.splitext(os.path.basename(fp))[0]
            self.program.save(fp); self.refresh()

    def _load_program(self):
        global current_program
        if self.program.modified:
            if not messagebox.askyesno("Unsaved Changes","Discard changes and load?",parent=self.win): return
        fp=filedialog.askopenfilename(
            parent=self.win,title="Load Program",
            filetypes=[("Robot Program","*.json"),("All Files","*.*")],
            initialdir=os.path.expanduser("~"))
        if fp:
            current_program=Program(); current_program.load(fp)
            self.program=current_program; self.selected_idx=None; self.refresh()

# ─────────────────────────────────────────────
# MENU WINDOW
# ─────────────────────────────────────────────
class MenuWindow:
    def __init__(self,root,osk):
        self.root=root; self.osk=osk; self.win=None

    def toggle(self):
        if self.win and self.win.winfo_exists(): self.win.lift(); self.win.attributes("-topmost",False)
        else: self._open()

    def _open(self):
        self.win=tk.Toplevel(self.root)
        self.win.title("SYSTEM MENU"); self.win.configure(bg=BG)
        self.win.geometry("860x620+80+40")
        self.win.protocol("WM_DELETE_WINDOW",lambda:(self.win.destroy(),setattr(self,"win",None)))
        tb=tk.Frame(self.win,bg="#111111",height=36); tb.pack(fill="x"); tb.pack_propagate(False)
        badge=tk.Frame(tb,bg=ORANGE,width=120); badge.pack(side="left",fill="y"); badge.pack_propagate(False)
        tk.Label(badge,text="SYSTEM MENU",font=("Courier New",8,"bold"),fg="white",bg=ORANGE).pack(expand=True)
        tk.Button(tb,text="  CLOSE  ",font=FT_SM,bg=RED,fg="white",
                  activebackground=RED_LT,relief="flat",cursor="hand2",
                  command=self.win.destroy).pack(side="right",padx=8,pady=4)
        nb=ttk.Notebook(self.win); nb.pack(fill="both",expand=True,padx=6,pady=6)
        self._tab_sysinfo(nb); self._tab_motorconfig(nb)
        self._tab_alarms(nb); self._tab_network(nb); self._tab_kinematics(nb)

    def _mk_tab(self,nb,title):
        f=tk.Frame(nb,bg=BG2); nb.add(f,text="  {}  ".format(title)); return f

    def _row(self,parent,label,value,row_idx):
        bg=PANEL if row_idx%2==0 else BG3
        f=tk.Frame(parent,bg=bg); f.pack(fill="x",pady=1)
        tk.Label(f,text=label,font=FT_SM,fg=GREY,bg=bg,width=28,anchor="w").pack(side="left",padx=8,pady=4)
        lbl=tk.Label(f,text=value,font=FT_MED,fg=SCRN_GRN,bg=bg,anchor="w"); lbl.pack(side="left",padx=4)
        return lbl

    def _tab_sysinfo(self,nb):
        f=self._mk_tab(nb,"SYSTEM INFO")
        tk.Label(f,text="  HARDWARE & OS INFORMATION",font=FT_MED,fg=ORANGE,bg=BG2).pack(anchor="w",padx=8,pady=4)
        self.si={}
        items=[("CPU Temp","--"),("Uptime","--"),("IP","--"),("Memory","--"),
               ("Disk","--"),("Session","--"),("Alarms","0"),("Running Motors","0"),
               ("All Homed","NO"),("Fan","0%"),("E-STOP","CLEAR")]
        for i,(l,v) in enumerate(items): self.si[l]=self._row(f,l,v,i)
        tk.Button(f,text="REFRESH",font=FT_MED,bg=DARKGREY,fg=ORANGE,
                  activebackground=ORANGE,activeforeground="black",
                  relief="raised",bd=2,cursor="hand2",command=self._refresh_si).pack(pady=8)
        self._refresh_si()

    def _refresh_si(self):
        if not(self.win and self.win.winfo_exists()): return
        um,tm=get_memory(); du,dp=get_disk()
        sess=int(time.time()-start_time)
        data={"CPU Temp":"{:.1f} C".format(get_cpu_temp()),"Uptime":get_uptime(),
              "IP":get_ip(),"Memory":"{} / {} MB".format(um,tm),"Disk":"{} ({})".format(du,dp),
              "Session":"{}h {}m {}s".format(sess//3600,(sess%3600)//60,sess%60),
              "Alarms":str(len(alarm_log)),
              "Running Motors":str(sum(1 for m in motors.values() if m["running"])),
              "All Homed":"YES" if all_homed else "NO","Fan":"{}%".format(fan_speed),
              "E-STOP":"ACTIVE" if estop_active else "CLEAR"}
        for k,v in data.items():
            if k in self.si:
                fg=RED if("ACTIVE" in v or v=="NO") else SCRN_GRN
                self.si[k].config(text=v,fg=fg)

    def _tab_motorconfig(self,nb):
        f=self._mk_tab(nb,"MOTOR CONFIG")
        c=tk.Canvas(f,bg=BG2,highlightthickness=0)
        sb=ttk.Scrollbar(f,orient="vertical",command=c.yview)
        inner=tk.Frame(c,bg=BG2)
        inner.bind("<Configure>",lambda e:c.configure(scrollregion=c.bbox("all")))
        c.create_window((0,0),window=inner,anchor="nw"); c.configure(yscrollcommand=sb.set)
        sb.pack(side="right",fill="y"); c.pack(side="left",fill="both",expand=True)
        tk.Label(inner,text="  MOTOR PARAMETERS",font=FT_MED,fg=ORANGE,bg=BG2).pack(anchor="w",padx=8,pady=4)
        self.mc={}
        for name in JOINT_ORDER:
            m=motors[name]; lo_i,hi_i=JOINT_LIMITS_IK[name]; lo_p,hi_p=JOINT_LIMITS_PHYS[name]
            sf=tk.Frame(inner,bg=PANEL,pady=4); sf.pack(fill="x",pady=2,padx=4)
            tk.Label(sf,text=name,font=FT_LG,fg=ORANGE,bg=PANEL,width=4).pack(side="left",padx=8)
            fields=[("Steps/Deg",str(STEPS_PER_DEG[name])),("Max Freq",str(m["target_freq"])),
                    ("IK Lo",str(lo_i)),("IK Hi",str(hi_i)),("Phys Lo",str(lo_p)),("Phys Hi",str(hi_p)),
                    ("Home Target",str(HOME_TARGET[name]))]
            entries={}
            for fl,fv in fields:
                cf=tk.Frame(sf,bg=PANEL); cf.pack(side="left",padx=6)
                tk.Label(cf,text=fl,font=FT_TINY,fg=GREY,bg=PANEL).pack()
                e=tk.Entry(cf,font=FT_SM,width=8,bg=SCRN_BG,fg=SCRN_GRN,
                           insertbackground=SCRN_GRN,relief="sunken",bd=1)
                e.insert(0,fv); e.pack()
                e.bind("<FocusIn>",lambda ev,ew=e: self.osk.attach(ew))
                entries[fl]=e
            tk.Button(sf,text="APPLY",font=FT_TINY,bg=DARKGREY,fg=ORANGE,
                      activebackground=ORANGE,activeforeground="black",
                      relief="raised",bd=1,cursor="hand2",
                      command=lambda n=name,en=entries: self._apply_motor(n,en)).pack(side="left",padx=8)
            self.mc[name]=entries

    def _apply_motor(self,name,entries):
        try:
            STEPS_PER_DEG[name]=float(entries["Steps/Deg"].get())
            motors[name]["target_freq"]=int(entries["Max Freq"].get())
            JOINT_LIMITS_IK[name]=(float(entries["IK Lo"].get()),float(entries["IK Hi"].get()))
            JOINT_LIMITS_PHYS[name]=(float(entries["Phys Lo"].get()),float(entries["Phys Hi"].get()))
            HOME_TARGET[name]=float(entries["Home Target"].get())
            log("{} config updated — home target {:.1f}deg".format(name, HOME_TARGET[name]))
        except Exception as e: log("Config error {}: {}".format(name,e),alarm=True)

    def _tab_alarms(self,nb):
        f=self._mk_tab(nb,"ALARMS")
        hdr=tk.Frame(f,bg="#181818"); hdr.pack(fill="x")
        tk.Label(hdr,text="  ALARM / FAULT HISTORY",font=FT_MED,fg=ORANGE,bg="#181818").pack(side="left",padx=8,pady=4)
        tk.Button(hdr,text="CLEAR",font=FT_SM,bg=DARKGREY,fg=RED,
                  activebackground=RED,activeforeground="white",relief="raised",bd=1,cursor="hand2",
                  command=lambda:(alarm_log.clear(),self._refresh_alarms())).pack(side="right",padx=8,pady=4)
        self.alarm_text=tk.Text(f,bg=SCRN_BG,fg=RED_LT,font=("Courier New",9),relief="flat",state="disabled")
        asb=ttk.Scrollbar(f,orient="vertical",command=self.alarm_text.yview)
        self.alarm_text.configure(yscrollcommand=asb.set)
        asb.pack(side="right",fill="y"); self.alarm_text.pack(fill="both",expand=True,padx=4,pady=4)
        tk.Button(f,text="REFRESH",font=FT_SM,bg=DARKGREY,fg=ORANGE,
                  activebackground=ORANGE,activeforeground="black",relief="raised",bd=1,cursor="hand2",
                  command=self._refresh_alarms).pack(pady=4)
        self._refresh_alarms()

    def _refresh_alarms(self):
        if not(self.win and self.win.winfo_exists()): return
        self.alarm_text.config(state="normal"); self.alarm_text.delete("1.0","end")
        if alarm_log:
            for e in alarm_log: self.alarm_text.insert("end",e+"\n")
        else: self.alarm_text.insert("end","No alarms recorded.\n")
        self.alarm_text.config(state="disabled"); self.alarm_text.see("end")

    def _tab_network(self,nb):
        f=self._mk_tab(nb,"NETWORK")
        tk.Label(f,text="  NETWORK & CONNECTIVITY",font=FT_MED,fg=ORANGE,bg=BG2).pack(anchor="w",padx=8,pady=4)
        self.net_text=tk.Text(f,bg=SCRN_BG,fg=SCRN_GRN,font=("Courier New",9),relief="flat",state="disabled")
        nsb=ttk.Scrollbar(f,orient="vertical",command=self.net_text.yview)
        self.net_text.configure(yscrollcommand=nsb.set)
        nsb.pack(side="right",fill="y"); self.net_text.pack(fill="both",expand=True,padx=4,pady=4)
        tk.Button(f,text="REFRESH",font=FT_SM,bg=DARKGREY,fg=ORANGE,
                  activebackground=ORANGE,activeforeground="black",relief="raised",bd=1,cursor="hand2",
                  command=self._refresh_network).pack(pady=4)
        self._refresh_network()

    def _refresh_network(self):
        if not(self.win and self.win.winfo_exists()): return
        try: ifconfig=subprocess.check_output(["ip","addr"],timeout=3).decode()
        except: ifconfig="Could not run ip addr"
        try: routes=subprocess.check_output(["ip","route"],timeout=3).decode()
        except: routes="Could not get routes"
        self.net_text.config(state="normal"); self.net_text.delete("1.0","end")
        self.net_text.insert("end","=== INTERFACES ===\n"+ifconfig+"\n=== ROUTES ===\n"+routes)
        self.net_text.config(state="disabled")

    def _tab_kinematics(self,nb):
        f=self._mk_tab(nb,"KINEMATICS")
        c=tk.Canvas(f,bg=BG2,highlightthickness=0)
        sb=ttk.Scrollbar(f,orient="vertical",command=c.yview)
        inner=tk.Frame(c,bg=BG2)
        inner.bind("<Configure>",lambda e:c.configure(scrollregion=c.bbox("all")))
        c.create_window((0,0),window=inner,anchor="nw"); c.configure(yscrollcommand=sb.set)
        sb.pack(side="right",fill="y"); c.pack(side="left",fill="both",expand=True)
        tk.Label(inner,text="LINK LENGTHS (mm)",font=FT_MED,fg=ORANGE,bg=BG2).pack(anchor="w",padx=8,pady=(8,2))
        self.kin={}
        for lbl,val in [("L1",L1),("L2",L2),("L3",L3),("L4",L4),("L5",L5),("TCP",TCP)]:
            rf=tk.Frame(inner,bg=PANEL,pady=3); rf.pack(fill="x",pady=1,padx=4)
            tk.Label(rf,text=lbl,font=FT_SM,fg=GREY,bg=PANEL,width=22,anchor="w").pack(side="left",padx=8)
            e=tk.Entry(rf,font=FT_MED,width=10,bg=SCRN_BG,fg=SCRN_GRN,insertbackground=SCRN_GRN,relief="sunken",bd=1)
            e.insert(0,str(val)); e.pack(side="left",padx=4)
            e.bind("<FocusIn>",lambda ev,ew=e: self.osk.attach(ew))
            self.kin[lbl]=e
        tk.Button(inner,text="APPLY",font=FT_MED,bg=DARKGREY,fg=ORANGE,
                  activebackground=ORANGE,activeforeground="black",relief="raised",bd=2,cursor="hand2",
                  command=self._apply_links).pack(pady=8,padx=8,anchor="w")

    def _apply_links(self):
        global L1,L2,L3,L4,L5,TCP
        try:
            L1=float(self.kin["L1"].get()); L2=float(self.kin["L2"].get())
            L3=float(self.kin["L3"].get()); L4=float(self.kin["L4"].get())
            L5=float(self.kin["L5"].get()); TCP=float(self.kin["TCP"].get())
            log("Link lengths updated"); _rebuild_chain()
        except Exception as e: log("Link error: {}".format(e),alarm=True)

# ─────────────────────────────────────────────
# MAIN GUI
# ─────────────────────────────────────────────
class RoboArmPrime:
    def __init__(self,root):
        self.root=root; self.root.title("ROBOARM OMEGA  v1.0")
        self.root.configure(bg=BG); self.root.attributes("-fullscreen",True)
        self.root.bind("<Escape>",lambda e: self._quit())
        self.osk=OSK(root)
        self.menu=MenuWindow(root,self.osk)
        self.prog_editor=ProgramEditor(root,self.osk)
        self.agv_panel=AGVPanel(root)
        self.ik_solution=None
        self._build_titlebar(); self._build_statusbar()
        self._build_main(); self._build_softkeys()
        start_ps4()
        log("ROBOARM OMEGA v1.0 ready  |  ikpy IK active"); log("Run HOMING before operating")
        log("Square = toggle AGV mode   Triangle = cycle ARM mode")
        self.update_loop()

    def _build_titlebar(self):
        bar=tk.Frame(self.root,bg="#111111",height=44); bar.pack(fill="x"); bar.pack_propagate(False)
        badge=tk.Frame(bar,bg=ORANGE,width=170); badge.pack(side="left",fill="y"); badge.pack_propagate(False)
        tk.Label(badge,text="ROBOARM OMEGA",font=("Courier New",10,"bold"),fg="white",bg=ORANGE).pack(expand=True)
        tk.Label(bar,text="6-AXIS + AGV  |  ikpy IK  |  T1 MANUAL  |  ESC=EXIT",font=FT_TTL,fg=WHITE,bg="#111111").pack(side="left",padx=14)
        right=tk.Frame(bar,bg="#111111"); right.pack(side="right",padx=10)
        self.time_lbl=tk.Label(right,text="00:00:00",font=("Courier New",16,"bold"),fg=ORANGE,bg="#111111")
        self.time_lbl.pack(side="right",padx=6)
        self.home_pill=tk.Label(right,text=" NOT HOMED ",font=FT_MED,fg="white",bg=RED)
        self.home_pill.pack(side="right",padx=8)
        for txt,cmd in [("PROG",self.prog_editor.toggle),
                        ("AGV",self.agv_panel.toggle),
                        ("MENU",self.menu.toggle),
                        ("KBD",self.osk.toggle)]:
            tk.Button(right,text=" {} ".format(txt),font=FT_MED,bg=DARKGREY,fg=ORANGE,
                      activebackground=ORANGE,activeforeground="black",
                      relief="raised",bd=2,cursor="hand2",command=cmd).pack(side="right",padx=3)

    def _build_statusbar(self):
        bar=tk.Frame(self.root,bg="#181818",height=28); bar.pack(fill="x"); bar.pack_propagate(False)
        self.sleds={}
        items=[("DRIVES",True,GREEN,GREEN_DK),("MOTORS",False,GREEN,GREEN_DK),
               ("HOMED",False,GREEN,GREEN_DK),("LIMITS",False,YELLOW,"#555200"),
               ("E-STOP",False,RED,RED_DK),("CTRL",False,ORANGE,ORANGE_DK),
               ("HOMING",False,YELLOW,"#555200"),("PLAYBACK",False,GREEN,GREEN_DK),
               ("AGV",False,AGV_BLUE,AGV_BLUE_DK)]
        for label,default,con,coff in items:
            f=tk.Frame(bar,bg="#181818"); f.pack(side="left",padx=7,pady=3)
            lc,lo=mk_led(f,"#181818",11); lc.pack(side="left",padx=2)
            sl(lc,lo,default,con,coff)
            tk.Label(f,text=label,font=FT_SM,fg=GREY,bg="#181818").pack(side="left")
            self.sleds[label]=(lc,lo,con,coff)
        right=tk.Frame(bar,bg="#181818"); right.pack(side="right",padx=10)
        self.ovr_badge=tk.Label(right,text=" OVR: 30% ",font=FT_SM,fg="black",bg=GREY); self.ovr_badge.pack(side="right",padx=3)
        self.mode_badge=tk.Label(right,text=" JOINT ",font=FT_MED,fg="black",bg=ORANGE); self.mode_badge.pack(side="right",padx=3)
        self.sel_badge=tk.Label(right,text=" SEL: J1 ",font=FT_MED,fg=ORANGE,bg="#181818"); self.sel_badge.pack(side="right",padx=8)

    def _build_main(self):
        body=tk.Frame(self.root,bg=BG); body.pack(fill="both",expand=True,padx=4,pady=3)
        lo=tk.Frame(body,bg=BG); lo.pack(side="left",fill="both",expand=True,padx=(0,3))
        self.mc=tk.Canvas(lo,bg=BG,highlightthickness=0)
        vsb=ttk.Scrollbar(lo,orient="vertical",command=self.mc.yview)
        self.sf=tk.Frame(self.mc,bg=BG)
        self.sf.bind("<Configure>",lambda e: self.mc.configure(scrollregion=self.mc.bbox("all")))
        self.mc.create_window((0,0),window=self.sf,anchor="nw")
        self.mc.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right",fill="y"); self.mc.pack(side="left",fill="both",expand=True)
        self.mc.bind_all("<MouseWheel>",lambda e: self.mc.yview_scroll(int(-1*(e.delta/120)),"units"))
        self.mc.bind_all("<Button-4>",lambda e: self.mc.yview_scroll(-1,"units"))
        self.mc.bind_all("<Button-5>",lambda e: self.mc.yview_scroll(1,"units"))
        self._build_axis_section(self.sf)
        self._build_cartesian_section(self.sf)
        self._build_ik_section(self.sf)
        self._build_agv_quickpanel(self.sf)
        self._build_log_section(self.sf)
        self._build_right_panel(body)

    def _sec_hdr(self,parent,text,color=ORANGE):
        f=tk.Frame(parent,bg="#181818",height=24); f.pack(fill="x",pady=(6,0)); f.pack_propagate(False)
        tk.Label(f,text="  "+text,font=FT_MED,fg=color,bg="#181818").pack(side="left",padx=4)
        tk.Frame(f,bg=color,height=1).pack(side="bottom",fill="x")

    def _build_axis_section(self,parent):
        self._sec_hdr(parent,"AXIS STATUS  /  JOG CONTROL")
        hdr=tk.Frame(parent,bg="#181818",height=20); hdr.pack(fill="x"); hdr.pack_propagate(False)
        for text,w in [("",2),("AXIS",5),("ANGLE(deg)",11),("STEPS",9),("STATUS",9),("LIM",4),("DIR",5),("SPEED",20),("JOG",12)]:
            tk.Label(hdr,text=text,font=FT_TINY,fg=GREY,bg="#181818",width=w,anchor="w").pack(side="left",padx=3)
        self.jw={}
        self._cart_jog_active=None
        self._cart_jog_after=None
        for i,name in enumerate(JOINT_ORDER):
            self._make_axis_row(parent,name,i)

    CART_ROW_MAP=[("X","X"),("Y","Y"),("Z","Z"),("Rx","Rx"),("Ry","Ry"),("Rz","Rz")]

    def _make_axis_row(self,parent,name,row_index):
        row=tk.Frame(parent,bg=PANEL,pady=1,cursor="hand2"); row.pack(fill="x",pady=2)
        row.bind("<Button-1>",lambda e,n=name: self._select(n))
        sel_lbl=tk.Label(row,text=" ",font=FT_LG,fg=ORANGE,bg=PANEL,width=2); sel_lbl.pack(side="left",padx=2)
        sel_lbl.bind("<Button-1>",lambda e,n=name: self._select(n))
        ax_lbl=tk.Label(row,text=name,font=FT_LG,fg=WHITE,bg=PANEL,width=4,cursor="hand2"); ax_lbl.pack(side="left")
        ax_lbl.bind("<Button-1>",lambda e,n=name: self._select(n))
        ang_bg=tk.Frame(row,bg=SCRN_BG,relief="sunken",bd=1); ang_bg.pack(side="left",padx=4)
        ang_lbl=tk.Label(ang_bg,text=" +000.00",font=("Courier New",11,"bold"),fg=SCRN_GRN,bg=SCRN_BG,width=9,anchor="e",padx=4,pady=2); ang_lbl.pack()
        stp_bg=tk.Frame(row,bg=SCRN_BG,relief="sunken",bd=1); stp_bg.pack(side="left",padx=2)
        stp_lbl=tk.Label(stp_bg,text="+000000",font=("Courier New",10,"bold"),fg="#008833",bg=SCRN_BG,width=8,anchor="e",padx=4,pady=2); stp_lbl.pack()
        stat_lbl=tk.Label(row,text="IDLE    ",font=FT,fg=GREY,bg=PANEL,width=9,anchor="w"); stat_lbl.pack(side="left",padx=4)
        lc,lo2=mk_led(row,PANEL,12); lc.pack(side="left",padx=4); sl(lc,lo2,False,RED,GREEN_DK)
        dir_lbl=tk.Label(row,text="FWD",font=FT_SM,fg=ORANGE,bg=PANEL,width=4); dir_lbl.pack(side="left",padx=2)
        spd_var=tk.IntVar(value=30)
        spd_scale=tk.Scale(row,from_=1,to=100,orient="horizontal",variable=spd_var,length=130,
                           bg=PANEL,fg=WHITE,troughcolor=BG,highlightthickness=0,bd=0,
                           activebackground=ORANGE,showvalue=False,
                           command=lambda v,n=name: set_motor_speed(n,int(v))); spd_scale.pack(side="left")
        spd_lbl=tk.Label(row,text=" 30%",font=FT_SM,fg=ORANGE,bg=PANEL,width=5); spd_lbl.pack(side="left")
        jog=tk.Frame(row,bg=PANEL); jog.pack(side="left",padx=4)
        btn_n=tk.Button(jog,text=" - ",font=FT_MED,bg="#3a1a0a",fg=WHITE,relief="raised",activebackground=RED_LT,bd=2,padx=4,pady=3,cursor="hand2"); btn_n.pack(side="left",padx=1)
        btn_n.bind("<ButtonPress-1>",  lambda e,n=name,ri=row_index: self._jog_dispatch(n,0,ri))
        btn_n.bind("<ButtonRelease-1>",lambda e,n=name,ri=row_index: self._jog_release(n,ri))
        btn_p=tk.Button(jog,text=" + ",font=FT_MED,bg="#0a2a0a",fg=WHITE,relief="raised",activebackground=GREEN,bd=2,padx=4,pady=3,cursor="hand2"); btn_p.pack(side="left",padx=1)
        btn_p.bind("<ButtonPress-1>",  lambda e,n=name,ri=row_index: self._jog_dispatch(n,1,ri))
        btn_p.bind("<ButtonRelease-1>",lambda e,n=name,ri=row_index: self._jog_release(n,ri))
        lo2v,hi2v=JOINT_LIMITS_PHYS[name]
        lim_lbl=tk.Label(row,text=" [{:.0f}~{:.0f}]".format(lo2v,hi2v),font=FT_TINY,fg=GREY,bg=PANEL); lim_lbl.pack(side="left")
        self.jw[name]={"row":row,"sel":sel_lbl,"ax":ax_lbl,"ang":ang_lbl,"stp":stp_lbl,
                       "stat":stat_lbl,"lc":lc,"lo":lo2,"dir":dir_lbl,"spd_var":spd_var,
                       "spd_lbl":spd_lbl,"btn_n":btn_n,"btn_p":btn_p,
                       "lim_lbl":lim_lbl,"row_index":row_index}

    def _jog_dispatch(self,name,direction,row_index):
        self._select(name)
        if control_mode=="JOINT":
            start_motor(name,direction)
        else:
            cart_axis=self.CART_ROW_MAP[row_index][1]
            sign=1 if direction==1 else -1
            self._cart_jog_active=(cart_axis,sign)
            self._do_cart_jog(cart_axis,sign)

    def _do_cart_jog(self,cart_axis,sign):
        if hasattr(self,"_cart_jog_active") and self._cart_jog_active:
            cartesian_jog(cart_axis,sign)
            self._cart_jog_after=self.root.after(80,lambda: self._do_cart_jog(cart_axis,sign))

    def _jog_release(self,name,row_index):
        if control_mode=="JOINT":
            stop_motor(name)
        else:
            self._cart_jog_active=None
            if hasattr(self,"_cart_jog_after") and self._cart_jog_after:
                self.root.after_cancel(self._cart_jog_after)
                self._cart_jog_after=None

    def _build_cartesian_section(self,parent):
        self._sec_hdr(parent,"CARTESIAN POSITION  /  TCP (mm)")
        cart=tk.Frame(parent,bg=BG2,pady=6); cart.pack(fill="x",pady=(0,2))
        crow=tk.Frame(cart,bg=BG2); crow.pack(fill="x",padx=8,pady=2)
        self.coord_lbls={}
        for axis in ["X","Y","Z","Rx","Ry","Rz"]:
            f=tk.Frame(crow,bg=SCRN_BG,relief="sunken",bd=1); f.pack(side="left",padx=4)
            tk.Label(f,text=axis,font=FT_SM,fg=GREY,bg=SCRN_BG,width=3).pack(side="left",padx=3)
            lbl=tk.Label(f,text="   0.000",font=("Courier New",11,"bold"),fg=SCRN_GRN,bg=SCRN_BG,width=9,anchor="e"); lbl.pack(side="left",padx=3,pady=3)
            self.coord_lbls[axis]=lbl

    def _build_ik_section(self,parent):
        self._sec_hdr(parent,"INVERSE KINEMATICS  /  WORLD MODE TARGET")
        ik=tk.Frame(parent,bg=BG2,pady=6); ik.pack(fill="x",pady=(0,2))
        r1=tk.Frame(ik,bg=BG2); r1.pack(fill="x",padx=8,pady=2)
        tk.Label(r1,text="TARGET XYZ (mm):",font=FT_SM,fg=GREY,bg=BG2).pack(side="left",padx=4)
        self.ik_entries={}
        for axis in ["X","Y","Z"]:
            tk.Label(r1,text=axis,font=FT_MED,fg=ORANGE,bg=BG2).pack(side="left",padx=(8,2))
            e=tk.Entry(r1,font=FT_MED,width=8,bg=SCRN_BG,fg=SCRN_GRN,insertbackground=SCRN_GRN,relief="sunken",bd=2)
            e.insert(0,"0.0"); e.pack(side="left",padx=2)
            e.bind("<FocusIn>",lambda ev,ew=e: self.osk.attach(ew)); self.ik_entries[axis]=e
        r2=tk.Frame(ik,bg=BG2); r2.pack(fill="x",padx=8,pady=2)
        tk.Label(r2,text="ORIENTATION Rx/Ry/Rz (deg):",font=FT_SM,fg=GREY,bg=BG2).pack(side="left",padx=4)
        for axis in ["Rx","Ry","Rz"]:
            tk.Label(r2,text=axis,font=FT_MED,fg=YELLOW,bg=BG2).pack(side="left",padx=(8,2))
            e=tk.Entry(r2,font=FT_MED,width=8,bg=SCRN_BG,fg="#ccaa00",insertbackground="#ccaa00",relief="sunken",bd=2)
            e.insert(0,"0.0"); e.pack(side="left",padx=2)
            e.bind("<FocusIn>",lambda ev,ew=e: self.osk.attach(ew)); self.ik_entries[axis]=e
        self.ik_result=tk.Label(ik,text="IK SOLUTION: -- enter target and press SOLVE --",font=FT_SM,fg=GREY,bg=BG2)
        self.ik_result.pack(anchor="w",padx=12,pady=2)
        br=tk.Frame(ik,bg=BG2); br.pack(fill="x",padx=8,pady=2)
        for txt,cmd,fg2 in [("SOLVE IK",self._solve_ik,ORANGE),("MOVE TO TARGET",self._move_to_ik,GREEN),("SET AS HOME",self._set_home,YELLOW)]:
            tk.Button(br,text=txt,font=FT_MED,bg=DARKGREY,fg=fg2,activebackground=fg2,activeforeground="black",
                      relief="raised",bd=2,cursor="hand2",command=cmd).pack(side="left",padx=4)

    def _build_agv_quickpanel(self,parent):
        """Inline AGV status strip in the main scroll area."""
        self._sec_hdr(parent,"AGV  /  EVA DRIVE STATUS", color=AGV_BLUE)
        af=tk.Frame(parent,bg=AGV_BG,pady=6); af.pack(fill="x",pady=(0,2))
        row=tk.Frame(af,bg=AGV_BG); row.pack(fill="x",padx=8)

        # Mode indicator
        mf=tk.Frame(row,bg=AGV_BG); mf.pack(side="left",padx=8)
        tk.Label(mf,text="MODE",font=FT_TINY,fg=GREY,bg=AGV_BG).pack()
        self.agv_mode_lbl=tk.Label(mf,text="ARM",font=FT_LG,fg=GREY,bg=AGV_BG,width=6)
        self.agv_mode_lbl.pack()

        # Throttle / steering
        for key,label in [("agv_t_lbl","THROTTLE"),("agv_r_lbl","STEERING")]:
            cf=tk.Frame(row,bg=SCRN_BG,relief="sunken",bd=1); cf.pack(side="left",padx=6)
            tk.Label(cf,text=label,font=FT_TINY,fg=GREY,bg=SCRN_BG).pack(padx=8,pady=(2,0))
            lbl=tk.Label(cf,text=" +0.000",font=("Courier New",11,"bold"),fg=AGV_BLUE,bg=SCRN_BG,width=8,anchor="e",padx=4,pady=2)
            lbl.pack(); setattr(self,key,lbl)

        # Left / right motor output (from status poll)
        for key,label in [("agv_left_lbl","L-MOTOR"),("agv_right_lbl","R-MOTOR")]:
            cf=tk.Frame(row,bg=SCRN_BG,relief="sunken",bd=1); cf.pack(side="left",padx=6)
            tk.Label(cf,text=label,font=FT_TINY,fg=GREY,bg=SCRN_BG).pack(padx=8,pady=(2,0))
            lbl=tk.Label(cf,text="  ---",font=("Courier New",11,"bold"),fg=GREY,bg=SCRN_BG,width=6,anchor="e",padx=4,pady=2)
            lbl.pack(); setattr(self,key,lbl)

        # AGV button
        tk.Button(row,text="OPEN\nAGV PANEL",font=FT_TINY,bg=AGV_BLUE_DK,fg=AGV_BLUE,
                  activebackground=AGV_BLUE,activeforeground="black",
                  relief="raised",bd=2,cursor="hand2",
                  command=self.agv_panel.toggle).pack(side="right",padx=12)

        tk.Label(af,text="Square = AGV  |  Left stick = drive  |  Right stick = turn  |  Cross = stop",
                 font=FT_TINY,fg=GREY,bg=AGV_BG).pack(anchor="w",padx=12,pady=(2,4))

    def _build_log_section(self,parent):
        self._sec_hdr(parent,"SYSTEM LOG")
        lf=tk.Frame(parent,bg=SCRN_BG,pady=2); lf.pack(fill="both",padx=4,pady=(0,4))
        self.log_text=tk.Text(lf,height=8,bg=SCRN_BG,fg=SCRN_GRN,font=("Courier New",8),relief="flat",state="disabled",insertbackground=SCRN_GRN)
        lsb=ttk.Scrollbar(lf,orient="vertical",command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=lsb.set)
        lsb.pack(side="right",fill="y"); self.log_text.pack(fill="both",expand=True,padx=4,pady=2)

    def _build_right_panel(self,parent):
        rp=tk.Frame(parent,bg=BG2,width=260); rp.pack(side="right",fill="y"); rp.pack_propagate(False)
        rpc=tk.Canvas(rp,bg=BG2,highlightthickness=0)
        rpsb=ttk.Scrollbar(rp,orient="vertical",command=rpc.yview)
        rpi=tk.Frame(rpc,bg=BG2)
        rpi.bind("<Configure>",lambda e: rpc.configure(scrollregion=rpc.bbox("all")))
        rpc.create_window((0,0),window=rpi,anchor="nw"); rpc.configure(yscrollcommand=rpsb.set)
        rpsb.pack(side="right",fill="y"); rpc.pack(side="left",fill="both",expand=True)
        r=rpi
        tk.Button(r,text="E - S T O P",font=("Courier New",14,"bold"),bg="#aa1100",fg="white",
                  activebackground=RED_LT,relief="raised",bd=5,height=2,cursor="hand2",
                  command=stop_all).pack(fill="x",padx=8,pady=(10,3))
        tk.Button(r,text="STOP ALL AXES",font=FT_MED,bg=DARKGREY,fg=YELLOW,
                  activebackground=YELLOW,activeforeground="black",relief="raised",bd=2,cursor="hand2",
                  command=stop_all).pack(fill="x",padx=8,pady=2)
        self._div(r)
        tk.Label(r,text="HOMING",font=FT_SM,fg=GREY,bg=BG2).pack(anchor="w",padx=10)
        tk.Button(r,text="RUN HOMING SEQUENCE",font=FT_MED,bg="#1a1a3a",fg=ORANGE,
                  activebackground=ORANGE,activeforeground="black",relief="raised",bd=2,cursor="hand2",
                  command=start_homing).pack(fill="x",padx=8,pady=2)
        self.home_detail=tk.Label(r,text="NOT HOMED",font=FT_SM,fg=RED,bg=BG2,wraplength=220,justify="left")
        self.home_detail.pack(anchor="w",padx=10,pady=2)
        self._div(r)
        tk.Label(r,text="ACTIVE AXIS",font=FT_SM,fg=GREY,bg=BG2).pack(anchor="w",padx=10)
        self.sel_xl=tk.Label(r,text="J1",font=("Courier New",48,"bold"),fg=ORANGE,bg=BG2); self.sel_xl.pack()
        self.sel_spd=tk.Label(r,text="SPEED:  30%",font=FT_MED,fg=WHITE,bg=BG2); self.sel_spd.pack()
        self.sel_ang=tk.Label(r,text="ANGLE: +000.00 deg",font=("Courier New",9),fg=SCRN_GRN,bg=BG2); self.sel_ang.pack()
        self.sel_stat=tk.Label(r,text="STATUS: IDLE",font=FT_SM,fg=GREY,bg=BG2); self.sel_stat.pack()
        sr=tk.Frame(r,bg=BG2); sr.pack(pady=4)
        tk.Button(sr,text="SPD-",font=FT_MED,bg=DARKGREY,fg=WHITE,activebackground=RED,relief="raised",bd=2,cursor="hand2",command=lambda: self._nudge(-5)).pack(side="left",padx=3)
        tk.Button(sr,text="SPD+",font=FT_MED,bg=DARKGREY,fg=WHITE,activebackground=GREEN,relief="raised",bd=2,cursor="hand2",command=lambda: self._nudge(5)).pack(side="left",padx=3)
        self._div(r)
        tk.Label(r,text="GLOBAL OVERRIDE",font=FT_SM,fg=GREY,bg=BG2).pack(anchor="w",padx=10)
        self.ovr_var=tk.IntVar(value=30)
        tk.Scale(r,from_=1,to=100,orient="horizontal",variable=self.ovr_var,length=230,bg=BG2,fg=WHITE,troughcolor=BG,highlightthickness=0,bd=0,activebackground=ORANGE,showvalue=False,command=self._set_override).pack(padx=8)
        self.ovr_lbl=tk.Label(r,text="OVR:  30%",font=FT_MED,fg=ORANGE,bg=BG2); self.ovr_lbl.pack()
        self._div(r)
        tk.Label(r,text="COOLING FAN",font=FT_SM,fg=GREY,bg=BG2).pack(anchor="w",padx=10)
        fb=tk.Frame(r,bg=BG2); fb.pack(fill="x",padx=8,pady=2)
        for lbl,val in [("OFF",0),("25",25),("50",50),("75",75),("100",100)]:
            tk.Button(fb,text=lbl,font=FT_TINY,bg=DARKGREY,fg=WHITE,activebackground=ORANGE,activeforeground="black",relief="raised",bd=1,cursor="hand2",command=lambda v=val: self._set_fan(v)).pack(side="left",expand=True,fill="x",padx=1)
        self.fan_var=tk.IntVar(value=2)
        tk.Scale(r,from_=0,to=100,orient="horizontal",variable=self.fan_var,length=230,bg=BG2,fg=WHITE,troughcolor=BG,highlightthickness=0,bd=0,activebackground=ORANGE,showvalue=False,command=lambda v: self._set_fan(int(v))).pack(padx=8)
        self.fan_lbl=tk.Label(r,text="FAN:   2%",font=FT_MED,fg=ORANGE,bg=BG2); self.fan_lbl.pack()
        self._div(r)
        tk.Label(r,text="PENDANT INPUT",font=FT_SM,fg=GREY,bg=BG2).pack(anchor="w",padx=10)
        self.ctrl_lbl=tk.Label(r,text="NO DEVICE",font=FT_MED,fg=RED,bg=BG2); self.ctrl_lbl.pack(pady=2)
        tk.Button(r,text="CONNECT CONTROLLER",font=FT_SM,bg=DARKGREY,fg=ORANGE,activebackground=ORANGE,activeforeground="black",relief="raised",bd=2,cursor="hand2",command=start_ps4).pack(fill="x",padx=8,pady=2)
        self._div(r)
        tk.Label(r,text="CONTROL MODE",font=FT_SM,fg=GREY,bg=BG2).pack(anchor="w",padx=10)
        self.mode_var=tk.StringVar(value="JOINT")
        tk.Radiobutton(r,text="Joint Mode",variable=self.mode_var,value="JOINT",font=FT,fg=WHITE,bg=BG2,selectcolor=BG,activebackground=BG2,command=self._set_mode).pack(anchor="w",padx=14)
        tk.Radiobutton(r,text="World Mode  (IK)",variable=self.mode_var,value="WORLD",font=FT,fg=WHITE,bg=BG2,selectcolor=BG,activebackground=BG2,command=self._set_mode).pack(anchor="w",padx=14)
        tk.Radiobutton(r,text="Tool Mode   (IK)",variable=self.mode_var,value="TOOL",font=FT,fg=WHITE,bg=BG2,selectcolor=BG,activebackground=BG2,command=self._set_mode).pack(anchor="w",padx=14)
        tk.Radiobutton(r,text="AGV Mode",variable=self.mode_var,value="AGV",font=FT,fg=AGV_BLUE,bg=BG2,selectcolor=BG,activebackground=BG2,command=self._set_mode).pack(anchor="w",padx=14)
        self._div(r)
        tk.Label(r,text="ARM GEOMETRY",font=FT_SM,fg=GREY,bg=BG2).pack(anchor="w",padx=10)
        self.geo_lbl=tk.Label(r,text="",font=FT_TINY,fg=GREY,bg=BG2,justify="left"); self.geo_lbl.pack(anchor="w",padx=10)

    def _build_softkeys(self):
        bar=tk.Frame(self.root,bg="#111111",height=36); bar.pack(fill="x",side="bottom"); bar.pack_propagate(False)
        keys=[("F1 HOME",start_homing),("F2 ZERO",self._zero),("F3 STOP",stop_all),
              ("F4 MODE",self._toggle_mode),("F5 SOLVE",self._solve_ik),("F6 MOVE IK",self._move_to_ik),
              ("F7 PROG",self.prog_editor.toggle),("F8 FAN50",lambda: self._set_fan(50)),
              ("F9 FAN0",lambda: self._set_fan(0)),("F10 OVR100",lambda: self._set_override(100)),
              ("AGV",self.agv_panel.toggle),("MENU",self.menu.toggle),
              ("KBD",self.osk.toggle),("ESC EXIT",self._quit)]
        for label,cmd in keys:
            tk.Button(bar,text=label,font=FT_TINY,bg="#1e1e1e",fg=WHITE,
                      activebackground=ORANGE,activeforeground="black",relief="flat",bd=0,padx=4,cursor="hand2",
                      command=cmd).pack(side="left",fill="y",expand=True,padx=1,pady=3)

    def _div(self,parent): tk.Frame(parent,bg=BORDER,height=1).pack(fill="x",padx=8,pady=5)

    def _select(self,name):
        global selected_joint; selected_joint=JOINT_ORDER.index(name); log("Selected: "+name)

    def _nudge(self,delta):
        jname=get_joint_name(); new_spd=max(1,min(100,motors[jname]["speed"]+delta))
        set_motor_speed(jname,new_spd); self.jw[jname]["spd_var"].set(new_spd)
        log("{} spd {}%".format(jname,new_spd))

    def _set_override(self,val):
        pct=int(val); self.ovr_var.set(pct)
        self.ovr_lbl.config(text="OVR: {:3d}%".format(pct))
        self.ovr_badge.config(text=" OVR: {}% ".format(pct))
        for name in JOINT_ORDER: set_motor_speed(name,pct); self.jw[name]["spd_var"].set(pct)

    def _set_fan(self,val):
        set_fan(val); self.fan_var.set(val); self.fan_lbl.config(text="FAN: {:3d}%".format(val))
        log("Fan: {}%".format(val))

    def _set_mode(self):
        global control_mode,_last_arm_mode
        control_mode=self.mode_var.get()
        if control_mode != "AGV":
            _last_arm_mode = control_mode
        else:
            stop_all()
        if control_mode in ("WORLD","TOOL") and not all_homed:
            log("WARNING: home robot before cartesian mode")
        log("Mode: "+control_mode)

    def _toggle_mode(self):
        global control_mode,_last_arm_mode
        all_modes=["JOINT","WORLD","TOOL","AGV"]
        idx=all_modes.index(control_mode)
        control_mode=all_modes[(idx+1)%len(all_modes)]
        if control_mode != "AGV": _last_arm_mode = control_mode
        self.mode_var.set(control_mode); log("Mode: "+control_mode)

    def _zero(self):
        for m in motors.values(): m["position"]=0; m["angle_deg"]=0.0
        log("All positions zeroed")

    def _set_home(self):
        for m in motors.values(): m["position"]=0; m["angle_deg"]=0.0; m["homed"]=True
        log("Current position set as home")

    def _solve_ik(self):
        try:
            tx=float(self.ik_entries["X"].get()); ty=float(self.ik_entries["Y"].get()); tz=float(self.ik_entries["Z"].get())
            rx=float(self.ik_entries["Rx"].get()); ry=float(self.ik_entries["Ry"].get()); rz=float(self.ik_entries["Rz"].get())
        except ValueError:
            self.ik_result.config(text="IK ERROR: invalid input",fg=RED); return
        sol=inverse_kinematics(tx,ty,tz,rx,ry,rz)
        if sol is None:
            self.ik_result.config(text="IK: NO SOLUTION - target unreachable or out of limits",fg=RED)
            self.ik_solution=None; log("IK: no solution for ({},{},{})".format(tx,ty,tz))
        else:
            self.ik_solution=sol
            result="  ".join("{}:{:+.1f}".format(JOINT_ORDER[i],sol[i]) for i in range(6))
            self.ik_result.config(text="IK OK: "+result,fg=GREEN); log("IK solved: "+result)

    def _move_to_ik(self):
        if self.ik_solution is None: log("IK: solve first"); return
        if not all_homed: log("IK: home robot first"); return
        log("Moving to IK target...")
        for i,name in enumerate(JOINT_ORDER):
            threading.Thread(target=move_to_angle,args=(name,self.ik_solution[i]),daemon=True).start()

    def _quit(self):
        agv.stop(); agv.running=False
        fan_off_hard(); stop_all(); lgpio.gpiochip_close(h); self.root.destroy()

    def update_loop(self):
        self.time_lbl.config(text=time.strftime("%H:%M:%S"))

        # Overheat check — throttled to once per 30s so log stays clean
        _ct = get_cpu_temp()
        if _ct >= 48.0:
            if not hasattr(self, '_last_overheat_log') or time.time() - self._last_overheat_log > 30:
                log("!! OVERHEAT {:.1f}C — check cooling !!".format(_ct), alarm=True)
                self._last_overheat_log = time.time()
            if fan_speed < 80:
                set_fan(80)

        # Status LEDs
        lc,lo,con,coff=self.sleds["E-STOP"]; sl(lc,lo,estop_active,con,coff)
        lc,lo,con,coff=self.sleds["CTRL"];   sl(lc,lo,ps4_connected,con,coff)
        lc,lo,con,coff=self.sleds["HOMED"];  sl(lc,lo,all_homed,con,coff)
        lc,lo,con,coff=self.sleds["HOMING"]; sl(lc,lo,homing_active,con,coff)
        lc,lo,con,coff=self.sleds["PLAYBACK"]; sl(lc,lo,playback_running,con,coff)
        lc,lo,con,coff=self.sleds["AGV"]; sl(lc,lo,control_mode=="AGV",con,coff)
        any_lim=any(m["at_limit"] for m in motors.values())
        lc,lo,con,coff=self.sleds["LIMITS"]; sl(lc,lo,any_lim,con,coff)
        any_run=any(m["running"] for m in motors.values())
        lc,lo,con,coff=self.sleds["MOTORS"]; sl(lc,lo,any_run,GREEN,GREEN_DK)

        # Home pill
        if all_homed: self.home_pill.config(text=" HOMED ",bg=GREEN,fg="black"); self.home_detail.config(text="All axes homed and ready",fg=GREEN)
        elif homing_active:
            self.home_pill.config(text=" HOMING... ",bg=YELLOW,fg="black")
            homed=[n for n in JOINT_ORDER if motors[n]["homed"]]
            self.home_detail.config(text="Homed: "+" ".join(homed) if homed else "In progress...",fg=YELLOW)
        else: self.home_pill.config(text=" NOT HOMED ",bg=RED,fg="white"); self.home_detail.config(text="Press HOME to start",fg=RED)

        # Mode badge
        if control_mode=="JOINT":
            self.mode_badge.config(text=" JOINT ",bg=ORANGE)
            self.mode_var.set("JOINT")
        elif control_mode=="WORLD":
            self.mode_badge.config(text=" WORLD ",bg=YELLOW if all_homed else RED)
            self.mode_var.set("WORLD")
        elif control_mode=="TOOL":
            self.mode_badge.config(text=" TOOL  ",bg="#4a9eff" if all_homed else RED)
            self.mode_var.set("TOOL")
        elif control_mode=="AGV":
            self.mode_badge.config(text="  AGV  ",bg=AGV_BLUE)
            self.mode_var.set("AGV")

        self.sel_badge.config(text=" SEL: {} ".format(get_joint_name()))
        self.ctrl_lbl.config(text="CONTROLLER OK" if ps4_connected else "NO DEVICE",fg=GREEN if ps4_connected else RED)
        self.fan_lbl.config(text="FAN: {:3d}%".format(fan_speed))
        self.geo_lbl.config(text="L1={} L2={} L3={}\nL4={} L5={} TCP={}".format(L1,L2,L3,L4,L5,TCP))

        # Active axis display
        jname=get_joint_name(); m=motors[jname]
        self.sel_xl.config(text=jname if control_mode!="AGV" else "AGV",
                           fg=AGV_BLUE if control_mode=="AGV" else ORANGE)
        self.sel_spd.config(text="SPEED: {:3d}%".format(m["speed"]))
        self.sel_ang.config(text="ANGLE: {:+.2f} deg".format(m["angle_deg"]))
        if estop_active: self.sel_stat.config(text="STATUS: E-STOP",fg=RED)
        elif control_mode=="AGV": self.sel_stat.config(text="STATUS: AGV MODE",fg=AGV_BLUE)
        elif m["at_limit"]: self.sel_stat.config(text="STATUS: AT LIMIT",fg=YELLOW)
        elif m["running"]: self.sel_stat.config(text="STATUS: RUNNING",fg=GREEN)
        elif not m["homed"]: self.sel_stat.config(text="STATUS: NOT HOMED",fg=YELLOW)
        else: self.sel_stat.config(text="STATUS: IDLE",fg=GREY)

        # Cartesian FK display
        angles=[motors[n]["angle_deg"] for n in JOINT_ORDER]
        try:
            tcp=forward_kinematics(angles)
            self.coord_lbls["X"].config(text="{:+10.3f}".format(tcp[0]))
            self.coord_lbls["Y"].config(text="{:+10.3f}".format(tcp[1]))
            self.coord_lbls["Z"].config(text="{:+10.3f}".format(tcp[2]))
        except: pass

        # AGV quick panel
        is_agv = (control_mode=="AGV")
        self.agv_mode_lbl.config(
            text="AGV" if is_agv else "ARM",
            fg=AGV_BLUE if is_agv else GREY)
        self.agv_t_lbl.config(
            text="{:+.3f}".format(agv.translate),
            fg=AGV_BLUE if abs(agv.translate)>0.05 else GREY)
        self.agv_r_lbl.config(
            text="{:+.3f}".format(agv.rotate),
            fg=AGV_BLUE if abs(agv.rotate)>0.05 else GREY)
        st=agv.status_json
        self.agv_left_lbl.config(text=str(st.get("left","---")),
                                  fg=AGV_BLUE if "left" in st else GREY)
        self.agv_right_lbl.config(text=str(st.get("right","---")),
                                   fg=AGV_BLUE if "right" in st else GREY)

        # Axis rows
        cart_labels=["X","Y","Z","Rx","Ry","Rz"]
        tool_labels=["Xt","Yt","Zt","Rxt","Ryt","Rzt"]
        for i,name in enumerate(JOINT_ORDER):
            w=self.jw[name]; m=motors[name]
            is_sel=(JOINT_ORDER[selected_joint]==name)
            bg=PANEL_SEL if is_sel else PANEL
            w["row"].config(bg=bg); w["sel"].config(text=">" if is_sel else " ",bg=bg)
            w["ax"].config(bg=bg,fg=ORANGE if is_sel else WHITE)
            w["ang"].config(text="{:+.2f}".format(m["angle_deg"]))
            pos=m["position"]; psign="+" if pos>=0 else "-"
            w["stp"].config(text="{}{:06d}".format(psign,abs(pos)))
            if estop_active: w["stat"].config(text="E-STOP  ",fg=RED)
            elif m["at_limit"]: w["stat"].config(text="AT LIMIT",fg=YELLOW)
            elif m["running"]: w["stat"].config(text="RUNNING ",fg=GREEN)
            elif not m["homed"]: w["stat"].config(text="NOT HOME",fg=YELLOW)
            else: w["stat"].config(text="IDLE    ",fg=GREY)
            sl(w["lc"],w["lo"],m["at_limit"],RED,GREEN_DK)
            w["spd_lbl"].config(text="{:3d}%".format(m["speed"]))
            if control_mode=="JOINT":
                w["ax"].config(text=name)
                lo2v,hi2v=JOINT_LIMITS_PHYS[name]
                w["lim_lbl"].config(text=" [{:.0f}~{:.0f}]".format(lo2v,hi2v))
                w["dir"].config(text="FWD" if m["direction"]==1 else "REV",
                                fg=GREEN if m["direction"]==1 else RED)
            elif control_mode=="WORLD":
                w["ax"].config(text=cart_labels[i]); w["lim_lbl"].config(text=" [mm/deg]"); w["dir"].config(text="",fg=GREY)
            elif control_mode=="TOOL":
                w["ax"].config(text=tool_labels[i]); w["lim_lbl"].config(text=" [tool]"); w["dir"].config(text="",fg=GREY)
            else:  # AGV
                w["ax"].config(text=name,fg=GREY); w["lim_lbl"].config(text=" [AGV]"); w["dir"].config(text="",fg=GREY)

        # Log
        self.log_text.config(state="normal"); self.log_text.delete("1.0","end")
        for entry in system_log[-20:]: self.log_text.insert("end",entry+"\n")
        self.log_text.config(state="disabled"); self.log_text.see("end")

        # Refresh program editor if open
        if self.prog_editor.win and self.prog_editor.win.winfo_exists():
            self.prog_editor.refresh()

        self.root.after(100,self.update_loop)

# ─────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────
root=tk.Tk()
app=RoboArmPrime(root)
root.protocol("WM_DELETE_WINDOW",app._quit)
root.mainloop()
