import lgpio
import time
import threading

GPIOCHIP      = 0
FAN_PIN       = 267
ESTOP_PIN     = 266
DEBOUNCE_TIME = 0.05
FAN_PWM_FREQ  = 100

motors = {
    "J1": {"step": 226, "dir": 227, "target_freq": 500,  "limit": 260, "running": False, "direction": 1, "at_limit": False},
    "J2": {"step": 228, "dir": 229, "target_freq": 2000, "limit": 261, "running": False, "direction": 1, "at_limit": False},
    "J3": {"step": 230, "dir": 231, "target_freq": 4000, "limit": 262, "running": False, "direction": 1, "at_limit": False},
    "J4": {"step": 232, "dir": 233, "target_freq": 500,  "limit": 263, "running": False, "direction": 1, "at_limit": False},
    "J5": {"step": 256, "dir": 257, "target_freq": 4000, "limit": 264, "running": False, "direction": 1, "at_limit": False},
    "J6": {"step": 258, "dir": 259, "target_freq": 4000, "limit": 265, "running": False, "direction": 1, "at_limit": False},
}

h = lgpio.gpiochip_open(GPIOCHIP)
estop_active = False
fan_speed = 0

for name, m in motors.items():
    lgpio.gpio_claim_output(h, m["step"], 0)
    lgpio.gpio_claim_output(h, m["dir"],  0)
    lgpio.gpio_claim_input(h,  m["limit"], lgpio.SET_PULL_UP)

lgpio.gpio_claim_input(h,  ESTOP_PIN, lgpio.SET_PULL_UP)
lgpio.gpio_claim_output(h, FAN_PIN, 0)

def set_fan(speed_percent):
    global fan_speed
    speed_percent = max(0, min(100, speed_percent))
    fan_speed = speed_percent
    if speed_percent == 0:
        lgpio.gpio_claim_output(h, FAN_PIN, 0)
    else:
        lgpio.tx_pwm(h, FAN_PIN, FAN_PWM_FREQ, speed_percent)
    print("Fan speed: {}%".format(speed_percent))

def stop_motor(name):
    m = motors[name]
    lgpio.tx_pwm(h, m["step"], 1, 0)
    lgpio.gpio_claim_output(h, m["step"], 0)
    m["running"] = False

def stop_all():
    for name in motors:
        stop_motor(name)

def start_motor(name, direction):
    m = motors[name]
    lgpio.tx_pwm(h, m["step"], 1, 0)
    lgpio.gpio_claim_output(h, m["step"], 0)
    time.sleep(0.05)
    m["direction"] = direction
    m["running"] = True
    lgpio.gpio_write(h, m["dir"], direction)
    time.sleep(0.01)
    lgpio.tx_pwm(h, m["step"], m["target_freq"], 50)

def monitor_limits():
    global estop_active
    last_trigger = {name: 0 for name in motors}
    last_estop = 0
    while True:
        try:
            if lgpio.gpio_read(h, ESTOP_PIN) == 1:
                now = time.time()
                if now - last_estop > DEBOUNCE_TIME:
                    if not estop_active:
                        print("E-STOP triggered - all motors stopped")
                        estop_active = True
                        stop_all()
                    last_estop = now
            else:
                if estop_active:
                    print("E-STOP released - ready")
                    estop_active = False
            for name, m in motors.items():
                state = lgpio.gpio_read(h, m["limit"])
                if state == 1:
                    now = time.time()
                    if now - last_trigger[name] > DEBOUNCE_TIME:
                        if not m["at_limit"]:
                            print(name + " limit hit - stopped")
                            m["at_limit"] = True
                            stop_motor(name)
                        last_trigger[name] = now
                else:
                    if m["at_limit"]:
                        print(name + " limit released - free to move")
                        m["at_limit"] = False
            time.sleep(0.01)
        except Exception:
            break

monitor_thread = threading.Thread(target=monitor_limits, daemon=True)
monitor_thread.start()

print("OPi Zero 2W - Motor control")
print("Commands: J1 F, J1 B, stop, FAN 0-100, quit")

try:
    while True:
        cmd = input("> ").strip().upper().split()
        if not cmd:
            continue
        if cmd[0] == "QUIT":
            break
        elif cmd[0] == "STOP":
            stop_all()
            print("All stopped")
        elif cmd[0] == "FAN":
            if len(cmd) == 2 and cmd[1].isdigit():
                set_fan(int(cmd[1]))
            else:
                print("Fan at {}% - usage: FAN 0-100".format(fan_speed))
        elif len(cmd) == 2 and cmd[0] in motors and cmd[1] in ["F", "B"]:
            name = cmd[0]
            direction = 1 if cmd[1] == "F" else 0
            m = motors[name]
            if estop_active:
                print("E-STOP active - release first")
            elif m["at_limit"] and m["direction"] == direction:
                print(name + " at limit - reverse first")
            else:
                start_motor(name, direction)
        else:
            print("Unknown command")
except KeyboardInterrupt:
    pass
finally:
    set_fan(0)
    stop_all()
    lgpio.gpiochip_close(h)
    print("Done")
