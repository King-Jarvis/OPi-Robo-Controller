"""
Cooling fan — V4 :464-606. Idle below t_lo, linear to 100% at t_hi, with 4%
hysteresis so it doesn't hunt. The H618 idles near 45 °C and throttles ~85 °C.
"""

import time


def cpu_temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return float(f.read()) / 1000.0
    except Exception:
        return 0.0


class Fan:
    def __init__(self, gpio, board, idle=2, t_lo=45.0, t_hi=72.0, auto=True):
        self.gpio = gpio
        self.pin = board.fan_pin
        self.pwm_freq = board.fan_pwm_freq
        self.idle, self.t_lo, self.t_hi = idle, t_lo, t_hi
        self.auto = auto
        self.speed = 0
        self._target = idle

    def claim(self):
        if self.pin is None:
            return
        self.gpio.claim_output(self.pin, 0)
        self.set(self.idle)

    def set(self, pct):
        pct = max(0, min(100, int(pct)))
        self.speed = pct
        if self.pin is None:
            return
        if pct == 0:
            self.gpio.pwm(self.pin, 1, 0)
            time.sleep(0.02)
            self.gpio.write(self.pin, 0)
        else:
            self.gpio.pwm(self.pin, self.pwm_freq, pct)

    def off_hard(self):
        if self.pin is None:
            return
        self.gpio.pwm(self.pin, 1, 0); time.sleep(0.05)
        self.gpio.write(self.pin, 0); time.sleep(0.05)

    def curve(self, temp):
        if temp <= self.t_lo:
            return self.idle
        if temp >= self.t_hi:
            return 100
        span = max(1.0, self.t_hi - self.t_lo)
        return int(self.idle + (100 - self.idle) * (temp - self.t_lo) / span)

    def tick(self, temp):
        """Call once a second."""
        if not self.auto:
            return
        want = self.curve(temp)
        if abs(want - self._target) >= 4 or (want == 100 and self._target != 100):
            self._target = want
            self.set(want)
