"""
GPIO backends.

LgpioBackend   the real thing on the Orange Pi (V4 behaviour, same calls).
SimGPIO        a fake board with simulated joints: it integrates the step
               pulse trains into a "true" joint angle and closes each limit
               switch when its joint reaches the switch. The whole stack —
               homing included — runs against it on a desktop or in tests.
"""

import threading
import time

from ..kinematics import mapping


class LgpioBackend:
    def __init__(self, chip=0):
        import lgpio
        self._lg = lgpio
        self.h = lgpio.gpiochip_open(chip)

    def claim_output(self, pin, level=0):
        self._lg.gpio_claim_output(self.h, pin, level)

    def claim_input_pullup(self, pin):
        self._lg.gpio_claim_input(self.h, pin, self._lg.SET_PULL_UP)

    def write(self, pin, level):
        self._lg.gpio_write(self.h, pin, int(level))

    def read(self, pin):
        return self._lg.gpio_read(self.h, pin)

    def pwm(self, pin, freq, duty_pct):
        """Start a pulse train. lgpio generates it in the background."""
        self._lg.tx_pwm(self.h, pin, freq, duty_pct)

    def pulses(self, pin, freq, count):
        """
        Exactly `count` step pulses at `freq`, then stop by itself. Absolute
        moves use this so arrival doesn't depend on Python waking up on time.
        """
        period = max(2, int(round(1e6 / freq)))
        on = period // 2
        self._lg.tx_pulse(self.h, pin, on, period - on, 0, int(count))

    def busy(self, pin):
        return bool(self._lg.tx_busy(self.h, pin, self._lg.TX_PWM))

    def pwm_off(self, pin):
        # V4's sequence: stop the train, then reclaim the pin driven low.
        # (tx_pwm also deletes anything queued by tx_pulse.)
        self._lg.tx_pwm(self.h, pin, 1, 0)
        self._lg.gpio_claim_output(self.h, pin, 0)

    def close(self):
        try:
            self._lg.gpiochip_close(self.h)
        except Exception:
            pass


class _SimJoint:
    def __init__(self, joint, start_deg):
        self.j = joint
        self.deg = float(start_deg)
        self.freq = 0.0
        self.count_left = None            # pulses still to send (counted train)
        self.level = 0
        self.t0 = time.monotonic()


class SimGPIO:
    """
    Simulated board for a RobotModel. `start_deg` is where each joint really
    is at power-on (unknown to the software until it homes). Switches close
    at their datum; the joint can over-travel `overtravel_deg` past it and no
    further (a hard stop).
    """

    def __init__(self, model, start_deg=None, overtravel_deg=3.0):
        self.model = model
        self.levels = {}
        self.estop_open = False
        self.fan_duty = 0
        self.overtravel = overtravel_deg
        self._lock = threading.Lock()
        start = start_deg if start_deg is not None else model.park_pose()
        self.joints = [_SimJoint(j, s) for j, s in zip(model.joints, start)]
        self._by_step = {sj.j.step_pin: sj for sj in self.joints}
        self._by_dir = {sj.j.dir_pin: sj for sj in self.joints}
        self._by_limit = {sj.j.limit_pin: sj for sj in self.joints if sj.j.limit_pin is not None}

    # ── simulated physics ────────────────────────────────────────────────
    def _advance(self, sj, now=None):
        now = time.monotonic() if now is None else now
        if sj.freq > 0:
            n = sj.freq * (now - sj.t0)
            if sj.count_left is not None:
                n = min(n, sj.count_left)
                sj.count_left -= n
                if sj.count_left <= 1e-9:
                    sj.count_left, sj.freq = None, 0.0
            step_deg = n / sj.j.steps_per_deg
            sgn = 1.0 if mapping.level_is_positive(sj.j, sj.level) else -1.0
            sj.deg += sgn * step_deg
            h = sj.j.home
            if h.switch == "max":
                sj.deg = min(sj.deg, h.datum_deg + self.overtravel)
            elif h.switch == "min":
                sj.deg = max(sj.deg, h.datum_deg - self.overtravel)
        sj.t0 = now

    def true_deg(self):
        with self._lock:
            for sj in self.joints:
                self._advance(sj)
            return [sj.deg for sj in self.joints]

    # ── backend API ──────────────────────────────────────────────────────
    def claim_output(self, pin, level=0):
        self.write(pin, level)

    def claim_input_pullup(self, pin):
        pass

    def write(self, pin, level):
        with self._lock:
            sj = self._by_dir.get(pin)
            if sj is not None:
                self._advance(sj)
                sj.level = int(level)
            self.levels[pin] = int(level)

    def read(self, pin):
        b = self.model.board
        if pin == b.estop_pin:
            return b.estop_triggered if self.estop_open else 1 - b.estop_triggered
        with self._lock:
            sj = self._by_limit.get(pin)
            if sj is None:
                return self.levels.get(pin, 0)
            self._advance(sj)
            h = sj.j.home
            hit = ((h.switch == "max" and sj.deg >= h.datum_deg) or
                   (h.switch == "min" and sj.deg <= h.datum_deg))
            return b.limit_triggered if hit else 1 - b.limit_triggered

    def pwm(self, pin, freq, duty_pct):
        with self._lock:
            sj = self._by_step.get(pin)
            if sj is not None:
                self._advance(sj)
                sj.freq = float(freq) if duty_pct > 0 else 0.0
                sj.count_left = None
            elif pin == self.model.board.fan_pin:
                self.fan_duty = duty_pct

    def pulses(self, pin, freq, count):
        with self._lock:
            sj = self._by_step[pin]
            self._advance(sj)
            sj.freq, sj.count_left = float(freq), float(count)

    def busy(self, pin):
        with self._lock:
            sj = self._by_step.get(pin)
            if sj is None:
                return False
            self._advance(sj)
            return sj.freq > 0

    def pwm_off(self, pin):
        self.pwm(pin, 0, 0)

    def close(self):
        pass


def open_backend(model, sim=False, **sim_kw):
    if sim:
        return SimGPIO(model, **sim_kw)
    return LgpioBackend(model.board.gpiochip)
