"""
Stepper motors and the step model — V4 :441-560, one object per joint.

lgpio generates each pulse train in the background, so nothing counts real
edges: position is the integral of commanded frequency over elapsed time.
That integral is only right if it is flushed *before* frequency, direction or
run state changes, so every such change goes through flush() under the
motor's lock. (V4 had the same model but no lock; the safety thread and a
mover thread could flush the same joint at once.)

Angles here are kinematic degrees (see kinematics/mapping.py). Directions are
"positive" (angle increasing) or not — never raw dir-pin levels.
"""

import threading
import time

from ..kinematics import mapping

MIN_FREQ = 10


class Motor:
    def __init__(self, joint, bank):
        self.j = joint
        self.name = joint.name
        self.bank = bank
        self.gpio = bank.gpio
        self.running = False
        self.level = 0                    # dir pin level
        self.pulses = 0.0                 # signed, 0 at the datum
        self.freq = 0.0
        self.t0 = time.monotonic()
        self.target = None                # pulse count a counted train ends at
        self.at_limit = False
        self.limit_side = None            # True if the switch was hit moving positive
        self.homed = False
        self.homing = False               # limit stop is left to the homing routine
        self.speed = joint.speed
        self.soft_block = None            # "lo" | "hi" | None
        self._lock = threading.RLock()

    def claim(self):
        g = self.gpio
        g.claim_output(self.j.step_pin, 0)
        g.claim_output(self.j.dir_pin, 0)
        if self.j.limit_pin is not None:
            g.claim_input_pullup(self.j.limit_pin)

    # ── step model ───────────────────────────────────────────────────────
    def flush(self, now=None):
        with self._lock:
            now = time.monotonic() if now is None else now
            if self.running and self.freq > 0.0:
                dt = now - self.t0
                if dt > 0.0:
                    self.pulses += self.freq * dt * (1 if self.level == 1 else -1)
                    if self.target is not None:
                        # a counted train stops itself at the target
                        self.pulses = (min(self.pulses, self.target) if self.level == 1
                                       else max(self.pulses, self.target))
            self.t0 = now

    @property
    def angle(self):
        """Current angle (kinematic degrees), integral brought up to date."""
        self.flush()
        return mapping.pulses_to_deg(self.j, self.pulses)

    @property
    def moving_positive(self):
        return mapping.level_is_positive(self.j, self.level)

    def rate(self, speed=None):
        """Step frequency for a speed percentage (default: this motor's)."""
        pct = self.speed if speed is None else speed
        return max(MIN_FREQ, int(self.j.max_freq * pct / 100.0))

    # ── interlocks ───────────────────────────────────────────────────────
    def toward_switch(self, positive):
        sw = self.j.home.switch
        if sw is not None:
            return positive == (sw == "max")
        # no declared switch side: block the direction we were moving when it hit
        return self.limit_side is not None and positive == self.limit_side

    def blocked(self, positive):
        """Reason a move this way is refused, or None."""
        if self.bank.estop_active:
            return "estop"
        if self.at_limit and self.toward_switch(positive):
            return "limit"
        if self.homed:
            a = self.angle
            lo, hi = self.j.limits_deg
            if positive and a >= hi:
                return "hi"
            if not positive and a <= lo:
                return "lo"
        return None

    # ── drive ────────────────────────────────────────────────────────────
    def stop(self):
        with self._lock:
            self.flush()
            self.gpio.pwm_off(self.j.step_pin)
            self.running = False
            self.freq = 0.0
            self.target = None

    def start(self, positive, freq=None, check=True):
        """Run continuously toward +/- angle. Returns the refusal reason or None."""
        if check:
            why = self.blocked(positive)
            if why:
                if why in ("hi", "lo"):
                    self.soft_block = why
                return why
        level = mapping.dir_level(self.j, positive)
        freq = max(MIN_FREQ, int(freq or self.rate()))
        with self._lock:
            if self.running and self.level == level:
                self.set_freq(freq)
                return None
            self.stop()
        time.sleep(0.01)                  # let the driver see STEP low first
        with self._lock:
            self.level = level
            self.gpio.write(self.j.dir_pin, level)
        time.sleep(0.005)                 # DIR setup time before the first edge
        with self._lock:
            self.flush()
            self.running = True
            self.freq = float(freq)
            self.t0 = time.monotonic()
            self.gpio.pwm(self.j.step_pin, freq, 50)
        return None

    def run_count(self, positive, count, freq):
        """
        Send exactly `count` pulses toward +/- angle, then stop in hardware.
        Returns the refusal reason or None. Poll counted_done().
        """
        why = self.blocked(positive)
        if why:
            return why
        count = int(round(count))
        if count <= 0:
            return None
        level = mapping.dir_level(self.j, positive)
        freq = max(MIN_FREQ, int(freq))
        self.stop()
        time.sleep(0.01)
        with self._lock:
            self.level = level
            self.gpio.write(self.j.dir_pin, level)
        time.sleep(0.005)
        with self._lock:
            self.flush()
            self.running = True
            self.freq = float(freq)
            self.target = self.pulses + (count if level == 1 else -count)
            self.t0 = time.monotonic()
            self.gpio.pulses(self.j.step_pin, freq, count)
        return None

    def counted_done(self):
        """True once a counted train has been fully sent."""
        with self._lock:
            if self.target is None:
                return not self.running
            self.flush()
            return self.pulses == self.target and not self.gpio.busy(self.j.step_pin)

    def finish_count(self):
        """Book a completed counted train: exact by construction."""
        with self._lock:
            if self.target is not None:
                self.pulses = self.target
            self.target = None
            self.running = False
            self.freq = 0.0

    def set_freq(self, freq):
        with self._lock:
            if not self.running or self.target is not None:
                # a counted train can't be retuned without losing count;
                # the new speed applies from the next move
                return
            freq = max(MIN_FREQ, int(freq))
            self.flush()                  # bank the pulses at the old rate first
            self.freq = float(freq)
            self.gpio.pwm(self.j.step_pin, freq, 50)

    def set_speed(self, pct):
        self.speed = max(1, min(100, int(pct)))
        if self.running:
            self.set_freq(self.rate())

    def set_angle(self, deg):
        """Declare where the joint is (datum found, or zeroed in place)."""
        with self._lock:
            self.flush()
            self.pulses = mapping.deg_to_pulses(self.j, deg)


class MotorBank:
    """All joints, plus the E-STOP state they all obey."""

    def __init__(self, model, gpio, log):
        self.model = model
        self.gpio = gpio
        self.log = log
        self.estop_active = False
        self.estop_pin_raw = False
        self.motors = [Motor(j, self) for j in model.joints]
        self.by_name = {m.name: m for m in self.motors}

    def __iter__(self):
        return iter(self.motors)

    def __len__(self):
        return len(self.motors)

    def __getitem__(self, k):
        return self.by_name[k] if isinstance(k, str) else self.motors[k]

    def claim(self):
        for m in self.motors:
            m.claim()

    def angles(self):
        return [m.angle for m in self.motors]

    @property
    def all_homed(self):
        return all(m.homed for m in self.motors)

    @property
    def any_running(self):
        return any(m.running for m in self.motors)

    def stop_all(self, reason="STOP ALL", alarm=True):
        for m in self.motors:
            m.stop()
        if reason:
            self.log(reason, alarm=alarm)

    def set_all_speeds(self, pct):
        for m in self.motors:
            m.set_speed(pct)
