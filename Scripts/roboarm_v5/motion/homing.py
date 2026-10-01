"""
Homing — V4 :956-1034, driven by each joint's `home` block in the robot file.

For a joint with a switch: drive toward the switch's end of travel until it
closes, back off, creep back onto it at SLOW_DEG_S reading the pin directly
(so the switch point doesn't depend on how fast the joint was going when it
first hit it), declare that point `datum_deg`, then back away to `park_deg`.
For a joint without one: declare where it stands to be `datum_deg`.

V4 recorded the switch point as 0 and then drove to a *positive* home target
for every joint, so the joints that home in the positive direction (J1, J5)
were driven straight back into their switches. Here the switch side is
explicit, so the park move always backs away.
"""

import threading
import time

from ..kinematics import mapping

SEEK_SPEED_PCT = 20
PARK_SPEED_PCT = 20
BACKOFF_DEG = 3.0
SLOW_DEG_S = 2.0                          # final approach: ~1 ms latency = 0.002°


class Homing:
    def __init__(self, bank, mover, log):
        self.bank = bank
        self.mover = mover
        self.log = log
        self.active = False

    def _wait(self, cond, timeout, poll=0.005):
        end = time.monotonic() + timeout
        while not cond():
            if self.bank.estop_active:
                return "aborted"
            if time.monotonic() > end:
                return "timed out"
            time.sleep(poll)
        return None

    def _on_switch(self, m):
        """Read the limit pin directly — no safety-loop poll or debounce lag."""
        b = self.bank.model.board
        return self.bank.gpio.read(m.j.limit_pin) == b.limit_triggered

    def home_joint(self, i):
        m = self.bank[i]
        j = m.j
        toward = mapping.homing_toward_positive(j)
        m.homed = False

        if toward is None:
            m.set_angle(j.home.datum_deg)
            m.homed = True
            self.log("{} has no switch — set to {:+.1f}° where it stands"
                     .format(m.name, j.home.datum_deg))
            return True

        freq = m.rate(SEEK_SPEED_PCT)
        # time to cross the whole range at seek speed, plus margin
        timeout = j.span_deg * j.steps_per_deg / freq * 1.5 + 5.0
        if self.bank.estop_active:
            return False
        m.homing = True
        try:
            # 1. fast seek (skipped if already sitting on the switch)
            if not self._on_switch(m):
                self.log("{} seeking {} switch".format(m.name, j.home.switch))
                m.start(toward, freq, check=False)
                why = self._wait(lambda: self._on_switch(m), timeout, poll=0.002)
                m.stop()
                if why:
                    self.log("{} homing {}".format(m.name, why), alarm=True)
                    return False
                time.sleep(0.05)

            # 2. back off until the switch is clearly open
            m.run_count(not toward, BACKOFF_DEG * j.steps_per_deg, freq)
            why = self._wait(m.counted_done, BACKOFF_DEG * j.steps_per_deg / freq + 5.0)
            if why:
                m.stop()
            else:
                m.finish_count()
            if why or self._on_switch(m):
                self.log("{} could not back off its switch ({})".format(
                    m.name, why or "still closed"), alarm=True)
                return False
            time.sleep(0.05)

            # 3. creep back on; this is the point that defines the datum
            m.start(toward, max(10, SLOW_DEG_S * j.steps_per_deg), check=False)
            why = self._wait(lambda: self._on_switch(m),
                             2 * BACKOFF_DEG / SLOW_DEG_S + 5.0, poll=0.0005)
            m.stop()
            if why:
                self.log("{} switch not found on slow approach ({})".format(m.name, why),
                         alarm=True)
                return False
            m.set_angle(j.home.datum_deg)
            self.log("{} datum found at {:+.1f}°".format(m.name, j.home.datum_deg))
        finally:
            m.homing = False

        time.sleep(0.15)
        m.homed = True
        saved = m.speed
        m.speed = PARK_SPEED_PCT
        try:
            ok = self.mover.move_to(i, j.home.park_deg)
        finally:
            m.speed = saved
        if not ok:
            self.log("{} did not reach park {:+.1f}°".format(m.name, j.home.park_deg),
                     alarm=True)
            return False
        self.log("{} home {:+.1f}°".format(m.name, m.angle))
        return True

    def run(self, order=None):
        self.active = True
        try:
            self.log("HOMING SEQUENCE START")
            for i in (order or range(len(self.bank))):
                if self.bank.estop_active:
                    self.log("Homing aborted by E-STOP", alarm=True)
                    return False
                if not self.home_joint(i):
                    return False
                time.sleep(0.3)
            self.log("HOMING COMPLETE  —  " + "  ".join(
                "{}:{:+.1f}".format(m.name, m.angle) for m in self.bank))
            return True
        finally:
            self.active = False

    def start(self, order=None):
        if self.active:
            return
        if self.bank.estop_active:
            self.log("Cannot home while E-STOP is active", alarm=True)
            return
        threading.Thread(target=self.run, args=(order,), name="homing", daemon=True).start()
