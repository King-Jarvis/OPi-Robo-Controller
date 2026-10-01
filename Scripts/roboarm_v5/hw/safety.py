"""
Safety loop — V4 :611. Runs at 100 Hz on its own thread:

  · E-STOP: wired normally-closed, so a cut wire reads as pressed. Latches
    until reset() when `latching` (V4 default), otherwise clears on release.
  · Limit switches: stop the joint (unless it is homing, in which case the
    homing routine owns the stop). Moving *away* from a switch stays allowed.
  · Soft limits: a homed joint is stopped at the edge of its range.
  · Integrates every running joint's step model.
"""

import threading
import time

DEBOUNCE = 0.05
PERIOD = 0.01


class Safety:
    def __init__(self, bank, latching=True):
        self.bank = bank
        self.gpio = bank.gpio
        self.board = bank.model.board
        self.log = bank.log
        self.latching = latching
        self._thread = None
        self._run = False

    def claim(self):
        if self.board.estop_pin is not None:
            self.gpio.claim_input_pullup(self.board.estop_pin)

    def start(self):
        self._run = True
        self._thread = threading.Thread(target=self._loop, name="safety", daemon=True)
        self._thread.start()

    def stop(self):
        self._run = False
        if self._thread:
            self._thread.join(timeout=1.0)

    def reset(self):
        """Clear a latched E-STOP. Refused while the loop is still open."""
        if self.bank.estop_pin_raw:
            self.log("E-STOP reset refused — loop still open", alarm=True)
            return False
        self.bank.estop_active = False
        self.log("E-STOP reset — drives re-enabled")
        return True

    def trigger(self, reason="E-STOP (software)"):
        self.bank.estop_active = True
        self.bank.stop_all(reason)

    def _loop(self):
        last_limit = {m.name: 0.0 for m in self.bank}
        while self._run:
            try:
                now = time.monotonic()
                self._estop()
                for m in self.bank:
                    self._limit(m, now, last_limit)
                    if m.running:
                        m.flush(now)
                    self._soft(m)
            except Exception as e:
                self.log("Safety loop fault: {}".format(e), alarm=True)
                time.sleep(0.2)
            time.sleep(PERIOD)

    def _estop(self):
        # No debounce on the way in: the first open reading stops everything.
        b, bank = self.board, self.bank
        if b.estop_pin is None:
            return
        if self.gpio.read(b.estop_pin) == b.estop_triggered:
            bank.estop_pin_raw = True
            if not bank.estop_active:
                bank.estop_active = True
                bank.stop_all("E-STOP TRIGGERED")
            return
        if bank.estop_pin_raw:
            bank.estop_pin_raw = False
            self.log("E-STOP loop closed" + ("  —  press RESET to re-enable"
                                             if self.latching else ""))
        if bank.estop_active and not self.latching:
            bank.estop_active = False

    def _limit(self, m, now, last):
        pin = m.j.limit_pin
        if pin is None:
            return
        if self.gpio.read(pin) == self.board.limit_triggered:
            if now - last[m.name] > DEBOUNCE and not m.at_limit:
                m.at_limit = True
                m.limit_side = m.moving_positive if m.running else m.limit_side
                if not m.homing and m.running and m.toward_switch(m.moving_positive):
                    m.stop()
                    self.log("{} limit switch".format(m.name), alarm=True)
            last[m.name] = now
        elif m.at_limit:
            m.at_limit = False
            self.log("{} limit cleared".format(m.name))

    def _soft(self, m):
        # A soft limit is a property of where the joint IS, not of whether it
        # is moving; evaluating it only while running would latch it forever.
        if not m.homed or m.homing:
            m.soft_block = None
            return
        a = m.angle
        lo, hi = m.j.limits_deg
        blk = "hi" if a >= hi else "lo" if a <= lo else None
        if blk and m.running and (m.moving_positive == (blk == "hi")):
            m.stop()
            self.log("{} soft limit ({})".format(m.name, blk), alarm=True)
        m.soft_block = blk
