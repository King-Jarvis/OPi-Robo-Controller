"""
Absolute and coordinated joint moves — V4 :824-918, in kinematic degrees.

move_to sends an exact pulse count (lgpio tx_pulse with pulse_cycles), so the
joint stops on its target in hardware however late this thread wakes up —
V4 ran a free pulse train and stopped it when the mover noticed arrival, so
every scheduling hiccup became overshoot. A speed change made mid-move
applies from the next move. Starting another move on the same joint
pre-empts the one in flight.
"""

import threading
import time

from ..kinematics import mapping


class Mover:
    def __init__(self, bank, log):
        self.bank = bank
        self.log = log
        self._cancel = [threading.Event() for _ in bank]

    def cancel_all(self):
        for ev in self._cancel:
            ev.set()

    def move_to(self, i, target_deg, freq=None):
        """Drive joint i to an absolute angle. Returns True on arrival."""
        m = self.bank[i]
        if not m.homed or self.bank.estop_active:
            return False
        lo, hi = m.j.limits_deg
        target_deg = max(lo, min(hi, target_deg))
        target_p = mapping.deg_to_pulses(m.j, target_deg)

        ev = self._cancel[i]
        ev.set()
        time.sleep(0.015)                 # let any previous mover fall out
        ev.clear()
        m.stop()                          # flushes the step integral first

        count = int(round(abs(target_p - m.pulses)))
        if count < 2:
            return True
        positive = target_deg > m.angle
        freq = max(10, int(freq or m.rate()))
        why = m.run_count(positive, count, freq)
        if why:
            if why != "estop":
                self.log("{} move refused ({})".format(m.name, why))
            return False
        # Generous ceiling: nominal duration plus 50% plus a second of slack.
        deadline = time.monotonic() + count / freq * 1.5 + 1.0
        while True:
            if self.bank.estop_active or ev.is_set() or not m.running:
                break
            if m.at_limit and m.toward_switch(positive):
                break
            if m.counted_done():
                m.finish_count()
                return True
            if time.monotonic() > deadline:
                self.log("{} move timed out {:.1f}° short".format(
                    m.name, abs(m.target - m.pulses) / m.j.steps_per_deg), alarm=True)
                break
            time.sleep(0.002)
        m.stop()                          # interrupted: kill the train
        return False

    def move_all(self, angles_deg, wait=True, speed=None):
        """
        Coordinated move: each joint's rate is scaled so they all arrive at the
        same instant (V3 let them finish whenever, swinging through
        unintended poses). Returns True if every joint arrived (wait only).
        """
        plan, longest = [], 0.0
        for m, tgt in zip(self.bank, angles_deg):
            lo, hi = m.j.limits_deg
            tgt = max(lo, min(hi, tgt))
            dist = abs(mapping.deg_to_pulses(m.j, tgt) - (m.flush() or m.pulses))
            fmax = float(m.rate(speed))
            dur = dist / fmax if dist > 2 else 0.0
            longest = max(longest, dur)
            plan.append((m, tgt, dist, fmax))

        results, threads = [], []
        for i, (m, tgt, dist, fmax) in enumerate(plan):
            if dist <= 2:
                continue
            freq = max(10.0, min(fmax, dist / longest if longest > 0 else fmax))
            t = threading.Thread(target=lambda i=i, tgt=tgt, f=freq:
                                 results.append(self.move_to(i, tgt, f)), daemon=True)
            threads.append(t)
            t.start()
        if not wait:
            return None
        for t in threads:
            t.join()
        return all(results)
