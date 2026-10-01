"""
System and alarm log — V4 :268, as an object so every layer shares one.
The UI tails it by watching `seq`, which bumps on every entry.
"""

import threading
import time


class Log:
    def __init__(self, keep=400, keep_alarms=200, echo=False):
        self.system, self.alarms = [], []
        self.seq = 0
        self.keep, self.keep_alarms = keep, keep_alarms
        self.echo = echo
        self._lock = threading.Lock()

    def __call__(self, msg, alarm=False):
        entry = "{}  {}".format(time.strftime("%H:%M:%S"), msg)
        with self._lock:
            self.system.append(entry)
            if len(self.system) > self.keep:
                del self.system[:self.keep // 4]
            if alarm:
                self.alarms.append(entry)
                if len(self.alarms) > self.keep_alarms:
                    del self.alarms[:self.keep_alarms // 4]
            self.seq += 1
        if self.echo:
            print(("!! " if alarm else "   ") + entry, flush=True)

    def tail(self, since_seq):
        """Entries added after `since_seq` (best effort if trimmed)."""
        with self._lock:
            n = min(self.seq - since_seq, len(self.system))
            return (self.system[-n:] if n > 0 else []), self.seq

    def clear_alarms(self):
        with self._lock:
            self.alarms.clear()
            self.seq += 1
