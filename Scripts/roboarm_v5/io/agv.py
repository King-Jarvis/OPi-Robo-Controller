"""
AGV link — V4 :355-430, wire protocol unchanged.

8-byte UDP packets to the ESP32 at 50 Hz while enabled:
    b'EVA' + int16be(t*10000) + int16be(r*10000) + CRC8(poly 0x07)
    t = translate  -1 reverse .. +1 forward   (left stick Y)
    r = rotate     -1 left    .. +1 right     (right stick X)
/status is polled twice a second for telemetry.
"""

import json
import socket
import struct
import threading
import time
import urllib.request

SEND_HZ = 50


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def pack(t, r):
    ti = int(max(-1.0, min(1.0, t)) * 10000)
    ri = int(max(-1.0, min(1.0, r)) * 10000)
    header = b'EVA' + struct.pack('>hh', ti, ri)
    return header + bytes([crc8(header)])


class AGVLink:
    def __init__(self, ip="192.168.4.1", port=5005, start=True):
        self.ip, self.port = ip, port
        self.enabled = False              # True while the pendant is in AGV mode
        self.translate = 0.0
        self.rotate = 0.0
        self.status_json = {}
        self.online = False
        self._run = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if start:
            threading.Thread(target=self._send_loop, name="agv-tx", daemon=True).start()
            threading.Thread(target=self._status_loop, name="agv-status", daemon=True).start()

    def _send_loop(self):
        interval = 1.0 / SEND_HZ
        while self._run:
            if self.enabled:
                try:
                    self.sock.sendto(pack(self.translate, self.rotate), (self.ip, self.port))
                except Exception:
                    pass
            time.sleep(interval)

    def _status_loop(self):
        while self._run:
            try:
                with urllib.request.urlopen(
                        "http://{}/status".format(self.ip), timeout=0.4) as r:
                    self.status_json = json.loads(r.read().decode())
                    self.online = True
            except Exception:
                self.online = False
            time.sleep(0.5)

    def set(self, translate, rotate):
        self.translate = max(-1.0, min(1.0, translate))
        self.rotate = max(-1.0, min(1.0, rotate))

    def stop(self):
        self.translate = 0.0
        self.rotate = 0.0

    def close(self):
        self._run = False
        self.stop()
