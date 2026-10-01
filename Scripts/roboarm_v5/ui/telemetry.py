"""
System telemetry for the status page — V4 :1367-1408, unchanged.
"""

import os
import subprocess


def get_ip():
    try:
        r = subprocess.check_output(["hostname", "-I"], timeout=2).decode().strip()
        return r.split()[0] if r else "—"
    except Exception:
        return "—"


def get_memory():
    try:
        with open("/proc/meminfo") as f:
            lines = f.readlines()
        total = int([l for l in lines if "MemTotal" in l][0].split()[1])
        avail = int([l for l in lines if "MemAvailable" in l][0].split()[1])
        return (total - avail) // 1024, total // 1024
    except Exception:
        return 0, 0


def get_uptime():
    try:
        with open("/proc/uptime") as f:
            s = float(f.read().split()[0])
        return "{}h {:02d}m".format(int(s // 3600), int((s % 3600) // 60))
    except Exception:
        return "—"


def get_load():
    try:
        return "{:.2f}".format(os.getloadavg()[0])
    except Exception:
        return "—"


def get_disk():
    try:
        p = subprocess.check_output(["df", "-h", "/"], timeout=2
                                    ).decode().strip().split("\n")[1].split()
        return p[2], p[4]
    except Exception:
        return "—", "—"
