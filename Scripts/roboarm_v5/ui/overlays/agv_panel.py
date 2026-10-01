"""
AGV pendant overlay — V4 :1869-2013, reading the runtime instead of globals.
"""

import math
import time
import tkinter as tk

from ..theme import Ink, Type, SP2, SP3, SP4, SP5
from ..widgets import Overlay, InkButton, Dot, cfg, hairline, text, track


class AGVPanel(Overlay):
    TITLE = "agv pendant"
    KANJI = "搬送車"
    GEOM = "440x600+780+30"

    JOY, RAD, NUB = 300, 122, 30

    def __init__(self, root, rt):
        super().__init__(root)
        self.rt = rt
        self._canvas = self._nub = None
        self._dragging = False
        self._last_send = 0.0
        self._cx = self._cy = self.JOY // 2
        self.telem = {}

    def on_close(self):
        self.rt.agv.stop()

    def build(self, body):
        st = tk.Frame(body, bg=Ink.ground)
        st.pack(fill="x", pady=(SP4, 0))
        tk.Label(st, text=track("link"), font=Type.label, fg=Ink.text_3,
                 bg=Ink.ground).pack(side="left", padx=(SP5, SP3))
        self.link_dot = Dot(st, bg=Ink.ground)
        self.link_dot.pack(side="left")
        self.link_lbl = text(st, "—", Type.body, Ink.text_3)
        self.link_lbl.pack(side="left", padx=SP2)
        self.src_lbl = text(st, self.rt.agv.ip, Type.body, Ink.text_4)
        self.src_lbl.pack(side="right", padx=SP5)

        self.mode_lbl = text(body, "standby", Type.body, Ink.text_3)
        self.mode_lbl.pack(pady=(SP4, SP3))

        mid = tk.Frame(body, bg=Ink.ground)
        mid.pack()
        c = tk.Canvas(mid, width=self.JOY, height=self.JOY, bg=Ink.void,
                      highlightthickness=0, bd=0, cursor="hand2")
        c.pack()
        self._canvas = c
        cx = cy = self.JOY // 2
        R = self.RAD
        c.create_oval(cx - R, cy - R, cx + R, cy + R, outline=Ink.line_lit, width=1)
        c.create_oval(cx - R // 2, cy - R // 2, cx + R // 2, cy + R // 2,
                      outline=Ink.line, width=1)
        c.create_line(cx - R, cy, cx + R, cy, fill=Ink.line)
        c.create_line(cx, cy - R, cx, cy + R, fill=Ink.line)
        for txt, x, y in [("▲", cx, cy - R - 14), ("▼", cx, cy + R + 14),
                          ("◀", cx - R - 14, cy), ("▶", cx + R + 14, cy)]:
            c.create_text(x, y, text=txt, fill=Ink.text_4, font=Type.micro)
        self._nub = c.create_oval(cx - self.NUB, cy - self.NUB,
                                  cx + self.NUB, cy + self.NUB,
                                  fill=Ink.ai, outline="")
        c.bind("<ButtonPress-1>", self._joy_press)
        c.bind("<B1-Motion>", self._joy_move)
        c.bind("<ButtonRelease-1>", self._joy_release)
        for seq in ("<Button-4>", "<Button-5>", "<MouseWheel>"):
            c.bind(seq, lambda e: "break")

        InkButton(body, "STOP AGV", command=self._stop, variant="danger",
                  font=Type.body_b, pady=SP3).pack(fill="x", padx=SP5, pady=SP5)

        hairline(body)
        strip = tk.Frame(body, bg=Ink.ground)
        strip.pack(fill="x", side="bottom", pady=SP4)
        for key, label in [("t", "throttle"), ("r", "steering"),
                           ("left", "l-motor"), ("right", "r-motor")]:
            cell = tk.Frame(strip, bg=Ink.ground)
            cell.pack(side="left", expand=True)
            v = tk.Label(cell, text="—", font=Type.data, fg=Ink.text_3, bg=Ink.ground)
            v.pack()
            tk.Label(cell, text=track(label), font=Type.micro, fg=Ink.text_4,
                     bg=Ink.ground).pack()
            self.telem[key] = v
        self._tick()

    # ── joystick ──────────────────────────────────────────────────────
    def _joy_press(self, e):
        self._dragging = True
        self._last_send = 0.0
        self._joy_move(e)

    def _joy_move(self, e):
        if not self._dragging:
            return
        dx, dy = e.x - self._cx, e.y - self._cy
        d = math.hypot(dx, dy)
        if d > self.RAD:
            dx, dy = dx / d * self.RAD, dy / d * self.RAD
        self._place_nub(dx, dy)
        now = time.time()
        if now - self._last_send < 0.02:
            return
        self._last_send = now
        t = -(dy / self.RAD)
        r = dx / self.RAD
        PDEAD = 0.08
        if self.rt.pendant.mode == "AGV":
            self.rt.agv.set(t if abs(t) > PDEAD else 0.0, r if abs(r) > PDEAD else 0.0)

    def _joy_release(self, _e):
        self._dragging = False
        self._place_nub(0, 0)
        self.rt.agv.stop()

    def _place_nub(self, dx, dy):
        n, cx, cy = self.NUB, self._cx, self._cy
        self._canvas.coords(self._nub, cx + dx - n, cy + dy - n,
                            cx + dx + n, cy + dy + n)

    def _stop(self):
        self.rt.agv.stop()
        self.rt.log("AGV stop")

    def _tick(self):
        if not self.alive():
            return
        st = self.rt.agv.status_json
        t, r = self.rt.agv.translate, self.rt.agv.rotate
        is_agv = (self.rt.pendant.mode == "AGV")

        if not self._dragging:
            dx, dy = r * self.RAD, -t * self.RAD
            d = math.hypot(dx, dy)
            if d > self.RAD:
                dx, dy = dx / d * self.RAD, dy / d * self.RAD
            self._place_nub(dx, dy)
        self._canvas.itemconfig(self._nub, fill=Ink.ai if is_agv else Ink.line_lit)

        cfg(self.mode_lbl,
            text="active  —  joystick or pendant" if is_agv
                 else "standby  —  ○ on the pendant to activate",
            fg=Ink.ai if is_agv else Ink.text_3)
        self.link_dot.set(self.rt.agv.online, Ink.wakatake)
        cfg(self.link_lbl, text="online" if self.rt.agv.online else "offline",
            fg=Ink.text_2 if self.rt.agv.online else Ink.text_4)
        cfg(self.src_lbl, text="{}  {}".format(self.rt.agv.ip, st.get("source", "—")))
        cfg(self.telem["t"], text="{:+.2f}".format(t),
            fg=Ink.ai if abs(t) > 0.05 else Ink.text_3)
        cfg(self.telem["r"], text="{:+.2f}".format(r),
            fg=Ink.ai if abs(r) > 0.05 else Ink.text_3)
        cfg(self.telem["left"], text=str(st.get("left", "—")),
            fg=Ink.ai if "left" in st else Ink.text_3)
        cfg(self.telem["right"], text=str(st.get("right", "—")),
            fg=Ink.ai if "right" in st else Ink.text_3)
        self.win.after(120, self._tick)
