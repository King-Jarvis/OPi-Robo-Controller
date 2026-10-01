"""
Target — a cartesian goal and the inverse kinematics for it (V4 :2016-2141).

Changes from V4:
  · Orientation is a real roll/pitch/yaw of the tool, shown for the current
    pose too. V4 treated Rx=Ry=Rz=0 as "no orientation", but 0/0/0 is a
    perfectly good orientation (tool pointing up); "keep" now means keep.
  · The IK mode can be chosen. Arms with fewer than six joints can't hold an
    arbitrary orientation, so "auto" picks what the arm can do.
  · A rejected target says why (out of reach / joint limits, and how close).
  · MOVE LINEAR drives the tool along a straight line to the goal.
"""

import threading
import tkinter as tk

from ..theme import Ink, Type, SP1, SP2, SP3, SP4, SP5
from ..widgets import Overlay, InkButton, cfg, field_entry, section, track

MODES = [("auto", "auto"), ("pose", "full pose"), ("axis", "tool axis"),
         ("position", "position")]


class TargetPanel(Overlay):
    TITLE = "target"
    KANJI = "目標"
    GEOM = "680x580+160+10"

    def __init__(self, root, rt, osk):
        super().__init__(root)
        self.rt = rt
        self.osk = osk
        self.e = {}
        self.solution = None
        self.goal = None
        self.mode = "auto"
        self.keep_ori = False

    def build(self, body):
        # actions first, docked to the bottom, so a short panel can never
        # squeeze them off the window (V4's main screen does the same)
        acts = tk.Frame(body, bg=Ink.ground)
        acts.pack(fill="x", padx=SP5, pady=SP4, side="bottom")
        InkButton(acts, "SOLVE", command=self.solve, variant="primary",
                  font=Type.body_b, padx=SP5, pady=SP3).pack(side="left")
        self.btn_move = InkButton(acts, "MOVE", command=self.move, variant="solid",
                                  font=Type.body_b, padx=SP4, pady=SP3)
        self.btn_move.pack(side="left", padx=SP2)
        self.btn_lin = InkButton(acts, "MOVE LINEAR", command=self.move_linear,
                                 variant="solid", font=Type.body_b, padx=SP4, pady=SP3)
        self.btn_lin.pack(side="left")
        for b in (self.btn_move, self.btn_lin):
            b.set_enabled(False)
        InkButton(acts, "ARM IS AT PARK\nMARK HOMED", command=self._mark_homed,
                  variant="quiet", font=Type.micro, padx=SP3, pady=SP2).pack(side="right")

        section(body, "current tool centre point", "現在位置", pady=(SP4, SP2))
        self.cur = {}
        for axes, unit in ((("X", "Y", "Z"), "mm"), (("Rx", "Ry", "Rz"), "deg")):
            row = tk.Frame(body, bg=Ink.ground)
            row.pack(fill="x", padx=SP5)
            for ax in axes:
                cell = tk.Frame(row, bg=Ink.ground)
                cell.pack(side="left", expand=True, fill="x")
                tk.Label(cell, text=track(ax), font=Type.label, fg=Ink.text_3,
                         bg=Ink.ground).pack(anchor="w")
                l = tk.Label(cell, text="—", font=Type.data_lg if unit == "mm" else Type.data,
                             fg=Ink.text if unit == "mm" else Ink.text_2,
                             bg=Ink.ground, anchor="w")
                l.pack(anchor="w")
                self.cur[ax] = l
            tk.Label(row, text=unit, font=Type.label, fg=Ink.text_4,
                     bg=Ink.ground).pack(side="left", padx=SP3)

        section(body, "goal position", "位置", pady=(SP3, SP1))
        r1 = tk.Frame(body, bg=Ink.ground)
        r1.pack(fill="x", padx=SP5)
        for ax in ("X", "Y", "Z"):
            self._field(r1, ax, Ink.asagi)
        InkButton(r1, "← FROM\nCURRENT", command=self._from_current, variant="quiet",
                  font=Type.micro).pack(side="left", padx=SP3, pady=(SP4, 0))

        section(body, "goal orientation  (roll · pitch · yaw)", "姿勢", pady=(SP3, SP1))
        r2 = tk.Frame(body, bg=Ink.ground)
        r2.pack(fill="x", padx=SP5)
        for ax in ("Rx", "Ry", "Rz"):
            self._field(r2, ax, Ink.yamabuki)
        self.keep_btn = InkButton(r2, "KEEP\nCURRENT", command=self._toggle_keep,
                                  variant="quiet", font=Type.micro)
        self.keep_btn.pack(side="left", padx=SP3, pady=(SP4, 0))

        mrow = tk.Frame(body, bg=Ink.ground)
        mrow.pack(fill="x", padx=SP5, pady=(SP3, 0))
        tk.Label(mrow, text=track("solve for"), font=Type.label, fg=Ink.text_3,
                 bg=Ink.ground).pack(side="left", padx=(0, SP3))
        self.mode_btns = {}
        for key, label in MODES:
            b = InkButton(mrow, label, variant="quiet", padx=SP3, pady=SP1,
                          command=lambda k=key: self._set_mode(k))
            b.pack(side="left", padx=(0, SP1))
            self.mode_btns[key] = b
        self._set_mode(self.mode)

        section(body, "solution", "解", pady=(SP3, SP1))
        self.result = tk.Label(body, text="—", font=(Type.mono, 10), fg=Ink.text_3,
                               bg=Ink.ground, anchor="w", justify="left", wraplength=630)
        self.result.pack(fill="x", padx=SP5)

        self._tick()

    def _field(self, parent, ax, color):
        cell = tk.Frame(parent, bg=Ink.ground)
        cell.pack(side="left", padx=(0, SP4))
        tk.Label(cell, text=track(ax), font=Type.label, fg=color,
                 bg=Ink.ground).pack(anchor="w", pady=(0, SP1))
        wrap, e = field_entry(cell, "0.0", width=9, osk=self.osk)
        wrap.pack()
        self.e[ax] = e

    def _set_mode(self, key):
        self.mode = key
        for k, b in self.mode_btns.items():
            b.set_colors(fg=Ink.asagi if k == key else Ink.text_3)

    def _toggle_keep(self):
        self.keep_ori = not self.keep_ori
        self.keep_btn.set_colors(fg=Ink.asagi if self.keep_ori else Ink.text_2)
        for ax in ("Rx", "Ry", "Rz"):
            self.e[ax].config(fg=Ink.text_4 if self.keep_ori else Ink.text)

    def _from_current(self):
        tcp = self.rt.kin.fk(self.rt.bank.angles())
        if not tcp:
            return
        for ax, v in zip(("X", "Y", "Z", "Rx", "Ry", "Rz"), tcp.xyz + tcp.rpy_deg):
            self.e[ax].delete(0, "end")
            self.e[ax].insert(0, "{:.2f}".format(v))

    def _mark_homed(self):
        self.rt.assume_homed_at(self.rt.model.park_pose())
        self.rt.log("Arm declared at park pose and homed — switches not used", alarm=True)

    def solve(self):
        kin = self.rt.kin
        if not kin.ready.is_set():
            cfg(self.result, text="solver still loading", fg=Ink.yamabuki)
            return
        try:
            v = {k: float(self.e[k].get()) for k in ("X", "Y", "Z", "Rx", "Ry", "Rz")}
        except ValueError:
            cfg(self.result, text="invalid input", fg=Ink.shu)
            return
        q = self.rt.bank.angles()
        xyz = [v["X"], v["Y"], v["Z"]]
        rpy = None if self.keep_ori else [v["Rx"], v["Ry"], v["Rz"]]
        sol = kin.ik_xyz(xyz, rpy, seed_deg=q, mode=self.mode)
        if sol is None or not sol.ok:
            self.solution = self.goal = None
            for b in (self.btn_move, self.btn_lin):
                b.set_enabled(False)
            cfg(self.result, fg=Ink.shu,
                text="no solution — " + (sol.reason if sol else "solver not ready"))
            return
        self.solution = sol.deg
        self.goal = kin.fk(sol.deg).T
        homed = self.rt.bank.all_homed
        for b in (self.btn_move, self.btn_lin):
            b.set_enabled(homed)
        names = self.rt.model.names
        cfg(self.result, fg=Ink.wakatake, text="  ".join(
            "{} {:+.2f}".format(n, a) for n, a in zip(names, sol.deg)) +
            "\n{} mode · {:.2f} mm · {:.2f}°".format(sol.mode, sol.pos_err_mm, sol.ori_err_deg))
        self.rt.log("IK solved ({})".format(sol.mode))

    def _ready_to_move(self):
        if self.solution is None:
            return False
        if not self.rt.bank.all_homed:
            self.rt.log("Home the arm before moving to a target", alarm=True)
            return False
        return True

    def move(self):
        if self._ready_to_move():
            threading.Thread(target=self.rt.mover.move_all, args=(self.solution, True),
                             daemon=True).start()
            self.rt.log("Moving to solved target")

    def move_linear(self):
        if self._ready_to_move():
            threading.Thread(target=self.rt.cart.linear, args=(self.goal,),
                             daemon=True).start()
            self.rt.log("Linear move to solved target")

    def _tick(self):
        if not self.alive():
            return
        tcp = self.rt.kin.fk(self.rt.bank.angles())
        vals = (tcp.xyz + tcp.rpy_deg) if tcp else [None] * 6
        for ax, v in zip(("X", "Y", "Z", "Rx", "Ry", "Rz"), vals):
            cfg(self.cur[ax], text="{:+.2f}".format(v) if v is not None else "—")
        self.win.after(250, self._tick)
