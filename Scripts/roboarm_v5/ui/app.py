"""
Main screen — V4 :2874-3422, built from the robot model instead of six
hardcoded joints.

Fukinsei — a wide data field on the left against a narrow rail on the right.
Weight sits left; the clock drifts to the far edge.

The field holds two row sets and shows the one that fits the mode:
  joint rows      one per joint of the loaded robot (JOINT / AGV)
  cartesian rows  X Y Z plus the rotations this arm can actually command
                  (WORLD / TOOL). V4 relabelled its six joint rows as
                  X…Rz, which only works when there happen to be six joints.

Three update tiers: 100 ms motion, 300 ms log, 1 s system telemetry.
"""

import time
import tkinter as tk

from ..hw.fan import cpu_temp
from .theme import Ink, Type, Metrics, SP1, SP2, SP3, SP4, SP5, SP6
from .widgets import (InkButton, InkSlider, TravelBar, Dot, cfg, hairline, vrule,
                      section, track)
from .overlays.osk import OSK
from .overlays.agv_panel import AGVPanel
from .overlays.target import TargetPanel
from .overlays.program import ProgramEditor
from .overlays.system import SystemMenu

CART_JOG_MM = 8.0
CART_JOG_DEG = 3.0
CART_REPEAT_MS = 90


def cart_axes(dof):
    """Cartesian axes an arm with this many joints can be jogged along."""
    if dof >= 6:
        return ["X", "Y", "Z", "Rx", "Ry", "Rz"]
    if dof == 5:
        return ["X", "Y", "Z", "Rx", "Ry"]
    return ["X", "Y", "Z"]


class RoboArm:
    def __init__(self, root, rt, on_switch_robot=None, fullscreen=True):
        self.root = root
        self.rt = rt
        self.model = rt.model
        root.title("ROBOARM V5 — " + self.model.name)
        root.configure(bg=Ink.ground)
        if fullscreen:
            root.attributes("-fullscreen", True)

        self.osk = OSK(root)
        self.system = SystemMenu(root, rt, self.osk, on_switch_robot=on_switch_robot)
        self.program = ProgramEditor(root, rt, self.osk)
        self.target = TargetPanel(root, rt, self.osk)
        self.agv_panel = AGVPanel(root, rt)

        self.jw = []                      # joint row widgets, by index
        self.cw = {}                      # cartesian row widgets, by axis
        self.axes = cart_axes(self.model.dof)
        self._cart_hold = None
        self._cart_after = None
        self._log_seen = 0
        self._fk_key = None
        self._fk = None
        self._tick_n = 0
        self._shown = None
        self._name_w = max(4, min(8, max(len(n) for n in self.model.names)))

        self._header()
        hairline(root)
        # Fixed chrome top and bottom first. An expanding body packed before
        # them takes the whole window and pushes the footer off the edge.
        self._command_bar()
        hairline(root, side="bottom")
        body = tk.Frame(root, bg=Ink.ground)
        body.pack(fill="both", expand=True)
        self._rail(body)
        vrule(body, side="right")
        self._field(body)

        self._bind_keys()
        rt.pendant.start()
        rt.log("ROBOARM V5 ready")
        rt.log("Home the arm before cartesian modes or program playback")
        self._tick()
        self._tick_log()
        self._tick_slow()

    @property
    def pendant(self):
        return self.rt.pendant

    # ── header ────────────────────────────────────────────────────────
    def _header(self):
        bar = tk.Frame(self.root, bg=Ink.raised, height=64)
        bar.pack(fill="x")
        bar.pack_propagate(False)

        left = tk.Frame(bar, bg=Ink.raised)
        left.pack(side="left", padx=(SP6, 0))
        tk.Label(left, text=track("roboarm"), font=(Type.sans, 14),
                 fg=Ink.text, bg=Ink.raised).pack(side="left")
        tk.Label(left, text="{}{}".format(self.model.name, "  · sim" if self.rt.sim else ""),
                 font=Type.label, fg=Ink.text_3, bg=Ink.raised).pack(side="left", padx=SP4)

        self.mode_chip = tk.Label(bar, text=track("joint"), font=Type.label,
                                  fg=Ink.void, bg=Ink.asagi, padx=SP4, pady=SP1)
        self.mode_chip.pack(side="left", padx=SP5)
        self.sel_chip = tk.Label(bar, text=self.model.names[0], font=Type.data,
                                 fg=Ink.text_3, bg=Ink.raised)
        self.sel_chip.pack(side="left")

        self.clock = tk.Label(bar, text="", font=(Type.mono, 18), fg=Ink.text_2,
                              bg=Ink.raised)
        self.clock.pack(side="right", padx=SP6)

        pill = tk.Frame(bar, bg=Ink.raised)
        pill.pack(side="right", padx=SP4)
        self.state_dot = Dot(pill, bg=Ink.raised, size=10)
        self.state_dot.pack(side="left", padx=(0, SP2))
        self.state_lbl = tk.Label(pill, text=track("not homed"), font=Type.label,
                                  fg=Ink.yamabuki, bg=Ink.raised)
        self.state_lbl.pack(side="left")

        self.solver_lbl = tk.Label(bar, text=track("solver loading"),
                                   font=Type.micro, fg=Ink.text_4, bg=Ink.raised)
        self.solver_lbl.pack(side="right", padx=SP4)

        pen = tk.Frame(bar, bg=Ink.raised)
        pen.pack(side="right", padx=SP4)
        self.pen_dot = Dot(pen, bg=Ink.raised)
        self.pen_dot.pack(side="left", padx=(0, SP2))
        self.pen_lbl = tk.Label(pen, text=track("pendant"), font=Type.micro,
                                fg=Ink.text_4, bg=Ink.raised)
        self.pen_lbl.pack(side="left")

    # ── left field ────────────────────────────────────────────────────
    def _field(self, parent):
        f = tk.Frame(parent, bg=Ink.ground)
        f.pack(side="left", fill="both", expand=True)

        self.axis_hdr = section(f, "axes", "軸", pady=Metrics.sec_pady, bg=Ink.ground)
        self.axis_hint = tk.Label(self.axis_hdr, text="", font=Type.micro,
                                  fg=Ink.text_4, bg=Ink.ground)
        self.axis_hint.pack(side="right", padx=SP5)

        self.rows_holder = tk.Frame(f, bg=Ink.ground)
        self.rows_holder.pack(fill="x", padx=SP4)
        self.joint_rows = tk.Frame(self.rows_holder, bg=Ink.ground)
        self.cart_rows = tk.Frame(self.rows_holder, bg=Ink.ground)
        for i, m in enumerate(self.rt.bank):
            self._joint_row(self.joint_rows, m, i)
        for ax in self.axes:
            self._cart_row(self.cart_rows, ax)

        section(f, "tool centre point", "先端位置", pady=Metrics.sec_pady, bg=Ink.ground)
        tcp = tk.Frame(f, bg=Ink.ground)
        tcp.pack(fill="x", padx=SP5)
        self.tcp_lbls = {}
        for ax in ("X", "Y", "Z"):
            cell = tk.Frame(tcp, bg=Ink.ground)
            cell.pack(side="left", padx=(0, SP6))
            tk.Label(cell, text=track(ax), font=Type.label, fg=Ink.text_3,
                     bg=Ink.ground).pack(anchor="w")
            l = tk.Label(cell, text="—", font=Metrics.tcp_font, fg=Ink.text,
                         bg=Ink.ground, anchor="w", width=10)
            l.pack(anchor="w")
            self.tcp_lbls[ax] = l
        tk.Label(tcp, text="mm", font=Type.label, fg=Ink.text_4,
                 bg=Ink.ground).pack(side="left", pady=(SP4, 0))

        section(f, "log", "記録", pady=Metrics.sec_pady, bg=Ink.ground)
        lw = tk.Frame(f, bg=Ink.void)
        lw.pack(fill="both", expand=True, padx=SP4, pady=(0, SP4))
        self.log_text = tk.Text(lw, bg=Ink.void, fg=Ink.text_3,
                                font=(Type.mono, 9), relief="flat", bd=0,
                                highlightthickness=0, state="disabled",
                                wrap="none", padx=SP3, pady=SP2,
                                height=Metrics.log_lines)
        self.log_text.pack(fill="both", expand=True)
        self.log_text.tag_configure("alarm", foreground=Ink.beni)
        self.log_text.tag_configure("plain", foreground=Ink.text_3)

    def _row_frame(self, parent):
        row = tk.Frame(parent, bg=Ink.panel, height=Metrics.row_h)
        row.pack(fill="x", pady=1)
        row.pack_propagate(False)
        mark = tk.Frame(row, bg=Ink.panel, width=3)
        mark.pack(side="left", fill="y")
        return row, mark

    def _jog_buttons(self, row, press, release):
        plus = InkButton(row, "+", variant="solid", font=(Type.sans, 15),
                         padx=Metrics.jog_padx, pady=SP2,
                         on_press=lambda: press(True), on_release=release)
        plus.pack(side="right", padx=(SP2, SP4))
        minus = InkButton(row, "−", variant="solid", font=(Type.sans, 15),
                          padx=Metrics.jog_padx, pady=SP2,
                          on_press=lambda: press(False), on_release=release)
        minus.pack(side="right")
        return plus, minus

    def _joint_row(self, parent, m, idx):
        row, mark = self._row_frame(parent)
        ax = tk.Label(row, text=m.name, font=Type.data_lg, fg=Ink.text_2,
                      bg=Ink.panel, width=self._name_w, cursor="hand2", anchor="w")
        ax.pack(side="left", padx=(SP4, SP2))
        ang = tk.Label(row, text="—", font=Type.data_lg, fg=Ink.text,
                       bg=Ink.panel, width=9, anchor="e")
        ang.pack(side="left")
        unit = tk.Label(row, text="°", font=Type.label, fg=Ink.text_3,
                        bg=Ink.panel, width=3, anchor="w")
        unit.pack(side="left")
        travel = TravelBar(row, width=Metrics.travel_w, bg=Ink.panel)
        travel.pack(side="left", padx=SP4)
        state = tk.Label(row, text="", font=Type.label, fg=Ink.text_4,
                         bg=Ink.panel, width=Metrics.state_w, anchor="w")
        state.pack(side="left", padx=SP2)

        # jog on the right edge — thumbs reach there on a panel-mounted screen
        plus, minus = self._jog_buttons(
            row, lambda pos, i=idx: self._jog_joint(i, pos),
            lambda i=idx: self._jog_joint_stop(i))
        spd_lbl = tk.Label(row, text="{}%".format(m.speed), font=Type.body, fg=Ink.text_3,
                           bg=Ink.panel, width=5, anchor="e")
        spd_lbl.pack(side="right", padx=SP2)
        slider = InkSlider(row, 1, 100, m.speed, width=Metrics.slider_w, bg=Ink.panel,
                           command=lambda v, i=idx: self._set_speed(i, v))
        slider.pack(side="right")

        for w in (row, ax, ang, unit, state):
            w.bind("<Button-1>", lambda e, i=idx: self.pendant.select(index=i))
        self.jw.append(dict(row=row, mark=mark, ax=ax, ang=ang, unit=unit,
                            travel=travel, state=state, spd_lbl=spd_lbl,
                            slider=slider, plus=plus, minus=minus))

    def _cart_row(self, parent, axis):
        row, mark = self._row_frame(parent)
        ax = tk.Label(row, text=axis, font=Type.data_lg, fg=Ink.text_2,
                      bg=Ink.panel, width=self._name_w, anchor="w")
        ax.pack(side="left", padx=(SP4, SP2))
        val = tk.Label(row, text="—", font=Type.data_lg, fg=Ink.text,
                       bg=Ink.panel, width=9, anchor="e")
        val.pack(side="left")
        unit = tk.Label(row, text="mm" if axis in "XYZ" else "°", font=Type.label,
                        fg=Ink.text_3, bg=Ink.panel, width=3, anchor="w")
        unit.pack(side="left")
        state = tk.Label(row, text="", font=Type.label, fg=Ink.text_4,
                         bg=Ink.panel, anchor="w")
        state.pack(side="left", padx=SP4)
        self._jog_buttons(row, lambda pos, a=axis: self._jog_cart(a, pos),
                          self._jog_cart_stop)
        self.cw[axis] = dict(row=row, mark=mark, ax=ax, val=val, unit=unit, state=state)

    # ── right rail ────────────────────────────────────────────────────
    def _rail(self, parent):
        r = tk.Frame(parent, bg=Ink.ground, width=Metrics.rail_w)
        r.pack(side="right", fill="y")
        r.pack_propagate(False)

        # E-STOP is claimed first so it can never be squeezed off a short
        # panel by the readouts above it. Safety outranks layout.
        stop_zone = tk.Frame(r, bg=Ink.ground)
        stop_zone.pack(side="bottom", fill="x", pady=Metrics.stop_pady)
        self.estop_btn = InkButton(stop_zone, track("stop"), variant="danger",
                                   font=(Type.sans, 16), pady=Metrics.stop_pady,
                                   command=self._estop_button)
        self.estop_btn.pack(fill="x", padx=SP4)
        self.estop_note = tk.Label(stop_zone, text="", font=Type.micro,
                                   fg=Ink.text_4, bg=Ink.ground)
        self.estop_note.pack(pady=(SP1, 0))

        pad = tk.Frame(r, bg=Ink.ground)
        pad.pack(fill="x", pady=(Metrics.rule_pady, 0))
        # long joint names ("Shoulder") get a smaller face so they still fit the rail
        self._hero_small = (Metrics.hero_font[0], max(14, Metrics.hero_font[1] * 9 // 20))
        self.hero = tk.Label(pad, text=self.model.names[0][:10], font=Metrics.hero_font,
                             fg=Ink.asagi, bg=Ink.ground)
        self.hero.pack()
        self.hero_sub = tk.Label(pad, text="", font=Type.rail, fg=Ink.text_4,
                                 bg=Ink.ground)
        self.hero_sub.pack(pady=(0, Metrics.rule_pady))

        self.rail_rows = {}
        for key in ("angle", "speed", "state"):
            row = tk.Frame(r, bg=Ink.ground)
            row.pack(fill="x", padx=SP5, pady=SP1)
            tk.Label(row, text=track(key), font=Type.label, fg=Ink.text_3,
                     bg=Ink.ground).pack(side="left")
            v = tk.Label(row, text="—", font=Type.data, fg=Ink.text, bg=Ink.ground)
            v.pack(side="right")
            self.rail_rows[key] = v

        hairline(r, pad=SP5, pady=Metrics.rule_pady)

        ov = tk.Frame(r, bg=Ink.ground)
        ov.pack(fill="x", padx=SP5)
        tk.Label(ov, text=track("override"), font=Type.label, fg=Ink.text_3,
                 bg=Ink.ground).pack(side="left")
        self.ovr_lbl = tk.Label(ov, text="30%", font=Type.data, fg=Ink.text,
                                bg=Ink.ground)
        self.ovr_lbl.pack(side="right")
        self.ovr_slider = InkSlider(r, 1, 100, 30, width=Metrics.rail_w - 2 * SP5,
                                    bg=Ink.ground, command=self._set_override)
        self.ovr_slider.pack(padx=SP5, pady=(SP1, Metrics.rule_pady))

        fn = tk.Frame(r, bg=Ink.ground)
        fn.pack(fill="x", padx=SP5)
        tk.Label(fn, text=track("cooling"), font=Type.label, fg=Ink.text_3,
                 bg=Ink.ground).pack(side="left")
        self.fan_lbl = tk.Label(fn, text="{}%".format(self.rt.fan.speed), font=Type.data,
                                fg=Ink.text, bg=Ink.ground)
        self.fan_lbl.pack(side="right")
        self.fan_slider = InkSlider(r, 0, 100, self.rt.fan.idle,
                                    width=Metrics.rail_w - 2 * SP5,
                                    bg=Ink.ground, command=self._set_fan_manual)
        self.fan_slider.pack(padx=SP5, pady=(SP1, SP2))
        self.temp_lbl = tk.Label(r, text="—", font=Type.label, fg=Ink.text_4,
                                 bg=Ink.ground)
        self.temp_lbl.pack(padx=SP5, anchor="w")

        hairline(r, pad=SP5, pady=Metrics.rule_pady)

    # ── command bar ───────────────────────────────────────────────────
    def _command_bar(self):
        bar = tk.Frame(self.root, bg=Ink.raised, height=60)
        bar.pack(fill="x", side="bottom")
        bar.pack_propagate(False)
        items = [("home", "原点", self.rt.homing.start),
                 ("mode", "座標", self._cycle_mode),
                 ("target", "目標", self.target.toggle),
                 ("program", "教示", self.program.toggle),
                 ("agv", "搬送", self.agv_panel.toggle),
                 ("system", "設定", self.system.toggle)]
        for i, (label, kanji, cmd) in enumerate(items):
            b = InkButton(bar, track(label), command=cmd, variant="quiet",
                          font=Type.body, padx=SP5, pady=SP2,
                          sub="F{}  {}".format(i + 1, kanji) if Type.cjk
                              else "F{}".format(i + 1))
            b.pack(side="left", fill="y", padx=(SP4 if i == 0 else SP2, 0))
        InkButton(bar, track("exit"), command=self.quit_app, variant="quiet",
                  font=Type.body, padx=SP5, pady=SP2, sub="esc"
                  ).pack(side="right", padx=SP4)
        InkButton(bar, track("keyboard"), command=self.osk.toggle, variant="quiet",
                  font=Type.body, padx=SP4, pady=SP2, sub="on-screen"
                  ).pack(side="right", padx=SP2)

    def _bind_keys(self):
        r = self.root
        r.bind("<Escape>", lambda e: self.quit_app())
        r.bind("<F1>", lambda e: self.rt.homing.start())
        r.bind("<F2>", lambda e: self._cycle_mode())
        r.bind("<F3>", lambda e: self.target.toggle())
        r.bind("<F4>", lambda e: self.program.toggle())
        r.bind("<F5>", lambda e: self.agv_panel.toggle())
        r.bind("<F6>", lambda e: self.system.toggle())
        r.bind("<space>", lambda e: self._stop_all("Keyboard stop"))

    # ── actions ───────────────────────────────────────────────────────
    def _stop_all(self, why):
        self.rt.mover.cancel_all()
        self.rt.playback.running = False
        self.rt.bank.stop_all(why, alarm=False)

    def _set_speed(self, i, v):
        self.rt.bank[i].set_speed(v)
        cfg(self.jw[i]["spd_lbl"], text="{}%".format(v))

    def _set_override(self, v):
        self.rt.bank.set_all_speeds(v)
        cfg(self.ovr_lbl, text="{}%".format(v))
        for w in self.jw:
            w["slider"].set(v)
            cfg(w["spd_lbl"], text="{}%".format(v))

    def _set_fan_manual(self, v):
        fan = self.rt.fan
        if fan.auto:
            fan.auto = False
            self.rt.log("Fan control: manual")
        fan.set(v)
        cfg(self.fan_lbl, text="{}%".format(v))

    def _jog_joint(self, i, positive):
        self.pendant.selected = i
        if self.pendant.mode == "JOINT":
            why = self.rt.bank[i].start(positive)
            if why and why != "estop":
                self.rt.log("{} jog refused ({})".format(self.rt.bank[i].name, why))

    def _jog_joint_stop(self, i):
        if self.pendant.mode == "JOINT":
            self.rt.bank[i].stop()

    def _jog_cart(self, axis, positive):
        if not self.rt.bank.all_homed:
            self.rt.log("Cartesian jog needs a homed arm", alarm=True)
            return
        self._cart_hold = (axis, 1 if positive else -1)
        self._cart_repeat()

    def _cart_repeat(self):
        if not self._cart_hold:
            return
        axis, sign = self._cart_hold
        d_mm, d_deg = [0.0] * 3, [0.0] * 3
        if axis in ("X", "Y", "Z"):
            d_mm["XYZ".index(axis)] = sign * CART_JOG_MM
        else:
            d_deg[("Rx", "Ry", "Rz").index(axis)] = sign * CART_JOG_DEG
        frame = "tool" if self.pendant.mode == "TOOL" else "world"
        self.rt.cart.jog(d_mm, d_deg, frame=frame)
        self._cart_after = self.root.after(CART_REPEAT_MS, self._cart_repeat)

    def _jog_cart_stop(self):
        self._cart_hold = None
        if self._cart_after:
            self.root.after_cancel(self._cart_after)
            self._cart_after = None

    def _cycle_mode(self):
        order = ["JOINT", "WORLD", "TOOL", "AGV"]
        mode = order[(order.index(self.pendant.mode) + 1) % len(order)]
        self.pendant.set_mode(mode)
        if mode in ("WORLD", "TOOL") and not self.rt.bank.all_homed:
            self.rt.log("Cartesian modes need a homed arm", alarm=True)

    def _estop_button(self):
        if self.rt.bank.estop_active and self.rt.safety.latching:
            self.rt.safety.reset()
        else:
            self._stop_all("Screen stop")

    def quit_app(self):
        try:
            self.rt.log("Shutting down")
            self.rt.save_config()
            self.rt.shutdown()
        except Exception:
            pass
        self.root.destroy()

    # ── update: motion tier, 100 ms ───────────────────────────────────
    def _show_rows(self, cart):
        if self._shown == cart:
            return
        self._shown = cart
        (self.joint_rows if cart else self.cart_rows).pack_forget()
        (self.cart_rows if cart else self.joint_rows).pack(fill="x")

    def _status(self, m, estop):
        if estop:
            return "e-stop", Ink.beni
        if m.at_limit:
            return "limit", Ink.yamabuki
        if m.soft_block:
            return "soft limit", Ink.yamabuki
        if m.running:
            return ("moving +" if m.moving_positive else "moving −"), Ink.asagi
        if not m.homed:
            return "not homed", Ink.text_4
        return "idle", Ink.text_4

    def _tick(self):
        self._tick_n += 1
        rt, bank = self.rt, self.rt.bank
        mode = self.pendant.mode
        sel = self.pendant.selected
        estop = bank.estop_active
        cart = mode in ("WORLD", "TOOL")
        self._show_rows(cart)

        # header
        chip_bg = {"JOINT": Ink.asagi, "WORLD": Ink.ai,
                   "TOOL": Ink.wakatake, "AGV": Ink.ai}[mode]
        cfg(self.mode_chip, text=track(mode.lower()), bg=chip_bg)
        cfg(self.sel_chip, text=bank[sel].name if mode == "JOINT" else "—")

        if estop:
            st = ("e-stop", Ink.beni, True, Ink.shu)
        elif rt.homing.active:
            st = ("homing", Ink.yamabuki, True, Ink.yamabuki)
        elif rt.playback.running:
            st = ("running", Ink.asagi, True, Ink.asagi)
        elif bank.all_homed:
            st = ("ready", Ink.wakatake, True, Ink.wakatake)
        else:
            st = ("not homed", Ink.yamabuki, False, Ink.text_4)
        cfg(self.state_lbl, text=track(st[0]), fg=st[1])
        self.state_dot.set(st[2], st[3])

        # e-stop control reads RESET only while latched
        if estop and rt.safety.latching:
            self.estop_btn.set_label(track("reset"))
            cfg(self.estop_note,
                text="loop open" if bank.estop_pin_raw else "release, then reset",
                fg=Ink.beni)
        else:
            self.estop_btn.set_label(track("stop"))
            cfg(self.estop_note, text="all axes", fg=Ink.text_4)

        cfg(self.axis_hint, text={
            "JOINT": "jog drives the selected joint",
            "WORLD": "jog moves the tool in world axes",
            "TOOL":  "jog moves the tool in its own frame",
            "AGV":   "arm jog disabled while driving",
        }[mode])

        angles = bank.angles()
        if cart:
            self._tick_cart(angles)
        else:
            self._tick_joints(angles, mode, sel, estop)

        # rail
        m = bank[sel]
        hero = m.name if mode != "AGV" else "AGV"
        cfg(self.hero, text=hero[:10], fg=Ink.ai if mode == "AGV" else Ink.asagi,
            font=Metrics.hero_font if len(hero) <= 4 else self._hero_small)
        cfg(self.hero_sub, text=Type.jp("軸", "axis") if mode != "AGV"
            else Type.jp("搬送車", "vehicle"))
        cfg(self.rail_rows["angle"], text="{:+.2f}°".format(angles[sel]))
        cfg(self.rail_rows["speed"], text="{}%".format(m.speed))
        if estop:
            cfg(self.rail_rows["state"], text="e-stop", fg=Ink.beni)
        elif mode == "AGV":
            cfg(self.rail_rows["state"],
                text="{:+.2f} / {:+.2f}".format(rt.agv.translate, rt.agv.rotate), fg=Ink.ai)
        elif m.running:
            cfg(self.rail_rows["state"], text="moving", fg=Ink.asagi)
        elif m.at_limit or m.soft_block:
            cfg(self.rail_rows["state"], text="at limit", fg=Ink.yamabuki)
        else:
            cfg(self.rail_rows["state"], text="idle", fg=Ink.text_3)

        self.pen_dot.set(self.pendant.connected, Ink.wakatake)
        cfg(self.pen_lbl, text=track("pendant"),
            fg=Ink.text_3 if self.pendant.connected else Ink.text_4)

        # tool centre point — only re-solved when a joint actually moved
        if self._tick_n % 3 == 0 or cart:
            fk = self._fk_for(angles)
            for i, ax in enumerate(("X", "Y", "Z")):
                cfg(self.tcp_lbls[ax],
                    text="{:+.2f}".format(fk.xyz[i]) if fk else "—",
                    fg=Ink.text if fk else Ink.text_4)

        if self.program.alive():
            self.program.refresh()

        self.root.after(100, self._tick)

    def _fk_for(self, angles):
        key = tuple(round(a, 2) for a in angles)
        if key != self._fk_key:
            self._fk_key = key
            self._fk = self.rt.kin.fk(list(key))
        return self._fk

    def _tick_joints(self, angles, mode, sel, estop):
        for i, (w, m) in enumerate(zip(self.jw, self.rt.bank)):
            selected = (i == sel) and mode != "AGV"
            bg = Ink.sel if selected else Ink.panel
            for k in ("row", "ax", "ang", "unit", "state", "spd_lbl"):
                cfg(w[k], bg=bg)
            cfg(w["mark"], bg=Ink.asagi if selected else bg)
            cfg(w["travel"], bg=bg)
            cfg(w["slider"], bg=bg)
            w["plus"].set_base_bg(bg)
            w["minus"].set_base_bg(bg)

            cfg(w["ax"], fg=Ink.asagi if selected else Ink.text_2)
            cfg(w["ang"], text="{:+.2f}".format(angles[i]),
                fg=Ink.text_4 if mode == "AGV" or not m.homed else Ink.text)
            cfg(w["spd_lbl"], text="{}%".format(m.speed))
            if w["slider"].get() != m.speed:
                w["slider"].set(m.speed)
            st, c = self._status(m, estop)
            cfg(w["state"], text=st, fg=c)
            if mode == "AGV" or not m.homed:
                w["travel"].blank()
            else:
                lo, hi = m.j.limits_deg
                w["travel"].set(angles[i], lo, hi, Ink.asagi if selected else Ink.text_3)

    def _tick_cart(self, angles):
        fk = self._fk_for(angles)
        vals = dict(zip(("X", "Y", "Z", "Rx", "Ry", "Rz"), fk.xyz + fk.rpy_deg)) if fk else {}
        holding = self._cart_hold[0] if self._cart_hold else None
        for ax, w in self.cw.items():
            v = vals.get(ax)
            cfg(w["val"], text="{:+.2f}".format(v) if v is not None else "—",
                fg=Ink.text if self.rt.bank.all_homed else Ink.text_4)
            active = (ax == holding)
            bg = Ink.sel if active else Ink.panel
            for k in ("row", "ax", "val", "unit", "state"):
                cfg(w[k], bg=bg)
            cfg(w["mark"], bg=Ink.asagi if active else bg)
            cfg(w["state"], text="jogging" if active else
                ("" if self.rt.bank.all_homed else "home first"),
                fg=Ink.asagi if active else Ink.text_4)

    # ── update: log tier, 300 ms, append only ─────────────────────────
    def _tick_log(self):
        log = self.rt.log
        if log.seq != self._log_seen:
            lines, self._log_seen = log.tail(self._log_seen)
            alarms = set(log.alarms[-40:])
            self.log_text.config(state="normal")
            for line in lines:
                self.log_text.insert("end", line + "\n",
                                     "alarm" if line in alarms else "plain")
            # keep the widget bounded — this runs for hours
            excess = int(self.log_text.index("end-1c").split(".")[0]) - 200
            if excess > 0:
                self.log_text.delete("1.0", "{}.0".format(excess + 1))
            self.log_text.config(state="disabled")
            self.log_text.see("end")
        self.root.after(300, self._tick_log)

    # ── update: system tier, 1 s ──────────────────────────────────────
    def _tick_slow(self):
        cfg(self.clock, text=time.strftime("%H:%M:%S"))
        fan = self.rt.fan
        temp = cpu_temp()
        fan.tick(temp)
        cfg(self.temp_lbl, text="cpu {:.1f} °C   ·   {}".format(
            temp, "auto" if fan.auto else "manual"),
            fg=Ink.shu if temp > fan.t_hi else
               Ink.yamabuki if temp > fan.t_lo + 15 else Ink.text_4)
        cfg(self.fan_lbl, text="{}%".format(fan.speed))
        if fan.auto and self.fan_slider.get() != fan.speed:
            self.fan_slider.set(fan.speed)

        kin = self.rt.kin
        if kin.ready.is_set():
            cfg(self.solver_lbl, text=track("solver ready"), fg=Ink.text_4)
        elif kin.error:
            cfg(self.solver_lbl, text=track("solver failed"), fg=Ink.shu)
        else:
            cfg(self.solver_lbl, text=track("solver loading"), fg=Ink.yamabuki)
        self.root.after(1000, self._tick_slow)
