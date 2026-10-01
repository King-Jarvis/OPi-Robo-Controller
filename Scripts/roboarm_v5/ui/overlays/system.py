"""
System — telemetry, motor tuning, the robot description, faults, network.
V4 :2548-2871. Custom tab strip rather than ttk.Notebook, which cannot be
themed to match.

Changes from V4:
  · motors   one row per joint of whatever robot is loaded; one set of limits
             (kinematic degrees); park angle and dir-pin inversion editable;
             SAVE writes them back into the robot file.
  · robot    replaces "geometry": link lengths are no longer three globals but
             the robot file's DH table or URDF, shown here; a different robot
             file can be chosen (the app restarts onto it).
"""

import json
import os
import threading
import subprocess
import time
import tkinter as tk
from tkinter import filedialog, messagebox

from ...hw.fan import cpu_temp
from ...robot.loader import load_robot, save_joint_tuning, RobotFileError
from .. import telemetry
from ..theme import Ink, Type, SP1, SP2, SP3, SP4, SP5
from ..widgets import Overlay, InkButton, cfg, field_entry, hairline, section, track


class SystemMenu(Overlay):
    TITLE = "system"
    KANJI = "設定"
    GEOM = "940x640+50+30"
    RESIZE = (True, True)

    TABS = [("status", "状態"), ("motors", "軸"), ("robot", "機体"),
            ("faults", "警報"), ("network", "通信")]

    def __init__(self, root, rt, osk, on_switch_robot=None):
        super().__init__(root)
        self.rt = rt
        self.osk = osk
        self.on_switch_robot = on_switch_robot
        self.tab = "status"
        self.tab_btns = {}
        self.pages = {}
        self._alarm_len = -1
        self.start_time = time.time()

    def build(self, body):
        strip = tk.Frame(body, bg=Ink.ground)
        strip.pack(fill="x", padx=SP4, pady=SP3)
        for name, kanji in self.TABS:
            b = InkButton(strip, track(name), variant="quiet", padx=SP4, pady=SP2,
                          sub=kanji if Type.cjk else None,
                          command=lambda n=name: self.show(n))
            b.pack(side="left", padx=(0, SP2))
            self.tab_btns[name] = b
        hairline(body)

        self.holder = tk.Frame(body, bg=Ink.ground)
        self.holder.pack(fill="both", expand=True)
        for name, _ in self.TABS:
            p = tk.Frame(self.holder, bg=Ink.ground)
            self.pages[name] = p
            getattr(self, "_page_" + name)(p)
        self.show("status")
        self._tick()

    def show(self, name):
        self.tab = name
        for p in self.pages.values():
            p.pack_forget()
        self.pages[name].pack(fill="both", expand=True)
        for n, b in self.tab_btns.items():
            b.set_colors(bg=Ink.ground, fg=Ink.asagi if n == name else Ink.text_3,
                         hover_bg=Ink.hover, hover_fg=Ink.text, press=Ink.asagi)

    # ── status ────────────────────────────────────────────────────────
    def _page_status(self, p):
        self.si = {}
        grid = tk.Frame(p, bg=Ink.ground)
        grid.pack(fill="both", expand=True, padx=SP5, pady=SP4)
        rows = ["robot", "cpu temp", "fan", "load", "memory", "disk", "uptime",
                "session", "ip address", "solver", "homed", "e-stop",
                "axes moving", "faults logged"]
        for i, label in enumerate(rows):
            r = tk.Frame(grid, bg=Ink.ground)
            r.pack(fill="x")
            tk.Label(r, text=track(label), font=Type.label, fg=Ink.text_3,
                     bg=Ink.ground, width=22, anchor="w").pack(side="left", pady=SP1)
            v = tk.Label(r, text="—", font=Type.data, fg=Ink.text,
                         bg=Ink.ground, anchor="w")
            v.pack(side="left")
            self.si[label] = v
            if i < len(rows) - 1:
                hairline(grid, color=Ink.line)

    def _refresh_status(self):
        rt = self.rt
        um, tm = telemetry.get_memory()
        du, dp = telemetry.get_disk()
        sess = int(time.time() - self.start_time)
        temp = cpu_temp()
        fan = rt.fan
        kin_ok = rt.kin.ready.is_set()
        homed = rt.bank.all_homed
        estop = rt.bank.estop_active
        vals = {
            "robot": ("{}  ·  {} joints{}".format(rt.model.name, rt.model.dof,
                                                 "  ·  SIM" if rt.sim else ""), Ink.text),
            "cpu temp": ("{:.1f} °C".format(temp),
                         Ink.shu if temp > fan.t_hi else
                         Ink.yamabuki if temp > fan.t_lo else Ink.text),
            "fan": ("{}%{}".format(fan.speed, "  auto" if fan.auto else "  manual"), Ink.text),
            "load": (telemetry.get_load(), Ink.text),
            "memory": ("{} / {} MB".format(um, tm), Ink.text),
            "disk": ("{}  ({})".format(du, dp), Ink.text),
            "uptime": (telemetry.get_uptime(), Ink.text),
            "session": ("{}h {:02d}m".format(sess // 3600, (sess % 3600) // 60), Ink.text),
            "ip address": (telemetry.get_ip(), Ink.text),
            "solver": ("ready" if kin_ok else "failed" if rt.kin.error else "loading…",
                       Ink.wakatake if kin_ok else Ink.shu if rt.kin.error else Ink.yamabuki),
            "homed": ("yes" if homed else "no", Ink.wakatake if homed else Ink.yamabuki),
            "e-stop": ("ACTIVE" if estop else "clear", Ink.shu if estop else Ink.wakatake),
            "axes moving": (str(sum(1 for m in rt.bank if m.running)), Ink.text),
            "faults logged": (str(len(rt.log.alarms)),
                              Ink.yamabuki if rt.log.alarms else Ink.text_3),
        }
        for k, (v, c) in vals.items():
            cfg(self.si[k], text=v, fg=c)

    # ── motors ────────────────────────────────────────────────────────
    FIELDS = [("steps/deg", 8), ("max hz", 7), ("lo °", 7), ("hi °", 7), ("park °", 7)]

    def _page_motors(self, p):
        wrap = tk.Frame(p, bg=Ink.ground)
        wrap.pack(fill="both", expand=True)
        c = tk.Canvas(wrap, bg=Ink.ground, highlightthickness=0, bd=0)
        sb = tk.Scrollbar(wrap, orient="vertical", command=c.yview, bd=0,
                          bg=Ink.panel, troughcolor=Ink.ground, width=10,
                          relief="flat", highlightthickness=0)
        inner = tk.Frame(c, bg=Ink.ground)
        inner.bind("<Configure>", lambda e: c.configure(scrollregion=c.bbox("all")))
        c.create_window((0, 0), window=inner, anchor="nw")
        c.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        c.pack(side="left", fill="both", expand=True)

        tk.Label(inner, text="limits are kinematic degrees  ·  re-home after changing "
                 "steps/deg or invert", font=Type.micro, fg=Ink.text_4, bg=Ink.ground
                 ).pack(anchor="w", padx=SP5, pady=(SP3, 0))
        self.mc = {}
        for m in self.rt.bank:
            j = m.j
            block = tk.Frame(inner, bg=Ink.ground)
            block.pack(fill="x", padx=SP5, pady=(SP4, 0))
            tk.Label(block, text=j.name[:6], font=Type.data_lg, fg=Ink.asagi,
                     bg=Ink.ground, width=6, anchor="w").pack(side="left")
            es = {}
            vals = [j.steps_per_deg, int(j.max_freq), j.limits_deg[0], j.limits_deg[1],
                    j.home.park_deg]
            for (fl, fw), fv in zip(self.FIELDS, vals):
                cell = tk.Frame(block, bg=Ink.ground)
                cell.pack(side="left", padx=SP2)
                tk.Label(cell, text=track(fl), font=Type.micro, fg=Ink.text_4,
                         bg=Ink.ground).pack(anchor="w")
                wrap2, e = field_entry(cell, fv, width=fw, font=(Type.mono, 10), osk=self.osk)
                wrap2.pack()
                es[fl] = e
            inv = InkButton(block, "INVERT " + ("ON" if j.invert_dir else "OFF"),
                            variant="quiet", padx=SP2, font=Type.micro)
            inv._command = lambda b=inv, jj=j: self._toggle_invert(jj, b)
            inv.pack(side="left", padx=SP2, pady=(SP3, 0))
            InkButton(block, "APPLY", variant="primary", padx=SP3,
                      command=lambda jj=j, en=es: self._apply_motor(jj, en)
                      ).pack(side="left", padx=SP2, pady=(SP3, 0))
            self.mc[j.name] = es
            hairline(inner, pad=SP5, pady=(SP3, 0))

        foot = tk.Frame(inner, bg=Ink.ground)
        foot.pack(fill="x", padx=SP5, pady=SP5)
        InkButton(foot, "SAVE TO ROBOT FILE", variant="solid", padx=SP5, pady=SP3,
                  command=self._save_robot).pack(side="left")
        tk.Label(foot, text=self.rt.model.source or "(no file)", font=Type.micro,
                 fg=Ink.text_4, bg=Ink.ground).pack(side="left", padx=SP3)

    def _toggle_invert(self, j, btn):
        j.invert_dir = not j.invert_dir
        btn.set_label("INVERT " + ("ON" if j.invert_dir else "OFF"))
        m = self.rt.bank[j.name]
        m.homed = False
        self.rt.log("{} dir {} — joint must be re-homed".format(
            j.name, "inverted" if j.invert_dir else "normal"), alarm=True)

    def _apply_motor(self, j, es):
        try:
            spd = float(es["steps/deg"].get())
            hz = float(es["max hz"].get())
            lo, hi = float(es["lo °"].get()), float(es["hi °"].get())
            park = float(es["park °"].get())
            if spd <= 0 or hz <= 0 or not lo < hi or not lo <= park <= hi:
                raise ValueError("need steps/deg>0, hz>0, lo<hi, lo≤park≤hi")
        except ValueError as e:
            self.rt.log("{} tuning rejected: {}".format(j.name, e), alarm=True)
            return
        m = self.rt.bank[j.name]
        if spd != j.steps_per_deg and m.homed:
            m.homed = False
            self.rt.log("{} steps/deg changed — joint must be re-homed".format(j.name),
                        alarm=True)
        j.steps_per_deg, j.max_freq, j.limits_deg = spd, hz, (lo, hi)
        j.home.park_deg = park
        # the switch sits at the end of travel, so moving that end moves the datum
        datum = {"min": lo, "max": hi}.get(j.home.switch, j.home.datum_deg)
        if datum != j.home.datum_deg:
            j.home.datum_deg = datum
            if m.homed:
                m.homed = False
                self.rt.log("{} datum moved to {:+.1f}° — joint must be re-homed"
                            .format(j.name, datum), alarm=True)
        self.rt.kin.set_model(self.rt.model)       # IK limits follow
        self.rt.log("{} tuning applied".format(j.name))

    def _save_robot(self):
        if not self.rt.model.source:
            self.rt.log("This robot was not loaded from a file", alarm=True)
            return
        try:
            save_joint_tuning(self.rt.model)
            self.rt.save_config()
            self.rt.log("Saved tuning to " + os.path.basename(self.rt.model.source))
        except Exception as e:
            self.rt.log("Save failed: {}".format(e), alarm=True)

    # ── robot ─────────────────────────────────────────────────────────
    def _page_robot(self, p):
        m = self.rt.model
        section(p, "robot file", "機体", pady=(SP5, SP3))
        info = tk.Frame(p, bg=Ink.ground)
        info.pack(fill="x", padx=SP5)
        for label, val in (("name", m.name), ("file", m.source or "—"),
                           ("joints", "{}  ({})".format(m.dof, "  ".join(m.names))),
                           ("kinematics", m.kin_type.upper()),
                           ("ik tolerance", "{} mm  ·  {}°".format(m.ik_pos_tol_mm,
                                                                   m.ik_ori_tol_deg))):
            r = tk.Frame(info, bg=Ink.ground)
            r.pack(fill="x", pady=1)
            tk.Label(r, text=track(label), font=Type.label, fg=Ink.text_3, bg=Ink.ground,
                     width=16, anchor="w").pack(side="left")
            tk.Label(r, text=val, font=(Type.mono, 10), fg=Ink.text, bg=Ink.ground,
                     anchor="w").pack(side="left")

        section(p, "geometry", "寸法")
        w = tk.Frame(p, bg=Ink.void)
        w.pack(fill="both", expand=True, padx=SP5)
        t = tk.Text(w, bg=Ink.void, fg=Ink.text_2, font=(Type.mono, 9), relief="flat",
                    bd=0, highlightthickness=0, wrap="none", padx=SP3, pady=SP2, height=10)
        t.pack(fill="both", expand=True)
        t.insert("end", self._geometry_text())
        t.config(state="disabled")

        bar = tk.Frame(p, bg=Ink.ground)
        bar.pack(fill="x", padx=SP5, pady=SP4)
        InkButton(bar, "OPEN ROBOT FILE…", command=self._open_robot, variant="primary",
                  padx=SP4, pady=SP3).pack(side="left")
        tk.Label(bar, text="geometry is edited in the robot file (DH table or URDF)",
                 font=Type.micro, fg=Ink.text_4, bg=Ink.ground).pack(side="left", padx=SP4)

        section(p, "cooling", "冷却")
        cf = tk.Frame(p, bg=Ink.ground)
        cf.pack(fill="x", padx=SP5, pady=(0, SP4))
        fan = self.rt.fan
        self.fan_auto_btn = InkButton(cf, "AUTO  " + ("ON" if fan.auto else "OFF"),
                                      variant="solid", padx=SP4, pady=SP2,
                                      command=self._toggle_fan_auto)
        self.fan_auto_btn.pack(side="left")
        tk.Label(cf, text="idle {}%  ·  ramps {:.0f}→{:.0f} °C".format(
            fan.idle, fan.t_lo, fan.t_hi), font=Type.label, fg=Ink.text_4,
            bg=Ink.ground).pack(side="left", padx=SP4)

    def _geometry_text(self):
        src = self.rt.model.source
        try:
            with open(src) as f:
                d = json.load(f)
        except Exception:
            return "(robot file not readable)"
        k = d.get("kinematics", {})
        out = []
        if k.get("type", "dh") == "dh":
            out.append("{} DH".format(k.get("convention", "standard")))
            out.append("{:<6} {:>9} {:>8} {:>9} {:>8}".format("joint", "a mm", "α°", "d mm", "θ off°"))
            for n, r in zip(self.rt.model.names, k.get("dh", [])):
                out.append("{:<6} {:>9.2f} {:>8.1f} {:>9.2f} {:>8.1f}".format(
                    n[:6], r.get("a", 0), r.get("alpha", 0), r.get("d", 0),
                    r.get("theta_offset", 0)))
        else:
            out.append("URDF  {}   base {}   tip {}".format(
                k.get("file"), k.get("base", "(root)"), k.get("tip", "(leaf)")))
        tool = d.get("tool", {})
        out.append("")
        out.append("tool  xyz {}  rpy° {}".format(tool.get("xyz", [0, 0, 0]),
                                                  tool.get("rpy_deg", [0, 0, 0])))
        return "\n".join(out)

    def _open_robot(self):
        fp = filedialog.askopenfilename(
            parent=self.win, title="Robot file",
            filetypes=[("Robot file", "*.json"), ("All files", "*.*")],
            initialdir=os.path.dirname(self.rt.model.source or os.getcwd()))
        if not fp:
            return
        try:
            m = load_robot(fp)
        except (OSError, RobotFileError) as e:
            messagebox.showerror("Robot file rejected", str(e), parent=self.win)
            return
        if not messagebox.askyesno(
                "Switch robot", "Load '{}' ({} joints)?\nThe controller restarts and the "
                "arm must be homed again.".format(m.name, m.dof), parent=self.win):
            return
        if self.on_switch_robot:
            self.on_switch_robot(fp)

    def _toggle_fan_auto(self):
        fan = self.rt.fan
        fan.auto = not fan.auto
        self.fan_auto_btn.set_label("AUTO  " + ("ON" if fan.auto else "OFF"))
        self.rt.log("Fan control: " + ("automatic" if fan.auto else "manual"))

    # ── faults ────────────────────────────────────────────────────────
    def _page_faults(self, p):
        bar = tk.Frame(p, bg=Ink.ground)
        bar.pack(fill="x", padx=SP4, pady=SP3)
        InkButton(bar, "CLEAR HISTORY", variant="quiet", padx=SP4,
                  command=lambda: (self.rt.log.clear_alarms(), self._refresh_faults(True))
                  ).pack(side="right")
        w = tk.Frame(p, bg=Ink.void)
        w.pack(fill="both", expand=True, padx=SP4, pady=(0, SP4))
        self.fault_text = tk.Text(w, bg=Ink.void, fg=Ink.beni, font=(Type.mono, 9),
                                  relief="flat", bd=0, highlightthickness=0,
                                  state="disabled", wrap="none", padx=SP3, pady=SP3)
        sb = tk.Scrollbar(w, orient="vertical", command=self.fault_text.yview,
                          bd=0, bg=Ink.panel, troughcolor=Ink.void, width=10,
                          relief="flat", highlightthickness=0)
        self.fault_text.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.fault_text.pack(fill="both", expand=True)

    def _refresh_faults(self, force=False):
        alarms = self.rt.log.alarms
        if len(alarms) == self._alarm_len and not force:
            return
        self._alarm_len = len(alarms)
        self.fault_text.config(state="normal")
        self.fault_text.delete("1.0", "end")
        self.fault_text.insert("end", "\n".join(alarms) if alarms else "no faults recorded")
        self.fault_text.config(state="disabled")
        self.fault_text.see("end")

    # ── network ───────────────────────────────────────────────────────
    def _page_network(self, p):
        bar = tk.Frame(p, bg=Ink.ground)
        bar.pack(fill="x", padx=SP4, pady=SP3)
        tk.Label(bar, text=track("agv endpoint"), font=Type.label, fg=Ink.text_3,
                 bg=Ink.ground).pack(side="left")
        wrap, self.agv_ip_e = field_entry(bar, self.rt.agv.ip, width=16, osk=self.osk)
        wrap.pack(side="left", padx=SP3)
        InkButton(bar, "APPLY", variant="primary", padx=SP4,
                  command=self._apply_net).pack(side="left")
        InkButton(bar, "REFRESH", variant="quiet", padx=SP4,
                  command=lambda: self._refresh_net(True)).pack(side="right")
        w = tk.Frame(p, bg=Ink.void)
        w.pack(fill="both", expand=True, padx=SP4, pady=(0, SP4))
        self.net_text = tk.Text(w, bg=Ink.void, fg=Ink.text_2, font=(Type.mono, 9),
                                relief="flat", bd=0, highlightthickness=0,
                                state="disabled", wrap="none", padx=SP3, pady=SP3)
        sb = tk.Scrollbar(w, orient="vertical", command=self.net_text.yview,
                          bd=0, bg=Ink.panel, troughcolor=Ink.void, width=10,
                          relief="flat", highlightthickness=0)
        self.net_text.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.net_text.pack(fill="both", expand=True)
        self._net_loaded = False

    def _apply_net(self):
        v = self.agv_ip_e.get().strip()
        if v:
            self.rt.agv.ip = v
            self.rt.save_config()
            self.rt.log("AGV endpoint set to " + v)

    def _refresh_net(self, force=False):
        if self._net_loaded and not force:
            return
        self._net_loaded = True

        def worker():
            try:
                addr = subprocess.check_output(["ip", "-br", "addr"], timeout=3).decode()
            except Exception:
                addr = "ip addr unavailable"
            try:
                route = subprocess.check_output(["ip", "route"], timeout=3).decode()
            except Exception:
                route = "ip route unavailable"
            out = "INTERFACES\n{}\nROUTES\n{}".format(addr, route)
            if self.alive():
                self.win.after(0, lambda: self._set_net(out))

        threading.Thread(target=worker, daemon=True).start()

    def _set_net(self, s):
        if not self.alive():
            return
        self.net_text.config(state="normal")
        self.net_text.delete("1.0", "end")
        self.net_text.insert("end", s)
        self.net_text.config(state="disabled")

    def _tick(self):
        if not self.alive():
            return
        if self.tab == "status":
            self._refresh_status()
        elif self.tab == "faults":
            self._refresh_faults()
        elif self.tab == "network":
            self._refresh_net()
        self.win.after(1000, self._tick)
