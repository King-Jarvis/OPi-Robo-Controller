"""
Program editor — V4 :2144-2545. Teach points, edit them, play them back.
Columns and the point editor are sized to the robot's joint count.
"""

import os
import threading
import tkinter as tk
from tkinter import filedialog, messagebox

from ...io.programs import Program
from ..theme import Ink, Type, SP1, SP2, SP3, SP4, SP5
from ..widgets import (Overlay, InkButton, InkSlider, cfg, field_entry, hairline,
                       section, track)


class ProgramEditor(Overlay):
    TITLE = "program"
    KANJI = "教示"
    GEOM = "1000x640+30+30"
    RESIZE = (True, True)

    def __init__(self, root, rt, osk):
        super().__init__(root)
        self.rt = rt
        self.osk = osk
        self.step_mode = False
        self.sel = None
        self.speed = 50
        self._sig = None

    @property
    def program(self):
        return self.rt.program

    def build(self, body):
        tools = tk.Frame(body, bg=Ink.ground)
        tools.pack(fill="x", padx=SP4, pady=SP3)
        for label, cmd, variant in [
                ("NEW", self._new, "quiet"), ("OPEN", self._load, "quiet"),
                ("SAVE", self._save, "quiet"), ("SAVE AS", self._save_as, "quiet"),
                ("RECORD", self._record, "primary"), ("EDIT", self._edit, "solid"),
                ("UP", lambda: self._shift(-1), "solid"),
                ("DOWN", lambda: self._shift(1), "solid"),
                ("DELETE", self._delete, "danger"),
                ("REGISTERS", self._variables, "quiet")]:
            InkButton(tools, label, command=cmd, variant=variant,
                      padx=SP3).pack(side="left", padx=2)

        hairline(body)
        split = tk.Frame(body, bg=Ink.ground)
        split.pack(fill="both", expand=True)

        left = tk.Frame(split, bg=Ink.ground)
        left.pack(side="left", fill="both", expand=True, padx=(SP4, SP3), pady=SP3)
        tk.Label(left, text=self._header_line(), font=(Type.mono, 9),
                 fg=Ink.text_4, bg=Ink.ground, anchor="w"
                 ).pack(fill="x", padx=SP2)
        hairline(left, pady=(SP1, 0))
        lw = tk.Frame(left, bg=Ink.void)
        lw.pack(fill="both", expand=True)
        self.listbox = tk.Listbox(
            lw, bg=Ink.void, fg=Ink.text_2, font=(Type.mono, 9),
            selectbackground=Ink.sel, selectforeground=Ink.asagi,
            activestyle="none", relief="flat", bd=0, highlightthickness=0)
        sb = tk.Scrollbar(lw, orient="vertical", command=self.listbox.yview,
                          bg=Ink.panel, troughcolor=Ink.void, bd=0,
                          highlightthickness=0, relief="flat", width=10)
        self.listbox.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.listbox.pack(fill="both", expand=True, padx=SP2, pady=SP2)
        self.listbox.bind("<<ListboxSelect>>", self._on_select)
        self.listbox.bind("<Double-Button-1>", lambda e: self._edit())

        right = tk.Frame(split, bg=Ink.ground, width=270)
        right.pack(side="right", fill="y")
        right.pack_propagate(False)

        section(right, "playback", "再生", pady=(SP4, SP3))
        row = tk.Frame(right, bg=Ink.ground)
        row.pack(fill="x", padx=SP4)
        InkButton(row, "RUN", variant="primary", padx=SP4, pady=SP3,
                  command=lambda: self.rt.playback.start(self.program, self.speed, True,
                                                         step=self.step_mode)
                  ).pack(side="left")
        InkButton(row, "REVERSE", variant="solid", padx=SP3, pady=SP3,
                  command=lambda: self.rt.playback.start(self.program, self.speed, False,
                                                         step=self.step_mode)
                  ).pack(side="left", padx=SP2)
        InkButton(row, "STOP", variant="danger", padx=SP3, pady=SP3,
                  command=self.rt.playback.stop).pack(side="right")
        row2 = tk.Frame(right, bg=Ink.ground)
        row2.pack(fill="x", padx=SP4, pady=SP2)
        InkButton(row2, "SINGLE STEP", variant="quiet", padx=SP3,
                  command=lambda: self.rt.playback.start(self.program, self.speed, step=True)
                  ).pack(side="left")
        self.step_btn = InkButton(row2, "STEP MODE  OFF", variant="quiet",
                                  padx=SP3, command=self._toggle_step)
        self.step_btn.pack(side="right")

        sf = tk.Frame(right, bg=Ink.ground)
        sf.pack(fill="x", padx=SP4, pady=(SP4, 0))
        tk.Label(sf, text=track("speed"), font=Type.label, fg=Ink.text_3,
                 bg=Ink.ground).pack(side="left")
        self.spd_lbl = tk.Label(sf, text="50%", font=Type.data, fg=Ink.text,
                                bg=Ink.ground)
        self.spd_lbl.pack(side="right")
        InkSlider(right, 1, 100, 50, width=238, bg=Ink.ground,
                  command=self._set_speed).pack(padx=SP4, pady=SP2)

        section(right, "point", "点")
        self.detail = tk.Label(right, text="—", font=(Type.mono, 9),
                               fg=Ink.text_2, bg=Ink.ground, justify="left",
                               anchor="nw")
        self.detail.pack(fill="both", expand=True, padx=SP4, pady=SP2)

        self.status = tk.Label(body, text="", font=Type.label, fg=Ink.text_4,
                               bg=Ink.raised, anchor="w", padx=SP4, pady=SP2)
        self.status.pack(fill="x", side="bottom")
        self.refresh(force=True)

    def _set_speed(self, v):
        self.speed = v
        cfg(self.spd_lbl, text="{}%".format(v))

    def _toggle_step(self):
        self.step_mode = not self.step_mode
        self.step_btn.set_label("STEP MODE  " + ("ON" if self.step_mode else "OFF"))

    # ── list ──────────────────────────────────────────────────────────
    ROW_FMT = "{:>3}  {}  {:<7} {}  {:>4} {:>3}  {}"

    def _line(self, pt):
        return self.ROW_FMT.format(
            pt["index"], pt["move_type"], pt["label"],
            " ".join("{:+7.1f}".format(a) for a in pt["angles"]),
            "{}%".format(pt["speed_pct"]), pt["cnt"], pt["comment"])

    def _header_line(self):
        return self.ROW_FMT.format(
            "NO", "T", "POINT",
            " ".join("{:>7}".format(n[:7]) for n in self.rt.model.names),
            "SPD", "CNT", "NOTE")

    def refresh(self, force=False):
        if not self.alive():
            return
        sig = (len(self.program.points), self.program.modified,
               self.program.name, self.sel,
               tuple(id(p) for p in self.program.points))
        if sig == self._sig and not force:
            return
        self._sig = sig
        self.listbox.delete(0, "end")
        for pt in self.program.points:
            self.listbox.insert("end", self._line(pt))
        if self.sel is not None and self.sel < len(self.program.points):
            self.listbox.selection_set(self.sel)
            self.listbox.see(self.sel)
        cfg(self.status, text="{}   ·   {} points   ·   {}{}".format(
            self.program.name, len(self.program.points),
            self.program.filepath or "unsaved",
            "   ·   modified" if self.program.modified else ""),
            fg=Ink.yamabuki if self.program.modified else Ink.text_4)

    def _on_select(self, _e=None):
        s = self.listbox.curselection()
        if not s:
            return
        self.sel = s[0]
        self._detail(self.program.points[self.sel])

    def _detail(self, pt):
        tcp = pt.get("tcp")
        rpy = pt.get("rpy")
        lines = ["{}   {}".format(pt["label"],
                                  "joint" if pt["move_type"] == "J" else "linear"),
                 "speed {}%   cnt {}".format(pt["speed_pct"], pt["cnt"]),
                 pt["comment"] or "—", ""]
        lines += ["{:<4}{:+9.3f}".format(n[:4], a)
                  for n, a in zip(self.rt.model.names, pt["angles"])]
        if tcp:
            lines += [""] + ["{:<4}{:+9.3f}".format(k, v) for k, v in zip("XYZ", tcp)]
        if rpy:
            lines += ["{:<4}{:+9.3f}".format(k, v) for k, v in zip(("Rx", "Ry", "Rz"), rpy)]
        cfg(self.detail, text="\n".join(lines))

    # ── actions ───────────────────────────────────────────────────────
    def _record(self):
        pt = self.rt.record_point(move_type="J")
        self.sel = len(self.program.points) - 1
        self.refresh(force=True)
        self._detail(pt)

    def _delete(self):
        if self.sel is None:
            return
        if messagebox.askyesno("Delete", "Delete P[{}]?".format(self.sel + 1),
                               parent=self.win):
            self.program.delete_point(self.sel)
            self.sel = max(0, self.sel - 1) if self.program.points else None
            self.refresh(force=True)

    def _shift(self, delta):
        if self.sel is None:
            return
        if self.program.move_point(self.sel, delta):
            self.sel += delta
            self.refresh(force=True)

    def _new(self):
        if self.program.modified and not messagebox.askyesno(
                "Unsaved", "Discard changes?", parent=self.win):
            return
        self.rt.new_program()
        self.sel = None
        self.refresh(force=True)

    def _save(self):
        if not self.program.filepath:
            self._save_as()
        else:
            self.program.save()
            self.refresh(force=True)

    def _save_as(self):
        fp = filedialog.asksaveasfilename(
            parent=self.win, title="Save program", defaultextension=".json",
            filetypes=[("Robot program", "*.json"), ("All files", "*.*")],
            initialdir=os.path.expanduser("~"))
        if fp:
            self.program.name = os.path.splitext(os.path.basename(fp))[0]
            self.program.save(fp)
            self.refresh(force=True)

    def _load(self):
        if self.program.modified and not messagebox.askyesno(
                "Unsaved", "Discard changes?", parent=self.win):
            return
        fp = filedialog.askopenfilename(
            parent=self.win, title="Open program",
            filetypes=[("Robot program", "*.json"), ("All files", "*.*")],
            initialdir=os.path.expanduser("~"))
        if fp:
            p = Program(self.rt.model, self.rt.log)
            try:
                p.load(fp)
            except (OSError, ValueError) as e:
                messagebox.showerror("Cannot open", str(e), parent=self.win)
                return
            self.rt.program = p
            self.sel = None
            self.refresh(force=True)

    def _edit(self):
        if self.sel is None:
            return
        self._point_editor(self.program.points[self.sel])

    def _point_editor(self, pt):
        w = tk.Toplevel(self.win)
        w.title("Edit " + pt["label"])
        w.configure(bg=Ink.ground)
        w.geometry("480x{}+220+40".format(380 + 36 * self.rt.model.dof))
        w.resizable(False, False)
        w.bind("<Escape>", lambda e: w.destroy())
        head = tk.Frame(w, bg=Ink.raised, height=48)
        head.pack(fill="x"); head.pack_propagate(False)
        tk.Label(head, text=track("edit " + pt["label"]), font=Type.title,
                 fg=Ink.text, bg=Ink.raised).pack(side="left", padx=SP5)
        hairline(w)

        f = tk.Frame(w, bg=Ink.ground)
        f.pack(fill="both", expand=True, padx=SP5, pady=SP4)
        es = {}

        def field(label, value, row, fg=None):
            tk.Label(f, text=track(label), font=Type.label, fg=Ink.text_3,
                     bg=Ink.ground, anchor="w").grid(row=row, column=0,
                                                     sticky="w", pady=SP1)
            wrap, e = field_entry(f, value, width=12, fg=fg, osk=self.osk)
            wrap.grid(row=row, column=1, sticky="e", pady=SP1)
            es[label] = e

        mt = tk.StringVar(value=pt["move_type"])
        tk.Label(f, text=track("move"), font=Type.label, fg=Ink.text_3,
                 bg=Ink.ground).grid(row=0, column=0, sticky="w", pady=SP1)
        mf = tk.Frame(f, bg=Ink.ground)
        mf.grid(row=0, column=1, sticky="e")
        for val, lab in (("J", "joint"), ("L", "linear")):
            tk.Radiobutton(mf, text=lab, variable=mt, value=val, font=Type.body,
                           fg=Ink.text_2, bg=Ink.ground, selectcolor=Ink.void,
                           activebackground=Ink.ground, activeforeground=Ink.text,
                           bd=0, highlightthickness=0).pack(side="left")
        field("speed %", pt["speed_pct"], 1)
        field("cnt", pt["cnt"], 2)
        field("note", pt["comment"], 3)
        tk.Label(f, text=track("joint angles"), font=Type.label, fg=Ink.asagi,
                 bg=Ink.ground).grid(row=4, column=0, sticky="w", pady=(SP4, SP1))
        names = self.rt.model.names
        for i, n in enumerate(names):
            field(n, "{:.4f}".format(pt["angles"][i]), 5 + i)
        f.grid_columnconfigure(0, weight=1)

        def apply_changes():
            try:
                pt["move_type"] = mt.get()
                pt["speed_pct"] = int(es["speed %"].get())
                pt["cnt"] = int(es["cnt"].get())
                pt["comment"] = es["note"].get()
                for i, n in enumerate(names):
                    pt["angles"][i] = float(es[n].get())
                tcp = self.rt.kin.fk(pt["angles"])
                pt["tcp"], pt["rpy"] = (tcp.xyz, tcp.rpy_deg) if tcp else (None, None)
                self.program.modified = True
                self.refresh(force=True)
                self._detail(pt)
                self.rt.log("{} edited".format(pt["label"]))
                w.destroy()
            except Exception as ex:
                messagebox.showerror("Invalid", str(ex), parent=w)

        def from_current():
            for n, a in zip(names, self.rt.bank.angles()):
                es[n].delete(0, "end")
                es[n].insert(0, "{:.4f}".format(a))

        def go_here():
            if not self.rt.bank.all_homed:
                messagebox.showwarning("Not homed", "Home the arm first.", parent=w)
                return
            threading.Thread(target=self.rt.mover.move_all,
                             args=(list(pt["angles"]), True), daemon=True).start()

        bar = tk.Frame(w, bg=Ink.ground)
        bar.pack(fill="x", padx=SP5, pady=SP4)
        InkButton(bar, "APPLY", command=apply_changes, variant="primary",
                  padx=SP5, pady=SP3).pack(side="left")
        InkButton(bar, "TEACH FROM CURRENT", command=from_current, variant="quiet",
                  padx=SP3, pady=SP3).pack(side="left", padx=SP2)
        InkButton(bar, "MOVE HERE", command=go_here, variant="solid",
                  padx=SP3, pady=SP3).pack(side="left")
        InkButton(bar, "CANCEL", command=w.destroy, variant="quiet",
                  padx=SP3, pady=SP3).pack(side="right")

    def _variables(self):
        w = tk.Toplevel(self.win)
        w.title("Registers")
        w.configure(bg=Ink.ground)
        w.geometry("560x480+240+90")
        w.bind("<Escape>", lambda e: w.destroy())
        head = tk.Frame(w, bg=Ink.raised, height=48)
        head.pack(fill="x"); head.pack_propagate(False)
        tk.Label(head, text=track("registers"), font=Type.title, fg=Ink.text,
                 bg=Ink.raised).pack(side="left", padx=SP5)
        hairline(w)

        lw = tk.Frame(w, bg=Ink.void)
        lw.pack(fill="both", expand=True, padx=SP4, pady=SP4)
        lb = tk.Listbox(lw, bg=Ink.void, fg=Ink.text_2, font=(Type.mono, 9),
                        selectbackground=Ink.sel, selectforeground=Ink.asagi,
                        activestyle="none", relief="flat", bd=0,
                        highlightthickness=0)
        lb.pack(fill="both", expand=True, padx=SP2, pady=SP2)

        def refresh_vars():
            lb.delete(0, "end")
            for k, v in self.program.variables.items():
                val = v.get("value", "") if isinstance(v, dict) else v
                cmt = v.get("comment", "") if isinstance(v, dict) else ""
                if isinstance(val, dict):
                    val = "pose"
                lb.insert("end", "  {:<18} {:<20} {}".format(k, str(val)[:20], cmt))

        refresh_vars()
        af = tk.Frame(w, bg=Ink.ground)
        af.pack(fill="x", padx=SP4)
        fields = {}
        for label, wdt in (("name", 12), ("value", 12), ("note", 16)):
            cell = tk.Frame(af, bg=Ink.ground)
            cell.pack(side="left", padx=(0, SP3))
            tk.Label(cell, text=track(label), font=Type.micro, fg=Ink.text_4,
                     bg=Ink.ground).pack(anchor="w")
            wrap, e = field_entry(cell, "", width=wdt, osk=self.osk)
            wrap.pack()
            fields[label] = e

        def add_var():
            n = fields["name"].get().strip()
            if not n:
                return
            self.program.variables[n] = {"value": fields["value"].get().strip(),
                                         "comment": fields["note"].get().strip()}
            self.program.modified = True
            refresh_vars()

        def del_var():
            s = lb.curselection()
            keys = list(self.program.variables.keys())
            if s and s[0] < len(keys):
                del self.program.variables[keys[s[0]]]
                self.program.modified = True
                refresh_vars()

        def store_pose():
            n = fields["name"].get().strip() or "PR[{}]".format(
                len(self.program.variables) + 1)
            angles = self.rt.bank.angles()
            tcp = self.rt.kin.fk(angles)
            self.program.variables[n] = {
                "value": {"angles": angles, "tcp": tcp.xyz if tcp else None,
                          "rpy": tcp.rpy_deg if tcp else None},
                "comment": fields["note"].get().strip() or "position register"}
            self.program.modified = True
            refresh_vars()
            self.rt.log("Position register {} stored".format(n))

        bar = tk.Frame(w, bg=Ink.ground)
        bar.pack(fill="x", padx=SP4, pady=SP4)
        InkButton(bar, "ADD / UPDATE", command=add_var, variant="primary",
                  padx=SP4, pady=SP3).pack(side="left")
        InkButton(bar, "STORE CURRENT POSE", command=store_pose, variant="solid",
                  padx=SP3, pady=SP3).pack(side="left", padx=SP2)
        InkButton(bar, "DELETE", command=del_var, variant="danger",
                  padx=SP3, pady=SP3).pack(side="right")
