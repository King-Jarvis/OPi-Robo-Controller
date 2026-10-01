"""
Widget toolkit — V4 :1411-1863, unchanged apart from imports.

Everything here is flat: no bevels, no relief, hairlines instead of borders.
Colour is applied only to carry state. cfg() is the cheap-redraw gate that
keeps this affordable on an H618.
"""

import tkinter as tk

from .theme import Ink, Type, SP1, SP2, SP3, SP4, SP5


def cfg(widget, **kw):
    """config() the widget only with values that actually changed."""
    cache = getattr(widget, "_ink_cache", None)
    if cache is None:
        cache = widget._ink_cache = {}
    delta = {k: v for k, v in kw.items() if cache.get(k, _MISS) != v}
    if delta:
        cache.update(delta)
        widget.config(**delta)


_MISS = object()


def track(s):
    """Letterspaced caps. Tk has no letter-spacing, so space the glyphs."""
    return " ".join(s.upper())


def hairline(parent, color=None, pad=0, side=None, **pk):
    f = tk.Frame(parent, bg=color or Ink.line, height=1)
    if side:
        f.pack(side=side, fill="x", padx=pad, **pk)
    else:
        f.pack(fill="x", padx=pad, **pk)
    return f


def vrule(parent, color=None, **pk):
    f = tk.Frame(parent, bg=color or Ink.line, width=1)
    f.pack(fill="y", **pk)
    return f


def text(parent, s="", font=None, fg=None, bg=None, **kw):
    return tk.Label(parent, text=s, font=font or Type.body,
                    fg=fg or Ink.text_2, bg=bg or Ink.ground,
                    bd=0, highlightthickness=0, **kw)


class InkButton(tk.Frame):
    """
    Flat pressable. Hover exists for a mouse, press feedback for a finger.
    variant: quiet | solid | primary | danger
    Set hold=True for jog buttons — press/release callbacks instead of click.
    """

    VARIANTS = {
        "quiet":   (Ink.ground, Ink.text_2, Ink.hover, Ink.text,  Ink.line_lit),
        "solid":   (Ink.panel,  Ink.text_2, Ink.hover, Ink.text,  Ink.asagi),
        "primary": (Ink.panel,  Ink.asagi,  Ink.hover, Ink.asagi, Ink.asagi),
        "danger":  (Ink.panel,  Ink.beni,   Ink.hover, Ink.beni,  Ink.shu),
    }

    def __init__(self, parent, label, command=None, variant="solid",
                 font=None, padx=SP4, pady=SP2, width=None, sub=None,
                 on_press=None, on_release=None, **kw):
        bg, fg, hbg, hfg, pbg = self.VARIANTS.get(variant, self.VARIANTS["solid"])
        super().__init__(parent, bg=bg, bd=0, highlightthickness=0, **kw)
        self._c = (bg, fg, hbg, hfg, pbg)
        self._command = command
        self._on_press = on_press
        self._on_release = on_release
        self._enabled = True
        self._down = False

        self.lbl = tk.Label(self, text=label, font=font or Type.label, fg=fg, bg=bg,
                            bd=0, highlightthickness=0, padx=padx, pady=pady)
        if width:
            self.lbl.config(width=width)
        self.lbl.pack(fill="both", expand=True)
        self.sub = None
        if sub:
            self.sub = tk.Label(self, text=sub, font=Type.micro, fg=Ink.text_4,
                                bg=bg, bd=0, pady=0)
            self.sub.pack(fill="x", pady=(0, SP1))

        for w in self._parts():
            w.bind("<Enter>", self._enter)
            w.bind("<Leave>", self._leave)
            w.bind("<ButtonPress-1>", self._press)
            w.bind("<ButtonRelease-1>", self._release)

    def _parts(self):
        return [w for w in (self, self.lbl, self.sub) if w is not None]

    def _paint(self, bg, fg):
        for w in self._parts():
            cfg(w, bg=bg)
        cfg(self.lbl, fg=fg)
        if self.sub is not None:
            cfg(self.sub, fg=Ink.text_4)

    def _enter(self, _e=None):
        if self._enabled and not self._down:
            self._paint(self._c[2], self._c[3])

    def _leave(self, _e=None):
        self._down = False
        self._paint(self._c[0], self._c[1] if self._enabled else Ink.text_4)

    def _press(self, _e=None):
        if not self._enabled:
            return
        self._down = True
        self._paint(self._c[4], Ink.void)
        if self._on_press:
            self._on_press()

    def _release(self, _e=None):
        if not self._enabled:
            return
        was = self._down
        self._down = False
        self._paint(self._c[2], self._c[3])
        if self._on_release:
            self._on_release()
        if was and self._command:
            self._command()

    def set_label(self, s):
        cfg(self.lbl, text=s)

    def set_base_bg(self, bg):
        """Re-ground the button, e.g. to follow a row that became selected.
        Leaves a button that is currently held down alone."""
        if bg == self._c[0]:
            return
        self._c = (bg,) + self._c[1:]
        if not self._down:
            self._paint(bg, self._c[1] if self._enabled else Ink.text_4)

    def set_colors(self, bg=None, fg=None, hover_bg=None, hover_fg=None, press=None):
        c = list(self._c)
        for i, v in enumerate((bg, fg, hover_bg, hover_fg, press)):
            if v is not None:
                c[i] = v
        self._c = tuple(c)
        self._leave()

    def set_enabled(self, on):
        if on == self._enabled:
            return
        self._enabled = on
        self._paint(self._c[0], self._c[1] if on else Ink.text_4)


class InkSlider(tk.Canvas):
    """Hairline track, filled lead-in, small handle. Drag or tap anywhere."""

    def __init__(self, parent, from_=1, to=100, value=30, width=180, height=26,
                 command=None, accent=None, bg=None):
        bg = bg or Ink.panel
        super().__init__(parent, width=width, height=height, bg=bg,
                         highlightthickness=0, bd=0, cursor="hand2")
        self._from, self._to = from_, to
        self._px_w, self._px_h = width, height   # _w is tkinter's; do not reuse
        self._pad = 9
        self._accent = accent or Ink.asagi
        self._command = command
        self._value = value

        y = height // 2
        self._track = self.create_line(self._pad, y, width - self._pad, y,
                                       fill=Ink.line_lit, width=1)
        self._fill = self.create_line(self._pad, y, self._pad, y,
                                      fill=self._accent, width=2)
        self._knob = self.create_oval(0, 0, 0, 0, fill=self._accent, outline="")
        self.bind("<ButtonPress-1>", self._drag)
        self.bind("<B1-Motion>", self._drag)
        self.bind("<ButtonRelease-1>", self._drag)
        self._redraw()

    def _x_for(self, v):
        span = max(1e-9, self._to - self._from)
        t = (v - self._from) / span
        return self._pad + t * (self._px_w - 2 * self._pad)

    def _redraw(self):
        x, y = self._x_for(self._value), self._px_h // 2
        self.coords(self._fill, self._pad, y, x, y)
        self.coords(self._knob, x - 5, y - 5, x + 5, y + 5)

    def set(self, v, notify=False):
        v = max(self._from, min(self._to, v))
        if v == self._value:
            return
        self._value = v
        self._redraw()
        if notify and self._command:
            self._command(v)

    def get(self):
        return self._value

    def _drag(self, e):
        span = self._px_w - 2 * self._pad
        t = max(0.0, min(1.0, (e.x - self._pad) / max(1, span)))
        v = int(round(self._from + t * (self._to - self._from)))
        if v != self._value:
            self._value = v
            self._redraw()
            if self._command:
                self._command(v)


class TravelBar(tk.Canvas):
    """Where a joint sits inside its physical range. Read-only, very quiet."""

    def __init__(self, parent, width=104, height=20, bg=None):
        bg = bg or Ink.panel
        super().__init__(parent, width=width, height=height, bg=bg,
                         highlightthickness=0, bd=0)
        self._px_w, self._px_h = width, height   # _w is tkinter's; do not reuse
        y = height // 2
        self._track = self.create_line(2, y, width - 2, y, fill=Ink.line_lit, width=1)
        self._mark = self.create_rectangle(0, 0, 0, 0, fill=Ink.text_3, outline="")
        self._pos = None

    def set(self, value, lo, hi, color=None):
        span = max(1e-9, hi - lo)
        t = max(0.0, min(1.0, (value - lo) / span))
        x = 2 + t * (self._px_w - 4)
        key = (round(x, 1), color)
        if key == self._pos:
            return
        self._pos = key
        y = self._px_h // 2
        self.coords(self._mark, x - 1, y - 6, x + 1, y + 6)
        self.itemconfig(self._mark, fill=color or Ink.text_3)

    def blank(self):
        if self._pos == "blank":
            return
        self._pos = "blank"
        self.coords(self._mark, 0, 0, 0, 0)


class Dot(tk.Canvas):
    """A 7px state dot. Dormant is nearly invisible — silence is the default."""

    def __init__(self, parent, bg=None, size=9):
        bg = bg or Ink.raised
        super().__init__(parent, width=size, height=size, bg=bg,
                         highlightthickness=0, bd=0)
        self._o = self.create_oval(1, 1, size - 1, size - 1,
                                   fill=Ink.text_4, outline="")
        self._last = None

    def set(self, on, color, off=None):
        c = color if on else (off or Ink.text_4)
        if c != self._last:
            self._last = c
            self.itemconfig(self._o, fill=c)


def well(parent, width=10, font=None, fg=None, anchor="e", pady=SP1):
    """A recessed numeric readout. Deepest ground so the number floats."""
    f = tk.Frame(parent, bg=Ink.void, bd=0, highlightthickness=0)
    l = tk.Label(f, text="", font=font or Type.data, fg=fg or Ink.text,
                 bg=Ink.void, width=width, anchor=anchor, padx=SP2, pady=pady)
    l.pack()
    return f, l


def section(parent, title, kanji=None, pady=(SP5, SP2), bg=None):
    """Section mark: tracked caps, a kanji whisper, then air. Ma before Kanso."""
    bg = bg or Ink.ground
    f = tk.Frame(parent, bg=bg)
    f.pack(fill="x", pady=pady)
    tk.Label(f, text=track(title), font=Type.label, fg=Ink.text_3, bg=bg,
             anchor="w").pack(side="left", padx=(SP4, 0))
    if kanji and Type.cjk:
        tk.Label(f, text=kanji, font=(Type.cjk, 9), fg=Ink.text_4, bg=bg
                 ).pack(side="left", padx=SP3)
    return f


# ═════════════════════════════════════════════════════════════════════════
#  OVERLAY BASE
#  Yūgen — depth is behind a door, not spread across the main screen.
# ═════════════════════════════════════════════════════════════════════════
class Overlay:
    TITLE = ""
    KANJI = None
    GEOM = "900x620+60+40"
    RESIZE = (False, False)

    def __init__(self, root):
        self.root = root
        self.win = None
        self.body = None

    def toggle(self):
        if self.win and self.win.winfo_exists():
            self.close()
        else:
            self.open()

    def open(self):
        if self.win and self.win.winfo_exists():
            self.win.lift()
            return
        self.win = tk.Toplevel(self.root)
        self.win.title(self.TITLE)
        self.win.configure(bg=Ink.ground)
        self.win.geometry(self.GEOM)
        self.win.resizable(*self.RESIZE)
        self.win.protocol("WM_DELETE_WINDOW", self.close)
        self.win.bind("<Escape>", lambda e: self.close())
        self._header()
        self.body = tk.Frame(self.win, bg=Ink.ground)
        self.body.pack(fill="both", expand=True)
        self.build(self.body)

    def close(self):
        self.on_close()
        if self.win and self.win.winfo_exists():
            self.win.destroy()
        self.win = None

    def alive(self):
        return bool(self.win and self.win.winfo_exists())

    def on_close(self):
        pass

    def build(self, body):
        raise NotImplementedError

    def _header(self):
        bar = tk.Frame(self.win, bg=Ink.raised, height=52)
        bar.pack(fill="x")
        bar.pack_propagate(False)
        tk.Label(bar, text=track(self.TITLE), font=Type.title, fg=Ink.text,
                 bg=Ink.raised).pack(side="left", padx=(SP5, 0))
        if self.KANJI and Type.cjk:
            tk.Label(bar, text=self.KANJI, font=(Type.cjk, 12), fg=Ink.text_4,
                     bg=Ink.raised).pack(side="left", padx=SP4)
        InkButton(bar, "CLOSE", command=self.close, variant="quiet",
                  padx=SP4).pack(side="right", padx=SP4, pady=SP3)
        hairline(self.win)


def bind_osk(entry, osk):
    entry.bind("<FocusIn>", lambda e, w=entry: osk.attach(w))
    return entry


def field_entry(parent, value="", width=10, font=None, fg=None, osk=None):
    """Flat entry: no relief, a hairline underline instead of a sunken box."""
    wrap = tk.Frame(parent, bg=Ink.void)
    e = tk.Entry(wrap, font=font or Type.data, width=width,
                 bg=Ink.void, fg=fg or Ink.text, insertbackground=Ink.asagi,
                 relief="flat", bd=0, highlightthickness=0, justify="right")
    e.insert(0, str(value))
    e.pack(padx=SP2, pady=SP2)
    tk.Frame(wrap, bg=Ink.line_lit, height=1).pack(fill="x")
    if osk:
        bind_osk(e, osk)
    return wrap, e
