"""
On-screen keyboard — V4 :1765-1844, unchanged.
"""

from ..theme import Ink, Type, SP1, SP2, SP4
from ..widgets import Overlay, InkButton

import tkinter as tk


class OSK(Overlay):
    TITLE = "keyboard"
    KANJI = "鍵盤"
    GEOM = "840x310+70+400"

    ROWS = [
        list("`1234567890-="), ["BKSP"],
        list("qwertyuiop[]\\"),
        list("asdfghjkl;'"), ["ENTER"],
        ["SHIFT"] + list("zxcvbnm,./"),
        ["SPACE", "CLEAR", "◀", "▶"],
    ]
    SHIFTED = {"`": "~", "1": "!", "2": "@", "3": "#", "4": "$", "5": "%",
               "6": "^", "7": "&", "8": "*", "9": "(", "0": ")", "-": "_",
               "=": "+", "[": "{", "]": "}", "\\": "|", ";": ":", "'": '"',
               ",": "<", ".": ">", "/": "?"}

    def __init__(self, root):
        super().__init__(root)
        self.target = None
        self.shift = False

    def attach(self, entry):
        self.target = entry

    def build(self, body):
        self.keys_frame = tk.Frame(body, bg=Ink.ground)
        self.keys_frame.pack(expand=True, pady=SP4)
        self._draw()

    def _draw(self):
        for w in self.keys_frame.winfo_children():
            w.destroy()
        # Flatten into visual rows, keeping the wide keys with their row.
        layout = [self.ROWS[0] + self.ROWS[1], self.ROWS[2],
                  self.ROWS[3] + self.ROWS[4], self.ROWS[5], self.ROWS[6]]
        for row in layout:
            rf = tk.Frame(self.keys_frame, bg=Ink.ground)
            rf.pack(pady=SP1)
            for key in row:
                disp = key
                if self.shift:
                    disp = key.upper() if key.isalpha() else self.SHIFTED.get(key, key)
                wide = {"BKSP": 6, "ENTER": 6, "SHIFT": 6, "CLEAR": 6, "SPACE": 22}
                w = wide.get(key, 3)
                variant = "quiet" if key in wide else "solid"
                InkButton(rf, disp, variant=variant, font=Type.body, width=w,
                          padx=SP2, pady=SP2,
                          command=lambda k=key, d=disp: self._press(k, d)
                          ).pack(side="left", padx=2)

    def _press(self, key, disp):
        t = self.target
        if key == "SHIFT":
            self.shift = not self.shift
            self._draw()
            return
        if t is None:
            return
        try:
            if key == "BKSP":
                p = t.index("insert")
                if p > 0:
                    t.delete(p - 1, p)
            elif key == "CLEAR":
                t.delete(0, "end")
            elif key == "ENTER":
                t.event_generate("<Return>")
            elif key == "SPACE":
                t.insert("insert", " ")
            elif key == "◀":
                t.icursor(max(0, t.index("insert") - 1))
            elif key == "▶":
                t.icursor(t.index("insert") + 1)
            else:
                t.insert("insert", disp)
        except Exception:
            pass
        if self.alive():
            self.win.lift()
