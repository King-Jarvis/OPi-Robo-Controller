"""
Design tokens — V4 :136-238, unchanged. Kanso, Ma, Shibui, Seijaku, Fukinsei,
Yūgen: see the V4 header for the design pillars this palette carries.
"""

import tkinter.font as tkfont


class Ink:
    """Sumi-ink palette. Grounds are warm neutrals; saturation is rationed."""
    void     = "#0B0C0E"   # deepest ground — readouts, wells
    ground   = "#121417"   # page
    raised   = "#181B1F"   # header / footer
    panel    = "#1E2226"   # cards, rows
    sel      = "#232A2E"   # selected row
    hover    = "#262C31"
    line     = "#282D33"   # hairline
    line_lit = "#3B434B"

    text     = "#EDEAE3"   # 白練 shironeri — primary
    text_2   = "#A5A39B"   # 生成 kinari    — secondary
    text_3   = "#6E7378"   # 鈍色 nibi      — labels
    text_4   = "#464B51"   # disabled / dormant

    asagi    = "#7FB3AE"   # 浅葱 — selection, the working accent
    ai       = "#5B84B1"   # 藍   — information, AGV
    wakatake = "#8AAE79"   # 若竹 — ready, ok
    yamabuki = "#D2A24C"   # 山吹 — caution
    shu      = "#BE4E3A"   # 朱   — danger (the only saturated red)
    beni     = "#D9604A"   # 紅   — danger, lit

# Ma — one spacing scale, used everywhere, no ad-hoc numbers.
SP1, SP2, SP3, SP4, SP5, SP6 = 4, 8, 12, 16, 24, 32

class Type:
    """Resolved after the root window exists — families vary by image."""
    mono = sans = cjk = None
    micro = label = body = body_b = data = data_lg = hero = title = rail = None

    @classmethod
    def resolve(cls, root):
        fams = set(tkfont.families(root))
        def pick(cands, fallback):
            for c in cands:
                if c in fams:
                    return c
            return fallback
        cls.mono = pick(["JetBrains Mono", "IBM Plex Mono", "Roboto Mono",
                         "DejaVu Sans Mono", "Liberation Mono", "Noto Sans Mono"],
                        "Courier")
        cls.sans = pick(["Inter", "IBM Plex Sans", "Noto Sans", "DejaVu Sans",
                         "Liberation Sans"], "Helvetica")
        # Only show kanji if something on this image can actually draw them,
        # otherwise the accents render as tofu and the screen looks broken.
        cls.cjk = pick(["Noto Sans CJK JP", "Noto Serif CJK JP", "Noto Sans JP",
                        "Source Han Sans JP", "IPAGothic", "TakaoGothic",
                        "WenQuanYi Zen Hei", "Droid Sans Fallback"], None)

        cls.micro   = (cls.sans, 7)
        cls.label   = (cls.sans, 8)
        cls.body    = (cls.sans, 10)
        cls.body_b  = (cls.sans, 10, "bold")
        cls.title   = (cls.sans, 12)
        cls.data    = (cls.mono, 13)
        cls.data_lg = (cls.mono, 17)
        cls.hero    = (cls.mono, 40)
        cls.rail    = (cls.cjk, 13) if cls.cjk else (cls.sans, 9)

    @classmethod
    def jp(cls, kanji, romaji):
        """Kanji when the image can render it, romaji when it can't."""
        return kanji if cls.cjk else romaji

class Metrics:
    """
    Layout density, chosen from the actual panel height at start-up. A 7"
    1024x600 touchscreen cannot carry desktop spacing and still show the log,
    so below 700px everything tightens by one step. Ma is preserved in
    proportion — the scale shrinks, the rhythm does not change.
    """
    compact = False
    row_h = 42
    rail_w = 252
    sec_pady = (SP4, SP2)
    log_lines = 5
    travel_w = 110
    slider_w = 132
    state_w = 10
    jog_padx = SP5
    rule_pady = SP5          # air around the rail's dividing rules
    stop_pady = SP4
    hero_font = None
    tcp_font = None

    @classmethod
    def resolve(cls, root):
        # Fullscreen means window height == screen height.
        cls.compact = root.winfo_screenheight() < 700
        if cls.compact:
            cls.row_h, cls.rail_w = 36, 224
            cls.sec_pady = (SP3, SP1)
            cls.log_lines = 3
            cls.travel_w, cls.slider_w, cls.state_w = 84, 104, 9
            cls.jog_padx = SP4
            cls.rule_pady, cls.stop_pady = SP2, SP3
            cls.hero_font = (Type.mono, 30)
            cls.tcp_font = Type.data
        else:
            cls.hero_font = Type.hero
            cls.tcp_font = Type.data_lg

