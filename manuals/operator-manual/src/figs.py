"""Diagrams for the AMR manual, one set per language (labels from L[lang])."""
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch, Polygon, Rectangle, Wedge

INK, INK2, GRID = "#0b0b0b", "#52514e", "#e1e0d9"
BLUE, ORANGE, AQUA, YELLOW = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"
GOOD, WARN, SERIOUS, CRIT, GRAY, PAPER = "#0ca30c", "#fab219", "#ec835a", "#d03b3b", "#a9a89f", "#f4f3ef"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9})

L = {
    "en": {
        "front": "FRONT", "lidar": "Safety laser scanner\n(SICK nanoScan3)", "estop": "E-stop\n(red)",
        "panel": "START / STOP\nbuttons", "wheelL": "Drive wheel L\n+ encoder", "wheelR": "Drive wheel R\n+ encoder",
        "caster": "caster", "pc": "PC + I/O\nmodules", "dims": "1.18 m × 0.675 m (top view, schematic)",
        "start": "START", "stop": "STOP", "estop1": "E-STOP",
        "start_m": "MANUAL: go to AUTO\nAUTO: start / continue", "stop_m": "MANUAL: reset a fault\nAUTO: back to MANUAL\n(aborts the run)",
        "estop_m": "Stops the motors at once,\nbrakes on. Twist to release.\nIn a run: START continues.",
        "kp": "USB keypad", "arrows": "arrows", "kp_drive": "drive (hold) - MANUAL only", "kp_g": "= START", "kp_enter": "= STOP",
        "kp_q": "MANUAL ⇄ AUTO", "kp_e": "keypad stop (ENTER releases)", "kp_f": "Go to POS 1-6 (AUTO)",
        "manual": "MANUAL", "auto": "AUTO", "s1": "START", "s2": "STOP", "pwr": "power on",
        "manual_d": "drive by hand\n(web jog / keypad)", "auto_d": "the route drives\nthe AMR",
        "idle": "no mission", "ready": "READY\nmission loaded", "exec": "EXECUTING\ndriving", "done": "DONE",
        "blocked": "BLOCKED\nheld (E-stop, lidar,\npath, position)", "waiting": "WAITING\nat a wait step",
        "fault": "FAULT\nneeds you", "a_start": "START", "a_auto": "loads by itself", "a_clear": "cleared",
        "a_cont": "Continue / START", "a_ack": "Ack fault /\nSTOP (MANUAL)", "modes_t": "Vehicle mode", "run_t": "Run state",
        "good": "GOOD: red scan dots lie on the black walls", "bad": "WRONG: dots beside the walls - set the pose again",
        "amr": "AMR", "fields_t": "What the scanner does in AUTO", "green": "clear: full speed",
        "yellow": "yellow field: half speed", "red": "red field:\nSTOP\n(goes on by itself\nwhen clear)",
        "dir": "driving direction",
        "flow": ["Switch on; E-stop released;\narea clear", "Open the web page:\nHome shows IDLE, MANUAL,\nDrives ARMED",
                 "Run page: map active,\nposition set and\nconfirmed (READY)", "Put the AMR on its\nhomepost (±20 cm, ±10°)",
                 "START once → AUTO\n(mission loads itself)", "START again → 2 s\nwarning → it drives"],
        "tr_t": "The AMR does not move - check in this order",
        "tr": [("E-stop pressed? (red banner)", "Release (twist) it. In a run: press START"),
               ("Selector MANUAL or AUTO as you expect?", "START = AUTO, STOP = MANUAL"),
               ("Drives tile ARMED? (AUTO standby = OFF is normal)", "OFF + 'push mode': Manual page → Brakes ON"),
               ("Localisation READY?", "Run page: set position, Confirm"),
               ("Run state READY / BLOCKED / WAITING / FAULT?", "Read the reason under the run state"),
               ("Something in front? (yellow / red scanner LED)", "Clear the area; it continues by itself"),
               ("Still nothing? Alarms page → Standing now", "Follow the action; Save report; call the engineer")],
        "yes": "if yes:", "home": "Home", "run": "Run", "man": "Manual", "alarms": "Alarms",
        "web_t": "The web page (operator view)", "rail": "status tiles\n(left rail)", "tabs": "page tabs",
        "map": "map: AMR (blue arrow + outline),\nroute (line), scanner dots", "zoom": "+ / − / FIT /\nFOLLOW",
        "side": "left column:\nMap & pose / Mission\nbuttons and tiles", "foot": "Pause · Abort · Ack fault\n(always visible)",
    },
    "id": {
        "front": "DEPAN", "lidar": "Laser scanner safety\n(SICK nanoScan3)", "estop": "E-stop\n(merah)",
        "panel": "Tombol START /\nSTOP", "wheelL": "Roda penggerak kiri\n+ encoder", "wheelR": "Roda penggerak kanan\n+ encoder",
        "caster": "caster", "pc": "PC + modul\nI/O", "dims": "1,18 m × 0,675 m (tampak atas, skema)",
        "start": "START", "stop": "STOP", "estop1": "E-STOP",
        "start_m": "MANUAL: pindah ke AUTO\nAUTO: mulai / lanjut", "stop_m": "MANUAL: reset fault\nAUTO: kembali ke MANUAL\n(run dibatalkan)",
        "estop_m": "Motor langsung berhenti,\nrem aktif. Putar untuk lepas.\nSaat run: START melanjutkan.",
        "kp": "Keypad USB", "arrows": "panah", "kp_drive": "jalan (tahan) - hanya MANUAL", "kp_g": "= START", "kp_enter": "= STOP",
        "kp_q": "MANUAL ⇄ AUTO", "kp_e": "stop keypad (lepas dengan ENTER)", "kp_f": "Ke POS 1-6 (AUTO)",
        "manual": "MANUAL", "auto": "AUTO", "s1": "START", "s2": "STOP", "pwr": "dinyalakan",
        "manual_d": "jalan manual\n(jog web / keypad)", "auto_d": "route yang\nmenjalankan AMR",
        "idle": "tanpa misi", "ready": "READY\nmisi dimuat", "exec": "EXECUTING\nberjalan", "done": "DONE",
        "blocked": "BLOCKED\nditahan (E-stop, lidar,\njalur, posisi)", "waiting": "WAITING\ndi titik tunggu",
        "fault": "FAULT\nperlu operator", "a_start": "START", "a_auto": "dimuat otomatis", "a_clear": "sudah aman",
        "a_cont": "Continue / START", "a_ack": "Ack fault /\nSTOP (MANUAL)", "modes_t": "Mode kendaraan", "run_t": "Status run",
        "good": "BENAR: titik scan merah tepat di dinding hitam", "bad": "SALAH: titik di samping dinding - set posisi lagi",
        "amr": "AMR", "fields_t": "Yang dilakukan scanner saat AUTO", "green": "aman: kecepatan penuh",
        "yellow": "field kuning: setengah kecepatan", "red": "field merah:\nBERHENTI\n(lanjut sendiri\nkalau aman)",
        "dir": "arah jalan",
        "flow": ["Nyalakan; E-stop dilepas;\narea aman", "Buka halaman web:\nHome = IDLE, MANUAL,\nDrives ARMED",
                 "Halaman Run: peta aktif,\nposisi di-set dan\ndikonfirmasi (READY)", "Taruh AMR di homepost\n(±20 cm, ±10°)",
                 "START sekali → AUTO\n(misi dimuat otomatis)", "START lagi → peringatan\n2 detik → AMR jalan"],
        "tr_t": "AMR tidak mau jalan - cek dengan urutan ini",
        "tr": [("E-stop ditekan? (banner merah)", "Lepas (putar). Saat run: tekan START"),
               ("Selector MANUAL atau AUTO sesuai yang diinginkan?", "START = AUTO, STOP = MANUAL"),
               ("Tile Drives ARMED? (AUTO standby = OFF itu normal)", "OFF + 'push mode': halaman Manual → Brakes ON"),
               ("Lokalisasi READY?", "Halaman Run: set posisi, Confirm"),
               ("Status run READY / BLOCKED / WAITING / FAULT?", "Baca alasan di bawah status run"),
               ("Ada benda di depan? (LED scanner kuning / merah)", "Bersihkan area; AMR lanjut sendiri"),
               ("Masih tidak jalan? Halaman Alarms → Standing now", "Ikuti tindakannya; Save report; hubungi engineer")],
        "yes": "jika ya:", "home": "Home", "run": "Run", "man": "Manual", "alarms": "Alarms",
        "web_t": "Halaman web (tampilan operator)", "rail": "tile status\n(kolom kiri)", "tabs": "tab halaman",
        "map": "peta: AMR (panah biru + outline),\nroute (garis), titik scanner", "zoom": "+ / − / FIT /\nFOLLOW",
        "side": "kolom kiri:\nMap & pose / Mission\ntombol dan tile", "foot": "Pause · Abort · Ack fault\n(selalu terlihat)",
    },
}


def _ax(w, h):
    fig = plt.figure(figsize=(w, h))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, w * 10)
    ax.set_ylim(0, h * 10)
    ax.axis("off")
    return fig, ax


def box(ax, x, y, w, h, text, fc=PAPER, ec=INK2, tc=INK, fs=8.5, bold=False, r=0.8):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle=f"round,pad=0,rounding_size={r}", fc=fc, ec=ec, lw=1.2))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs, color=tc, fontweight="bold" if bold else "normal")


def arrow(ax, a, b, color=INK2, text=None, fs=7.5, rad=0.0, off=(0, 1.2)):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle="-|>", mutation_scale=12, color=color, lw=1.4, connectionstyle=f"arc3,rad={rad}"))
    if text:
        ax.text((a[0] + b[0]) / 2 + off[0], (a[1] + b[1]) / 2 + off[1], text, ha="center", va="center", fontsize=fs, color=color,
                bbox=dict(fc="white", ec="none", pad=0.6))


def overview(t, out):
    fig, ax = _ax(7.2, 3.6)
    cx, cy = 30, 18
    ax.add_patch(FancyBboxPatch((cx - 18, cy - 10), 36, 20, boxstyle="round,pad=0,rounding_size=1.5", fc="#eef3fb", ec=INK, lw=1.6))
    ax.add_patch(Wedge((cx + 18, cy), 4, -95, 95, fc=YELLOW, ec=INK, lw=1))
    ax.add_patch(Rectangle((cx - 1, cy - 12), 6, 3, fc=INK, ec=INK))
    ax.add_patch(Rectangle((cx - 1, cy + 9), 6, 3, fc=INK, ec=INK))
    for px, py in ((cx - 15, cy - 7), (cx - 15, cy + 7), (cx + 13, cy - 7), (cx + 13, cy + 7)):
        ax.add_patch(Circle((px, py), 1.1, fc=GRAY, ec=INK))
    ax.add_patch(Circle((cx - 10, cy + 5), 1.7, fc=CRIT, ec=INK))
    ax.add_patch(Rectangle((cx - 13, cy - 6), 6, 3.2, fc="#cfe0f6", ec=INK2))
    ax.add_patch(Rectangle((cx - 4, cy - 3), 10, 6, fc="white", ec=INK2, ls="--"))
    ax.text(cx + 1, cy, t["pc"], ha="center", va="center", fontsize=7.2, color=INK2)
    ax.annotate("", (cx + 26, cy), (cx + 19, cy), arrowprops=dict(arrowstyle="-|>", color=BLUE, lw=2))
    ax.text(cx + 27, cy, t["front"], va="center", fontsize=10, color=BLUE, fontweight="bold")
    lab = [(t["lidar"], (cx + 20, cy + 3), (52, 33)), (t["estop"], (cx - 10, cy + 6.5), (8, 33)),
           (t["panel"], (cx - 13, cy - 5), (7, 8)), (t["wheelL"], (cx + 2, cy + 12), (30, 33)),
           (t["wheelR"], (cx + 4, cy - 12), (56, 4)), (t["caster"], (cx - 15, cy - 8), (22, 3.5))]
    for txt, p, q in lab:
        ax.annotate(txt, p, q, fontsize=7.8, ha="center", va="center", arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8))
    ax.text(1, 0.8, t["dims"], fontsize=7.5, color=INK2, ha="left")
    fig.savefig(out, dpi=200); plt.close(fig)


def panel(t, out):
    fig, ax = _ax(7.2, 4.4)
    for i, (lbl, col, tc, desc) in enumerate(((t["start"], GOOD, "white", t["start_m"]), (t["stop"], "#3d3d3a", "white", t["stop_m"]),
                                              (t["estop1"], CRIT, "white", t["estop_m"]))):
        x = 7 + i * 13
        ax.add_patch(Circle((x, 33), 4.0, fc=col, ec=INK, lw=1.4))
        if i == 2:
            ax.add_patch(Circle((x, 33), 4.8, fc="none", ec=WARN, lw=3))
        ax.text(x, 33, lbl, ha="center", va="center", color=tc, fontsize=7.4, fontweight="bold")
        ax.text(x, 21, desc, ha="center", va="center", fontsize=6.4, color=INK)
    kx = 43
    ax.text(kx + 14, 41.5, t["kp"], ha="center", fontsize=9, fontweight="bold")
    keys = {"F1": (0, 36), "F2": (4.2, 36), "F3": (8.4, 36), "F4": (12.6, 36), "F5": (16.8, 36), "F6": (21, 36),
            "Q": (0, 31), "W": (4.2, 31), "E": (8.4, 31), "A": (0, 26.5), "S": (4.2, 26.5), "D": (8.4, 26.5),
            "G": (14, 26.5), "ENTER": (19, 26.5)}
    col = {"W": BLUE, "A": BLUE, "S": BLUE, "D": BLUE, "G": GOOD, "ENTER": "#3d3d3a", "E": CRIT, "Q": ORANGE}
    for k, (x, y) in keys.items():
        w = 6.5 if k == "ENTER" else 3.6
        c = col.get(k, YELLOW if k.startswith("F") else GRAY)
        ax.add_patch(FancyBboxPatch((kx + x, y - 1.8), w, 3.6, boxstyle="round,pad=0,rounding_size=0.5", fc=c, ec=INK, lw=0.8))
        ax.text(kx + x + w / 2, y, k, ha="center", va="center", fontsize=6.4, color="white" if c not in (YELLOW, GRAY) else INK, fontweight="bold")
    leg = [(BLUE, "W A S D / " + t["arrows"] + ": " + t["kp_drive"]), (GOOD, "G " + t["kp_g"]), ("#3d3d3a", "ENTER " + t["kp_enter"]),
           (ORANGE, "Q: " + t["kp_q"]), (CRIT, "E: " + t["kp_e"]), (YELLOW, "F1-F6: " + t["kp_f"])]
    for i, (c, s_) in enumerate(leg):
        y = 19 - i * 3.0
        ax.add_patch(Rectangle((kx, y - 0.8), 1.6, 1.6, fc=c, ec=INK, lw=0.5))
        ax.text(kx + 2.4, y, s_, fontsize=6.6, va="center")
    fig.savefig(out, dpi=200); plt.close(fig)


def modes(t, out):
    fig, ax = _ax(7.2, 4.6)
    ax.text(2, 43.5, t["modes_t"], fontsize=10, fontweight="bold")
    box(ax, 4, 30, 14, 8, t["manual"] + "\n" + t["manual_d"], fc="#eef3fb", ec=BLUE, fs=7.8)
    box(ax, 34, 30, 14, 8, t["auto"] + "\n" + t["auto_d"], fc="#eef8ee", ec=GOOD, fs=7.8)
    arrow(ax, (18, 36), (34, 36), color=GOOD, text=t["s1"], off=(0, 1.3))
    arrow(ax, (34, 32), (18, 32), color=INK, text=t["s2"], off=(0, -1.4))
    ax.annotate(t["pwr"], (8, 38), (2, 41.5), fontsize=7.5, color=INK2, arrowprops=dict(arrowstyle="-|>", color=INK2))
    ax.text(52, 43.5, t["run_t"] + " (AUTO)", fontsize=10, fontweight="bold")
    box(ax, 52, 34, 9, 6, t["idle"], fs=7.2)
    box(ax, 52, 22, 9, 7, t["ready"], fc="#eef3fb", ec=BLUE, fs=7)
    box(ax, 52, 10, 9, 7, t["exec"], fc="#eef8ee", ec=GOOD, fs=7)
    box(ax, 52, 1, 9, 5, t["done"], fc="#eef8ee", ec=GOOD, fs=7.5)
    box(ax, 63, 14, 8.5, 9, t["blocked"], fc="#fff6e0", ec=WARN, fs=6.1)
    box(ax, 63, 3, 8.5, 8, t["waiting"], fc="#fff6e0", ec=WARN, fs=6.4)
    box(ax, 40, 3, 9, 7, t["fault"], fc="#fbe9e9", ec=CRIT, fs=7)
    arrow(ax, (56.5, 34), (56.5, 29), text=t["a_auto"], off=(-5.5, 0))
    arrow(ax, (56.5, 22), (56.5, 17), text=t["a_start"], off=(-3.4, 0))
    arrow(ax, (56.5, 10), (56.5, 6))
    arrow(ax, (61, 14), (63, 17), color=WARN)
    arrow(ax, (63, 20), (61, 15.5), color=WARN, text=t["a_clear"], off=(1.5, 3.2), fs=6.5)
    arrow(ax, (61, 11), (63, 8), color=WARN)
    arrow(ax, (63, 5), (61, 10.5), color=WARN, text=t["a_cont"], off=(3.5, -3.0), fs=6.5)
    arrow(ax, (52, 12), (49, 8), color=CRIT)
    arrow(ax, (44.5, 10), (52, 25), color=CRIT, text=t["a_ack"], off=(-6.0, 2.5), fs=6.5, rad=-0.25)
    fig.savefig(out, dpi=200); plt.close(fig)


def localise(t, out):
    fig, axs = plt.subplots(1, 2, figsize=(7.2, 2.6))
    walls = [[(0, 0), (8, 0), (8, 4), (0, 4), (0, 0)], [(2.5, 1.5), (6, 1.5), (6, 2.5), (2.5, 2.5), (2.5, 1.5)]]
    import numpy as np
    rng = np.random.default_rng(3)
    for ax, ok, title in ((axs[0], True, t["good"]), (axs[1], False, t["bad"])):
        for w in walls:
            p = np.array(w); ax.plot(p[:, 0], p[:, 1], color=INK, lw=2.4)
        pts = []
        for w in walls:
            for (x0, y0), (x1, y1) in zip(w, w[1:]):
                for s in np.linspace(0, 1, 18):
                    pts.append((x0 + s * (x1 - x0), y0 + s * (y1 - y0)))
        pts = np.array(pts) + rng.normal(0, 0.03, (len(pts), 2))
        if not ok:
            a = math.radians(9); c, s_ = math.cos(a), math.sin(a)
            pts = np.c_[c * (pts[:, 0] - 4) - s_ * (pts[:, 1] - 2) + 4.5, s_ * (pts[:, 0] - 4) + c * (pts[:, 1] - 2) + 1.7]
        ax.scatter(pts[:, 0], pts[:, 1], s=6, color=CRIT, zorder=3)
        ax.annotate("", (1.9, 0.9), (1.0, 0.9), arrowprops=dict(arrowstyle="-|>", color=BLUE, lw=2.4))
        ax.set_xlim(-0.6, 8.8); ax.set_ylim(-0.6, 4.6); ax.set_aspect("equal"); ax.axis("off")
        ax.set_title(title, fontsize=8.2, color=GOOD if ok else CRIT, fontweight="bold")
    fig.tight_layout(); fig.savefig(out, dpi=200); plt.close(fig)


def fields(t, out):
    fig, ax = _ax(7.2, 2.6)
    ax.add_patch(Polygon([(10, 4), (70, 2), (70, 24), (10, 22)], closed=True, fc="#eef8ee", ec="none"))
    ax.add_patch(Polygon([(10, 6), (40, 5), (40, 21), (10, 20)], closed=True, fc="#fff1c9", ec=YELLOW, lw=1))
    ax.add_patch(Polygon([(10, 8), (22, 8), (22, 18), (10, 18)], closed=True, fc="#f8d3d3", ec=CRIT, lw=1))
    ax.add_patch(FancyBboxPatch((1, 8), 9, 10, boxstyle="round,pad=0,rounding_size=0.8", fc="#eef3fb", ec=INK, lw=1.4))
    ax.text(5.5, 13, t["amr"], ha="center", va="center", fontweight="bold")
    ax.text(16, 13, t["red"], ha="center", va="center", fontsize=6.3, color=CRIT)
    ax.text(31, 13, t["yellow"], ha="center", va="center", fontsize=7.4, color="#9a6a00")
    ax.text(56, 13, t["green"], ha="center", va="center", fontsize=7.4, color=GOOD)
    ax.annotate("", (68, 1.5), (12, 1.5), arrowprops=dict(arrowstyle="-|>", color=INK2))
    ax.text(40, 0.2, t["dir"], ha="center", fontsize=7, color=INK2)
    ax.text(1, 24.5, t["fields_t"], fontsize=9.5, fontweight="bold")
    fig.savefig(out, dpi=200); plt.close(fig)


def startflow(t, out):
    fig, ax = _ax(7.2, 2.9)
    w, h = 19, 10
    for i, s_ in enumerate(t["flow"]):
        r, c = divmod(i, 3)
        x, y = 2 + c * 24, 16 - r * 14
        last = i == len(t["flow"]) - 1
        box(ax, x, y, w, h, s_, fc="#eef8ee" if last else PAPER, ec=GOOD if last else INK2, fs=7.2)
        ax.text(x + 1, y + h - 1.6, str(i + 1), fontsize=9, fontweight="bold", color=BLUE)
        if c < 2 and i < len(t["flow"]) - 1:
            arrow(ax, (x + w + 0.2, y + h / 2), (x + 24 - 0.2, y + h / 2))
    arrow(ax, (2 + 2 * 24 + w / 2, 16), (2 + w / 2, 12), rad=-0.08)
    fig.savefig(out, dpi=200); plt.close(fig)


def trouble(t, out):
    rows = t["tr"]
    fig, ax = _ax(7.2, 0.62 * len(rows) + 0.6)
    H = 0.62 * len(rows) * 10 + 6
    ax.text(1, H - 3, t["tr_t"], fontsize=10, fontweight="bold")
    for i, (q, a) in enumerate(rows):
        y = H - 9 - i * 6.2
        box(ax, 1, y, 36, 4.8, f"{i + 1}. {q}", fc="#eef3fb", ec=BLUE, fs=7.3)
        box(ax, 44, y, 27, 4.8, a, fc="#eef8ee", ec=GOOD, fs=7.0)
        arrow(ax, (37.2, y + 2.4), (43.8, y + 2.4), color=GOOD, text=t["yes"], off=(0, 1.6), fs=6.3)
        if i < len(rows) - 1:
            arrow(ax, (19, y), (19, y - 1.3), color=INK2)
    fig.savefig(out, dpi=200); plt.close(fig)


def webpage(t, out):
    fig, ax = _ax(7.2, 3.8)
    ax.add_patch(Rectangle((1, 1), 70, 36, fc="white", ec=INK, lw=1.4))
    ax.add_patch(Rectangle((1, 33), 70, 4, fc="#2b2b29", ec=INK))
    for i, s in enumerate((t["home"], t["run"], t["man"], t["alarms"])):
        ax.add_patch(Rectangle((14 + i * 7.5, 33.6), 7, 2.8, fc=BLUE if i == 1 else "#4a4a46", ec="none"))
        ax.text(17.5 + i * 7.5, 35, s, ha="center", va="center", color="white", fontsize=6.8)
    ax.add_patch(Rectangle((1, 1), 9, 32, fc="#f0efea", ec=INK2))
    for i in range(8):
        ax.add_patch(Rectangle((1.7, 29 - i * 3.6), 7.6, 3, fc="white", ec=GRID))
    ax.add_patch(Rectangle((10.5, 1.5), 19, 31, fc="#fafaf7", ec=INK2))
    ax.add_patch(Rectangle((10.9, 2), 18.2, 4.5, fc="#fff6e0", ec=WARN))
    ax.add_patch(Rectangle((30, 1.5), 40.5, 31, fc="#ecebe5", ec=INK2))
    ax.plot([34, 66, 66, 34, 34], [8, 8, 26, 26, 8], color=BLUE, lw=1.5)
    ax.annotate("", (52, 8), (46, 8), arrowprops=dict(arrowstyle="-|>", color=BLUE, lw=2.6))
    for i in range(4):
        ax.add_patch(Rectangle((66.8, 27.5 - i * 2.4), 3, 2, fc="white", ec=INK2))
    lab = [(t["rail"], (5.5, 20), (5.5, -5.5)), (t["tabs"], (25, 35), (25, 41)), (t["side"], (20, 20), (20, -7.5)),
           (t["foot"], (20, 4), (38, -7.0)), (t["map"], (50, 17), (55, -6.0)), (t["zoom"], (68.3, 26), (68, 41))]
    ax.set_ylim(-11, 44)
    for s, p, q in lab:
        ax.annotate(s, p, q, fontsize=6.8, ha="center", va="center", arrowprops=dict(arrowstyle="-", color=CRIT, lw=0.8))
    ax.text(1, 42, t["web_t"], fontsize=9.5, fontweight="bold")
    fig.savefig(out, dpi=200); plt.close(fig)


def make_all(lang: str, outdir: str) -> dict:
    os.makedirs(outdir, exist_ok=True)
    t = L[lang]
    out = {}
    for name, fn in (("overview", overview), ("panel", panel), ("modes", modes), ("localise", localise), ("fields", fields),
                     ("startflow", startflow), ("trouble", trouble), ("webpage", webpage)):
        p = os.path.join(outdir, f"{name}.png")
        fn(t, p)
        out[name] = p
    return out


if __name__ == "__main__":
    import sys
    d = os.path.dirname(os.path.abspath(__file__))
    for lang in sys.argv[1:] or ["en"]:
        make_all(lang, os.path.join(d, "fig_" + lang))
