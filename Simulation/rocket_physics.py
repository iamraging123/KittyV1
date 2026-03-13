"""
KittyV1 — Single-Fin Roll Force (Barrowman Equations)
=====================================================
GUI application for computing aerodynamic forces on a single
trapezoidal fin using the Barrowman normal-force method.

Requires: matplotlib, tkinter (built-in)
"""

import json
import math
import os
import tkinter as tk
from tkinter import ttk

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


# ═══════════════════════════════════════════════════════════════
#  BARROWMAN EQUATIONS
# ═══════════════════════════════════════════════════════════════

def barrowman_fin_CNa(body_d, semi_span, cr, ct):
    """CNα for one fin [per radian] with fin-body + body carryover interference.

    Ref area: body cross-section π(d/2)².
    K_fb  = fin-on-body   = 1 + r/(s+r)
    K_bf  = body carryover = r/(s+r)
    Total = K_fb + K_bf   = 1 + 2r/(s+r)
    """
    CNa = (4.0 * (semi_span / body_d) ** 2) / \
          (1.0 + math.sqrt(1.0 + (2.0 * semi_span / (cr + ct)) ** 2))
    r = body_d / 2.0
    Kfb = 1.0 + r / (semi_span + r)          # fin-on-body
    Kbf = r / (semi_span + r)                 # body carryover
    return CNa * (Kfb + Kbf)


def fin_planform_area(cr, ct, semi_span):
    return 0.5 * (cr + ct) * semi_span


def fin_spanwise_cp(cr, ct, semi_span):
    """Spanwise CP from body surface [m]."""
    return (semi_span / 3.0) * (cr + 2.0 * ct) / (cr + ct)


def fin_chordwise_cp(cr, ct, semi_span, sweep_deg):
    """Chordwise CP from root LE [m]."""
    Xle = semi_span * math.tan(math.radians(sweep_deg))
    x1 = (Xle / 3.0) * (cr + 2.0 * ct) / (cr + ct)
    x2 = (1.0 / 6.0) * (cr + ct - (cr * ct) / (cr + ct))
    return x1 + x2


def roll_moment_arm(body_d, cr, ct, semi_span):
    return body_d / 2.0 + fin_spanwise_cp(cr, ct, semi_span)


def compute_forces(cant_deg, params):
    """Compute forces on a single fin at a given cant angle."""
    S_ref = math.pi * (params["body_d"] / 2.0) ** 2   # body cross-section
    CNa = barrowman_fin_CNa(params["body_d"], params["semi_span"],
                            params["cr"], params["ct"])
    arm = roll_moment_arm(params["body_d"], params["cr"],
                          params["ct"], params["semi_span"])

    cant_rad = math.radians(cant_deg)
    aoa_rad = math.radians(params["aoa"])
    alpha_eff = aoa_rad + cant_rad

    q = 0.5 * params["rho"] * params["airspeed"] ** 2
    F_normal = q * S_ref * CNa * alpha_eff * params["cfd_scale"]

    F_roll = F_normal * math.sin(cant_rad)
    F_axial = F_normal * math.cos(cant_rad)
    tau_roll = F_roll * arm

    return {
        "q": q, "alpha_eff": alpha_eff,
        "F_normal": F_normal, "F_roll": F_roll,
        "F_axial": F_axial, "tau_roll": tau_roll,
    }


# ═══════════════════════════════════════════════════════════════
#  DARK THEME COLOURS
# ═══════════════════════════════════════════════════════════════
BG       = "#1a1a1a"
BG_PANEL = "#222222"
BG_INPUT = "#2a2a2a"
FG       = "#d0d0d0"
FG_DIM   = "#888888"
FG_HEAD  = "#ffffff"
ACCENT   = "#8a9bae"
BORDER   = "#444444"
COL_NORM = "#c27c6e"
COL_ROLL = "#7aab85"
COL_TORQ = "#8a9bae"


CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "rocket_physics_defaults.json")


def _load_saved_defaults():
    """Load user-saved defaults from JSON, or return empty dict."""
    try:
        with open(CONFIG_PATH, "r") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


# ═══════════════════════════════════════════════════════════════
#  GUI APPLICATION
# ═══════════════════════════════════════════════════════════════

class App:
    def __init__(self, root):
        self.root = root
        root.title("KittyV1 — Fin Roll Force Calculator")
        root.configure(bg=BG)
        root.minsize(1100, 700)

        # ── Style ─────────────────────────────────────────
        style = ttk.Style()
        style.theme_use("clam")
        style.configure(".", background=BG, foreground=FG,
                        fieldbackground=BG_INPUT, borderwidth=0)
        style.configure("TLabel", background=BG, foreground=FG,
                        font=("Consolas", 10))
        style.configure("Head.TLabel", foreground=FG_HEAD,
                        font=("Consolas", 11, "bold"))
        style.configure("Dim.TLabel", foreground=FG_DIM,
                        font=("Consolas", 9))
        style.configure("Result.TLabel", foreground=ACCENT,
                        font=("Consolas", 10))
        style.configure("TButton", background="#333333", foreground=FG,
                        font=("Consolas", 10, "bold"), padding=(12, 6))
        style.map("TButton",
                  background=[("active", "#555555")],
                  foreground=[("active", "#ffffff")])
        style.configure("TFrame", background=BG)
        style.configure("Panel.TFrame", background=BG_PANEL)

        # ── Layout: left inputs | right plot ──────────────
        self.left = ttk.Frame(root, style="Panel.TFrame", padding=16)
        self.left.pack(side=tk.LEFT, fill=tk.Y, padx=(8, 0), pady=8)

        self.right = ttk.Frame(root, padding=4)
        self.right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True,
                        padx=8, pady=8)

        # ── Input fields ──────────────────────────────────
        self.vars = {}
        self._build_inputs()

        # ── Buttons ───────────────────────────────────────
        btn_row = ttk.Frame(self.left, style="Panel.TFrame")
        btn_row.pack(fill=tk.X, pady=(16, 4))
        ttk.Button(btn_row, text="Calculate",
                   command=self.calculate).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 4))
        ttk.Button(btn_row, text="Fin Preview",
                   command=self.show_fin_preview).pack(side=tk.RIGHT, expand=True, fill=tk.X, padx=(4, 0))

        btn_row2 = ttk.Frame(self.left, style="Panel.TFrame")
        btn_row2.pack(fill=tk.X, pady=(4, 0))
        ttk.Button(btn_row2, text="Save Defaults",
                   command=self.save_defaults).pack(fill=tk.X)

        # ── Fin planform area box ────────────────────────
        area_row = ttk.Frame(self.left, style="Panel.TFrame")
        area_row.pack(fill=tk.X, pady=(10, 0))
        ttk.Label(area_row, text="Fin planform area:",
                  style="Dim.TLabel").pack(side=tk.LEFT)
        self.area_var = tk.StringVar(value="—")
        tk.Entry(
            area_row, textvariable=self.area_var, width=14,
            bg=BG_INPUT, fg=ACCENT, font=("Consolas", 10, "bold"),
            relief=tk.FLAT, borderwidth=0, highlightthickness=1,
            highlightbackground=BORDER, highlightcolor=ACCENT,
            readonlybackground=BG_INPUT, state="readonly", justify=tk.RIGHT,
        ).pack(side=tk.RIGHT)

        # ── Results text ──────────────────────────────────
        ttk.Label(self.left, text="Results",
                  style="Head.TLabel").pack(anchor=tk.W, pady=(12, 4))

        self.result_text = tk.Text(
            self.left, height=22, width=48,
            bg=BG_INPUT, fg=FG, insertbackground=FG,
            font=("Consolas", 9), relief=tk.FLAT,
            borderwidth=0, highlightthickness=1,
            highlightbackground=BORDER, highlightcolor=ACCENT,
            state=tk.DISABLED, wrap=tk.NONE,
        )
        self.result_text.pack(fill=tk.X, pady=(0, 4))

        # ── Matplotlib figure ─────────────────────────────
        self.fig = Figure(figsize=(7, 6), dpi=100, facecolor=BG)
        self.ax = self.fig.add_subplot(111)

        self.ax.set_facecolor(BG_PANEL)
        self.ax.tick_params(colors=FG_DIM, labelsize=8)
        for spine in self.ax.spines.values():
            spine.set_color(BORDER)
        self.ax.grid(True, alpha=0.2, color="#555555")
        self.ax.axhline(0, color="#555555", lw=0.5)

        self.ax.set_ylabel("Normal force (N)", color=FG, fontsize=9)
        self.ax.set_xlabel("Fin cant angle (deg)", color=FG, fontsize=9)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.right)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # ── Initial calculation ───────────────────────────
        self.calculate()

    # ── Build input fields ────────────────────────────────
    def _build_inputs(self):
        saved = _load_saved_defaults()
        fields = [
            ("Air", None),
            ("Air density (kg/m³)", "rho", 1.225),
            ("Body", None),
            ("Body diameter (mm)", "body_d", 56.3),
            ("Fin Geometry", None),
            ("Root chord (mm)", "cr", 63.0),
            ("Tip chord (mm)", "ct", 51.0),
            ("Semi-span (mm)", "semi_span", 50.0),
            ("Sweep angle (deg)", "sweep", 27.0),
            ("Fin count", "fin_count", 3),
            ("Flight", None),
            ("Angle of attack (deg)", "aoa", 0.0),
            ("Airspeed (m/s)", "airspeed", 60.0),
            ("CFD scale factor", "cfd_scale", 1.0),
            ("Cant Sweep", None),
            ("Cant min (deg)", "cant_min", -20.0),
            ("Cant max (deg)", "cant_max", 20.0),
            ("Cant steps", "cant_steps", 20),
        ]

        for item in fields:
            if item[1] is None:
                # Section header
                ttk.Label(self.left, text=item[0],
                          style="Head.TLabel").pack(
                    anchor=tk.W, pady=(10, 2))
                continue

            label, key, default = item
            default = saved.get(key, default)
            row = ttk.Frame(self.left, style="Panel.TFrame")
            row.pack(fill=tk.X, pady=1)

            ttk.Label(row, text=label, style="Dim.TLabel").pack(
                side=tk.LEFT, padx=(0, 8))

            var = tk.StringVar(value=str(default))
            entry = tk.Entry(
                row, textvariable=var, width=10,
                bg=BG_INPUT, fg=FG, insertbackground=FG,
                font=("Consolas", 10), relief=tk.FLAT,
                borderwidth=0, highlightthickness=1,
                highlightbackground=BORDER, highlightcolor=ACCENT,
                justify=tk.RIGHT,
            )
            entry.pack(side=tk.RIGHT)
            entry.bind("<Return>", lambda e: self.calculate())
            self.vars[key] = var

    # ── Save / load defaults ─────────────────────────────
    def save_defaults(self):
        data = {key: var.get() for key, var in self.vars.items()}
        with open(CONFIG_PATH, "w") as f:
            json.dump(data, f, indent=2)

    # ── Read inputs → params dict ─────────────────────────
    def _read_params(self):
        def f(key):
            return float(self.vars[key].get())

        return {
            "rho":       f("rho"),
            "body_d":    f("body_d") / 1000.0,    # mm → m
            "cr":        f("cr") / 1000.0,
            "ct":        f("ct") / 1000.0,
            "semi_span": f("semi_span") / 1000.0,
            "sweep":     f("sweep"),
            "fin_count": int(f("fin_count")),
            "aoa":       f("aoa"),
            "airspeed":  f("airspeed"),
            "cfd_scale": f("cfd_scale"),
            "cant_min":  f("cant_min"),
            "cant_max":  f("cant_max"),
            "cant_steps": int(f("cant_steps")),
        }

    # ── Calculate & update ────────────────────────────────
    def calculate(self):
        try:
            p = self._read_params()
        except ValueError:
            return

        n = max(p["cant_steps"], 2)
        cants = [p["cant_min"] + (p["cant_max"] - p["cant_min"]) * i / (n - 1)
                 for i in range(n)]
        results = [compute_forces(c, p) for c in cants]

        # ── Update plot ───────────────────────────────────
        self.ax.clear()
        self.ax.set_facecolor(BG_PANEL)
        self.ax.tick_params(colors=FG_DIM, labelsize=8)
        for spine in self.ax.spines.values():
            spine.set_color(BORDER)
        self.ax.grid(True, alpha=0.2, color="#555555")
        self.ax.axhline(0, color="#555555", lw=0.5)

        self.ax.plot(cants, [r["F_normal"] for r in results],
                     color=COL_NORM, lw=1.8, label="Normal")
        self.ax.plot(cants, [r["F_axial"] for r in results],
                     color=COL_TORQ, lw=1.8, label="Axial")
        self.ax.legend(facecolor=BG_PANEL, edgecolor=BORDER,
                       labelcolor=FG, fontsize=8)
        self.ax.set_ylabel("Force (N)", color=FG, fontsize=9)
        self.ax.set_xlabel("Fin cant angle (deg)", color=FG, fontsize=9)

        self.fig.suptitle(
            f"Single-Fin Forces  —  V = {p['airspeed']:.0f} m/s,  "
            f"AoA = {p['aoa']:.1f}°,  Ø{p['body_d']*1000:.1f} mm",
            fontweight="bold", color=FG_HEAD, fontsize=11,
        )
        self.canvas.draw()

        # ── Update results text ───────────────────────────
        CNa = barrowman_fin_CNa(p["body_d"], p["semi_span"],
                                p["cr"], p["ct"])
        arm = roll_moment_arm(p["body_d"], p["cr"], p["ct"],
                              p["semi_span"])
        area = fin_planform_area(p["cr"], p["ct"], p["semi_span"])
        self.area_var.set(f"{area * 1e4:.2f} cm²")
        y_cp = fin_spanwise_cp(p["cr"], p["ct"], p["semi_span"])
        x_cp = fin_chordwise_cp(p["cr"], p["ct"], p["semi_span"],
                                p["sweep"])
        q = 0.5 * p["rho"] * p["airspeed"] ** 2

        # Find peak values
        peak_norm = max(results, key=lambda r: abs(r["F_normal"]))
        peak_roll = max(results, key=lambda r: abs(r["F_roll"]))
        peak_axl  = max(results, key=lambda r: abs(r["F_axial"]))
        peak_torq = max(results, key=lambda r: abs(r["tau_roll"]))

        nf = p["fin_count"]
        lines = [
            f"  Fin CNα:         {CNa:.4f}  /rad",
            f"  Planform area:   {area * 1e4:.2f}  cm²",
            f"  Spanwise CP:     {y_cp * 1000:.2f}  mm  (from body)",
            f"  Chordwise CP:    {x_cp * 1000:.2f}  mm  (from root LE)",
            f"  Moment arm:      {arm * 1000:.2f}  mm  (from axis)",
            f"  Dyn. pressure:   {q:.1f}  Pa",
            f"  Fin count:       {nf}",
            f"  ─────────────────────────────────────",
            f"  Peak normal:     {peak_norm['F_normal']:.4f}  N",
            f"  Peak axial:      {peak_axl['F_axial']:.4f}  N",
            f"  Peak roll force: {peak_roll['F_roll']:.5f}  N",
            f"  Peak torque:     {peak_torq['tau_roll']*1000:.4f}  mN·m  (1 fin)",
            f"  Total torque:    {peak_torq['tau_roll']*1000*nf:.4f}  mN·m  ({nf} fins)",
            f"",
            f"  Cant°  Normal    Axial     Roll      Torque",
            f"  ─────────────────────────────────────────────",
        ]
        for deg in [5, 10, 15, 20]:
            r = compute_forces(deg, p)
            lines.append(
                f"  {deg:4d}°  {r['F_normal']:8.4f}  "
                f"{r['F_axial']:8.4f}  {r['F_roll']:8.5f}  "
                f"{r['tau_roll']*1000:8.4f}"
            )

        self.result_text.config(state=tk.NORMAL)
        self.result_text.delete("1.0", tk.END)
        self.result_text.insert(tk.END, "\n".join(lines))
        self.result_text.config(state=tk.DISABLED)

    # ── Fin preview window ────────────────────────────────
    def show_fin_preview(self):
        try:
            p = self._read_params()
        except ValueError:
            return

        # All dims in mm for display
        cr_mm   = p["cr"] * 1000
        ct_mm   = p["ct"] * 1000
        S_mm    = p["semi_span"] * 1000
        d_mm    = p["body_d"] * 1000
        r_mm    = d_mm / 2.0
        sweep   = p["sweep"]

        # Leading-edge offset at tip
        xle = S_mm * math.tan(math.radians(sweep))

        # Fin corners (x = chordwise, y = spanwise from centreline)
        # Root sits at body surface: y = r_mm
        fin_x = [0, xle, xle + ct_mm, cr_mm, 0]
        fin_y = [r_mm, r_mm + S_mm, r_mm + S_mm, r_mm, r_mm]

        # Centre of pressure
        y_cp_mm = r_mm + fin_spanwise_cp(p["cr"], p["ct"], p["semi_span"]) * 1000
        x_cp_mm = fin_chordwise_cp(p["cr"], p["ct"], p["semi_span"], sweep) * 1000

        # Body tube rectangle (partial, behind fin)
        body_len = max(cr_mm * 1.6, xle + ct_mm + 10)
        body_x0  = -cr_mm * 0.2

        # ── Create window ─────────────────────────────────
        win = tk.Toplevel(self.root)
        win.title("Fin Preview")
        win.configure(bg=BG)
        win.geometry("520x480")

        fig = Figure(figsize=(5, 4.5), dpi=100, facecolor=BG)
        ax = fig.add_subplot(111)
        ax.set_facecolor(BG_PANEL)
        ax.set_aspect("equal")
        ax.tick_params(colors=FG_DIM, labelsize=8)
        for spine in ax.spines.values():
            spine.set_color(BORDER)
        ax.grid(True, alpha=0.15, color="#555555")

        # Body tube (rectangle from -r to +r)
        from matplotlib.patches import Rectangle
        body_rect = Rectangle((body_x0, -r_mm), body_len, d_mm,
                               linewidth=1.2, edgecolor="#666666",
                               facecolor="#2a2a2a", zorder=1)
        ax.add_patch(body_rect)

        # Centreline
        ax.axhline(0, color="#555555", lw=0.8, ls="--", zorder=0)
        ax.text(body_x0 + 2, 1.5, "CL", color="#555555", fontsize=7)

        # Fin polygon (top side)
        ax.fill(fin_x, fin_y, color="#3a5a4a", alpha=0.6, zorder=2)
        ax.plot(fin_x, fin_y, color=COL_ROLL, lw=1.8, zorder=3)

        # Mirror fin (bottom side)
        fin_y_bot = [-y for y in fin_y]
        ax.fill(fin_x, fin_y_bot, color="#3a5a4a", alpha=0.6, zorder=2)
        ax.plot(fin_x, fin_y_bot, color=COL_ROLL, lw=1.8, zorder=3)

        # CP marker (top fin only)
        ax.plot(x_cp_mm, y_cp_mm, "o", color=COL_NORM, ms=8, zorder=5)
        ax.annotate(f"CP", (x_cp_mm, y_cp_mm),
                    textcoords="offset points", xytext=(8, 6),
                    color=COL_NORM, fontsize=9, fontweight="bold")

        # ── Dimension annotations ─────────────────────────
        dim_col = "#888888"
        dim_fs = 8

        # Root chord
        ax.annotate("", xy=(cr_mm, r_mm - 3), xytext=(0, r_mm - 3),
                     arrowprops=dict(arrowstyle="<->", color=dim_col, lw=0.8))
        ax.text(cr_mm / 2, r_mm - 5.5, f"Cr = {cr_mm:.1f}",
                ha="center", color=dim_col, fontsize=dim_fs)

        # Tip chord
        ax.annotate("", xy=(xle + ct_mm, r_mm + S_mm + 3),
                     xytext=(xle, r_mm + S_mm + 3),
                     arrowprops=dict(arrowstyle="<->", color=dim_col, lw=0.8))
        ax.text(xle + ct_mm / 2, r_mm + S_mm + 5,
                f"Ct = {ct_mm:.1f}",
                ha="center", color=dim_col, fontsize=dim_fs)

        # Semi-span (right side)
        x_right = max(cr_mm, xle + ct_mm) + 6
        ax.annotate("", xy=(x_right, r_mm + S_mm), xytext=(x_right, r_mm),
                     arrowprops=dict(arrowstyle="<->", color=dim_col, lw=0.8))
        ax.text(x_right + 3, r_mm + S_mm / 2,
                f"S = {S_mm:.1f}",
                color=dim_col, fontsize=dim_fs, va="center")

        # Body diameter (left side)
        ax.annotate("", xy=(body_x0 - 4, r_mm), xytext=(body_x0 - 4, -r_mm),
                     arrowprops=dict(arrowstyle="<->", color=dim_col, lw=0.8))
        ax.text(body_x0 - 6, 0,
                f"Ø{d_mm:.1f}",
                color=dim_col, fontsize=dim_fs, va="center", ha="right")

        # Sweep angle arc
        arc_r = S_mm * 0.35
        arc_angles = [90 - sweep + i * sweep / 20 for i in range(21)]
        arc_xs = [arc_r * math.cos(math.radians(a)) for a in arc_angles]
        arc_ys = [r_mm + arc_r * math.sin(math.radians(a)) for a in arc_angles]
        ax.plot(arc_xs, arc_ys, color=dim_col, lw=0.7)
        ax.text(arc_r * 0.4, r_mm + arc_r * 0.85,
                f"{sweep:.0f}°", color=dim_col, fontsize=dim_fs)

        ax.set_xlabel("Chordwise (mm)", color=FG, fontsize=9)
        ax.set_ylabel("Spanwise (mm)", color=FG, fontsize=9)
        ax.set_title("Fin Profile  (front view)", color=FG_HEAD,
                     fontsize=11, fontweight="bold", pad=10)

        # Auto-margin
        margin = max(S_mm, cr_mm) * 0.2
        ax.set_xlim(body_x0 - 15, max(cr_mm, xle + ct_mm) + 20)
        ax.set_ylim(-(r_mm + S_mm + margin), r_mm + S_mm + margin)

        fig.tight_layout()
        canvas = FigureCanvasTkAgg(fig, master=win)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        canvas.draw()


# ═══════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════
if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
