# Barrowman Single-Fin Roll Force Equations

Reference: Barrowman, J.S. (1967) — *The Practical Calculation of the Aerodynamic Characteristics of Slender Finned Vehicles*

---

## 1. Fin Geometry

Trapezoidal fin defined by:

| Symbol | Description |
|--------|-------------|
| Cr | Root chord (at body surface) |
| Ct | Tip chord |
| S | Semi-span (fin height from body surface) |
| d | Body tube outer diameter |
| r | Body radius = d / 2 |
| Λ | Leading-edge sweep angle |

Planform area:

    A_fin = ½ · (Cr + Ct) · S

Leading-edge offset at tip:

    X_le = S · tan(Λ)

---

## 2. Normal Force Coefficient Gradient (CNα)

For a single fin (n = 1):

    CNα = [4 · (S/d)²] / [1 + √(1 + (2·S / (Cr + Ct))²)]

This gives the rate of change of normal force coefficient with angle of attack, per radian.

### Fin-Body Interference Factor

A fin mounted on a cylindrical body generates more lift than a fin in freestream due to the body acting as a reflection plane. The correction factor:

    K_fb = 1 + d / (2·S + d)

Final single-fin CNα:

    CNα_fin = CNα · K_fb

---

## 3. Centre of Pressure

### Spanwise CP (distance from body surface)

    y_cp = (S / 3) · (Cr + 2·Ct) / (Cr + Ct)

This is the spanwise location where the resultant aerodynamic force acts. For a rectangular fin (Cr = Ct), this simplifies to S/3. For a highly tapered fin, it shifts inboard.

### Chordwise CP (distance from root leading edge)

    x_cp = (X_le / 3) · (Cr + 2·Ct) / (Cr + Ct)  +  (1/6) · (Cr + Ct − Cr·Ct / (Cr + Ct))

where X_le = S · tan(Λ).

### Roll Moment Arm

Distance from the rocket centreline (roll axis) to the spanwise CP:

    r_arm = r + y_cp = d/2 + y_cp

---

## 4. Dynamic Pressure

    q = ½ · ρ · V²

where:
- ρ = air density (kg/m³)
- V = airspeed (m/s)

---

## 5. Normal Force on the Fin

    F_normal = q · S_ref · CNα_fin · α_eff

where:
- S_ref = π · r² = body cross-sectional area (reference area)
- α_eff = effective angle of attack seen by the fin (radians)

### Effective Angle of Attack

For a canted fin at cant angle δ with rocket at angle of attack α:

    α_eff = α + δ

At zero rocket AoA (typical in straight flight): α_eff = δ

---

## 6. Force Decomposition

The normal force acts perpendicular to the fin surface. For a canted fin, decompose into:

### Roll Force (circumferential component)

    F_roll = F_normal · sin(δ)

This is the tangential force that drives rotation about the roll axis.

### Axial Force (in-plane component)

    F_axial = F_normal · cos(δ)

This acts in the fin plane (contributes to pitch/yaw restoring moment, not roll).

---

## 7. Roll Torque

    τ_roll = F_roll · r_arm

This is the moment about the rocket centreline that causes the rocket to spin.

Expanding fully:

    τ_roll = ½ · ρ · V² · π·r² · CNα_fin · (α + δ) · sin(δ) · (r + y_cp)

---

## 8. Variable Summary

| Variable | Unit | Description |
|----------|------|-------------|
| ρ | kg/m³ | Air density |
| V | m/s | Airspeed |
| d | m | Body diameter |
| Cr | m | Root chord |
| Ct | m | Tip chord |
| S | m | Semi-span |
| Λ | deg | Sweep angle |
| δ | deg | Fin cant angle |
| α | deg | Rocket angle of attack |
| q | Pa | Dynamic pressure |
| CNα_fin | /rad | Normal force coefficient gradient (single fin) |
| K_fb | — | Fin-body interference factor |
| y_cp | m | Spanwise centre of pressure |
| x_cp | m | Chordwise centre of pressure |
| r_arm | m | Roll moment arm (centreline to CP) |
| F_normal | N | Total normal force on fin |
| F_roll | N | Circumferential roll-driving force |
| F_axial | N | In-plane force component |
| τ_roll | N·m | Roll torque about centreline |

---

## 9. Assumptions & Limitations

- **Subsonic flow** — Barrowman equations assume incompressible flow (M < 0.3). Corrections needed above Mach 0.3.
- **Small angles** — CNα is a linear approximation valid for small α (< ~15°). At large angles, non-linear effects dominate.
- **Thin fins** — Fin thickness is ignored in the aerodynamic model. Only planform shape matters.
- **No fin-fin interference** — The single-fin CNα doesn't account for wake effects from other fins. K_fb only models the body reflection effect.
- **Rigid body** — No aeroelastic deflection of the fin.
- **Zero sideslip** — The rocket is assumed to fly with zero sideslip angle.
