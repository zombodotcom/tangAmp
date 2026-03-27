# Proper WDF Triode Stage Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the simplified load-line model in `wdf_triode.v` with a proper Wave Digital Filter binary tree that matches the Python reference simulation's 29.1dB gain and ~14V output swing.

**Architecture:** WDF binary tree with series/parallel adaptors connecting linear one-port elements (resistors, capacitors, voltage source) to a nonlinear triode root. The triode root uses 1-2 fixed Newton-Raphson iterations per sample with the existing 2D LUTs for Ip and dIp/dVgk. Upward pass collects reflected waves from leaves, root scatters through the tube model, downward pass distributes incident waves back to update capacitor states.

**Tech Stack:** SystemVerilog (iverilog -g2012 for simulation), Q16.16 fixed-point, Python for reference/validation, existing tube_lut_gen.py for LUTs.

---

## Background: Why the Current Code is Wrong

The current `wdf_triode.v` does a **single LUT lookup** per sample using stale voltage estimates from the previous sample. There is no iteration to find the self-consistent operating point where the load line intersects the tube curve. This produces 47.2dB gain and ±60-100V output swings instead of the correct 29.1dB and ±14V.

Additionally:
- DC output subtraction uses hardcoded 100V instead of the actual quiescent Vplate (~146V)
- Cathode voltage is computed from previous sample's Ip (stale)
- HP filter uses a single coefficient instead of proper bilinear transform
- Only 500 settling samples (needs ~2000)

## WDF Circuit Topology (Binary Tree)

```
Common-Cathode 12AX7 Triode Stage:

        Vin --[Cin]--+--[Rg]-- Grid
                     |
              Triode (Grid, Plate, Cathode)
                     |
        B+ --[Rp]---+--- Plate
                     |
        GND --[Ck]--+--[Rk]-- Cathode


WDF Binary Tree (triode at root):

                    [TRIODE 3-PORT ROOT]
                   /        |          \
            plate_tree   grid_tree   cathode_tree
               |            |              |
           [Series]     [Series]      [Parallel]
           /      \     /      \      /        \
         Rp    Vs(B+) Cin      Rg   Rk         Ck
```

**Port Resistances (at 48kHz):**
- Rp: R0 = 100kΩ
- Vs (B+ ideal voltage source): R0 ≈ 0 (use small value, e.g. 1Ω)
- Cin (22nF): R0 = 1/(2×48000×22e-9) = 473.5Ω
- Rg: R0 = 1MΩ
- Rk: R0 = 1.5kΩ
- Ck (22µF): R0 = 1/(2×48000×22e-6) = 0.4735Ω

**Adaptor Port Resistances (propagate up):**
- Plate series: R_plate = R_Rp + R_Vs = 100000 + 1 = 100001Ω
- Grid series: R_grid = R_Cin + R_Rg = 473.5 + 1000000 = 1000473.5Ω
- Cathode parallel: R_cath = 1/(1/R_Rk + 1/R_Ck) = 1/(1/1500 + 1/0.4735) = ~0.4734Ω

**Scattering Coefficients:**
- Plate series: γ_plate = R_Rp / (R_Rp + R_Vs) = 100000/100001 ≈ 0.99999
- Grid series: γ_grid = R_Cin / (R_Cin + R_Rg) = 473.5/1000473.5 ≈ 0.000473
- Cathode parallel: γ_cath = G_Rk / (G_Rk + G_Ck) = (1/1500) / (1/1500 + 1/0.4735) ≈ 0.000316

## File Structure

| File | Responsibility | Action |
|------|----------------|--------|
| `wdf_triode_wdf.v` | New proper WDF triode module | Create |
| `wdf_triode_wdf_tb.v` | Testbench for new WDF module | Create |
| `wdf_triode_sim_wdf.py` | Python WDF reference sim (wave variables) | Create |
| `validate_wdf.py` | Cross-validate Python WDF vs Python Newton vs Verilog WDF | Create |
| `tube_lut_gen.py` | LUT generator — add dIp/dVpk derivative LUT | Modify |
| `wdf_triode.v` | Keep as-is for comparison | No change |
| `wdf_triode_tb.v` | Keep as-is for comparison | No change |
| `analyze_tb.py` | Reuse for new testbench output | No change |

---

## Task 1: Add dIp/dVpk Derivative LUT to Generator

Newton-Raphson at the triode root needs both partial derivatives: dIp/dVgk (already generated) AND dIp/dVpk (needed for the full Jacobian).

**Files:**
- Modify: `tube_lut_gen.py:54-74` (add dIp/dVpk function)
- Modify: `tube_lut_gen.py:86-112` (generate and write third LUT)
- Modify: `tube_lut_gen.py:115-142` (add third LUT params to Verilog output)

- [ ] **Step 1: Add dIp/dVpk analytical derivative function**

Add after the existing `koren_dip_dvgk` function (line 74):

```python
def koren_dip_dvpk(Vpk, Vgk, mu, ex, kg1, kp, kvb):
    """
    Analytical derivative dIp/dVpk of the Koren model.
    Needed for Newton-Raphson Jacobian in WDF nonlinear root.
    """
    sqrt_term = np.sqrt(kvb + Vpk**2 + 1e-9)
    inner = kp * (1.0/mu + Vgk / sqrt_term)
    inner_c = np.clip(inner, -500, 500)

    Ed = (Vpk / kp) * np.log1p(np.exp(inner_c))
    Ed = np.maximum(Ed, 0.0)

    log1p_val = np.log1p(np.exp(inner_c))
    sig = np.exp(inner_c) / (1.0 + np.exp(inner_c))

    # dEd/dVpk has two terms via product rule:
    # d/dVpk [(Vpk/kp) * log1p(exp(inner))]
    # = (1/kp)*log1p(exp(inner)) + (Vpk/kp)*sig*d(inner)/dVpk
    # d(inner)/dVpk = kp * Vgk * (-Vpk) / (kvb + Vpk^2)^(3/2)
    dinner_dvpk = kp * Vgk * (-Vpk) / (sqrt_term**3)
    dEd_dVpk = (1.0/kp) * log1p_val + (Vpk/kp) * sig * dinner_dvpk

    dIp = np.where(Ed > 0,
                   (ex / kg1) * Ed**(ex - 1.0) * dEd_dVpk,
                   0.0)
    return dIp
```

- [ ] **Step 2: Generate third LUT grid and write hex file**

After line 87 (`dip_grid = ...`), add:

```python
dip_vpk_grid = koren_dip_dvpk(Vpk_grid, Vgk_grid, **p)
print(f"  dIp/dVpk range: {dip_vpk_grid.min():.6f} to {dip_vpk_grid.max():.6f} A/V")
```

After line 100 (`dip_fixed = ...`), add:

```python
DIP_VPK_SCALE = 1e5  # same scale as dIp/dVgk
dip_vpk_fixed = to_fixed16(dip_vpk_grid, DIP_VPK_SCALE)
```

After line 112 (`write_hex("dip_dvgk_lut.hex", ...)`), add:

```python
write_hex("dip_dvpk_lut.hex", dip_vpk_fixed)
```

Update the Verilog params string (line 134) to add:

```python
parameter DIP_VPK_SCALE = {int(DIP_VPK_SCALE)};  // dIp/dVpk * scale = 16-bit int
```

- [ ] **Step 3: Regenerate LUTs and verify**

Run: `python tube_lut_gen.py`

Expected output includes:
```
  Written: ip_lut.hex (65536 entries)
  Written: dip_dvgk_lut.hex (65536 entries)
  Written: dip_dvpk_lut.hex (65536 entries)
```

- [ ] **Step 4: Commit**

```bash
git add tube_lut_gen.py lut_params.v
git commit -m "feat: add dIp/dVpk derivative LUT for WDF Newton-Raphson Jacobian"
```

---

## Task 2: Python WDF Reference Simulation

Build a Python simulation using actual WDF wave variables (incident/reflected waves) that we can validate the Verilog against. This is the ground truth.

**Files:**
- Create: `wdf_triode_sim_wdf.py`

- [ ] **Step 1: Write the WDF one-port element classes**

Create `wdf_triode_sim_wdf.py`:

```python
"""
wdf_triode_sim_wdf.py
WDF simulation of common-cathode 12AX7 using wave variables.

This is the reference implementation for the Verilog WDF module.
Uses actual incident/reflected wave pairs through a binary tree,
NOT the simplified Newton-Raphson-on-circuit-equations approach.
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ═══════════════════════════════════════════════════════════════════════════════
# Circuit Parameters
# ═══════════════════════════════════════════════════════════════════════════════

VB  = 200.0       # B+ supply (V)
RP  = 100000.0    # Plate resistor (Ω)
RG  = 1000000.0   # Grid resistor (Ω)
RK  = 1500.0      # Cathode resistor (Ω)
CIN = 22e-9       # Input coupling cap (F)
CK  = 22e-6       # Cathode bypass cap (F)
FS  = 48000.0     # Sample rate (Hz)

# Koren 12AX7 parameters
MU, EX, KG1, KP, KVB = 100.0, 1.4, 1060.0, 600.0, 300.0

# ═══════════════════════════════════════════════════════════════════════════════
# WDF One-Port Elements
# ═══════════════════════════════════════════════════════════════════════════════

class Resistor:
    def __init__(self, R):
        self.R = R          # port resistance = resistance value
        self.a = 0.0        # incident wave
        self.b = 0.0        # reflected wave

    def reflected(self):
        self.b = 0.0        # resistor absorbs everything
        return self.b

    def incident(self, x):
        self.a = x

class Capacitor:
    """Bilinear-transform discretized capacitor."""
    def __init__(self, C, fs):
        self.R = 1.0 / (2.0 * fs * C)  # port resistance
        self.a = 0.0
        self.b = 0.0
        self.z = 0.0        # state (previous incident wave)

    def reflected(self):
        self.b = self.z     # reflect stored state
        return self.b

    def incident(self, x):
        self.a = x
        self.z = x          # store for next sample

class VoltageSource:
    """Ideal voltage source with small series resistance."""
    def __init__(self, V, R_series=1.0):
        self.V = V
        self.R = R_series   # small but nonzero for WDF compatibility
        self.a = 0.0
        self.b = 0.0

    def reflected(self):
        self.b = 2.0 * self.V - self.a  # reflects to maintain voltage
        return self.b

    def incident(self, x):
        self.a = x

# ═══════════════════════════════════════════════════════════════════════════════
# WDF Adaptors
# ═══════════════════════════════════════════════════════════════════════════════

class SeriesAdaptor:
    """3-port series adaptor. port1 and port2 are children, parent faces up."""
    def __init__(self, port1, port2):
        self.port1 = port1
        self.port2 = port2
        self.R = port1.R + port2.R   # series: resistances add
        self.gamma = port1.R / self.R  # scattering coefficient
        self.a = 0.0   # incident from parent
        self.b = 0.0   # reflected to parent

    def reflected(self):
        """Upward pass: collect reflected waves from children."""
        b1 = self.port1.reflected()
        b2 = self.port2.reflected()
        self.b = -(b1 + b2)  # series junction
        return self.b

    def incident(self, x):
        """Downward pass: scatter incident wave to children."""
        self.a = x
        b1 = self.port1.b
        b2 = self.port2.b
        # Scatter to port1
        a1 = b1 - self.gamma * (x + b1 + b2)
        self.port1.incident(a1)
        # Scatter to port2 (from KVL constraint)
        a2 = -(x + a1)
        self.port2.incident(a2)

class ParallelAdaptor:
    """3-port parallel adaptor. port1 and port2 are children, parent faces up."""
    def __init__(self, port1, port2):
        self.port1 = port1
        self.port2 = port2
        G1 = 1.0 / port1.R
        G2 = 1.0 / port2.R
        self.R = 1.0 / (G1 + G2)     # parallel: conductances add
        self.gamma = G1 / (G1 + G2)   # scattering coefficient
        self.a = 0.0
        self.b = 0.0
        self._bDiff = 0.0

    def reflected(self):
        """Upward pass: collect reflected waves from children."""
        b1 = self.port1.reflected()
        b2 = self.port2.reflected()
        self._bDiff = b2 - b1
        self.b = b2 - self.gamma * self._bDiff
        return self.b

    def incident(self, x):
        """Downward pass: scatter incident wave to children."""
        self.a = x
        b2_down = x + self.b - self.port2.b  # simplification from chowdsp_wdf
        self.port2.incident(b2_down)
        self.port1.incident(b2_down + self._bDiff)

# ═══════════════════════════════════════════════════════════════════════════════
# Koren Triode Model
# ═══════════════════════════════════════════════════════════════════════════════

def koren_ip(vpk, vgk):
    """Plate current in amps."""
    vpk = np.clip(vpk, 0.0, 500.0)
    vgk = np.clip(vgk, -10.0, 1.0)
    if vpk <= 0:
        return 0.0
    inner = KP * (1.0/MU + vgk / np.sqrt(KVB + vpk**2))
    inner = np.clip(inner, -500, 500)
    Ed = (vpk / KP) * np.log1p(np.exp(float(inner)))
    if Ed <= 0:
        return 0.0
    return float((Ed ** EX) / KG1)

def koren_dip_dvgk(vpk, vgk, h=0.001):
    """Numerical dIp/dVgk."""
    return (koren_ip(vpk, vgk + h) - koren_ip(vpk, vgk - h)) / (2*h)

def koren_dip_dvpk(vpk, vgk, h=0.01):
    """Numerical dIp/dVpk."""
    return (koren_ip(vpk + h, vgk) - koren_ip(vpk - h, vgk)) / (2*h)

# ═══════════════════════════════════════════════════════════════════════════════
# Build WDF Tree
# ═══════════════════════════════════════════════════════════════════════════════

# Leaf elements
res_rp = Resistor(RP)
vs_vb  = VoltageSource(VB, R_series=1.0)
cap_cin = Capacitor(CIN, FS)
res_rg  = Resistor(RG)
res_rk  = Resistor(RK)
cap_ck  = Capacitor(CK, FS)

# Adaptors
plate_tree   = SeriesAdaptor(res_rp, vs_vb)      # Rp + Vs(B+)
grid_tree    = SeriesAdaptor(cap_cin, res_rg)     # Cin + Rg
cathode_tree = ParallelAdaptor(res_rk, cap_ck)    # Rk || Ck

print(f"=== WDF Port Resistances ===")
print(f"Rp:       {res_rp.R:.1f} Ω")
print(f"Vs(B+):   {vs_vb.R:.1f} Ω")
print(f"Cin:      {cap_cin.R:.1f} Ω")
print(f"Rg:       {res_rg.R:.1f} Ω")
print(f"Rk:       {res_rk.R:.1f} Ω")
print(f"Ck:       {cap_ck.R:.1f} Ω")
print(f"Plate tree (series Rp+Vs): {plate_tree.R:.1f} Ω,  γ={plate_tree.gamma:.6f}")
print(f"Grid tree (series Cin+Rg): {grid_tree.R:.1f} Ω,  γ={grid_tree.gamma:.6f}")
print(f"Cathode tree (par Rk||Ck): {cathode_tree.R:.4f} Ω,  γ={cathode_tree.gamma:.6f}")

# ═══════════════════════════════════════════════════════════════════════════════
# Nonlinear Root Solver (Triode at Root of Tree)
# ═══════════════════════════════════════════════════════════════════════════════

def solve_triode_root(a_p, a_g, a_k, R_p, R_g, R_k, ip_prev):
    """
    Given incident waves from plate/grid/cathode subtrees and their port resistances,
    solve for the triode operating point and return reflected waves.

    Wave-voltage relations:
      v = (a + b) / 2
      i = (a - b) / (2*R)

    Triode constraints:
      Ip = koren_ip(Vpk, Vgk)    (plate current from Koren model)
      Ig ≈ 0                      (no grid current for 12AX7 in normal operation)
      Ik = -(Ip + Ig) = -Ip       (KCL at cathode)

    From wave relations:
      Vp = (a_p + b_p) / 2,  ip = (a_p - b_p) / (2*R_p)
      Vg = (a_g + b_g) / 2,  ig = (a_g - b_g) / (2*R_g)
      Vk = (a_k + b_k) / 2,  ik = (a_k - b_k) / (2*R_k)

    Since ig = 0: b_g = a_g (grid reflects fully)
    Since ip = Ip: b_p = a_p - 2*R_p*Ip
    Since ik = -Ip: b_k = a_k + 2*R_k*Ip

    Voltages:
      Vp = (a_p + (a_p - 2*R_p*Ip)) / 2 = a_p - R_p*Ip
      Vg = (a_g + a_g) / 2 = a_g
      Vk = (a_k + (a_k + 2*R_k*Ip)) / 2 = a_k + R_k*Ip

    Therefore:
      Vpk = Vp - Vk = a_p - R_p*Ip - a_k - R_k*Ip = (a_p - a_k) - (R_p + R_k)*Ip
      Vgk = Vg - Vk = a_g - a_k - R_k*Ip

    Equation to solve: f(Ip) = Ip - koren_ip(Vpk(Ip), Vgk(Ip)) = 0
    """
    ip = ip_prev  # initial guess

    for _ in range(8):
        vpk = (a_p - a_k) - (R_p + R_k) * ip
        vgk = a_g - a_k - R_k * ip

        ip_model = koren_ip(vpk, vgk)
        f = ip - ip_model

        if abs(f) < 1e-9:
            break

        # Jacobian: df/dIp = 1 - dIp_model/dIp
        # dIp_model/dIp = dIp/dVpk * dVpk/dIp + dIp/dVgk * dVgk/dIp
        #               = dIp/dVpk * (-(R_p+R_k)) + dIp/dVgk * (-R_k)
        dip_dvpk_val = koren_dip_dvpk(vpk, vgk)
        dip_dvgk_val = koren_dip_dvgk(vpk, vgk)
        df_dip = 1.0 - (dip_dvpk_val * (-(R_p + R_k)) + dip_dvgk_val * (-R_k))

        if abs(df_dip) < 1e-15:
            break

        ip -= f / df_dip
        ip = max(ip, 0.0)

    # Compute reflected waves
    b_p = a_p - 2.0 * R_p * ip     # plate reflected
    b_g = a_g                        # grid reflected (no grid current)
    b_k = a_k + 2.0 * R_k * ip     # cathode reflected

    return b_p, b_g, b_k, ip

# ═══════════════════════════════════════════════════════════════════════════════
# Simulation
# ═══════════════════════════════════════════════════════════════════════════════

n_settle = 2000
n_audio  = 4800
n_total  = n_settle + n_audio

t = np.arange(n_total) / FS
audio_in = np.zeros(n_total)
audio_in[n_settle:] = 0.5 * np.sin(2 * np.pi * 440 * t[n_settle:])

print(f"\nSimulating {n_total} samples (WDF wave variables)...")

# State
ip_prev = 0.5e-3  # initial guess ~0.5mA

out = np.zeros(n_total)
out_vp = np.zeros(n_total)
out_ip = np.zeros(n_total)

for n in range(n_total):
    vin = audio_in[n]

    # Input signal enters through the grid tree's Cin
    # The voltage source model for the input: set Cin's "input side"
    # In WDF, the input voltage modifies the incident wave to Cin
    # For a series adaptor (Cin + Rg), the input signal effectively
    # shifts the grid voltage. We model this by adding vin to the
    # Cin state (the input is a Thevenin source through Cin).
    #
    # Simpler approach: treat input as modifying Cin's reflected wave.
    # The input couples through Cin as an AC signal. In the WDF framework,
    # we add the input voltage to Cin's incident wave calculation.
    # Since Cin state z stores the previous incident wave, and the coupling
    # makes b_cin = z + vin_contribution, we model the input source
    # by modifying the grid tree:
    #
    # Actually, the cleanest way: the input is a voltage source in series
    # with Cin. But to keep the tree binary, we fold the input into Cin.
    # Cin with input: b = z + 2*vin (the vin appears as a Thevenin source)
    # No — let's think more carefully.
    #
    # The input chain is: Vin -> Cin -> Rg -> Grid
    # As a WDF: Vin is a voltage source with ~0Ω series R, in series with Cin.
    # That forms a 3-port series adaptor, but we can simplify:
    # A voltage source in series with a capacitor is equivalent to
    # a capacitor whose state is shifted by the source voltage.
    #
    # For bilinear-transform cap: b = z, incident a updates z = a
    # With a series voltage source vin: the reflected wave becomes b = z,
    # but the voltage across Cin changes. The simplest correct approach:
    # after the adaptor scattering, shift the cap's incident wave by vin.
    #
    # Better: model as vin feeding directly into the grid subtree.
    # Replace cap_cin with a modified element that reflects b = z + 2*vin
    # (the factor 2 comes from the Thevenin equivalent: the reflected wave
    # of an ideal voltage source is b = 2*V - a, for a cap storing vin
    # it becomes b = z + 2*vin... but this isn't quite right either).
    #
    # Cleanest correct approach from chowdsp_wdf:
    # Use a ResistiveVoltageSource: R = R_cin, b = 2*vin + z
    # Actually no. Let me use the standard approach:
    #
    # Add vin as a voltage source (Vs_in) with R=0 (or tiny R) in series
    # with Cin, making a 3-element series chain. But binary tree requires
    # 2-port adaptors. So: Series(Vs_in, Cin) -> new element with R ≈ R_cin.
    # Or more simply: modify Cin's reflected wave to include vin.
    #
    # The standard trick: for a cap with input voltage source in series,
    # b_cin = z (from cap state) and we add 2*vin to the reflected wave.
    # This is because the Thevenin voltage adds to the port voltage.

    # ── Step 1: Set input voltage into Cin ──
    # Store original Cin reflect, override to include vin
    original_z = cap_cin.z
    cap_cin.z = original_z + 2.0 * vin  # input source adds to cap state reflection

    # ── Step 2: Upward pass (reflected waves from leaves to root) ──
    a_p = plate_tree.reflected()
    a_g = grid_tree.reflected()
    a_k = cathode_tree.reflected()

    # Restore Cin state (the +2*vin was only for the reflected wave)
    cap_cin.z = original_z

    # ── Step 3: Nonlinear root (triode scattering) ──
    b_p, b_g, b_k, ip = solve_triode_root(
        a_p, a_g, a_k,
        plate_tree.R, grid_tree.R, cathode_tree.R,
        ip_prev
    )
    ip_prev = ip

    # ── Step 4: Downward pass (incident waves from root to leaves) ──
    plate_tree.incident(b_p)
    grid_tree.incident(b_g)
    cathode_tree.incident(b_k)

    # ── Step 5: Extract output ──
    # Plate voltage from wave variables: Vp = (a_p + b_p) / 2
    # But a_p here is the reflected wave from plate_tree (going up)
    # and b_p is the incident from triode (going down).
    # Actually in our convention:
    #   a_p = plate_tree.b (reflected up = incident to triode = "a" for triode)
    #   b_p = what triode sends down = incident to plate_tree
    # The plate voltage is at the junction between plate_tree and triode.
    # v_plate = (a_triode_plate + b_triode_plate) / 2 = (a_p + b_p) / 2
    v_plate = (a_p + b_p) / 2.0

    out_vp[n] = v_plate
    out_ip[n] = ip

# DC operating point from settling
vp_dc = np.mean(out_vp[n_settle-100:n_settle])
print(f"\n=== DC Operating Point (WDF) ===")
print(f"Vplate_dc = {vp_dc:.2f}V")
print(f"Ip_dc = {np.mean(out_ip[n_settle-100:n_settle])*1000:.3f}mA")

# AC-coupled output
out = out_vp - vp_dc

# ═══════════════════════════════════════════════════════════════════════════════
# Analysis
# ═══════════════════════════════════════════════════════════════════════════════

ap = out[n_settle:]
inp = audio_in[n_settle:]
in_rms = np.sqrt(np.mean(inp**2))
out_rms = np.sqrt(np.mean(ap**2))
gain_db = 20*np.log10(out_rms / in_rms + 1e-12) if in_rms > 0 else 0

print(f"\n=== Audio (WDF) ===")
print(f"Input:  {inp.min():.3f}V to {inp.max():.3f}V  rms={in_rms:.3f}V")
print(f"Output: {ap.min():.2f}V to {ap.max():.2f}V  rms={out_rms:.2f}V")
print(f"Gain:   {gain_db:.1f} dB")

# ═══════════════════════════════════════════════════════════════════════════════
# Plots
# ═══════════════════════════════════════════════════════════════════════════════

t_ms = t * 1000

fig, axes = plt.subplots(3, 1, figsize=(14, 10))
fig.suptitle("12AX7 WDF Triode Stage — Wave Digital Filter", fontsize=14)

t_a = t_ms[n_settle:] - t_ms[n_settle]
mask = t_a < 10

ax = axes[0]
ax.plot(t_a[mask], inp[mask], 'b', linewidth=1.5, label='Input')
ax.set_ylabel("V"); ax.set_title("Input — 440Hz, 0.5V"); ax.legend(); ax.grid(True, alpha=0.3)

ax = axes[1]
ax.plot(t_a[mask], ap[mask], 'r', linewidth=1.5, label='Output (AC)')
ax.set_ylabel("V"); ax.set_title(f"WDF Output ({gain_db:.1f} dB gain)"); ax.legend(); ax.grid(True, alpha=0.3)

ax = axes[2]
in_n = inp[mask] / (np.max(np.abs(inp[mask])) + 1e-12)
out_n = ap[mask] / (np.max(np.abs(ap[mask])) + 1e-12)
ax.plot(t_a[mask], in_n, 'b', alpha=0.6, label='Input (norm)')
ax.plot(t_a[mask], out_n, 'r', alpha=0.6, label='Output (norm)')
ax.set_xlabel("Time (ms)"); ax.set_ylabel("Norm"); ax.set_title("Overlay")
ax.legend(); ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("wdf_sim_wdf_waveforms.png", dpi=150)
print("\nSaved: wdf_sim_wdf_waveforms.png")

# Write output for cross-validation
with open("wdf_sim_wdf_output.txt", "w") as f:
    for n in range(n_total):
        f.write(f"{n} {int(audio_in[n]*65536)} {int(out[n]*65536)}\n")
print("Saved: wdf_sim_wdf_output.txt")
print("Done.")
```

- [ ] **Step 2: Run the WDF Python simulation**

Run: `python wdf_triode_sim_wdf.py`

Expected output should show:
- DC operating point close to the original sim (~146V plate, ~0.54mA)
- Gain close to 29dB
- Output swing of ~±15V

If the gain or operating point differs significantly from `wdf_triode_sim.py`, debug the wave variable input coupling (the `cap_cin.z + 2*vin` trick). The most likely issue is the factor of 2 on vin — try without it if gain is too high.

- [ ] **Step 3: Commit**

```bash
git add wdf_triode_sim_wdf.py
git commit -m "feat: add Python WDF reference simulation with wave variable tree"
```

---

## Task 3: Cross-Validation Script

Validate that the WDF Python sim matches the original Newton-Raphson Python sim.

**Files:**
- Create: `validate_wdf.py`

- [ ] **Step 1: Write the cross-validation script**

Create `validate_wdf.py`:

```python
"""
validate_wdf.py
Cross-validate WDF simulation against Newton-Raphson reference.
Both should produce nearly identical output (within numerical precision).
"""

import numpy as np
import sys

def load_sim(filename):
    data = np.loadtxt(filename, dtype=int)
    return data[:, 0], data[:, 1] / 65536.0, data[:, 2] / 65536.0

# Load both outputs
print("Loading simulations...")
n1, in1, out1 = load_sim("wdf_sim_output.txt")       # Newton-Raphson
n2, in2, out2 = load_sim("wdf_sim_wdf_output.txt")    # WDF

# Align lengths
min_len = min(len(out1), len(out2))
out1 = out1[:min_len]
out2 = out2[:min_len]

# Skip settling period
settle = 2000
o1 = out1[settle:]
o2 = out2[settle:]

# Compare
diff = o1 - o2
max_diff = np.max(np.abs(diff))
rms_diff = np.sqrt(np.mean(diff**2))
rms_out = np.sqrt(np.mean(o1**2))

print(f"\n=== Cross-Validation ===")
print(f"Newton-Raphson: rms={np.sqrt(np.mean(o1**2)):.3f}V, range=[{o1.min():.2f}, {o1.max():.2f}]V")
print(f"WDF:            rms={np.sqrt(np.mean(o2**2)):.3f}V, range=[{o2.min():.2f}, {o2.max():.2f}]V")
print(f"Max difference: {max_diff:.4f}V")
print(f"RMS difference: {rms_diff:.4f}V")
print(f"Relative error: {rms_diff/rms_out*100:.2f}%")

if rms_diff / rms_out < 0.05:  # less than 5% relative error
    print("\nPASS: WDF matches Newton-Raphson within 5%")
    sys.exit(0)
else:
    print(f"\nFAIL: WDF differs from Newton-Raphson by {rms_diff/rms_out*100:.1f}%")
    print("Debug: check input coupling, adaptor scattering, or triode root solver")
    sys.exit(1)
```

- [ ] **Step 2: Run the validation**

Run: `python validate_wdf.py`

Expected: `PASS: WDF matches Newton-Raphson within 5%`

If it fails, the WDF Python sim needs debugging before we proceed to Verilog. Common issues:
- Input coupling factor (2x vs 1x on vin)
- Parallel adaptor incident wave formula (several equivalent forms, easy to get signs wrong)
- Capacitor state update timing (should update z AFTER downward pass, not before)

- [ ] **Step 3: Commit**

```bash
git add validate_wdf.py
git commit -m "feat: add cross-validation script for WDF vs Newton-Raphson sims"
```

---

## Task 4: Verilog WDF Module — One-Port Elements and Adaptors

Implement the WDF tree in Verilog. This is the core new module.

**Files:**
- Create: `wdf_triode_wdf.v`

- [ ] **Step 1: Write the module header, constants, and LUT memory**

Create `wdf_triode_wdf.v`:

```verilog
// ============================================================================
// wdf_triode_wdf.v
// Common-Cathode 12AX7 Triode Stage — Proper WDF Binary Tree
//
// WDF tree structure:
//   [Triode 3-port nonlinear ROOT]
//     ├── Plate:   Series(Rp, Vs_B+)
//     ├── Grid:    Series(Cin, Rg)
//     └── Cathode: Parallel(Rk, Ck)
//
// Fixed point: Q16.16 signed throughout
// Sample rate: 48kHz, Clock: 27MHz (562 cycles/sample)
// ============================================================================

module wdf_triode_wdf #(
    parameter real VB       = 200.0,
    parameter real RP       = 100000,
    parameter real RG       = 1000000,
    parameter real RK       = 1500,
    parameter real CIN_F    = 22e-9,    // 22nF
    parameter real CK_F     = 22e-6,    // 22µF
    parameter real FS       = 48000.0,
    parameter FP_FRAC       = 16,
    parameter FP_WIDTH      = 32,
    parameter LUT_BITS      = 8,
    parameter LUT_SIZE      = 256,
    parameter IP_SCALE      = 10000,
    parameter DIP_SCALE     = 100000,
    parameter DIP_VPK_SCALE = 100000,
    parameter integer VPK_MIN_MV = 0,
    parameter integer VPK_MAX_MV = 300000,
    parameter integer VGK_MIN_MV = -4000,
    parameter integer VGK_MAX_MV = 0
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sample_en,
    input  wire signed [FP_WIDTH-1:0] audio_in,
    output reg  signed [FP_WIDTH-1:0] audio_out,
    output reg                        out_valid
);

// ============================================================================
// Port Resistances (Q16.16)
// Computed at elaboration from circuit parameters
// ============================================================================

// One-port element resistances
// Resistors: R0 = R
// Capacitors: R0 = 1/(2*Fs*C) — bilinear transform
// Voltage source: R0 ≈ 1Ω (small series resistance)
localparam real R_RP_REAL  = RP;                    // 100000
localparam real R_VS_REAL  = 1.0;                   // ~0 for ideal source
localparam real R_CIN_REAL = 1.0/(2.0*FS*CIN_F);   // 473.5
localparam real R_RG_REAL  = RG;                    // 1000000
localparam real R_RK_REAL  = RK;                    // 1500
localparam real R_CK_REAL  = 1.0/(2.0*FS*CK_F);    // 0.4735

// Adaptor port resistances (propagate up tree)
localparam real R_PLATE_REAL   = R_RP_REAL + R_VS_REAL;           // series
localparam real R_GRID_REAL    = R_CIN_REAL + R_RG_REAL;          // series
localparam real R_CATHODE_REAL = 1.0/(1.0/R_RK_REAL + 1.0/R_CK_REAL); // parallel

// Scattering coefficients (Q16.16)
// Series: gamma = R_port1 / (R_port1 + R_port2)
// Parallel: gamma = G_port1 / (G_port1 + G_port2) = R_port2 / (R_port1 + R_port2)
localparam signed [FP_WIDTH-1:0] GAMMA_PLATE =
    $rtoi((R_RP_REAL / R_PLATE_REAL) * (1 << FP_FRAC));
localparam signed [FP_WIDTH-1:0] GAMMA_GRID =
    $rtoi((R_CIN_REAL / R_GRID_REAL) * (1 << FP_FRAC));
localparam signed [FP_WIDTH-1:0] GAMMA_CATHODE =
    $rtoi(((1.0/R_RK_REAL) / (1.0/R_RK_REAL + 1.0/R_CK_REAL)) * (1 << FP_FRAC));

// Port resistances as Q16.16 (for Newton-Raphson: Vpk = a_p - a_k - (R_p+R_k)*Ip)
// These are large values — R_plate = 100001 doesn't fit Q16.16 (max ~32767)
// So store as plain integers and multiply carefully
localparam integer R_PLATE_INT  = $rtoi(R_PLATE_REAL);   // 100001
localparam integer R_GRID_INT   = $rtoi(R_GRID_REAL);    // 1000474
localparam integer R_CATHODE_INT = 1;  // ~0.47Ω, rounds to ≈0 — use scaled version

// For Newton: we need (R_plate + R_cathode) * Ip
// R_plate ≈ 100001, R_cathode ≈ 0.47, so R_plate + R_cathode ≈ 100001
// The cathode resistance is negligible compared to plate for this product
localparam integer R_PK_INT = $rtoi(R_PLATE_REAL + R_CATHODE_REAL); // ~100001

// B+ voltage (Q16.16)
localparam signed [FP_WIDTH-1:0] VB_FP = VB * (1 << FP_FRAC);

// 2*VB for voltage source reflection: b_vs = 2*VB - a_vs
localparam signed [FP_WIDTH-1:0] TWO_VB_FP = 2 * VB * (1 << FP_FRAC);

// 2*R_plate and 2*R_cathode as integers for reflected wave computation
// b_p = a_p - 2*R_p*Ip,  b_k = a_k + 2*R_k*Ip
localparam integer TWO_R_PLATE = $rtoi(2.0 * R_PLATE_REAL);    // 200002
localparam integer TWO_R_CATHODE_SCALED = $rtoi(2.0 * R_CATHODE_REAL * 65536.0);
// R_cathode is ~0.47Ω, so 2*R_cath = ~0.95. In Q16.16: ~62259
// We pre-scale so that TWO_R_CATHODE_SCALED * Ip_Q16 >> 16 gives Q16.16 result

// ============================================================================
// LUT Memory (3 tables)
// ============================================================================

reg signed [15:0] ip_lut     [0 : LUT_SIZE*LUT_SIZE - 1];
reg signed [15:0] dip_vgk_lut[0 : LUT_SIZE*LUT_SIZE - 1];
reg signed [15:0] dip_vpk_lut[0 : LUT_SIZE*LUT_SIZE - 1];

initial begin
    $readmemh("ip_lut.hex",       ip_lut);
    $readmemh("dip_dvgk_lut.hex", dip_vgk_lut);
    $readmemh("dip_dvpk_lut.hex", dip_vpk_lut);
end

// ============================================================================
// LUT Address Functions (same as original)
// ============================================================================

function automatic [LUT_BITS-1:0] vpk_to_idx;
    input signed [FP_WIDTH-1:0] vpk_fp;
    reg signed [63:0] tmp;
    begin
        tmp = (vpk_fp * 1000) >>> FP_FRAC;
        if (tmp < VPK_MIN_MV) tmp = VPK_MIN_MV;
        if (tmp > VPK_MAX_MV) tmp = VPK_MAX_MV;
        vpk_to_idx = ((tmp - VPK_MIN_MV) * (LUT_SIZE-1)) / (VPK_MAX_MV - VPK_MIN_MV);
    end
endfunction

function automatic [LUT_BITS-1:0] vgk_to_idx;
    input signed [FP_WIDTH-1:0] vgk_fp;
    reg signed [63:0] tmp;
    begin
        tmp = (vgk_fp * 1000) >>> FP_FRAC;
        if (tmp < VGK_MIN_MV) tmp = VGK_MIN_MV;
        if (tmp > VGK_MAX_MV) tmp = VGK_MAX_MV;
        vgk_to_idx = ((tmp - VGK_MIN_MV) * (LUT_SIZE-1)) / (VGK_MAX_MV - VGK_MIN_MV);
    end
endfunction

// ============================================================================
// State Registers
// ============================================================================

// Capacitor states (WDF: z = previous incident wave)
reg signed [FP_WIDTH-1:0] cin_z;      // Cin state
reg signed [FP_WIDTH-1:0] ck_z;       // Ck state

// Voltage source state
reg signed [FP_WIDTH-1:0] vs_a_prev;  // previous incident to Vs

// Previous Ip (for Newton initial guess)
reg signed [FP_WIDTH-1:0] ip_prev;

// DC output offset (computed during settling)
reg signed [FP_WIDTH-1:0] vp_dc;
reg [15:0] settle_count;
reg signed [63:0] vp_dc_accum;

// Wave variables (pipeline registers)
reg signed [FP_WIDTH-1:0] b_rp, b_vs, b_cin, b_rg, b_rk, b_ck;  // leaf reflected
reg signed [FP_WIDTH-1:0] a_plate, a_grid, a_cathode;  // adaptor reflected (to root)
reg signed [FP_WIDTH-1:0] bp_root, bg_root, bk_root;   // root scattered (from triode)

// LUT interface
reg [LUT_BITS*2-1:0] lut_addr;
reg signed [15:0] ip_raw, dip_vgk_raw, dip_vpk_raw;
reg signed [FP_WIDTH-1:0] ip_lut_val, dip_vgk_val, dip_vpk_val;

// Newton-Raphson iteration
reg signed [FP_WIDTH-1:0] ip_est;     // current Ip estimate
reg signed [FP_WIDTH-1:0] vpk_est, vgk_est;  // voltage estimates
reg [2:0] newton_iter;

// Intermediates
reg signed [63:0] temp64, temp64b;

// ============================================================================
// Pipeline State Machine
//
// States:
//  0  ST_IDLE      — wait for sample_en
//  1  ST_REFLECT   — compute leaf reflected waves (upward pass leaves)
//  2  ST_ADAPTOR   — compute adaptor reflected waves (upward pass adaptors)
//  3  ST_NR_ADDR   — Newton-Raphson: compute LUT address from Vpk/Vgk est
//  4  ST_NR_READ   — Newton-Raphson: BRAM read latency
//  5  ST_NR_CONV   — Newton-Raphson: convert LUT raw to Q16.16
//  6  ST_NR_STEP   — Newton-Raphson: compute f, f', update Ip
//  7  ST_SCATTER   — compute root reflected waves (triode output)
//  8  ST_INCIDENT  — downward pass: scatter to children, update cap states
//  9  ST_OUTPUT    — extract plate voltage, AC couple, output
// ============================================================================

localparam ST_IDLE     = 4'd0;
localparam ST_REFLECT  = 4'd1;
localparam ST_ADAPTOR  = 4'd2;
localparam ST_NR_ADDR  = 4'd3;
localparam ST_NR_READ  = 4'd4;
localparam ST_NR_CONV  = 4'd5;
localparam ST_NR_STEP  = 4'd6;
localparam ST_SCATTER  = 4'd7;
localparam ST_INCIDENT = 4'd8;
localparam ST_OUTPUT   = 4'd9;

reg [3:0] state;

// ============================================================================
// Main State Machine
// ============================================================================

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state        <= ST_IDLE;
        out_valid    <= 1'b0;
        audio_out    <= 0;
        cin_z        <= 0;
        ck_z         <= 0;
        vs_a_prev    <= 0;
        ip_prev      <= 32'sd35;  // ~0.5mA in Q16.16 (35/65536 ≈ 0.00053)
        ip_est       <= 0;
        vp_dc        <= 0;
        settle_count <= 0;
        vp_dc_accum  <= 0;
        newton_iter  <= 0;
        lut_addr     <= 0;
    end else begin
        out_valid <= 1'b0;

        case (state)

        // ── Wait for sample ─────────────────────────────────────────────
        ST_IDLE: begin
            if (sample_en) begin
                state <= ST_REFLECT;
            end
        end

        // ── Upward pass: leaf reflected waves ───────────────────────────
        // Resistor: b = 0
        // Capacitor: b = z (stored state)
        // Voltage source: b = 2*V - a_prev
        ST_REFLECT: begin
            b_rp  <= 0;                          // Rp absorbs
            b_vs  <= TWO_VB_FP - vs_a_prev;      // Vs(B+) reflects
            b_rg  <= 0;                          // Rg absorbs

            // Cin: b = z + 2*vin (input voltage source in series with cap)
            // 2*vin in Q16.16: audio_in is already Q16.16, shift left by 1
            b_cin <= cin_z + (audio_in <<< 1);

            b_rk  <= 0;                          // Rk absorbs
            b_ck  <= ck_z;                       // Ck reflects state

            state <= ST_ADAPTOR;
        end

        // ── Upward pass: adaptor scattering ─────────────────────────────
        // Series: b_parent = -(b_child1 + b_child2)
        // Parallel: b_parent = b2 - gamma*(b2 - b1)
        ST_ADAPTOR: begin
            // Plate tree (series): a_plate = -(b_rp + b_vs)
            a_plate <= -(b_rp + b_vs);

            // Grid tree (series): a_grid = -(b_cin + b_rg)
            a_grid <= -(b_cin + b_rg);

            // Cathode tree (parallel): a_cathode = b_ck - gamma*(b_ck - b_rk)
            temp64 = $signed(GAMMA_CATHODE) * $signed(b_ck - b_rk);
            a_cathode <= b_ck - temp64[47:16];

            // Initialize Newton-Raphson with previous Ip
            ip_est <= ip_prev;
            newton_iter <= 0;

            state <= ST_NR_ADDR;
        end

        // ── Newton-Raphson: compute Vpk, Vgk and LUT address ───────────
        // Vpk = (a_p - a_k) - (R_p + R_k) * Ip
        // Vgk = a_g - a_k - R_k * Ip
        // Note: a_p, a_g, a_k are the wave variables from adaptor outputs
        // R_k for cathode is R_CATHODE_REAL ≈ 0.47Ω, so R_k*Ip is tiny
        ST_NR_ADDR: begin
            // Vpk = (a_plate - a_cathode) - R_PK_INT * ip_est
            // R_PK_INT is plain integer, ip_est is Q16.16
            // product is Q16.16
            temp64 = $signed({{32{ip_est[31]}}, ip_est}) * $signed(R_PK_INT);
            vpk_est <= (a_plate - a_cathode) - temp64[31:0];

            // Vgk = a_grid - a_cathode - R_CATHODE * ip_est
            // R_CATHODE ≈ 0.47Ω — negligible, but include for correctness
            // Use pre-scaled: TWO_R_CATHODE_SCALED/2 * ip_est >> 16
            temp64b = $signed({{32{ip_est[31]}}, ip_est}) * $signed(TWO_R_CATHODE_SCALED / 2);
            vgk_est <= (a_grid - a_cathode) - temp64b[47:16];

            // LUT address from voltage estimates
            // Need to use the values we just computed — but they're in NBA registers
            // Use combinational versions for address
            begin
                reg signed [FP_WIDTH-1:0] vpk_comb, vgk_comb;
                reg signed [63:0] t1, t2;
                t1 = $signed({{32{ip_est[31]}}, ip_est}) * $signed(R_PK_INT);
                vpk_comb = (a_plate - a_cathode) - t1[31:0];
                t2 = $signed({{32{ip_est[31]}}, ip_est}) * $signed(TWO_R_CATHODE_SCALED / 2);
                vgk_comb = (a_grid - a_cathode) - t2[47:16];
                lut_addr <= vpk_to_idx(vpk_comb) * LUT_SIZE + vgk_to_idx(vgk_comb);
            end

            state <= ST_NR_READ;
        end

        // ── Newton-Raphson: BRAM read ───────────────────────────────────
        ST_NR_READ: begin
            ip_raw      <= ip_lut[lut_addr];
            dip_vgk_raw <= dip_vgk_lut[lut_addr];
            dip_vpk_raw <= dip_vpk_lut[lut_addr];
            state <= ST_NR_CONV;
        end

        // ── Newton-Raphson: convert LUT values to Q16.16 ───────────────
        ST_NR_CONV: begin
            ip_lut_val  <= ($signed(ip_raw) <<< FP_FRAC) / IP_SCALE;
            dip_vgk_val <= ($signed(dip_vgk_raw) <<< FP_FRAC) / DIP_SCALE;
            dip_vpk_val <= ($signed(dip_vpk_raw) <<< FP_FRAC) / DIP_VPK_SCALE;
            state <= ST_NR_STEP;
        end

        // ── Newton-Raphson: compute step ────────────────────────────────
        // f(Ip) = Ip - Ip_model
        // f'(Ip) = 1 - dIp/dVpk*(-(R_p+R_k)) - dIp/dVgk*(-R_k)
        //        = 1 + dIp/dVpk*(R_p+R_k) + dIp/dVgk*R_k
        // Ip_new = Ip - f/f'
        ST_NR_STEP: begin
            // f = ip_est - ip_lut_val
            // f' = 1 + dip_vpk * R_PK + dip_vgk * R_CATH
            // All in Q16.16 except R values are plain integers

            // dip_vpk * R_PK: Q16.16 * int = Q16.16
            // dip_vgk * R_CATH_INT: Q16.16 * ~0 ≈ 0 (cathode R is negligible)
            // f' ≈ 1 + dip_vpk * R_PK (in Q16.16)
            temp64 = $signed({{32{dip_vpk_val[31]}}, dip_vpk_val}) * $signed(R_PK_INT);
            // temp64 is Q32.32 effectively, but since R_PK is plain int and dip is Q16.16,
            // result is Q16.16 in temp64[31:0]

            begin
                reg signed [FP_WIDTH-1:0] f_val, fp_val, step;
                reg signed [63:0] t1;

                f_val = ip_est - ip_lut_val;

                // f' = 1.0 + dip_vpk * R_PK + dip_vgk * R_cathode
                // 1.0 in Q16.16 = 65536
                // dip_vpk * R_PK: already in temp64[31:0]
                t1 = $signed({{32{dip_vgk_val[31]}}, dip_vgk_val}) * $signed(1); // R_cathode ≈ 0-1
                fp_val = 32'sd65536 + temp64[31:0] + t1[31:0];

                // Newton step: ip_new = ip_est - f/f'
                // Division in Q16.16: (f << 16) / f'
                if (fp_val != 0) begin
                    step = ($signed({{32{f_val[31]}}, f_val}) <<< FP_FRAC) / fp_val;
                    ip_est <= (ip_est - step < 0) ? 0 : ip_est - step;
                end
            end

            // Check if we need more iterations
            if (newton_iter < 3'd1) begin  // 2 iterations total (0 and 1)
                newton_iter <= newton_iter + 1;
                state <= ST_NR_ADDR;  // loop back for another iteration
            end else begin
                state <= ST_SCATTER;
            end
        end

        // ── Root scattering: triode output ──────────────────────────────
        // b_p = a_plate - 2*R_plate*Ip
        // b_g = a_grid  (no grid current)
        // b_k = a_cathode + 2*R_cathode*Ip
        ST_SCATTER: begin
            // Plate reflected: b_p = a_plate - 2*R_plate*Ip
            temp64 = $signed({{32{ip_est[31]}}, ip_est}) * $signed(TWO_R_PLATE);
            bp_root <= a_plate - temp64[31:0];

            // Grid reflected: no grid current
            bg_root <= a_grid;

            // Cathode reflected: b_k = a_cathode + 2*R_cathode*Ip
            temp64b = $signed({{32{ip_est[31]}}, ip_est}) * $signed(TWO_R_CATHODE_SCALED);
            bk_root <= a_cathode + temp64b[47:16];

            // Save Ip for next sample
            ip_prev <= ip_est;

            state <= ST_INCIDENT;
        end

        // ── Downward pass: scatter to children, update states ───────────
        ST_INCIDENT: begin
            // Plate tree (series adaptor) downward:
            // a_rp  = b_rp  - gamma_plate * (bp_root + b_rp + b_vs)
            // a_vs  = -(bp_root + a_rp)
            begin
                reg signed [FP_WIDTH-1:0] a_rp_val, a_vs_val;
                reg signed [63:0] t1;
                t1 = $signed(GAMMA_PLATE) * $signed(bp_root + b_rp + b_vs);
                a_rp_val = b_rp - t1[47:16];
                a_vs_val = -(bp_root + a_rp_val);
                vs_a_prev <= a_vs_val;  // store for next sample's Vs reflection
            end

            // Grid tree (series adaptor) downward:
            // a_cin = b_cin - gamma_grid * (bg_root + b_cin + b_rg)
            // a_rg  = -(bg_root + a_cin)
            begin
                reg signed [FP_WIDTH-1:0] a_cin_val;
                reg signed [63:0] t1;
                t1 = $signed(GAMMA_GRID) * $signed(bg_root + b_cin + b_rg);
                a_cin_val = b_cin - t1[47:16];
                cin_z <= a_cin_val;  // update Cin state
            end

            // Cathode tree (parallel adaptor) downward:
            // b2_down = bk_root + a_cathode_b - b_ck  (using stored values)
            // Actually: a_ck = bk_root - a_cathode + b_ck ... let me use chowdsp form:
            // From reflected: bDiff = b_ck - b_rk
            // a_ck = bk_root + a_cathode_reflected - b_ck
            //      = bk_root + (b_ck - gamma*(b_ck-b_rk)) - b_ck
            // Simpler: just use the standard parallel incident formulas:
            // a_rk = bk_root - a_cathode + b_rk + bDiff  ... no
            //
            // chowdsp_wdf parallel incident:
            //   b2_down = x + wdf.b - port2.wdf.b
            //   port2.incident(b2_down)
            //   port1.incident(b2_down + bDiff)
            // where x = bk_root (incident from parent), wdf.b = a_cathode, port2 = Ck
            begin
                reg signed [FP_WIDTH-1:0] a_ck_val, a_rk_val, bDiff;
                bDiff = b_ck - b_rk;
                a_ck_val = bk_root + a_cathode - b_ck;
                a_rk_val = a_ck_val + bDiff;
                ck_z <= a_ck_val;  // update Ck state
            end

            state <= ST_OUTPUT;
        end

        // ── Extract output ──────────────────────────────────────────────
        ST_OUTPUT: begin
            // Plate voltage: Vp = (a_plate + bp_root) / 2
            // (a_plate is reflected from plate subtree, bp_root is from triode root)
            begin
                reg signed [FP_WIDTH-1:0] v_plate;
                v_plate = (a_plate + bp_root) >>> 1;

                // During first 2000 samples, accumulate DC average
                if (settle_count < 16'd2000) begin
                    settle_count <= settle_count + 1;
                    vp_dc_accum <= vp_dc_accum + v_plate;
                    if (settle_count == 16'd1999) begin
                        // vp_dc = accum / 2000
                        vp_dc <= vp_dc_accum / 2000;
                    end
                    audio_out <= 0;
                end else begin
                    // AC-coupled output
                    audio_out <= v_plate - vp_dc;
                end
            end

            out_valid <= 1'b1;
            state <= ST_IDLE;
        end

        default: state <= ST_IDLE;

        endcase
    end
end

endmodule
```

- [ ] **Step 2: Review the module for compilation**

Check: does the module compile with iverilog?

Run: `export PATH="$PATH:/c/iverilog/bin" && iverilog -g2012 -o /dev/null wdf_triode_wdf.v 2>&1`

Fix any syntax errors. Common issues:
- `begin/end` blocks inside always blocks for local variable declarations
- Sign extension on 64-bit multiplies
- $rtoi may not be supported in iverilog — may need to compute constants externally

If `$rtoi` doesn't work in iverilog, replace the localparam computations with pre-computed integer values (compute them in Python and hardcode).

- [ ] **Step 3: Commit**

```bash
git add wdf_triode_wdf.v
git commit -m "feat: add proper WDF triode module with binary tree and Newton-Raphson"
```

---

## Task 5: WDF Testbench

**Files:**
- Create: `wdf_triode_wdf_tb.v`

- [ ] **Step 1: Write the testbench**

Create `wdf_triode_wdf_tb.v` — identical to the existing `wdf_triode_tb.v` but instantiating the new WDF module:

```verilog
// ============================================================================
// wdf_triode_wdf_tb.v
// Testbench for the proper WDF triode stage
// ============================================================================

`timescale 1ns/1ps

module wdf_triode_wdf_tb;

// ── Clock and Reset ─────────────────────────────────────────────────────────
reg clk, rst_n;
localparam CLK_PERIOD = 37;  // ~27MHz

initial clk = 0;
always #(CLK_PERIOD/2) clk = ~clk;

// ── Sample Rate Enable ──────────────────────────────────────────────────────
localparam SAMPLE_DIV = 562;
reg [9:0] sample_cnt;
reg sample_en;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sample_cnt <= 0;
        sample_en  <= 0;
    end else begin
        sample_en <= 0;
        if (sample_cnt >= SAMPLE_DIV - 1) begin
            sample_cnt <= 0;
            sample_en  <= 1;
        end else begin
            sample_cnt <= sample_cnt + 1;
        end
    end
end

// ── DUT ─────────────────────────────────────────────────────────────────────
wire signed [31:0] audio_out;
wire               out_valid;
reg  signed [31:0] audio_in;

wdf_triode_wdf dut (
    .clk       (clk),
    .rst_n     (rst_n),
    .sample_en (sample_en),
    .audio_in  (audio_in),
    .audio_out (audio_out),
    .out_valid (out_valid)
);

// ── Sine Wave Generator ────────────────────────────────────────────────────
integer sine_idx;
reg signed [31:0] sine_table [0:107];

integer i;
real two_pi = 6.2831853;

initial begin
    for (i = 0; i < 108; i = i + 1) begin
        sine_table[i] = $rtoi($sin(two_pi * i / 108.0) * 0.5 * 65536.0);
    end
    sine_idx = 0;
end

// 2000 settling samples (matching Python WDF sim)
localparam DC_SETTLE_SAMPLES = 2000;

always @(posedge clk) begin
    if (sample_en) begin
        if (sample_count < DC_SETTLE_SAMPLES) begin
            audio_in <= 0;
        end else begin
            audio_in <= sine_table[sine_idx];
            if (sine_idx >= 107)
                sine_idx <= 0;
            else
                sine_idx <= sine_idx + 1;
        end
    end
end

// ── Output Capture ──────────────────────────────────────────────────────────
integer fd;
integer sample_count;

initial begin
    fd = $fopen("wdf_tb_output.txt", "w");
    sample_count = 0;
end

always @(posedge clk) begin
    if (out_valid) begin
        $fdisplay(fd, "%0d %0d %0d", sample_count, audio_in, audio_out);
        sample_count = sample_count + 1;
    end
end

// ── Run Simulation ──────────────────────────────────────────────────────────
initial begin
    rst_n = 0;
    audio_in = 0;
    #(CLK_PERIOD * 10);
    rst_n = 1;

    // 2000 settling + 4800 audio = 6800 samples
    #(6800 * SAMPLE_DIV * CLK_PERIOD);

    $fclose(fd);
    $display("WDF simulation complete. %0d samples written to wdf_tb_output.txt", sample_count);
    $finish;
end

// ── Waveform Dump ───────────────────────────────────────────────────────────
initial begin
    $dumpfile("wdf_triode_wdf.vcd");
    $dumpvars(0, wdf_triode_wdf_tb);
end

endmodule
```

- [ ] **Step 2: Compile and run the simulation**

```bash
export PATH="$PATH:/c/iverilog/bin"
iverilog -g2012 -o wdf_sim wdf_triode_wdf_tb.v wdf_triode_wdf.v
vvp wdf_sim
```

Expected: `WDF simulation complete. 6800 samples written to wdf_tb_output.txt`

- [ ] **Step 3: Analyze the output**

```bash
python analyze_tb.py wdf_tb_output.txt
```

Expected:
- Gain should be close to 29dB (matching Python WDF sim)
- Output range should be approximately ±15V
- Harmonics visible in FFT at 880Hz, 1320Hz, etc.

- [ ] **Step 4: Commit**

```bash
git add wdf_triode_wdf_tb.v
git commit -m "feat: add testbench for WDF triode module"
```

---

## Task 6: Full Cross-Validation (Python WDF vs Verilog WDF)

**Files:**
- Modify: `validate_wdf.py` — add Verilog output comparison

- [ ] **Step 1: Extend validate_wdf.py to include Verilog output**

Add after the existing comparison code:

```python
# Also compare against Verilog WDF output if available
try:
    n3, in3, out3 = load_sim("wdf_tb_output.txt")
    o3 = out3[settle:min(settle+len(o1), len(out3))]
    o1_trimmed = o1[:len(o3)]

    diff_v = o1_trimmed - o3
    max_diff_v = np.max(np.abs(diff_v))
    rms_diff_v = np.sqrt(np.mean(diff_v**2))

    print(f"\n=== Verilog WDF vs Newton-Raphson ===")
    print(f"Verilog WDF: rms={np.sqrt(np.mean(o3**2)):.3f}V, range=[{o3.min():.2f}, {o3.max():.2f}]V")
    print(f"Max difference: {max_diff_v:.4f}V")
    print(f"RMS difference: {rms_diff_v:.4f}V")
    print(f"Relative error: {rms_diff_v/rms_out*100:.2f}%")

    if rms_diff_v / rms_out < 0.10:  # 10% tolerance for fixed-point
        print("PASS: Verilog WDF matches reference within 10%")
    else:
        print(f"FAIL: Verilog WDF differs by {rms_diff_v/rms_out*100:.1f}%")
except FileNotFoundError:
    print("\nSkipping Verilog comparison (wdf_tb_output.txt not found)")
    print("Run: iverilog -g2012 -o wdf_sim wdf_triode_wdf_tb.v wdf_triode_wdf.v && vvp wdf_sim")
```

- [ ] **Step 2: Run full validation**

```bash
python validate_wdf.py
```

Expected: Both Python WDF and Verilog WDF should match Newton-Raphson reference within tolerance.

- [ ] **Step 3: Commit**

```bash
git add validate_wdf.py
git commit -m "feat: extend validation to include Verilog WDF cross-comparison"
```

---

## Task 7: Debug and Iterate (if needed)

This task exists for the likely case that the Verilog WDF doesn't match on first try.

**Common issues and fixes:**

1. **$rtoi not supported in iverilog** — Replace all `$rtoi(...)` localparam computations with hardcoded values. Add a Python script section to `tube_lut_gen.py` that prints the computed constants:

```python
# Add to tube_lut_gen.py at the end:
print(f"\n=== WDF Constants (for Verilog hardcoding) ===")
R_cin = 1.0/(2*48000*22e-9)
R_ck = 1.0/(2*48000*22e-6)
R_plate = 100000 + 1
R_grid = R_cin + 1000000
R_cath = 1.0/(1.0/1500 + 1.0/R_ck)
print(f"R_CIN = {R_cin:.4f}")
print(f"R_CK = {R_ck:.4f}")
print(f"R_PLATE = {R_plate:.4f}")
print(f"R_GRID = {R_grid:.4f}")
print(f"R_CATHODE = {R_cath:.4f}")
print(f"GAMMA_PLATE = {int(100000/R_plate * 65536)} // Q16.16")
print(f"GAMMA_GRID = {int(R_cin/R_grid * 65536)} // Q16.16")
print(f"GAMMA_CATHODE = {int((1.0/1500)/(1.0/1500+1.0/R_ck) * 65536)} // Q16.16")
print(f"TWO_VB_FP = {int(400 * 65536)} // 2*VB in Q16.16")
print(f"R_PK_INT = {int(R_plate + R_cath)}")
print(f"TWO_R_PLATE = {int(2*R_plate)}")
print(f"TWO_R_CATHODE_SCALED = {int(2*R_cath*65536)} // Q16.16")
```

2. **64-bit multiply overflow** — If intermediate products exceed 64-bit signed range, split into smaller operations or reduce the dynamic range.

3. **Division in Newton step** — The `(f << 16) / f'` operation may overflow if f is large. Add clamping: if `|f| > some_threshold`, clamp the step size.

4. **Capacitor state divergence** — If Ck state grows unbounded, the parallel adaptor may be scattering incorrectly. Check signs in ST_INCIDENT.

5. **DC offset computation** — The accumulator for vp_dc may overflow in 64 bits if plate voltage is large. Use a running average instead: `vp_dc <= vp_dc + (v_plate - vp_dc) >>> 4` (exponential moving average).

---

## Summary: Expected Clock Budget

| Stage | Clocks | Notes |
|-------|--------|-------|
| ST_IDLE | 1 | Wait for sample_en |
| ST_REFLECT | 1 | All leaf reflections in parallel |
| ST_ADAPTOR | 1 | All 3 adaptors in parallel |
| ST_NR_ADDR | 1 | Compute voltages + LUT address |
| ST_NR_READ | 1 | BRAM latency |
| ST_NR_CONV | 1 | Fixed-point conversion |
| ST_NR_STEP | 1 | Newton update |
| (repeat NR x2) | 4×2=8 | Two Newton iterations |
| ST_SCATTER | 1 | Triode reflected waves |
| ST_INCIDENT | 1 | Downward pass + state update |
| ST_OUTPUT | 1 | Extract voltage, output |
| **Total** | **~14** | **Well within 562-cycle budget** |

This leaves ~548 spare cycles per sample for future additions (cascaded stages, tone stack, etc.).
