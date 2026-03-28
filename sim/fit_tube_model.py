"""
Koren Tube Model Curve Fitter
Fits Koren equation parameters (mu, ex, kg1, kp, kvb) to published tube data
using global optimization (differential evolution) with log-space weighting.

Outputs:
  - Best-fit parameters for 12AX7 and 12AU7
  - Per-point error table
  - fit_12ax7.png / fit_12au7.png — old vs new vs datasheet overlay
"""

import numpy as np
from scipy.optimize import differential_evolution
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ─── Koren Model ─────────────────────────────────────────────────────────────
def koren_ip(Vpk, Vgk, mu, ex, kg1, kp, kvb):
    """Koren triode plate current (amps)."""
    Vpk = np.asarray(Vpk, dtype=float)
    Vgk = np.asarray(Vgk, dtype=float)
    inner = kp * (1.0/mu + Vgk / np.sqrt(kvb + Vpk**2 + 1e-9))
    Ed = (Vpk / kp) * np.log1p(np.exp(np.clip(inner, -500, 500)))
    Ip = (np.maximum(Ed, 0.0) ** ex) / kg1
    return Ip

# ─── 12AX7 Measured Data (RCA datasheet) ─────────────────────────────────────
ax7_data = [
    # (Vpk, Vgk, Ip_mA)
    # Vpk=250V
    (250, 0.0, 3.2), (250, -0.5, 2.3), (250, -1.0, 1.7), (250, -1.5, 1.1),
    (250, -2.0, 0.8), (250, -2.5, 0.4), (250, -3.0, 0.2), (250, -3.5, 0.05),
    # Vpk=200V
    (200, 0.0, 2.0), (200, -1.0, 1.0), (200, -2.0, 0.4), (200, -3.0, 0.05),
    # Vpk=150V
    (150, 0.0, 1.2), (150, -1.0, 0.5), (150, -2.0, 0.1),
    # Vpk=100V
    (100, 0.0, 1.0), (100, -0.5, 0.6), (100, -1.0, 0.3), (100, -1.5, 0.1),
]

# ─── 12AU7 Measured Data (RCA datasheet) ─────────────────────────────────────
au7_data = [
    # (Vpk, Vgk, Ip_mA)
    # Vpk=250V
    (250, 0.0, 22.0), (250, -5.0, 10.5), (250, -8.5, 5.5),
    (250, -10.0, 3.5), (250, -15.0, 0.5),
    # Vpk=100V
    (100, 0.0, 8.0), (100, -5.0, 2.5), (100, -8.0, 0.5),
]

# ─── Fitting Function ────────────────────────────────────────────────────────
def fit_tube(name, data, bounds, initial_guesses):
    """
    Fit Koren parameters to measured data using differential evolution.
    Uses log-space loss for equal weighting across decades.
    """
    Vpk = np.array([d[0] for d in data], dtype=float)
    Vgk = np.array([d[1] for d in data], dtype=float)
    Ip_data = np.array([d[2] for d in data], dtype=float) * 1e-3  # mA -> A

    eps = 1e-9  # avoid log(0)

    def loss(params):
        mu, ex, kg1, kp, kvb = params
        Ip_model = koren_ip(Vpk, Vgk, mu, ex, kg1, kp, kvb)
        # Log-space RMS error
        log_err = np.log(Ip_model + eps) - np.log(Ip_data + eps)
        return np.mean(log_err**2)

    best_result = None
    best_loss = np.inf

    for guess_name, x0 in initial_guesses:
        print(f"  Trying initial guess: {guess_name} = {x0}")
        # Use differential_evolution for global optimization
        result = differential_evolution(
            loss, bounds,
            seed=42, maxiter=2000, tol=1e-12,
            init='sobol', polish=True,
            x0=x0,
        )
        print(f"    Loss = {result.fun:.6f}, params = {result.x}")
        if result.fun < best_loss:
            best_loss = result.fun
            best_result = result

    return best_result.x

def evaluate_fit(name, data, params, old_params_list):
    """Print per-point error table and return arrays for plotting."""
    Vpk = np.array([d[0] for d in data], dtype=float)
    Vgk = np.array([d[1] for d in data], dtype=float)
    Ip_data = np.array([d[2] for d in data], dtype=float) * 1e-3

    mu, ex, kg1, kp, kvb = params
    Ip_new = koren_ip(Vpk, Vgk, mu, ex, kg1, kp, kvb)

    print(f"\n{'='*75}")
    print(f"  {name} — Per-Point Error Table")
    print(f"{'='*75}")
    header = f"  {'Vpk':>5s} {'Vgk':>6s} {'Ip_data':>10s}"
    for old_name, _ in old_params_list:
        header += f" {'Err_'+old_name:>12s}"
    header += f" {'Ip_new':>10s} {'Err_new':>10s}"
    print(header)
    print(f"  {'-'*len(header)}")

    for i in range(len(data)):
        line = f"  {Vpk[i]:5.0f}V {Vgk[i]:6.1f}V {Ip_data[i]*1000:8.3f}mA"
        for old_name, old_p in old_params_list:
            Ip_old_i = koren_ip(Vpk[i], Vgk[i], *old_p)
            err_old = (Ip_old_i - Ip_data[i]) / Ip_data[i] * 100
            line += f" {err_old:+10.1f}%"
        err_new = (Ip_new[i] - Ip_data[i]) / Ip_data[i] * 100
        line += f" {Ip_new[i]*1000:8.3f}mA {err_new:+8.1f}%"
        print(line)

    # Summary
    for old_name, old_p in old_params_list:
        Ip_old = koren_ip(Vpk, Vgk, *old_p)
        errs = np.abs((Ip_old - Ip_data) / Ip_data) * 100
        print(f"\n  {old_name}: mean abs error = {errs.mean():.1f}%, max = {errs.max():.1f}%")

    errs_new = np.abs((Ip_new - Ip_data) / Ip_data) * 100
    print(f"  NEW FIT: mean abs error = {errs_new.mean():.1f}%, max = {errs_new.max():.1f}%")

    return Ip_new

def plot_fit(name, data, new_params, old_params_list, filename):
    """Generate overlay plot: old vs new vs datasheet."""
    mu, ex, kg1, kp, kvb = new_params

    # Get unique Vpk values from data
    vpk_vals = sorted(set(d[0] for d in data))
    Vpk_smooth = np.linspace(0, max(vpk_vals) + 50, 500)

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle(f"{name} Koren Model Fit — Old vs New vs Datasheet", fontsize=13)

    # ─── Plate curves (Ip vs Vpk) ────────────────────────────────────────
    ax = axes[0]
    colors = plt.cm.tab10(np.linspace(0, 1, 10))
    vgk_vals = sorted(set(d[1] for d in data))

    for ci, vgk in enumerate(vgk_vals):
        color = colors[ci % len(colors)]
        # Datasheet points
        pts = [(d[0], d[2]) for d in data if d[1] == vgk]
        if pts:
            vpk_pts, ip_pts = zip(*pts)
            ax.plot(vpk_pts, ip_pts, 'o', color=color, markersize=6, label=f"Data Vgk={vgk:.1f}V")

        # New fit curve
        Ip_new_curve = koren_ip(Vpk_smooth, vgk, mu, ex, kg1, kp, kvb) * 1000
        ax.plot(Vpk_smooth, Ip_new_curve, '-', color=color, linewidth=1.5)

        # Old fit curve (first old params only, dashed)
        if old_params_list:
            old_p = old_params_list[0][1]
            Ip_old_curve = koren_ip(Vpk_smooth, vgk, *old_p) * 1000
            ax.plot(Vpk_smooth, Ip_old_curve, '--', color=color, linewidth=1, alpha=0.5)

    ax.set_xlabel("Plate Voltage Vpk (V)")
    ax.set_ylabel("Plate Current Ip (mA)")
    ax.set_title("Plate Curves (solid=new, dashed=old, dots=data)")
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, max(vpk_vals) + 50)
    ax.set_ylim(0)

    # ─── Transfer curves (Ip vs Vgk) ─────────────────────────────────────
    ax = axes[1]
    vgk_min = min(d[1] for d in data)
    Vgk_smooth = np.linspace(vgk_min - 0.5, 0, 500)

    for ci, vpk in enumerate(vpk_vals):
        color = colors[ci % len(colors)]
        # Datasheet points
        pts = [(d[1], d[2]) for d in data if d[0] == vpk]
        if pts:
            vgk_pts, ip_pts = zip(*pts)
            ax.plot(vgk_pts, ip_pts, 'o', color=color, markersize=6, label=f"Data Vpk={vpk:.0f}V")

        # New fit
        Ip_new_curve = koren_ip(vpk, Vgk_smooth, mu, ex, kg1, kp, kvb) * 1000
        ax.plot(Vgk_smooth, Ip_new_curve, '-', color=color, linewidth=1.5, label=f"New Vpk={vpk:.0f}V")

        # Old
        if old_params_list:
            old_p = old_params_list[0][1]
            Ip_old_curve = koren_ip(vpk, Vgk_smooth, *old_p) * 1000
            ax.plot(Vgk_smooth, Ip_old_curve, '--', color=color, linewidth=1, alpha=0.5)

    ax.set_xlabel("Grid Voltage Vgk (V)")
    ax.set_ylabel("Plate Current Ip (mA)")
    ax.set_title("Transfer Curves (solid=new, dashed=old, dots=data)")
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    print(f"\n  Written: {filename}")

# ─── Main ─────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    # ═══ 12AX7 ════════════════════════════════════════════════════════════════
    print("=" * 60)
    print("Fitting 12AX7...")
    print("=" * 60)

    ax7_old = (100.0, 1.4, 1060.0, 600.0, 300.0)
    ax7_paeng = (97.66, 1.17, 552.01, 621.78, 6979.1)

    ax7_bounds = [(50, 150), (0.8, 2.0), (200, 3000), (100, 2000), (1, 50000)]
    ax7_guesses = [
        ("current", list(ax7_old)),
        ("paengdesign", list(ax7_paeng)),
    ]

    ax7_fit = fit_tube("12AX7", ax7_data, ax7_bounds, ax7_guesses)

    print(f"\n  12AX7 Old constants:  mu={ax7_old[0]}, ex={ax7_old[1]}, kg1={ax7_old[2]}, kp={ax7_old[3]}, kvb={ax7_old[4]}")
    print(f"  12AX7 Paeng constants: mu={ax7_paeng[0]}, ex={ax7_paeng[1]}, kg1={ax7_paeng[2]}, kp={ax7_paeng[3]}, kvb={ax7_paeng[4]}")
    print(f"  12AX7 NEW constants:  mu={ax7_fit[0]:.2f}, ex={ax7_fit[1]:.2f}, kg1={ax7_fit[2]:.2f}, kp={ax7_fit[3]:.2f}, kvb={ax7_fit[4]:.2f}")

    evaluate_fit("12AX7", ax7_data, ax7_fit,
                 [("old", ax7_old), ("paeng", ax7_paeng)])
    plot_fit("12AX7", ax7_data, ax7_fit,
             [("old", ax7_old)], "fit_12ax7.png")

    # ═══ 12AU7 ════════════════════════════════════════════════════════════════
    print("\n" + "=" * 60)
    print("Fitting 12AU7...")
    print("=" * 60)

    au7_old = (21.5, 1.35, 1180.0, 84.0, 300.0)

    au7_bounds = [(10, 50), (0.7, 2.0), (100, 3000), (20, 500), (1, 50000)]
    au7_guesses = [
        ("current", list(au7_old)),
    ]

    au7_fit = fit_tube("12AU7", au7_data, au7_bounds, au7_guesses)

    print(f"\n  12AU7 Old constants: mu={au7_old[0]}, ex={au7_old[1]}, kg1={au7_old[2]}, kp={au7_old[3]}, kvb={au7_old[4]}")
    print(f"  12AU7 NEW constants: mu={au7_fit[0]:.2f}, ex={au7_fit[1]:.2f}, kg1={au7_fit[2]:.2f}, kp={au7_fit[3]:.2f}, kvb={au7_fit[4]:.2f}")

    evaluate_fit("12AU7", au7_data, au7_fit,
                 [("old", au7_old)])
    plot_fit("12AU7", au7_data, au7_fit,
             [("old", au7_old)], "fit_12au7.png")

    # ═══ Summary ══════════════════════════════════════════════════════════════
    print("\n" + "=" * 60)
    print("SUMMARY — Copy these to tube_lut_gen.py")
    print("=" * 60)
    print(f'    "12AX7": dict(mu={ax7_fit[0]:.2f}, ex={ax7_fit[1]:.2f}, kg1={ax7_fit[2]:.2f}, kp={ax7_fit[3]:.2f}, kvb={ax7_fit[4]:.2f}),')
    print(f'    "12AU7": dict(mu={au7_fit[0]:.2f}, ex={au7_fit[1]:.2f}, kg1={au7_fit[2]:.2f}, kp={au7_fit[3]:.2f}, kvb={au7_fit[4]:.2f}),')
