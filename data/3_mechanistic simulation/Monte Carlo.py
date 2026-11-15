from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from paper_style import setup_style, save_figure

SEED = 20260824
N = 30000
V0, D0 = 1.0, 0.8
SIGMA_V, SIGMA_D = 0.08, 0.04
EPS = 0.1
STATIC_D0 = 0.3


def rb(v, dmin, tau, ab, static_d=0.0):
    dreq = static_d + v * tau + v**2 / (2 * ab)
    return np.clip(dreq / (dmin + EPS), 0, 1)


def ecdf(x):
    x = np.sort(x)
    y = np.arange(1, x.size + 1) / x.size
    return x, y


def make_figure(outdir="outputs", save_pdf=False):
    setup_style()
    fig, axes = plt.subplots(1, 2, figsize=(10.2, 4.0))

    tau = np.linspace(0.1, 0.6, 220)
    ab = np.linspace(1.0, 4.0, 220)
    T, A = np.meshgrid(tau, ab)
    Z = rb(V0, D0, T, A)
    m = axes[0].pcolormesh(T, A, Z, shading="auto")
    fig.colorbar(m, ax=axes[0], fraction=0.047, pad=0.03)
    axes[0].scatter([0.3], [2.0], s=30, facecolors="none", edgecolors="black", linewidths=1)
    axes[0].annotate("Initial parameters", (0.3, 2.0), xytext=(0.36, 3.7),
                     arrowprops=dict(arrowstyle="->", lw=0.8), fontsize=8)
    axes[0].set_xlabel("Reaction time $\\tau$ (s)")
    axes[0].set_ylabel("Braking acceleration $a_b$ (m/s$^2$)")
    axes[0].set_title("(a) Parameter sensitivity: $R_b(\\tau,a_b)$")
    axes[0].grid(False)

    rng = np.random.default_rng(SEED)
    v = rng.normal(V0, SIGMA_V, N)
    d = rng.normal(D0, SIGMA_D, N)
    r_dec = rb(v, d, 0.3, 2.0, static_d=0.0)
    r_old = rb(v, d, 0.3, 2.0, static_d=STATIC_D0)
    for arr, label in [(r_dec, "Decoupled model"), (r_old, "With static $d_0$")]:
        x, y = ecdf(arr)
        axes[1].plot(x, y, lw=1.7, label=label)
    axes[1].axvline(1.0, ls="--", lw=0.9)
    axes[1].set_xlim(0.25, 1.02)
    axes[1].set_ylim(0, 1.02)
    axes[1].set_xlabel("Braking risk $R_b$ under perturbation")
    axes[1].set_ylabel("Empirical cumulative distribution")
    axes[1].set_title("(b) Risk distribution under speed/distance perturbation")
    axes[1].legend(frameon=False, loc="upper left")

    #stats = (
     #   f"解耦: mean={r_dec.mean():.3f}, P95={np.percentile(r_dec,95):.3f}, sat={100*np.mean(r_dec>=1):.3f}%\n"
     #   f"旧式: mean={r_old.mean():.3f}, P95={np.percentile(r_old,95):.3f}, sat={100*np.mean(r_old>=1):.1f}%"
    #)
    #axes[1].text(0.52, 0.08, stats, fontsize=7.5)

    fig.suptitle("")
    fig.tight_layout()
    return save_figure(fig, outdir, "fig04_sensitivity_montecarlo", save_pdf)


if __name__ == "__main__":
    make_figure(Path(__file__).parent / "outputs")
