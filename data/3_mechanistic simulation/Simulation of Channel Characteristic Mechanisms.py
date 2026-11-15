#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

try:
    from paper_style import setup_style, save_figure
except ImportError:
    def setup_style():
        plt.rcParams["font.sans-serif"] = [
            "Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "DejaVu Sans"
        ]
        plt.rcParams["axes.unicode_minus"] = False
        plt.rcParams["font.size"] = 10

    def save_figure(fig, outdir, stem, save_pdf=False):
        outdir = Path(outdir)
        outdir.mkdir(parents=True, exist_ok=True)
        png = outdir / f"{stem}.png"
        fig.savefig(png, dpi=600, bbox_inches="tight")
        if save_pdf:
            fig.savefig(outdir / f"{stem}.pdf", bbox_inches="tight")
        plt.close(fig)
        return png


ETA = 0.6
EPS = 1e-6
slice_id = np.arange(1, 11)

F_gradual = np.array([
    0.95, 0.91, 0.87, 0.83, 0.79,
    0.755, 0.715, 0.675, 0.640, 0.600
])

F_bottleneck = np.array([
    0.95, 0.935, 0.925, 0.915, 0.895,
    0.875, 0.855, 0.520, 0.500, 0.490
])

F_uniform = np.array([
    0.860, 0.870, 0.860, 0.850, 0.860,
    0.850, 0.860, 0.860, 0.850, 0.860
])


def contraction_components(F):
    F = np.asarray(F, dtype=float)
    F_near = np.mean(F[:3])
    F_far = np.mean(F[-3:])

    C_global = np.clip(
        (F_near - F_far) / (F_near + EPS),
        0.0, 1.0
    )

    local_drop = (F[:-1] - F[1:]) / (F[:-1] + EPS)
    G_local = np.max(np.clip(local_drop, 0.0, 1.0))

    R_c = ETA * C_global + (1.0 - ETA) * G_local
    return C_global, G_local, R_c


profiles = {
    "Gradual contraction": F_gradual,
    "Local bottleneck": F_bottleneck,
    "Approximately uniform width": F_uniform,
}

components = {
    name: contraction_components(F)
    for name, F in profiles.items()
}


def make_figure(outdir="outputs", save_pdf=False):
    setup_style()

    fig, axes = plt.subplots(
        1, 2,
        figsize=(10.2, 4.2),
        gridspec_kw={"width_ratios": [1.0, 1.05]},
    )

    # (a)
    ax = axes[0]
    ax.plot(slice_id, F_gradual, marker="o", linewidth=1.8, markersize=4.5, label="Gradual contraction")
    ax.plot(slice_id, F_bottleneck, marker="o", linewidth=1.8, markersize=4.5, label="Local bottleneck")
    ax.plot(slice_id, F_uniform, marker="o", linewidth=1.8, markersize=4.5, label="Approximately uniform width")

    ax.set_xlim(0.6, 10.4)
    ax.set_ylim(0.40, 1.00)
    ax.set_xlabel("Forward slice index")
    ax.set_ylabel(r"Cross-section openness $F_i$")
    ax.set_title("(a) Three typical openness profiles", loc="left", fontweight="bold")
    ax.legend(frameon=False, loc="lower left")
    ax.grid(True, linewidth=0.5, alpha=0.35)

    # (b)
    ax = axes[1]
    names = list(profiles.keys())
    x = np.arange(len(names))
    width = 0.24

    C_vals = [components[n][0] for n in names]
    G_vals = [components[n][1] for n in names]
    R_vals = [components[n][2] for n in names]

    ax.bar(x - width, C_vals, width, label=r"$C_{global}$")
    ax.bar(x, G_vals, width, label=r"$G_{local}$")
    ax.bar(x + width, R_vals, width, label=r"$R_c$")

    ax.set_xticks(x)
    ax.set_xticklabels(names)
    ax.set_ylim(0.0, 0.55)
    ax.set_ylabel("Risk component")
    ax.set_title("(b) Complementary response of global and local terms", loc="left", fontweight="bold")
    ax.legend(frameon=False, loc="upper right", ncol=3)
    ax.grid(True, axis="y", linewidth=0.5, alpha=0.35)

    fig.tight_layout()
    return save_figure(fig, outdir, "fig01_contraction_mechanism", save_pdf)


if __name__ == "__main__":
    for name, (cg, gl, rc) in components.items():
        print(f"{name}: C_global={cg:.3f}, G_local={gl:.3f}, R_c={rc:.3f}")

    out = make_figure(
        Path(__file__).resolve().parent / "outputs",
        save_pdf=False
    )
    print("saved:", out)
