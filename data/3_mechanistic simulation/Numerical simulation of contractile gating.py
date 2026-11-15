from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
from paper_style import setup_style, save_figure

TAU = 0.3
A_B = 2.0
EPS = 0.1
D_BASE, D_MAX = 0.5, 1.0
LAMBDA_MIN, LAMBDA_MAX = 0.5, 2.0
GAMMA, K_C = 2.0, 0.5


def rb(v, dmin):
    dbrake = v * TAU + v**2 / (2 * A_B)
    return np.clip(dbrake / (dmin + EPS), 0, 1)


def dsafe(rbar, rcbar):
    hc = 1 - rcbar
    return D_BASE + hc * rbar * (D_MAX - D_BASE)


def lambda_obs(rbar, rcbar):
    return LAMBDA_MIN + (1 - K_C * rcbar) * (rbar**GAMMA) * (LAMBDA_MAX - LAMBDA_MIN)


def make_figure(outdir="outputs", save_pdf=False):
    setup_style()
    fig, axes = plt.subplots(1, 3, figsize=(12.0, 3.7))

    # ---------- 关闭网格线、加粗边框 ----------
    for ax in axes:
        ax.grid(False)
        for spine in ax.spines.values():
            spine.set_linewidth(1.2)

    # =========================================================
    # (a) 动态制动风险 R_b
    # 色系：严格按参考图 -> viridis
    # =========================================================
    v = np.linspace(0, 2.5, 240)
    d = np.linspace(0.35, 1.5, 220)
    V, D = np.meshgrid(v, d)
    Z1 = rb(V, D)

    m1 = axes[0].pcolormesh(
        V, D, Z1,
        shading="auto",
        cmap="viridis",
        vmin=0.0, vmax=1.0
    )

    cs = axes[0].contour(
        V, D, Z1,
        levels=[0.25, 0.5, 0.75],
        linewidths=0.7,
        colors='white'
    )
    axes[0].clabel(cs, inline=True, fontsize=7, fmt="%.2g")

    cbar1 = fig.colorbar(m1, ax=axes[0], fraction=0.047, pad=0.03)
    cbar1.set_ticks(np.arange(0.0, 1.01, 0.2))
    cbar1.ax.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))

    axes[0].set_xlabel("Velocity $v$ (m s$^{-1}$)")
    axes[0].set_ylabel("Minimum obstacle distance $d_{min}$ (m)")
    axes[0].set_title("(a) Dynamic braking risk $R_b$")

    axes[0].set_xlim(0, 2.5)

    # 让图像显示范围严格贴合数据，不额外冒到 1.6
    axes[0].set_ylim(d.min(), d.max())  # 这里就是 0.35 到 1.5

    # x 轴刻度保持不变
    axes[0].set_xticks(np.arange(0, 2.6, 0.5))

    # y 轴只标到 1.4，不额外标 1.6
    axes[0].set_yticks(np.arange(0.4, 1.41, 0.2))

    # 让子图框看起来接近正方形
    axes[0].set_box_aspect(1)

    # =========================================================
    # (b) 门控安全距离 d_safe
    # 色系：严格按参考图 -> YlGnBu
    # 低值浅黄，高值深蓝
    # =========================================================
    r = np.linspace(0, 1, 220)
    rc = np.linspace(0, 1, 220)
    R, RC = np.meshgrid(r, rc)
    Z2 = dsafe(R, RC)

    m2 = axes[1].pcolormesh(
        R, RC, Z2,
        shading="auto",
        cmap="YlGnBu",
        vmin=0.5, vmax=1.0
    )

    cbar2 = fig.colorbar(m2, ax=axes[1], fraction=0.047, pad=0.03)
    cbar2.set_ticks(np.arange(0.5, 1.01, 0.1))
    cbar2.ax.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))

    axes[1].set_xlabel("Smoothed overall risk $\\bar{R}$")
    axes[1].set_ylabel("Contraction risk $\\bar{R}_c$")
    axes[1].set_title("(b) Gated safety distance $d_{safe}$")

    axes[1].set_xlim(0, 1)
    axes[1].set_ylim(0, 1)
    axes[1].set_xticks(np.arange(0, 1.01, 0.2))
    axes[1].set_yticks(np.arange(0, 1.01, 0.2))

    # =========================================================
    # (c) 障碍惩罚权重 lambda_obs
    # 色系：严格按参考图 -> magma
    # 低值深紫/黑，高值橙黄
    # =========================================================
    Z3 = lambda_obs(R, RC)

    m3 = axes[2].pcolormesh(
        R, RC, Z3,
        shading="auto",
        cmap="magma",
        vmin=0.5, vmax=2.0
    )

    cbar3 = fig.colorbar(m3, ax=axes[2], fraction=0.047, pad=0.03)
    cbar3.set_ticks(np.arange(0.6, 2.01, 0.2))
    cbar3.ax.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))

    axes[2].set_xlabel("Smoothed overall risk $\\bar{R}$")
    axes[2].set_ylabel("Contraction risk $\\bar{R}_c$")
    axes[2].set_title("(c) Obstacle penalty weight $\\lambda_{obs}$")

    axes[2].set_xlim(0, 1)
    axes[2].set_ylim(0, 1)
    axes[2].set_xticks(np.arange(0, 1.01, 0.2))
    axes[2].set_yticks(np.arange(0, 1.01, 0.2))

    fig.suptitle("")
    fig.tight_layout()
    return save_figure(fig, outdir, "fig03_response_surfaces", save_pdf)


if __name__ == "__main__":
    make_figure(Path(__file__).parent / "outputs")