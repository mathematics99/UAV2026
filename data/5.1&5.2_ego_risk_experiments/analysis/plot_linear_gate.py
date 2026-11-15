#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Section 6.4 - Linear contraction gate ablation
==============================================

Purpose
-------
Rebuild the gate ON/OFF experiment with an explicitly defined linear
contraction geometry, so the conclusion no longer depends on whether the
old map was actually a contracting corridor.

Controlled geometry
-------------------
- Opening width: 5.0 m
- Linear contraction: x = -5.0 ... 0.0 m
- Throat: x >= 0
- Two throat gaps: 1.4 m and 1.0 m
- UAV follows the corridor centerline
- Forward ROI: 5.0 m, 10 slices
- Openness of slice i: O_i = width_i / opening_width
- C_global = max(mean(O_near) - mean(O_far), 0)
- G_c = max positive adjacent openness drop
- R_c = 0.6*C_global + 0.4*G_c

Gate ablation
-------------
The motion state is held the same for gate ON/OFF.
A representative fixed speed v=1.2 m/s is used only to construct the
same-state braking-risk contribution. No uncalibrated occupancy surrogate
is added (rho = 0).

R_b = clip((v*tau + v^2/(2*a_b))/(d_min + eps), 0, 1)
R_g = clip(0.20*rho + 0.55*R_b + 0.25*R_c, 0, 1)

Both R_g and R_c use EMA smoothing lambda=0.9 and a per-step change limit
of +/-0.05.

Gate OFF:
    d_safe = d_base + r_bar*(d_max-d_base)

Gate ON:
    H_c = 1 - rc_bar
    d_safe = d_base + H_c*r_bar*(d_max-d_base)

Outputs
-------
- section64_linear_gap14.csv
- section64_linear_gap10.csv
- section64_linear_summary.csv
- fig08_linear_contraction_gate.png
- section64_linear_report.txt
"""

from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# ------------------------------- parameters -------------------------------
WIDE = 5.0
CONTRACTION_START = -5.0
THROAT_X = 0.0
SIM_START = -8.0
SIM_END = 2.0
DX = 0.05

ROI_LEN = 5.0
N_SLICES = 10
ETA = 0.6

V_FIXED = 1.2
TAU = 0.3
A_B = 2.0
EPS = 0.1

W_RHO = 0.20
W_B = 0.55
W_C = 0.25
RHO_CONTROLLED = 0.0

EMA_LAMBDA = 0.9
DELTA_MAX = 0.05

D_BASE = 0.5
D_MAX = 1.0

RC_ACTIVE_THRESHOLD = 0.05
GAPS = [1.4, 1.0]


def corridor_width(x: float, gap: float) -> float:
    """5 m opening -> linearly contracted gap -> constant throat."""
    if x <= CONTRACTION_START:
        return WIDE
    if x < THROAT_X:
        s = (x - CONTRACTION_START) / (THROAT_X - CONTRACTION_START)
        return WIDE - (WIDE - gap) * s
    return gap


def contraction_risk_at_x(x: float, gap: float):
    """
    Evaluate the paper's multi-slice contraction quantities on the
    controlled linear geometry.
    """
    slice_len = ROI_LEN / N_SLICES
    centers = x + (np.arange(N_SLICES) + 0.5) * slice_len
    widths = np.array([corridor_width(xx, gap) for xx in centers], dtype=float)

    openness = np.clip(widths / WIDE, 0.0, 1.0)

    near = openness[:3].mean()
    far = openness[-3:].mean()

    c_global = max(0.0, float(near - far))
    g_c = max(0.0, float(np.max(openness[:-1] - openness[1:])))
    r_c = np.clip(ETA * c_global + (1.0 - ETA) * g_c, 0.0, 1.0)

    return c_global, g_c, r_c


def ema_rate_limited(prev: float, raw: float) -> float:
    target = EMA_LAMBDA * prev + (1.0 - EMA_LAMBDA) * raw
    return float(np.clip(target, prev - DELTA_MAX, prev + DELTA_MAX))


def simulate_gap(gap: float) -> pd.DataFrame:
    xs = np.arange(SIM_START, SIM_END + 1e-12, DX)

    rows = []
    prev_rc_bar = None
    prev_r_bar = None

    d_brake = V_FIXED * TAU + V_FIXED ** 2 / (2.0 * A_B)

    for x in xs:
        width = corridor_width(float(x), gap)
        d_min = width / 2.0

        c_global, g_c, r_c = contraction_risk_at_x(float(x), gap)

        r_b = np.clip(d_brake / (d_min + EPS), 0.0, 1.0)
        r_g = np.clip(
            W_RHO * RHO_CONTROLLED + W_B * r_b + W_C * r_c,
            0.0,
            1.0,
        )

        if prev_rc_bar is None:
            rc_bar = float(r_c)
            r_bar = float(r_g)
        else:
            rc_bar = ema_rate_limited(prev_rc_bar, float(r_c))
            r_bar = ema_rate_limited(prev_r_bar, float(r_g))

        gate = 1.0 - rc_bar

        d_safe_off = D_BASE + r_bar * (D_MAX - D_BASE)
        d_safe_on = D_BASE + gate * r_bar * (D_MAX - D_BASE)

        rows.append(
            {
                "x_m": float(x),
                "gap_m": gap,
                "corridor_width_m": width,
                "d_min_m": d_min,
                "C_global": c_global,
                "G_c": g_c,
                "R_c": float(r_c),
                "rc_bar": rc_bar,
                "R_b": float(r_b),
                "R_g_raw": float(r_g),
                "r_bar": r_bar,
                "gate": gate,
                "d_safe_gate_on_m": d_safe_on,
                "d_safe_gate_off_m": d_safe_off,
                "suppression_m": d_safe_off - d_safe_on,
                "contraction_active": int(rc_bar >= RC_ACTIVE_THRESHOLD),
            }
        )

        prev_rc_bar = rc_bar
        prev_r_bar = r_bar

    return pd.DataFrame(rows)


def summarize(df: pd.DataFrame) -> dict:
    gap = float(df["gap_m"].iloc[0])
    active = df[df["contraction_active"] == 1].copy()

    idx_peak_rc = int(df["rc_bar"].idxmax())
    peak_rc_row = df.loc[idx_peak_rc]

    idx_peak_sup = int(df["suppression_m"].idxmax())
    peak_sup_row = df.loc[idx_peak_sup]

    mean_off = float(active["d_safe_gate_off_m"].mean())
    mean_on = float(active["d_safe_gate_on_m"].mean())

    return {
        "gap_m": gap,
        "max_Rc": float(df["R_c"].max()),
        "max_rc_bar": float(df["rc_bar"].max()),
        "min_gate": float(df["gate"].min()),
        "active_x_start_m": float(active["x_m"].min()),
        "active_x_end_m": float(active["x_m"].max()),
        "active_mean_dsafe_off_m": mean_off,
        "active_mean_dsafe_on_m": mean_on,
        "active_mean_suppression_m": mean_off - mean_on,
        "active_mean_suppression_pct": 100.0 * (mean_off - mean_on) / mean_off,
        "peak_paired_suppression_m": float(df["suppression_m"].max()),
        "peak_suppression_x_m": float(peak_sup_row["x_m"]),
        "peak_suppression_width_m": float(peak_sup_row["corridor_width_m"]),
        "peak_rc_x_m": float(peak_rc_row["x_m"]),
        "peak_rc_width_m": float(peak_rc_row["corridor_width_m"]),
        "dsafe_off_at_peak_rc_m": float(peak_rc_row["d_safe_gate_off_m"]),
        "dsafe_on_at_peak_rc_m": float(peak_rc_row["d_safe_gate_on_m"]),
    }


def make_figure(data_by_gap, out_path: Path):
    plt.rcParams["axes.unicode_minus"] = False

    fig, axes = plt.subplots(1, 2, figsize=(9.2, 3.8), sharey=True)

    for ax, gap in zip(axes, GAPS):
        df = data_by_gap[gap]

        ax.plot(
            df["x_m"],
            df["d_safe_gate_on_m"],
            marker="o",
            markevery=12,
            linewidth=1.8,
            markersize=3.8,
            label="gate ON",
        )
        ax.plot(
            df["x_m"],
            df["d_safe_gate_off_m"],
            linestyle="--",
            marker="s",
            markevery=12,
            linewidth=1.6,
            markersize=3.6,
            label="gate OFF",
        )

        ax.axvspan(CONTRACTION_START, THROAT_X, alpha=0.08)
        ax.axvline(THROAT_X, linewidth=0.9, linestyle=":")

        s = summarize(df)
        ax.text(
            0.03,
            0.05,
            (
                f"max $\\bar{{R}}_c$ = {s['max_rc_bar']:.3f}\n"
                f"min gate = {s['min_gate']:.3f}\n"
                f"peak suppression = {1000*s['peak_paired_suppression_m']:.1f} mm"
            ),
            transform=ax.transAxes,
            fontsize=8.5,
            va="bottom",
            bbox=dict(boxstyle="round,pad=0.25", facecolor="white", alpha=0.78, edgecolor="none"),
        )

        ax.set_title(f"gap = {gap:.1f} m", fontweight="bold")
        ax.set_xlabel("Position x (m)")
        ax.grid(False)

    axes[0].set_ylabel(r"$d_{\mathrm{safe}}$ (m)")
    axes[1].legend(frameon=False, loc="upper left")

    fig.suptitle(
        "Gate ablation in a linear contracting corridor",
        y=1.02,
        fontsize=11,
    )
    fig.tight_layout()
    fig.savefig(out_path, dpi=600, bbox_inches="tight")
    plt.close(fig)


def main():
    out_dir = Path(__file__).resolve().parent / "section64_linear_outputs"
    out_dir.mkdir(parents=True, exist_ok=True)

    data_by_gap = {}
    summaries = []

    for gap in GAPS:
        df = simulate_gap(gap)
        data_by_gap[gap] = df

        suffix = "14" if abs(gap - 1.4) < 1e-9 else "10"
        df.to_csv(
            out_dir / f"section64_linear_gap{suffix}.csv",
            index=False,
            encoding="utf-8-sig",
        )
        summaries.append(summarize(df))

    summary_df = pd.DataFrame(summaries)
    summary_df.to_csv(
        out_dir / "section64_linear_summary.csv",
        index=False,
        encoding="utf-8-sig",
    )

    fig_path = out_dir / "fig08_linear_contraction_gate.png"
    make_figure(data_by_gap, fig_path)

    report_lines = [
        "Section 6.4 controlled linear-contraction gate ablation",
        "=" * 68,
        f"Opening width = {WIDE:.1f} m",
        f"Linear contraction x = [{CONTRACTION_START:.1f}, {THROAT_X:.1f}] m",
        f"Forward ROI = {ROI_LEN:.1f} m, slices = {N_SLICES}",
        f"Fixed motion state v = {V_FIXED:.1f} m/s",
        f"Active contraction definition: rc_bar >= {RC_ACTIVE_THRESHOLD:.2f}",
        "",
    ]

    for s in summaries:
        report_lines += [
            f"gap = {s['gap_m']:.1f} m",
            f"  max Rc / max rc_bar : {s['max_Rc']:.4f} / {s['max_rc_bar']:.4f}",
            f"  min gate            : {s['min_gate']:.4f}",
            (
                "  active mean d_safe   : "
                f"OFF {s['active_mean_dsafe_off_m']:.4f} -> "
                f"ON {s['active_mean_dsafe_on_m']:.4f} m "
                f"({s['active_mean_suppression_pct']:.2f}% reduction)"
            ),
            (
                "  peak paired reduction: "
                f"{s['peak_paired_suppression_m']:.4f} m "
                f"at x={s['peak_suppression_x_m']:.2f} m"
            ),
            (
                "  at max rc_bar        : "
                f"OFF {s['dsafe_off_at_peak_rc_m']:.4f} -> "
                f"ON {s['dsafe_on_at_peak_rc_m']:.4f} m"
            ),
            "",
        ]

    report_lines += [
        "Interpretation:",
        "- The gate is strongest while the forward ROI sees a width gradient.",
        "- Once the UAV enters the constant-width throat, contraction risk naturally",
        "  decays because the geometry is narrow but no longer contracting.",
        "- Therefore the gate should be interpreted as suppressing safety-margin",
        "  inflation during the contraction process, not as permanently shrinking",
        "  d_safe throughout every narrow but uniform section.",
        "- This is a controlled model-level ablation, not a replacement for a",
        "  closed-loop ROS/Gazebo success-rate experiment.",
    ]

    (out_dir / "section64_linear_report.txt").write_text(
        "\n".join(report_lines),
        encoding="utf-8",
    )

    print(summary_df.to_string(index=False))
    print(f"\nOutputs: {out_dir}")


if __name__ == "__main__":
    main()
