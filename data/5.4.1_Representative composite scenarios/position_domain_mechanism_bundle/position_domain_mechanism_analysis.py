#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
复杂森林 + 收缩通道：位置域机制分析

输入：
  timeseries.csv
  replan_events.csv

输出：
  position_profile_0p1m.csv
  position_region_summary.csv
  risk_contribution_summary.csv
  replan_events_with_position.csv
  replan_region_summary.csv
  fig_position_risk.png
  fig_position_speed.png
  fig_position_contraction_components.png
  fig_position_risk_contributions.png
  fig_position_geometry.png
  position_domain_analysis.txt

说明：
1. 位置域曲线采用 0.1 m 位置分箱后的中位数，避免无人机低速位置因为
   50 Hz 时间采样而被过度加权。
2. scene_region 采用当前实验的分析边界：
   forest_before x<-5
   contraction  -5<=x<-1
   bottleneck   -1<=x<=1
   expansion     1<x<=5
   forest_after  x>5
3. 当前代码仍为原始风险驱动速度映射：
   v_max = 2.0 - 1.2*r_bar
   因此不使用 v_safe / R_b_target / constraint_active。
"""

import argparse
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


V_HIGH = 2.0
V_LOW = 0.8
BIN_W = 0.10

BOUNDARIES = [-5.0, -1.0, 1.0, 5.0]
REGION_ORDER = [
    "forest_before",
    "contraction",
    "throat",
    "expansion",
    "forest_after",
]


def region_from_x(x: float) -> str:
    if x < -5.0:
        return "forest_before"
    if x < -1.0:
        return "contraction"
    if x <= 1.0:
        return "throat"
    if x <= 5.0:
        return "expansion"
    return "forest_after"


def add_region_boundaries(ax):
    # 分区边界：-5 / -1 / +1 / +5 m。
    # 区域名称不直接压在曲线上，避免论文图中产生视觉遮挡。
    for x in BOUNDARIES:
        ax.axvline(x, linestyle=":", linewidth=0.9)


def save_plot(fig, path: Path):
    fig.tight_layout()
    fig.savefig(path, dpi=600, bbox_inches="tight")
    plt.close(fig)


def raw_extreme(df, col, mode="max", xlo=None, xhi=None):
    d = df
    if xlo is not None:
        d = d[d["x"] >= xlo]
    if xhi is not None:
        d = d[d["x"] <= xhi]
    idx = d[col].idxmax() if mode == "max" else d[col].idxmin()
    return float(d.loc[idx, "x"]), float(d.loc[idx, col])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeseries", required=True)
    parser.add_argument("--events", required=True)
    parser.add_argument("--out-dir", default="position_domain_outputs")
    args = parser.parse_args()

    ts_file = Path(args.timeseries)
    event_file = Path(args.events)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    ts = pd.read_csv(ts_file)
    events = pd.read_csv(event_file)

    required = [
        "t_s", "x", "v", "d_min", "rho", "C_global", "G_c",
        "R_c", "R_b", "R_g", "r_bar", "rc_bar", "v_max",
        "d_safe", "lambda_obs", "gate", "scene_region",
    ]
    missing = [c for c in required if c not in ts.columns]
    if missing:
        raise ValueError(f"timeseries 缺少列: {missing}")

    valid = ts.dropna(
        subset=["R_c", "R_b", "R_g", "v_max", "r_bar", "rc_bar"]
    ).copy()

    valid["speed_limit_reduction"] = 1.0 - valid["v_max"] / V_HIGH
    valid["adaptive_active_95"] = (
        valid["v_max"] < 0.95 * V_HIGH
    ).astype(int)

    # 风险组成：验证综合风险究竟主要被哪一项推动。
    valid["rho_contrib"] = 0.20 * valid["rho"]
    valid["Rb_contrib"] = 0.55 * valid["R_b"]
    valid["Rc_contrib"] = 0.25 * valid["R_c"]

    # ------------------------ 位置域分箱 ------------------------
    valid["x_bin"] = np.round(valid["x"] / BIN_W) * BIN_W

    profile_cols = [
        "v", "d_min", "rho", "C_global", "G_c", "R_c", "R_b", "R_g",
        "r_bar", "rc_bar", "v_max", "d_safe", "lambda_obs", "gate",
        "speed_limit_reduction", "rho_contrib", "Rb_contrib", "Rc_contrib",
    ]

    profile = (
        valid.groupby("x_bin", as_index=False)[profile_cols]
        .median()
    )
    profile.to_csv(
        out_dir / "position_profile_0p1m.csv",
        index=False,
        encoding="utf-8-sig",
    )

    # ------------------------ 区域统计 ------------------------
    rows = []
    for region in REGION_ORDER:
        d = valid[valid["scene_region"] == region]
        if d.empty:
            continue

        rows.append({
            "scene_region": region,
            "n": len(d),
            "x_min": d["x"].min(),
            "x_max": d["x"].max(),
            "mean_speed": d["v"].mean(),
            "max_speed": d["v"].max(),
            "mean_vmax": d["v_max"].mean(),
            "min_vmax": d["v_max"].min(),
            "max_vmax": d["v_max"].max(),
            "mean_speed_limit_reduction": d["speed_limit_reduction"].mean(),
            "adaptive_active_ratio_0p95": d["adaptive_active_95"].mean(),
            "mean_Rc": d["R_c"].mean(),
            "max_Rc": d["R_c"].max(),
            "mean_Rb": d["R_b"].mean(),
            "p95_Rb": d["R_b"].quantile(0.95),
            "max_Rb": d["R_b"].max(),
            "mean_Rg": d["R_g"].mean(),
            "p95_Rg": d["R_g"].quantile(0.95),
            "max_Rg": d["R_g"].max(),
            "min_dmin": d["d_min"].min(),
            "mean_gate": d["gate"].mean(),
            "mean_dsafe": d["d_safe"].mean(),
        })

    region_summary = pd.DataFrame(rows)
    region_summary.to_csv(
        out_dir / "position_region_summary.csv",
        index=False,
        encoding="utf-8-sig",
    )

    contrib = (
        valid.groupby("scene_region")[
            ["rho_contrib", "Rb_contrib", "Rc_contrib"]
        ]
        .mean()
        .reindex(REGION_ORDER)
    )
    contrib_pct = contrib.div(contrib.sum(axis=1), axis=0) * 100.0
    contrib_out = contrib.copy()
    contrib_out.columns = [
        "mean_rho_contribution",
        "mean_Rb_contribution",
        "mean_Rc_contribution",
    ]
    contrib_out["rho_share_pct"] = contrib_pct["rho_contrib"]
    contrib_out["Rb_share_pct"] = contrib_pct["Rb_contrib"]
    contrib_out["Rc_share_pct"] = contrib_pct["Rc_contrib"]
    contrib_out.to_csv(
        out_dir / "risk_contribution_summary.csv",
        encoding="utf-8-sig",
    )

    # ------------------------ 重规划事件映射到位置 ------------------------
    events2 = events.sort_values("timestamp_s").copy()
    ts_sorted = ts.sort_values("t_s").copy()

    events2["x"] = np.interp(
        events2["timestamp_s"],
        ts_sorted["t_s"],
        ts_sorted["x"],
    )
    events2["scene_region"] = events2["x"].map(region_from_x)

    events2.to_csv(
        out_dir / "replan_events_with_position.csv",
        index=False,
        encoding="utf-8-sig",
    )

    event_summary = (
        events2.groupby("scene_region")
        .agg(
            events=("count", "count"),
            success_rate=("success", "mean"),
            mean_planning_ms=("planning_time_ms", "mean"),
            p95_planning_ms=(
                "planning_time_ms",
                lambda s: s.quantile(0.95),
            ),
            max_planning_ms=("planning_time_ms", "max"),
        )
        .reindex(REGION_ORDER)
        .dropna(how="all")
    )
    event_summary.to_csv(
        out_dir / "replan_region_summary.csv",
        encoding="utf-8-sig",
    )

    # ------------------------ 图 1：风险 ------------------------
    fig, ax = plt.subplots(figsize=(9.6, 4.2))
    ax.plot(profile["x_bin"], profile["R_c"], label=r"$R_c$", linewidth=1.6)
    ax.plot(profile["x_bin"], profile["R_b"], label=r"$R_b$", linewidth=1.6)
    ax.plot(profile["x_bin"], profile["R_g"], label=r"$R_g$", linewidth=1.6)
    ax.set_xlabel("Position x (m)")
    ax.set_ylabel("Risk")
    ax.set_xlim(ts["x"].min(), ts["x"].max())
    ax.set_ylim(0, 1.05)
    add_region_boundaries(ax)
    ax.legend(frameon=False, ncol=3)
    ax.set_title("Position-domain risk response (0.1 m median bins)")
    save_plot(fig, out_dir / "fig_position_risk.png")

    # ------------------------ 图 2：速度 ------------------------
    fig, ax = plt.subplots(figsize=(9.6, 4.2))
    ax.plot(
        profile["x_bin"],
        profile["v_max"],
        label=r"$v_{\max}$",
        linewidth=1.7,
    )
    ax.plot(
        profile["x_bin"],
        profile["v"],
        label="actual v",
        linewidth=1.5,
    )
    ax.axhline(
        V_HIGH,
        linestyle="--",
        linewidth=0.9,
        label=r"$v_{\mathrm{high}}=2.0$",
    )
    ax.set_xlabel("Position x (m)")
    ax.set_ylabel("Speed (m/s)")
    ax.set_xlim(ts["x"].min(), ts["x"].max())
    ax.set_ylim(0, 2.1)
    add_region_boundaries(ax)
    ax.legend(frameon=False, ncol=3)
    ax.set_title("Position-domain adaptive speed response")
    save_plot(fig, out_dir / "fig_position_speed.png")

    # ------------------------ 图 3：Rc 组成 ------------------------
    fig, ax = plt.subplots(figsize=(9.6, 4.2))
    ax.plot(
        profile["x_bin"],
        profile["C_global"],
        label=r"$C_{\mathrm{global}}$",
        linewidth=1.5,
    )
    ax.plot(
        profile["x_bin"],
        profile["G_c"],
        label=r"$G_c$",
        linewidth=1.5,
    )
    ax.plot(
        profile["x_bin"],
        profile["R_c"],
        label=r"$R_c$",
        linewidth=1.7,
    )
    ax.plot(
        profile["x_bin"],
        profile["rc_bar"],
        label=r"$\bar{R}_c$",
        linewidth=1.5,
    )
    ax.set_xlabel("Position x (m)")
    ax.set_ylabel("Contraction signal")
    ax.set_xlim(ts["x"].min(), ts["x"].max())
    add_region_boundaries(ax)
    ax.legend(frameon=False, ncol=4)
    ax.set_title("Position-domain contraction-risk components")
    save_plot(fig, out_dir / "fig_position_contraction_components.png")

    # ------------------------ 图 4：综合风险组成 ------------------------
    fig, ax = plt.subplots(figsize=(9.6, 4.2))
    ax.plot(
        profile["x_bin"],
        profile["rho_contrib"],
        label=r"$0.20\rho$",
        linewidth=1.5,
    )
    ax.plot(
        profile["x_bin"],
        profile["Rb_contrib"],
        label=r"$0.55R_b$",
        linewidth=1.6,
    )
    ax.plot(
        profile["x_bin"],
        profile["Rc_contrib"],
        label=r"$0.25R_c$",
        linewidth=1.5,
    )
    ax.set_xlabel("Position x (m)")
    ax.set_ylabel("Contribution to raw risk")
    ax.set_xlim(ts["x"].min(), ts["x"].max())
    add_region_boundaries(ax)
    ax.legend(frameon=False, ncol=3)
    ax.set_title("Position-domain composition of the raw combined risk")
    save_plot(fig, out_dir / "fig_position_risk_contributions.png")

    # ------------------------ 图 5：几何距离 ------------------------
    fig, ax = plt.subplots(figsize=(9.6, 4.2))
    ax.plot(
        profile["x_bin"],
        profile["d_min"],
        label=r"$d_{\min}$",
        linewidth=1.5,
    )
    ax.plot(
        profile["x_bin"],
        profile["d_safe"],
        label=r"$d_{\mathrm{safe}}$",
        linewidth=1.6,
    )
    ax.set_xlabel("Position x (m)")
    ax.set_ylabel("Distance (m)")
    ax.set_xlim(ts["x"].min(), ts["x"].max())
    add_region_boundaries(ax)
    ax.legend(frameon=False, ncol=2)
    ax.set_title(
        "Position-domain geometric clearance and adaptive safety distance"
    )
    save_plot(fig, out_dir / "fig_position_geometry.png")

    # ------------------------ 关键数值 ------------------------
    rc_x, rc_peak = raw_extreme(valid, "R_c", "max")
    rcbar_x, rcbar_peak = raw_extreme(valid, "rc_bar", "max")
    throat_vmax_x, throat_vmax_min = raw_extreme(
        valid, "v_max", "min", -1.0, 1.0
    )
    throat_rb_x, throat_rb_max = raw_extreme(
        valid, "R_b", "max", -1.0, 1.0
    )

    vmax_formula = V_HIGH - valid["r_bar"] * (V_HIGH - V_LOW)
    vmax_err = float(
        np.max(np.abs(valid["v_max"] - vmax_formula))
    )
    gate_err = float(
        np.max(
            np.abs(valid["gate"] - (1.0 - valid["rc_bar"]))
        )
    )

    failures = events2[events2["success"] == 0]
    next_event_dt = np.nan
    if not failures.empty:
        fail_idx = failures.index[0]
        pos = events2.index.get_loc(fail_idx)
        if pos + 1 < len(events2):
            next_event_dt = float(
                events2.iloc[pos + 1]["timestamp_s"]
                - events2.iloc[pos]["timestamp_s"]
            )

    report = [
        "POSITION-DOMAIN MECHANISM ANALYSIS",
        "=" * 76,
        f"Rows: {len(ts)}, valid risk/adaptive rows: {len(valid)}",
        (
            f"Flight: x={ts['x'].min():.4f} -> "
            f"{ts['x'].max():.4f} m, "
            f"T={ts['t_s'].max()-ts['t_s'].min():.4f} s"
        ),
        "",
        "KEY POSITIONS",
        (
            f"max Rc={rc_peak:.4f} at x={rc_x:.4f} m "
            f"(about {-5.0-rc_x:.3f} m before x=-5 boundary)"
        ),
        f"max rc_bar={rcbar_peak:.4f} at x={rcbar_x:.4f} m",
        (
            f"bottleneck min v_max={throat_vmax_min:.4f} m/s "
            f"at x={throat_vmax_x:.4f} m"
        ),
        (
            f"bottleneck max Rb={throat_rb_max:.4f} "
            f"at x={throat_rb_x:.4f} m"
        ),
        "",
        "IMPLEMENTATION CONSISTENCY",
        f"max |v_max - [2.0 - 1.2*r_bar]| = {vmax_err:.3e}",
        f"max |gate - (1-rc_bar)| = {gate_err:.3e}",
        "",
        "REGION SUMMARY",
        region_summary.round(4).to_string(index=False),
        "",
        "MEAN RAW-RISK CONTRIBUTION SHARES (%)",
        contrib_pct.round(1).to_string(),
        "",
        "REPLAN SUMMARY",
        event_summary.round(4).to_string(),
        "",
        (
            f"All planning events: n={len(events2)}, "
            f"mean={events2['planning_time_ms'].mean():.4f} ms, "
            f"P95={events2['planning_time_ms'].quantile(0.95):.4f} ms, "
            f"max={events2['planning_time_ms'].max():.4f} ms"
        ),
        f"Failed events: {len(failures)}",
    ]

    if len(failures):
        report.append(
            f"First failure count={int(failures.iloc[0]['count'])}, "
            f"x≈{failures.iloc[0]['x']:.3f} m; "
            f"next planning event after {next_event_dt*1000:.1f} ms"
        )

    report += [
        "",
        "INTERPRETATION",
        (
            "1) Rc is not dead: the global raw Rc maximum appears at "
            "x≈-5.30 m, before the contraction-analysis boundary x=-5 m."
        ),
        (
            "2) Rc is a forward contraction/change indicator, not an absolute "
            "narrowness indicator; therefore its value can fall inside the "
            "bottleneck."
        ),
        (
            "3) The bottleneck speed-limit reduction is real, but the first "
            "forest is also strongly constrained by braking risk. Therefore "
            "this run does NOT support the claim that the whole forest-before "
            "region is almost unrestricted."
        ),
        (
            "4) Rb dominates the raw combined-risk budget in every region; "
            "Rc contributes less, but can act earlier around the corridor entrance."
        ),
        (
            "5) Expansion/forest also produce Rc responses. In a cluttered "
            "environment, the present Rc has limited specificity because local "
            "forest-density changes can also alter forward openness."
        ),
        (
            "6) adaptive_active_ratio with v_max<0.95*v_high is nearly saturated. "
            "Use continuous speed-limit reduction A_v=1-v_max/v_high instead."
        ),
    ]

    (out_dir / "position_domain_analysis.txt").write_text(
        "\n".join(report),
        encoding="utf-8",
    )

    print(region_summary[[
        "scene_region",
        "mean_speed",
        "mean_vmax",
        "min_vmax",
        "mean_Rc",
        "max_Rc",
        "mean_Rb",
        "p95_Rb",
        "mean_Rg",
        "min_dmin",
        "mean_speed_limit_reduction",
    ]].round(4).to_string(index=False))

    print("\nRisk contribution shares (%):")
    print(contrib_pct.round(1).to_string())

    print("\nOutputs:", out_dir)


if __name__ == "__main__":
    main()
