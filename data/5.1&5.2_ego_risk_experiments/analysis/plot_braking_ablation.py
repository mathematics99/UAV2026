from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from paper_style import setup_style, save_figure


# ============================================================
# 图 7：制动风险解耦消融（真实实验数据版）
#
# 核心思路：
# 1) 直接读取真实 exp3_g1.4_timeseries.csv；
# 2) 自动提取第一段“完整穿越”（排除启动阶段的不完整半程）；
# 3) D1 直接使用真实记录的 R_b / R_g；
# 4) D2 在完全相同的 v_risk、d_min、几何状态下，仅恢复 legacy d0；
# 5) 因而该图隔离的是“静态安全距离重复进入动态制动需求”的机制影响，
#    这正对应本文“动态风险—静态几何裕度解耦”的创新点。
#
# 注意：
# 当前 D2 是“同状态公式重算消融”，不是另一条独立 D2 飞行日志。
# 如果以后有真正独立 D2 CSV，应优先改为两套真实日志直接比较。
# ============================================================


# ------------------------ 真实数据路径 ------------------------
DATA_DIR = Path(r"F:\无人机暑假开发\看实验数据\data")
DATA_FILE = DATA_DIR / "exp3_g1.4_timeseries.csv"


# ------------------------ 模型参数 ----------------------------
TAU = 0.3
A_B = 2.0
EPS = 0.1

# legacy 模型恢复的静态安全距离项。
# 这里严格按论文机理仿真中使用的 d0=0.30 m，
# 不为了匹配论文旧数字而调参。
LEGACY_D0 = 0.30

# 综合风险中制动风险的权重
W_B = 0.55


# ------------------------ 核心通道窗口 ------------------------
# 仅用于附加分析，不用于“挑数据”替代完整穿越统计。
# 程序会同时输出：
#   A. 第一段完整穿越的统计（主统计）
#   B. 核心通道窗口 [-5, 8] m 的统计（机制增强区）
CORE_X_MIN = -5.0
CORE_X_MAX = 8.0


def braking_risk(v_risk, d_min, d0=0.0):
    """
    动态制动风险：
        d_req = d0 + v*tau + v^2/(2*a_b)
        R_b   = clip(d_req / (d_min + eps), 0, 1)

    D1: d0 = 0
    D2: d0 = LEGACY_D0
    """
    d_req = d0 + v_risk * TAU + v_risk**2 / (2.0 * A_B)
    return np.clip(d_req / (d_min + EPS), 0.0, 1.0)


def check_required_columns(df):
    required = {
        "t_s", "x", "v_risk", "d_min",
        "R_b", "R_g"
    }
    missing = sorted(required - set(df.columns))
    if missing:
        raise ValueError(f"CSV 缺少必要字段：{missing}")


def detect_monotonic_runs(df, x_smooth_window=101, dx_threshold=1e-4):
    """
    根据平滑后的 x(t) 自动识别正向/反向穿越段。
    返回 [(start_idx, end_idx, direction), ...]
    direction: +1 正向，-1 反向
    """
    d = df.sort_values("t_s").reset_index(drop=True)

    x_s = (
        d["x"]
        .rolling(x_smooth_window, center=True, min_periods=1)
        .mean()
        .to_numpy()
    )

    dx = np.diff(x_s)

    sign = np.where(
        dx > dx_threshold, 1,
        np.where(dx < -dx_threshold, -1, 0)
    )

    nz = np.where(sign != 0)[0]
    if len(nz) == 0:
        return [(0, len(d) - 1, 1)]

    # 填补开头和局部零速度区间
    sign[:nz[0]] = sign[nz[0]]
    last = sign[nz[0]]

    for i in range(nz[0] + 1, len(sign)):
        if sign[i] == 0:
            sign[i] = last
        else:
            last = sign[i]

    runs = []
    start = 0
    current = sign[0]

    for i in range(1, len(sign)):
        if sign[i] != current:
            runs.append((start, i, int(current)))
            start = i
            current = sign[i]

    runs.append((start, len(d) - 1, int(current)))
    return runs


def select_first_complete_pass(df, min_span_m=25.0):
    """
    自动选择第一段完整穿越。

    你的 exp3_g1.4_timeseries.csv 起始于一次已经进行中的半程，
    因此不能直接拿第一个单调段做主统计。

    这里定义“完整穿越”为 x 跨度 >= min_span_m 的第一段单调轨迹。
    对当前真实数据，该规则选中约 15.034 s ~ 39.799 s，
    持续约 24.765 s，是第一段完整往返中的单程。
    """
    d = df.sort_values("t_s").reset_index(drop=True)
    runs = detect_monotonic_runs(d)

    candidates = []

    for start, end, direction in runs:
        seg = d.iloc[start:end + 1]
        span = float(seg["x"].max() - seg["x"].min())

        if span >= min_span_m:
            candidates.append((start, end, direction, span))

    if not candidates:
        raise RuntimeError(
            "没有找到跨度足够大的完整穿越段。"
            "请检查 min_span_m 或原始数据。"
        )

    start, end, direction, span = candidates[0]

    seg = d.iloc[start:end + 1].copy().reset_index(drop=True)

    print("\n[自动选择完整穿越]")
    print(f"direction = {'正向' if direction > 0 else '反向'}")
    print(f"t = {seg['t_s'].iloc[0]:.3f} ~ {seg['t_s'].iloc[-1]:.3f} s")
    print(f"duration = {seg['t_s'].iloc[-1] - seg['t_s'].iloc[0]:.3f} s")
    print(f"x = {seg['x'].min():.4f} ~ {seg['x'].max():.4f} m")
    print(f"span = {span:.3f} m")
    print(f"N = {len(seg)}")

    return seg


def reconstruct_same_state_ablation(df):
    """
    在完全相同的运动/几何状态下构造 D2。

    D1：
        使用 CSV 真实记录 R_b、R_g。

    D2：
        保留相同 v_risk、d_min 和其余风险项，
        仅在制动需求中恢复 LEGACY_D0。

    综合风险：
        Rg_D2 = Rg_D1 + w_b * (Rb_D2 - Rb_D1)

    这样可以把变化严格归因于“静态 d0 是否重复进入动态制动风险通道”。
    """
    out = df.copy()

    v = out["v_risk"].to_numpy()
    dmin = out["d_min"].to_numpy()

    rb_recorded = out["R_b"].to_numpy()

    # 用公式重算 D1，只做一致性核验
    rb_formula_d1 = braking_risk(v, dmin, d0=0.0)
    abs_err = np.abs(rb_formula_d1 - rb_recorded)

    print("\n[D1 公式一致性检查]")
    print(f"mean absolute error = {abs_err.mean():.8f}")
    print(f"P99 absolute error  = {np.quantile(abs_err, 0.99):.8f}")
    print(f"|error| > 1e-3 数量 = {(abs_err > 1e-3).sum()} / {len(abs_err)}")

    # D1 使用真实记录值
    out["Rb_D1"] = rb_recorded
    out["Rg_D1"] = out["R_g"].to_numpy()

    # D2 同状态恢复 d0
    out["Rb_D2"] = braking_risk(v, dmin, d0=LEGACY_D0)

    out["Rg_D2"] = np.clip(
        out["Rg_D1"].to_numpy()
        + W_B * (out["Rb_D2"].to_numpy() - out["Rb_D1"].to_numpy()),
        0.0,
        1.0
    )

    return out


def calculate_stats(df):
    rb1 = df["Rb_D1"].to_numpy()
    rb2 = df["Rb_D2"].to_numpy()
    rg1 = df["Rg_D1"].to_numpy()
    rg2 = df["Rg_D2"].to_numpy()

    return {
        "N": len(df),

        "mean_Rb_D1": float(np.mean(rb1)),
        "mean_Rb_D2": float(np.mean(rb2)),

        "max_Rb_D1": float(np.max(rb1)),
        "max_Rb_D2": float(np.max(rb2)),

        "max_Rg_D1": float(np.max(rg1)),
        "max_Rg_D2": float(np.max(rg2)),

        # 风险饱和比例是非常适合体现“风险分辨率被压缩”的指标
        "sat_Rb_D1": float(np.mean(rb1 >= 0.999)),
        "sat_Rb_D2": float(np.mean(rb2 >= 0.999)),
    }


def print_stats(title, stats):
    print("\n" + "=" * 68)
    print(title)
    print("=" * 68)

    print(
        f"mean R_b : "
        f"{stats['mean_Rb_D1']:.4f} -> {stats['mean_Rb_D2']:.4f}"
    )

    print(
        f"max  R_b : "
        f"{stats['max_Rb_D1']:.4f} -> {stats['max_Rb_D2']:.4f}"
    )

    print(
        f"max  R_g : "
        f"{stats['max_Rg_D1']:.4f} -> {stats['max_Rg_D2']:.4f}"
    )

    print(
        f"R_b saturation ratio : "
        f"{100 * stats['sat_Rb_D1']:.3f}% -> "
        f"{100 * stats['sat_Rb_D2']:.3f}%"
    )

    if stats["mean_Rb_D1"] > 0:
        mean_inc = (
            stats["mean_Rb_D2"] / stats["mean_Rb_D1"] - 1.0
        ) * 100.0

        print(f"mean R_b increase = {mean_inc:.2f}%")

    if stats["max_Rg_D1"] > 0:
        rg_inc = (
            stats["max_Rg_D2"] / stats["max_Rg_D1"] - 1.0
        ) * 100.0

        print(f"max R_g increase = {rg_inc:.2f}%")


def make_position_profile(df, bin_width=0.05, smooth_bins=5):
    """
    把同一完整穿越按 x 分箱取中位数，
    避免原始高频采样导致折线过密。
    """
    d = df[
        ["x", "Rb_D1", "Rb_D2"]
    ].dropna().copy()

    xmin = float(d["x"].min())
    xmax = float(d["x"].max())

    bins = np.arange(
        xmin,
        xmax + bin_width,
        bin_width
    )

    d["x_bin"] = pd.cut(
        d["x"],
        bins=bins,
        include_lowest=True
    )

    profile = (
        d.groupby("x_bin", observed=False)[["Rb_D1", "Rb_D2"]]
        .median()
    )

    profile["x"] = [
        (interval.left + interval.right) / 2.0
        for interval in profile.index
    ]

    profile = profile.dropna().reset_index(drop=True)

    for col in ["Rb_D1", "Rb_D2"]:
        profile[col] = (
            profile[col]
            .rolling(
                smooth_bins,
                center=True,
                min_periods=1
            )
            .mean()
        )

    return profile


def write_report(outdir, full_stats, core_stats, selected_pass):
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    report = outdir / "fig07_real_data_analysis.txt"

    duration = (
        selected_pass["t_s"].iloc[-1]
        - selected_pass["t_s"].iloc[0]
    )

    lines = [
        "图7 制动风险解耦消融：真实数据分析",
        "=" * 60,
        "",
        f"数据文件: {DATA_FILE}",
        f"legacy d0 = {LEGACY_D0:.3f} m",
        f"selected pass duration = {duration:.3f} s",
        f"selected pass x range = "
        f"[{selected_pass['x'].min():.4f}, {selected_pass['x'].max():.4f}] m",
        "",
        "A. 第一段完整穿越（主统计）",
        f"mean Rb: {full_stats['mean_Rb_D1']:.4f} -> "
        f"{full_stats['mean_Rb_D2']:.4f}",
        f"max  Rb: {full_stats['max_Rb_D1']:.4f} -> "
        f"{full_stats['max_Rb_D2']:.4f}",
        f"max  Rg: {full_stats['max_Rg_D1']:.4f} -> "
        f"{full_stats['max_Rg_D2']:.4f}",
        f"Rb saturation: "
        f"{100*full_stats['sat_Rb_D1']:.3f}% -> "
        f"{100*full_stats['sat_Rb_D2']:.3f}%",
        "",
        f"B. 核心通道窗口 [{CORE_X_MIN}, {CORE_X_MAX}] m（附加机制统计）",
        f"mean Rb: {core_stats['mean_Rb_D1']:.4f} -> "
        f"{core_stats['mean_Rb_D2']:.4f}",
        f"max  Rb: {core_stats['max_Rb_D1']:.4f} -> "
        f"{core_stats['max_Rb_D2']:.4f}",
        f"max  Rg: {core_stats['max_Rg_D1']:.4f} -> "
        f"{core_stats['max_Rg_D2']:.4f}",
        f"Rb saturation: "
        f"{100*core_stats['sat_Rb_D1']:.3f}% -> "
        f"{100*core_stats['sat_Rb_D2']:.3f}%",
        "",
        "解释：",
        "D2 与 D1 使用完全相同的 v_risk、d_min 和几何状态，",
        "仅恢复静态 d0。因此 D2 风险整体抬升以及 Rb=1 饱和比例增加，",
        "可直接归因于静态安全距离被重复计入动态制动需求。",
        "这一结果支持本文的创新点：动态制动风险应与静态几何安全裕度解耦，",
        "否则固定风险偏置会压缩动态风险通道的有效分辨率。",
        "",
        "注意：当前 D2 为同状态公式重算消融，不应表述为独立 D2 飞行实验。",
    ]

    report.write_text("\n".join(lines), encoding="utf-8")
    return report


def make_figure(outdir="outputs", save_pdf=False):
    setup_style()

    if not DATA_FILE.exists():
        raise FileNotFoundError(
            f"找不到真实实验数据：\n{DATA_FILE}"
        )

    raw = pd.read_csv(DATA_FILE)
    check_required_columns(raw)

    # 1. 自动选第一段完整穿越
    selected = select_first_complete_pass(raw)

    # 2. 同状态风险消融
    ablation = reconstruct_same_state_ablation(selected)

    # 3. 主统计：整个完整穿越
    full_stats = calculate_stats(ablation)

    # 4. 附加统计：核心通道区域
    core = ablation[
        (ablation["x"] >= CORE_X_MIN)
        & (ablation["x"] <= CORE_X_MAX)
    ].copy()

    core_stats = calculate_stats(core)

    print_stats("A. 第一段完整穿越（主统计）", full_stats)
    print_stats(
        f"B. 核心通道窗口 [{CORE_X_MIN}, {CORE_X_MAX}] m",
        core_stats
    )

    # 5. 位置剖面
    profile = make_position_profile(ablation)

    # ========================================================
    # 绘图
    # ========================================================
    fig, axes = plt.subplots(1, 2, figsize=(10.2, 4.0))

    # ---------------- (a) 位置剖面 ----------------
    axes[0].plot(
        profile["x"],
        profile["Rb_D1"],
        lw=1.8,
        label="D1: decoupled"
    )

    axes[0].plot(
        profile["x"],
        profile["Rb_D2"],
        lw=1.8,
        ls="--",
        label=r"D2: + legacy $d_0$"
    )

    axes[0].axhline(
        1.0,
        ls=":",
        lw=0.8,
        alpha=0.5
    )

    axes[0].set_xlabel(r"Longitudinal position $x$ (m)")
    axes[0].set_ylabel(r"Braking risk $R_b$")
    axes[0].set_ylim(0.0, 1.05)
    axes[0].set_title("(a) Braking risk position profile", loc="left", weight="bold")
    axes[0].legend(frameon=False)

    # ---------------- (b) 主统计柱状图 ----------------
    labels = [
        r"mean $R_b$",
        r"max $R_b$",
        r"max $R_g$"
    ]

    d1_values = [
        full_stats["mean_Rb_D1"],
        full_stats["max_Rb_D1"],
        full_stats["max_Rg_D1"],
    ]

    d2_values = [
        full_stats["mean_Rb_D2"],
        full_stats["max_Rb_D2"],
        full_stats["max_Rg_D2"],
    ]

    x = np.arange(len(labels))
    width = 0.34

    bars1 = axes[1].bar(
        x - width / 2,
        d1_values,
        width,
        label="D1"
    )

    bars2 = axes[1].bar(
        x + width / 2,
        d2_values,
        width,
        label="D2",
        hatch="//",
        fill=False
    )

    axes[1].set_xticks(x, labels)
    axes[1].set_ylabel("Risk value")
    axes[1].set_ylim(0.0, 1.08)
    axes[1].set_title("(b) Complete-pass risk statistics", loc="left", weight="bold")
    axes[1].legend(frameon=False)

    # 柱顶标数值
    for bars in (bars1, bars2):
        for bar in bars:
            h = bar.get_height()
            axes[1].text(
                bar.get_x() + bar.get_width() / 2,
                h + 0.018,
                f"{h:.3f}",
                ha="center",
                va="bottom",
                fontsize=8
            )

    # ---------------- 统一样式 ----------------
    for ax in axes:
        ax.grid(False)
        for spine in ax.spines.values():
            spine.set_linewidth(1.1)

    fig.suptitle("")
    fig.tight_layout()

    # 输出分析报告
    report = write_report(
        outdir,
        full_stats,
        core_stats,
        selected
    )

    print(f"\n分析报告已保存：{report}")

    return save_figure(
        fig,
        outdir,
        "fig07_braking_ablation_real_data",
        save_pdf
    )


if __name__ == "__main__":
    make_figure(
        Path(__file__).parent / "outputs"
    )