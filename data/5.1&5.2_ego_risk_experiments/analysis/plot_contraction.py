from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from paper_style import setup_style, save_figure


# ============================================================
# 路径
# ============================================================

DATA_DIR = Path(r"F:\无人机暑假开发\看实验数据\data")


# ============================================================
# 图 5(a) 参数
# ============================================================

# 用于识别无人机运动方向时，对 x 进行轻微平滑
DIRECTION_SMOOTH_WINDOW = 101

# 判断 x 增大/减小的阈值
DX_THRESHOLD = 1e-4

# 只有横跨足够大空间范围的正向段才认为是“完整正向穿越”
COMPLETE_X_MIN = -10.0
COMPLETE_X_MAX = 10.0

# 最终图中显示的空间范围
PROFILE_X_MIN = -8.0
PROFILE_X_MAX = 2.6

# 按位置分箱，单位 m
BIN_WIDTH = 0.05

# 对位置剖面进行轻微平滑
SMOOTH_BINS = 5

# 论文中报告的提前位置
LEAD_POSITION = -3.196


# ============================================================
# 基础函数
# ============================================================

def rolling_mean(values, window):
    """
    一维滚动平均。
    """
    return (
        pd.Series(values)
        .rolling(
            window=window,
            center=True,
            min_periods=1
        )
        .mean()
        .to_numpy()
    )


# ============================================================
# 自动识别正向/反向穿越
# ============================================================

def detect_motion_runs(
    df,
    x_col="x",
    t_col="t_s",
    smooth_window=DIRECTION_SMOOTH_WINDOW,
    dx_threshold=DX_THRESHOLD
):
    """
    根据 x 随时间的变化方向，
    将整份 timeseries 自动拆分成正向/反向运动段。

    direction:
        +1 = x 增大，正向
        -1 = x 减小，反向
    """

    d = (
        df
        .sort_values(t_col)
        .reset_index(drop=True)
        .copy()
    )

    # 对 x 做轻微平滑，仅用于判断运动方向
    x_smooth = rolling_mean(
        d[x_col].to_numpy(),
        smooth_window
    )

    dx = np.diff(x_smooth)

    direction = np.where(
        dx > dx_threshold,
        1,
        np.where(
            dx < -dx_threshold,
            -1,
            0
        )
    )

    nonzero = np.where(direction != 0)[0]

    if len(nonzero) == 0:
        raise RuntimeError(
            "无法根据 x 数据识别无人机运动方向。"
        )

    # 开头的 0 用第一个有效方向补齐
    first = nonzero[0]
    direction[:first] = direction[first]

    # 中间的 0 延续前一个方向
    last_direction = direction[first]

    for i in range(first + 1, len(direction)):
        if direction[i] == 0:
            direction[i] = last_direction
        else:
            last_direction = direction[i]

    # 拆分连续运动段
    runs = []

    start = 0
    current_direction = direction[0]

    for i in range(1, len(direction)):
        if direction[i] != current_direction:
            runs.append(
                (start, i, current_direction)
            )

            start = i
            current_direction = direction[i]

    runs.append(
        (
            start,
            len(direction),
            current_direction
        )
    )

    return d, runs


# ============================================================
# 只选择完整正向穿越
# ============================================================

def extract_complete_forward_passes(df):
    """
    从整个实验 timeseries 中，
    自动挑选完整的正向穿越。

    要求：
        起点到达 x < -10 m
        终点到达 x > +10 m

    这样会排除第一段从 -3.196 m 才开始记录的不完整穿越。
    """

    d, runs = detect_motion_runs(df)

    forward_passes = []

    print("\n==============================")
    print("识别到的运动段")
    print("==============================")

    for run_id, (i0, i1, direction) in enumerate(runs):

        seg = (
            d.iloc[i0:i1 + 1]
            .copy()
            .reset_index(drop=True)
        )

        direction_name = (
            "正向"
            if direction == 1
            else "反向"
        )

        print(
            f"段 {run_id}: "
            f"{direction_name}, "
            f"t={seg['t_s'].iloc[0]:.3f}"
            f"~{seg['t_s'].iloc[-1]:.3f} s, "
            f"x={seg['x'].iloc[0]:.3f}"
            f"~{seg['x'].iloc[-1]:.3f} m, "
            f"max Rc={seg['R_c'].max():.4f}"
        )

        if direction != 1:
            continue

        # 判断是否为完整正向穿越
        if (
            seg["x"].min() <= COMPLETE_X_MIN
            and
            seg["x"].max() >= COMPLETE_X_MAX
        ):
            forward_passes.append(seg)

    if len(forward_passes) == 0:
        raise RuntimeError(
            "没有找到完整正向穿越数据。"
        )

    print("\n==============================")
    print(
        f"最终用于平均的完整正向穿越数量："
        f"{len(forward_passes)}"
    )
    print("==============================")

    for i, seg in enumerate(forward_passes, start=1):

        print(
            f"正向穿越 {i}: "
            f"t={seg['t_s'].iloc[0]:.3f}"
            f"~{seg['t_s'].iloc[-1]:.3f} s, "
            f"x={seg['x'].min():.3f}"
            f"~{seg['x'].max():.3f} m, "
            f"max Rc={seg['R_c'].max():.4f}"
        )

    return forward_passes


# ============================================================
# 单次实验：按 x 生成位置剖面
# ============================================================

def build_single_pass_profile(
    df,
    x_min=PROFILE_X_MIN,
    x_max=PROFILE_X_MAX,
    bin_width=BIN_WIDTH,
    smooth_bins=SMOOTH_BINS
):
    """
    将一整次正向穿越按纵向位置 x 分箱。

    对每个位置区间计算：
        R_c
        C_global
        G_c

    使用中位数降低异常点影响。
    """

    cols = [
        "R_c",
        "C_global",
        "G_c"
    ]

    d = df[
        (df["x"] >= x_min)
        &
        (df["x"] <= x_max)
    ].copy()

    bins = np.arange(
        x_min,
        x_max + bin_width,
        bin_width
    )

    centers = (
        bins[:-1] + bins[1:]
    ) / 2.0

    d["x_bin"] = pd.cut(
        d["x"],
        bins=bins,
        labels=False,
        include_lowest=True
    )

    grouped = (
        d.groupby(
            "x_bin",
            observed=True
        )[cols]
        .median()
    )

    profile = pd.DataFrame({
        "x": centers
    })

    for col in cols:

        values = np.full(
            len(centers),
            np.nan
        )

        indices = (
            grouped.index
            .to_numpy(dtype=int)
        )

        values[indices] = (
            grouped[col]
            .to_numpy()
        )

        # 少量缺失区间用插值补齐
        values = (
            pd.Series(values)
            .interpolate(
                limit_direction="both"
            )
            .to_numpy()
        )

        # 对空间剖面做轻微平滑
        values = rolling_mean(
            values,
            smooth_bins
        )

        profile[col] = values

    return profile


# ============================================================
# 多次完整正向实验平均
# ============================================================

def build_mean_forward_profile(
    forward_passes
):
    """
    将多次完整正向穿越按同一 x 坐标对齐。

    输出：
        mean
        std
    """

    profiles = [
        build_single_pass_profile(seg)
        for seg in forward_passes
    ]

    result = pd.DataFrame({
        "x": profiles[0]["x"]
    })

    for col in [
        "R_c",
        "C_global",
        "G_c"
    ]:

        stacked = np.vstack([
            p[col].to_numpy()
            for p in profiles
        ])

        result[col] = np.mean(
            stacked,
            axis=0
        )

        if len(profiles) > 1:

            result[col + "_std"] = np.std(
                stacked,
                axis=0,
                ddof=1
            )

        else:

            result[col + "_std"] = 0.0

    return result, profiles


# ============================================================
# 图 5(b)：真实通道宽度统计
# ============================================================

def load_gap_stats(data_dir):
    """
    优先读取真实 exp3 数据。

    支持两种常见命名：

        exp3_g1.4_timeseries.csv
        exp3_g1.2_timeseries.csv
        exp3_g1.0_timeseries.csv

    或：

        exp3_g14_timeseries.csv
        exp3_g12_timeseries.csv
        exp3_g10_timeseries.csv

    如果找不到，则读取：
        fig05_gap_stats.csv
    """

    file_candidates = {

        1.4: [
            "exp3_g1.4_timeseries.csv",
            "exp3_g14_timeseries.csv"
        ],

        1.2: [
            "exp3_g1.2_timeseries.csv",
            "exp3_g12_timeseries.csv"
        ],

        1.0: [
            "exp3_g1.0_timeseries.csv",
            "exp3_g10_timeseries.csv"
        ]
    }

    rows = []

    for gap, candidates in file_candidates.items():

        selected_file = None

        for name in candidates:

            candidate = (
                data_dir / name
            )

            if candidate.exists():
                selected_file = candidate
                break

        if selected_file is None:
            continue

        df = pd.read_csv(
            selected_file
        )

        if "R_c" not in df.columns:
            raise ValueError(
                f"{selected_file.name} "
                "中没有 R_c 列"
            )

        rows.append({
            "gap": gap,
            "max_Rc": df["R_c"].max(),
            "mean_Rc": df["R_c"].mean()
        })

        print(
            f"读取图5(b)："
            f"{selected_file.name}"
        )

    # 三种宽度都成功找到
    if len(rows) == 3:

        stats = pd.DataFrame(rows)

        return (
            stats
            .sort_values(
                "gap",
                ascending=False
            )
            .reset_index(drop=True)
        )

    # -------------------------
    # 回退到已有统计 CSV
    # -------------------------

    fallback = (
        data_dir
        /
        "fig05_gap_stats.csv"
    )

    if fallback.exists():

        print(
            "没有找到完整的三个 exp3 原始文件，"
            "使用 fig05_gap_stats.csv"
        )

        return pd.read_csv(
            fallback
        )

    raise FileNotFoundError(
        "没有找到完整的 exp3 通道宽度实验数据，"
        "同时也没有 fig05_gap_stats.csv。"
    )


# ============================================================
# 主绘图函数
# ============================================================

def make_figure(
    outdir="outputs",
    save_pdf=False
):

    setup_style()

    # ========================================================
    # 读取真实漏斗实验
    # ========================================================

    funnel_file = (
        DATA_DIR
        /
        "exp1_funnel_timeseries.csv"
    )

    if not funnel_file.exists():

        raise FileNotFoundError(
            f"找不到真实实验数据："
            f"{funnel_file}"
        )

    raw = pd.read_csv(
        funnel_file
    )

    required_columns = [
        "t_s",
        "x",
        "R_c",
        "C_global",
        "G_c"
    ]

    for col in required_columns:

        if col not in raw.columns:

            raise ValueError(
                f"真实 CSV 缺少列：{col}"
            )

    # ========================================================
    # 自动提取完整正向穿越
    # ========================================================

    forward_passes = (
        extract_complete_forward_passes(
            raw
        )
    )

    # ========================================================
    # 多次穿越按 x 对齐并取平均
    # ========================================================

    profile, individual_profiles = (
        build_mean_forward_profile(
            forward_passes
        )
    )

    print("\n==============================")
    print("平均剖面结果")
    print("==============================")

    print(
        "平均曲线 max Rc = "
        f"{profile['R_c'].max():.4f}"
    )

    print(
        "原始完整正向穿越 max Rc = "
        f"{max(seg['R_c'].max() for seg in forward_passes):.4f}"
    )

    # 查看 x=-3.196 附近数据
    lead_idx = (
        profile["x"] - LEAD_POSITION
    ).abs().idxmin()

    print(
        f"x≈{profile.loc[lead_idx, 'x']:.3f} m:"
    )

    print(
        f"Rc="
        f"{profile.loc[lead_idx, 'R_c']:.4f}"
        f" ± "
        f"{profile.loc[lead_idx, 'R_c_std']:.4f}"
    )

    # ========================================================
    # 图 5(b)
    # ========================================================

    gap_stats = load_gap_stats(
        DATA_DIR
    )

    # ========================================================
    # 创建画布
    # ========================================================

    fig, axes = plt.subplots(
        1,
        2,
        figsize=(10.5, 4.3)
    )

    ax0, ax1 = axes

    # ========================================================
    # 图 5(a)
    # ========================================================

    line_info = [
        (
            "R_c",
            "R_c_std",
            r"$R_c$",
            "-",
            "C0"
        ),
        (
            "C_global",
            "C_global_std",
            r"$C_{global}$",
            "--",
            "C1"
        ),
        (
            "G_c",
            "G_c_std",
            r"$G_c$",
            ":",
            "C2"
        )
    ]

    for (
        col,
        std_col,
        label,
        linestyle,
        color
    ) in line_info:

        mean_y = (
            profile[col]
            .to_numpy()
        )

        std_y = (
            profile[std_col]
            .to_numpy()
        )

        lower = np.clip(
            mean_y - std_y,
            0,
            None
        )

        upper = (
            mean_y + std_y
        )

        # ±1 标准差阴影
        ax0.fill_between(
            profile["x"],
            lower,
            upper,
            color=color,
            alpha=0.10,
            linewidth=0
        )

        # 平均曲线
        ax0.plot(
            profile["x"],
            mean_y,
            linestyle=linestyle,
            color=color,
            lw=2.0,
            label=label
        )

    # --------------------------------------------------------
    # 提前位置参考线
    # --------------------------------------------------------

    ax0.axvline(
        LEAD_POSITION,
        color="C0",
        linestyle=":",
        linewidth=1.3
    )

    # --------------------------------------------------------
    # 喉部 x = 0
    # --------------------------------------------------------

    ax0.axvline(
        0,
        color="0.35",
        linestyle="--",
        linewidth=1.0
    )

    # --------------------------------------------------------
    # 提前距离文字
    # --------------------------------------------------------

    ax0.annotate(
        r"$L_{lead}=3.196\ \mathrm{m}$",
        xy=(
            LEAD_POSITION,
            0.41
        ),
        xytext=(
            -2.8,
            0.425
        ),
        fontsize=9
    )

    # --------------------------------------------------------
    # 坐标设置
    # --------------------------------------------------------

    ax0.set_xlim(
        PROFILE_X_MIN,
        PROFILE_X_MAX
    )

    max_y = max(
        profile["R_c"].max(),
        profile["C_global"].max(),
        profile["G_c"].max()
    )

    ax0.set_ylim(
        0,
        max(
            0.46,
            max_y * 1.05
        )
    )

    ax0.set_xlabel(
        r"Longitudinal position $x$ (m)"
    )

    ax0.set_ylabel(
        "Contraction risk / contraction component"
    )

    ax0.set_title(
        "(a) Forward contraction risk",
        loc="left",
        weight="bold"
    )

    ax0.legend(
        frameon=False,
        loc="upper right"
    )

    ax0.grid(False)

    # ========================================================
    # 图 5(b)
    # ========================================================

    ax1.plot(
        gap_stats["gap"],
        gap_stats["max_Rc"],
        marker="o",
        lw=1.8,
        label=r"max $R_c$"
    )

    ax1.plot(
        gap_stats["gap"],
        gap_stats["mean_Rc"],
        marker="s",
        linestyle="--",
        lw=1.8,
        label=r"mean $R_c$"
    )

    # 最大值数字
    for x, y in zip(
        gap_stats["gap"],
        gap_stats["max_Rc"]
    ):

        ax1.text(
            x,
            y + 0.012,
            f"{y:.3f}",
            ha="center",
            fontsize=8
        )

    ax1.set_xlabel(
        "Channel width (m)"
    )

    ax1.set_ylabel(
        r"$R_c$"
    )

    ax1.set_title(
        "(b) Contraction risk at different channel widths",
        loc="left",
        weight="bold"
    )

    # 宽通道在左，窄通道在右
    ax1.set_xlim(
        max(gap_stats["gap"]) + 0.03,
        min(gap_stats["gap"]) - 0.03
    )

    ax1.set_ylim(
        0.05,
        max(
            0.61,
            gap_stats["max_Rc"].max()
            + 0.05
        )
    )

    ax1.legend(
        frameon=False
    )

    ax1.grid(False)

    # ========================================================
    # 统一边框
    # ========================================================

    for ax in axes:

        for spine in ax.spines.values():

            spine.set_linewidth(
                1.1
            )

    # ========================================================
    # 总标题
    # ========================================================

    fig.suptitle(
        "Fig. 5  Forward contraction risk experimental results"
    )

    fig.tight_layout()

    # ========================================================
    # 保存
    # ========================================================

    return save_figure(
        fig,
        outdir,
        "fig05_contraction_experiment",
        save_pdf
    )


# ============================================================
# 运行
# ============================================================

if __name__ == "__main__":

    make_figure(
        Path(__file__).parent
        /
        "outputs"
    )