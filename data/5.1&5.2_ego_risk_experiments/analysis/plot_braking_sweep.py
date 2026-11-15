from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ============================================================
# Real experiment data directory
# ============================================================
DATA_DIR = Path(r"F:\无人机暑假开发\看实验数据\data")
OUT_DIR = Path(__file__).parent / "outputs"
OUT_DIR.mkdir(parents=True, exist_ok=True)

FILES = {
    0.5: DATA_DIR / "exp2_v0.5_timeseries.csv",
    1.0: DATA_DIR / "exp2_v1.0_timeseries.csv",
    1.5: DATA_DIR / "exp2_v1.5_timeseries.csv",
    2.0: DATA_DIR / "exp2_v2.0_timeseries.csv",
    2.5: DATA_DIR / "exp2_v2.5_timeseries.csv",
}

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "DejaVu Sans"],
    "axes.unicode_minus": False,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.linewidth": 1.0,
    "xtick.direction": "out",
    "ytick.direction": "out",
    "savefig.dpi": 600,
})

rng = np.random.default_rng(20260827)

def load_data():
    data = {}
    rows = []
    for speed, path in FILES.items():
        if not path.exists():
            raise FileNotFoundError(f"找不到数据文件：{path}")
        df = pd.read_csv(path)
        data[speed] = df
        rows.append({
            "speed": speed,
            "mean_Rb": df["R_b"].mean(),
            "median_Rb": df["R_b"].median(),
            "p95_Rb": df["R_b"].quantile(0.95),
            "max_Rb": df["R_b"].max(),
            "saturation_pct": 100 * (df["R_b"] >= 0.9999).mean(),
            "min_dmin": df["d_min"].min(),
            "p05_dmin": df["d_min"].quantile(0.05),
            "median_dmin": df["d_min"].median(),
            "n": len(df),
        })
    summary = pd.DataFrame(rows).sort_values("speed")
    return data, summary

def plot_risk_distribution(data, summary):
    speeds = summary["speed"].to_numpy()
    samples = []

    for s in speeds:
        arr = data[s]["R_b"].dropna().to_numpy()
        n_sample = min(12000, len(arr))
        idx = rng.choice(len(arr), size=n_sample, replace=False)
        samples.append(arr[idx])

    fig, ax = plt.subplots(figsize=(7.4, 5.0))

    parts = ax.violinplot(
        samples,
        positions=speeds,
        widths=0.28,
        showmeans=False,
        showmedians=True,
        showextrema=False
    )
    for body in parts["bodies"]:
        body.set_alpha(0.24)

    ax.plot(
        speeds, summary["mean_Rb"],
        marker="o", linewidth=1.8,
        label=r"mean $R_b$"
    )
    ax.plot(
        speeds, summary["p95_Rb"],
        marker="D", linestyle="--", linewidth=1.5,
        label=r"P95 $R_b$"
    )

    ax.axhline(1.0, linestyle=":", linewidth=1.0)

    for _, r in summary.iterrows():
        if r["saturation_pct"] > 0:
            ax.annotate(
                f"{r['saturation_pct']:.1f}% saturated",
                (r["speed"], 0.965),
                xytext=(0, -22),
                textcoords="offset points",
                ha="center",
                fontsize=8
            )

    ax.set_xlim(0.28, 2.72)
    ax.set_ylim(0, 1.07)
    ax.set_xticks(speeds)
    ax.set_xlabel(r"Speed setting (m s$^{-1}$)")
    ax.set_ylabel(r"Dynamic braking risk $R_b$")
    ax.set_title("Dynamic braking risk distribution at different speed settings", weight="bold")
    ax.legend(frameon=False, loc="upper left")
    ax.grid(axis="y", alpha=0.18)

    fig.tight_layout()
    fig.savefig(OUT_DIR / "fig06_advanced_risk_distribution.png", bbox_inches="tight")
    plt.close(fig)

def plot_same_clearance(data, summary):
    speeds = summary["speed"].to_numpy()

    d_lo, d_hi = 0.80, 1.20
    bin_width = 0.02
    bins = np.arange(d_lo, d_hi + bin_width, bin_width)
    centers = (bins[:-1] + bins[1:]) / 2

    fig, ax = plt.subplots(figsize=(7.4, 5.0))

    for s in speeds:
        df = data[s][["d_min", "R_b"]].dropna()
        df = df[
            (df["d_min"] >= d_lo) &
            (df["d_min"] <= d_hi)
        ].copy()

        df["bin"] = pd.cut(
            df["d_min"],
            bins=bins,
            labels=False,
            include_lowest=True
        )

        g = df.groupby("bin", observed=True)["R_b"].agg(
            median="median",
            q25=lambda x: x.quantile(0.25),
            q75=lambda x: x.quantile(0.75),
            n="size"
        )

        xvals, med, q25, q75 = [], [], [], []

        for idx, row in g.iterrows():
            idx = int(idx)
            if row["n"] < 50:
                continue
            xvals.append(centers[idx])
            med.append(row["median"])
            q25.append(row["q25"])
            q75.append(row["q75"])

        xvals = np.asarray(xvals)
        med = np.asarray(med)
        q25 = np.asarray(q25)
        q75 = np.asarray(q75)

        ax.plot(
            xvals, med,
            linewidth=1.8,
            label=fr"{s:.1f} m s$^{{-1}}$"
        )
        ax.fill_between(
            xvals, q25, q75,
            alpha=0.08
        )

    ax.axhline(1.0, linestyle=":", linewidth=1.0)
    ax.set_xlim(d_lo, d_hi)
    ax.set_ylim(0, 1.04)
    ax.set_xlabel(r"Nearest obstacle distance $d_{\min}$ (m)")
    ax.set_ylabel(r"Median dynamic braking risk $R_b$")
    ax.set_title("Dynamic risk separation at the same geometric clearance", weight="bold")
    ax.legend(frameon=False, ncol=2, title="Speed setting")
    ax.grid(axis="both", alpha=0.16)

    fig.tight_layout()
    fig.savefig(OUT_DIR / "fig06_advanced_same_clearance.png", bbox_inches="tight")
    plt.close(fig)

if __name__ == "__main__":
    data, summary = load_data()

    print(summary.to_string(index=False))
    summary.to_csv(
        OUT_DIR / "fig06_speed_sweep_summary.csv",
        index=False,
        encoding="utf-8-sig"
    )

    plot_risk_distribution(data, summary)
    plot_same_clearance(data, summary)

    print(f"\n图片已保存到：{OUT_DIR}")