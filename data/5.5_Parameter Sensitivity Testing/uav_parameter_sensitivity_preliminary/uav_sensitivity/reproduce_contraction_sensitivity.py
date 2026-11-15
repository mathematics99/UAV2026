#!/usr/bin/env python3
"""Replay contraction fusion and the obstacle-weight gate on a recorded flight."""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parent
DATA = ROOT / "data" / "risk_run_20260904_212411_timeseries.csv"
FIGURES = ROOT / "figures"
LEVELS = np.linspace(0.0, 1.0, 21)
DEFAULT_ETA = 0.6
DEFAULT_GAIN = 0.5
EMA = 0.9
REGIONS = (
    ("contraction", "Contraction", "#14687C", "-"),
    ("throat", "Throat", "#A55124", "--"),
)


def read_flight():
    required = [
        "t_s", "rho", "R_b", "C_global", "G_c", "R_c", "R_g",
        "r_bar", "rc_bar", "lambda_obs", "scene_region",
    ]
    flight = pd.read_csv(DATA, usecols=required).dropna()
    if flight.empty or not flight.t_s.is_monotonic_increasing:
        raise ValueError("The flight log must contain valid, time-ordered samples.")
    return flight.drop_duplicates(subset="t_s", keep="last").reset_index(drop=True)


def replay(flight):
    global_cue = flight.C_global.to_numpy()
    local_cue = flight.G_c.to_numpy()
    contraction = LEVELS[:, None] * global_cue + (1 - LEVELS[:, None]) * local_cue
    fused = 0.20 * flight.rho.to_numpy() + 0.55 * flight.R_b.to_numpy() + 0.25 * contraction
    # Reuse the callback inference of the existing weight/EMA replay.
    events = (flight.R_g.diff().abs().gt(1e-6) |
              flight.r_bar.diff().abs().gt(1e-6)).to_numpy()
    events[0] = True
    filtered = np.empty_like(fused)
    filtered_contraction = np.empty_like(contraction)
    previous = np.zeros(len(LEVELS))
    previous_contraction = contraction[:, 0].copy()
    for sample in range(len(flight)):
        if events[sample]:
            current = fused[:, sample]
            candidate = current if sample == 0 else EMA * previous + (1 - EMA) * current
            previous += np.clip(candidate - previous, -0.05, 0.05)
            previous_contraction = (EMA * previous_contraction +
                                    (1 - EMA) * contraction[:, sample])
        filtered[:, sample] = previous
        filtered_contraction[:, sample] = previous_contraction
    return contraction, filtered, filtered_contraction, events


def summarize(flight, contraction, filtered, filtered_contraction):
    # The implemented quadratic obstacle-weight map has bounds 0.5 and 2.0.
    ungated_increment = 1.5 * filtered**2
    default_eta_index = np.flatnonzero(np.isclose(LEVELS, DEFAULT_ETA)).item()
    nominal_weight = (0.5 + ungated_increment[default_eta_index] *
                      (1 - DEFAULT_GAIN * filtered_contraction[default_eta_index]))
    errors = {
        "R_c": np.mean(abs(contraction[default_eta_index] - flight.R_c)),
        "filtered R_c": np.mean(abs(filtered_contraction[default_eta_index] - flight.rc_bar)),
        "obstacle weight": np.mean(abs(nominal_weight - flight.lambda_obs)),
    }
    if errors["R_c"] > 1e-5 or max(errors.values()) > 0.005:
        raise ValueError(f"Nominal replay differs from the recorded flight: {errors}")

    records = []
    region_curves = {}
    for name, *_ in REGIONS:
        region = flight.scene_region.to_numpy() == name
        if not region.any():
            raise ValueError(f"Missing samples for {name}")
        mean_risk = contraction[:, region].mean(axis=1)
        ungated_mean = (0.5 + ungated_increment[:, region]).mean(axis=1)
        # k_c scales the reduction at every sample. Compute the joint
        # grid from this linear term after replaying each eta trajectory.
        sensitivity = (filtered_contraction[:, region] * ungated_increment[:, region]).mean(axis=1)
        reduction = sensitivity[:, None] * LEVELS[None, :]
        region_curves[name] = (mean_risk, reduction)
        for i, eta in enumerate(LEVELS):
            for j, gain in enumerate(LEVELS):
                records.append({
                    "region": name,
                    "samples": int(region.sum()),
                    "eta": eta,
                    "k_c": gain,
                    "mean_R_c": mean_risk[i],
                    "mean_obstacle_weight_k0": ungated_mean[i],
                    "mean_obstacle_weight": ungated_mean[i] - reduction[i, j],
                    "mean_obstacle_weight_reduction": reduction[i, j],
                })
    return pd.DataFrame(records), region_curves, errors


def draw_figure(region_curves):
    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 9.5,
        "axes.labelsize": 9.5,
        "axes.titlesize": 10.2,
        "axes.titleweight": "bold",
        "axes.linewidth": 0.7,
        "axes.edgecolor": "#55616B",
        "axes.spines.top": False,
        "axes.spines.right": False,
        "legend.fontsize": 8.6,
        "xtick.labelsize": 8.5,
        "ytick.labelsize": 8.5,
        "savefig.facecolor": "white",
    })
    fig, (risk_ax, gain_ax) = plt.subplots(1, 2, figsize=(7.45, 3.15), layout="constrained")
    joint_fig, joint_axes = plt.subplots(1, 2, figsize=(7.45, 3.65), sharey=True,
                                        layout="constrained")
    for figure in (fig, joint_fig):
        figure.get_layout_engine().set(w_pad=0.065, h_pad=0.065, wspace=0.08)
    default_eta_index = np.flatnonzero(np.isclose(LEVELS, DEFAULT_ETA)).item()
    default_gain_index = np.flatnonzero(np.isclose(LEVELS, DEFAULT_GAIN)).item()
    vmax = np.ceil(max(reduction.max() for _, reduction in region_curves.values()) * 100)
    ticks = np.linspace(0, 1, 6)
    meshes = []
    for i, (name, label, color, linestyle) in enumerate(REGIONS):
        mean_risk, reduction = region_curves[name]
        gain_slice = 100 * reduction[default_eta_index]
        risk_ax.plot(LEVELS, mean_risk, linestyle, color=color, lw=2.0, label=label)
        gain_ax.plot(LEVELS, gain_slice, linestyle, color=color, lw=2.0)
        risk_ax.scatter(DEFAULT_ETA, mean_risk[default_eta_index], s=31, facecolor="white",
                        edgecolor=color, linewidth=1.5, zorder=3)
        gain_ax.scatter(DEFAULT_GAIN, gain_slice[default_gain_index], s=31, facecolor="white",
                        edgecolor=color, linewidth=1.5, zorder=3)

        ax = joint_axes[i]
        mesh = ax.pcolormesh(LEVELS, LEVELS, 100 * reduction.T, shading="nearest",
                             cmap="cividis", vmin=0, vmax=vmax, rasterized=True)
        meshes.append(mesh)
        ax.contour(LEVELS, LEVELS, 100 * reduction.T,
                   levels=np.arange(1, vmax), colors="white", linewidths=0.7, alpha=0.55)
        ax.plot([DEFAULT_ETA, DEFAULT_ETA], [0, DEFAULT_GAIN], ":", color="white", lw=0.9)
        ax.plot([0, DEFAULT_ETA], [DEFAULT_GAIN, DEFAULT_GAIN], ":", color="white", lw=0.9)
        ax.scatter(DEFAULT_ETA, DEFAULT_GAIN, marker="D", s=33, facecolor="white",
                   edgecolor="#202A34", linewidth=1.0, zorder=4)
        ax.set(xlim=(0, 1), ylim=(0, 1), xticks=ticks, yticks=ticks,
               xlabel=r"Fusion coefficient $\eta$")
        ax.set_title(f"({'ab'[i]}) {label}", loc="left", pad=8)

    for ax, nominal in ((risk_ax, DEFAULT_ETA), (gain_ax, DEFAULT_GAIN)):
        ax.axvline(nominal, color="#87949C", linestyle=":", linewidth=0.95, zorder=0)
        ax.grid(axis="y", color="#DCE4EA", linewidth=0.65)
        ax.set_axisbelow(True)
        ax.set(xlim=(0, 1), xticks=ticks, ylim=(0, None))
    risk_ax.set(xlabel=r"Fusion coefficient $\eta$", ylabel=r"Mean contraction risk $R_c$")
    gain_ax.set(xlabel=r"Gate gain $k_c$", ylabel=r"Mean weight reduction ($\times 10^{-2}$)")
    risk_ax.set_title("(a) Contraction cue", loc="left", pad=8)
    gain_ax.set_title(r"(b) Gate response at $\eta=0.6$", loc="left", pad=8)
    risk_ax.legend(loc="upper right", frameon=False, handlelength=2.3)
    joint_axes[0].set_ylabel(r"Gate gain $k_c$")
    colorbar = joint_fig.colorbar(meshes[-1], ax=joint_axes, orientation="horizontal",
                                 shrink=0.73, pad=0.10, aspect=36, ticks=np.arange(0, vmax + 1))
    colorbar.set_label(r"Mean obstacle-weight reduction relative to $k_c=0$ ($\times 10^{-2}$)")
    FIGURES.mkdir(exist_ok=True)
    fig.savefig(FIGURES / "Fig_S3_contraction_sensitivity.png", dpi=600)
    joint_fig.savefig(FIGURES / "Fig_S4_contraction_joint_sensitivity.png", dpi=600)
    plt.close(fig)
    plt.close(joint_fig)


def main():
    flight = read_flight()
    contraction, filtered, filtered_contraction, events = replay(flight)
    results, region_curves, errors = summarize(flight, contraction, filtered, filtered_contraction)
    results.to_csv(ROOT / "contraction_sensitivity_replay.csv", index=False, float_format="%.8f")
    draw_figure(region_curves)
    nominal = results[np.isclose(results.eta, DEFAULT_ETA) & np.isclose(results.k_c, DEFAULT_GAIN)]
    print(f"{len(flight)} distinct timestamps, {events.sum()} inferred risk updates")
    print("Nominal reconstruction MAE:", errors)
    print(nominal[["region", "samples", "mean_R_c", "mean_obstacle_weight_k0",
                   "mean_obstacle_weight", "mean_obstacle_weight_reduction"]].to_string(index=False))


if __name__ == "__main__":
    main()
