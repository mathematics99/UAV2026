"""Recompute the historical matched forest batch and draw its manuscript figure."""
import csv
from itertools import product
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

SOURCE = Path("F:/无人机暑假开发/看实验数据/data/复合环境/multi_trials")
OUT = Path(__file__).resolve().parents[2] / "experiments/section54_forest_seed_study/figures"
METHODS = ("baseline", "speed_only", "adaptive")
NAMES = ("Fixed EGO", "Speed-only", "Proposed")
COLORS = ("#4477AA", "#B47A21", "#AA3377")
STYLES = ("--", "-.", "-")
STATIONS = np.linspace(-14.0, 13.8, 140)


def load_flight(method, seed):
    folder = SOURCE / method / f"seed{seed:02d}"
    log, = folder.glob("*_timeseries.csv")
    with log.open(encoding="utf-8-sig", newline="") as file:
        rows = list(csv.DictReader(file))
    times = np.array([float(row["t_s"]) for row in rows])
    assert np.all(np.diff(times) >= 0)
    assert {int(row["seed"]) for row in rows} == {seed}
    start = next(i for i, row in enumerate(rows) if row["cmd_x"])
    stop = next(i for i in range(start, len(rows)) if float(rows[i]["x"]) >= 14.0)
    return rows[start:stop+1], times[stop]-times[start]


def samples(rows, fields, keep):
    # Check sensitivity to the ordering of rows with the same logged timestamp.
    ordered = rows if keep == "last" else reversed(rows)
    by_time = {float(row["t_s"]): row for row in ordered
               if all(row[field] for field in fields)}
    return np.array([[time, *(float(by_time[time][field]) for field in fields)]
                     for time in sorted(by_time)])


def exposure(rows, keep):
    t, vx, vy, vz, distance = samples(rows, ("vx", "vy", "vz", "d_min"), keep).T
    v = np.linalg.norm([vx, vy, vz], axis=0)
    margin = distance - (0.30*v + v**2/4)
    assert np.isfinite(margin).all() and t[-1] > t[0]
    percent = 100*np.trapezoid((margin < 0).astype(float), t)/(t[-1]-t[0])
    return percent, float(np.diff(t).max())


def first_crossing_speed(rows):
    _, x, vx, vy, vz = samples(rows, ("x", "vx", "vy", "vz"), "last").T
    right = np.searchsorted(np.maximum.accumulate(x), STATIONS)
    assert np.all((right > 0) & (right < len(x)))
    left = right - 1
    u = (STATIONS-x[left])/(x[right]-x[left])
    velocity = np.stack([vx, vy, vz], axis=1)
    return np.linalg.norm(velocity[left]+u[:, None]*(velocity[right]-velocity[left]), axis=1)


def plot(times, exposures, speed):
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11,
                         "axes.spines.top": False, "axes.spines.right": False})
    fig, (profile, trade) = plt.subplots(1, 2, figsize=(10.4, 3.9),
                                        gridspec_kw={"width_ratios": [1.22, 1]})
    fig.subplots_adjust(left=.08, right=.985, top=.89, bottom=.25, wspace=.34)
    profile.axvspan(-5, 5, color="#E6EDF4", zorder=0)
    profile.axvline(0, ls=":", color="#7890A3", lw=.9, zorder=1)
    for i, (name, color, style) in enumerate(zip(NAMES, COLORS, STYLES)):
        low, center, high = np.quantile(speed[i], [.25, .5, .75], axis=0)
        profile.fill_between(STATIONS, low, high, color=color, alpha=.10, linewidth=0)
        profile.plot(STATIONS, center, label=name, color=color, ls=style, lw=1.85)
    profile.set(xlim=(-14, 14), xlabel="Longitudinal position (m)",
                ylabel="Executed speed (m/s)", xticks=[-10, -5, 0, 5, 10])
    profile.legend(loc="upper center", bbox_to_anchor=(.5, -.25), ncols=3,
                   frameon=False, fontsize=9.5)

    trade.axhline(0, color="#718395", lw=.8, ls="--")
    trade.axvline(0, color="#718395", lw=.8, ls="--")
    handles = []
    for i, marker in enumerate(("o", "s")):
        dt = times[-1]-times[i]
        gain = exposures[i]-exposures[-1]
        trade.scatter(dt, gain, marker=marker, s=36, color=COLORS[i], alpha=.68,
                      edgecolor="white", lw=.45, zorder=3)
        trade.scatter(dt.mean(), gain.mean(), marker=marker, s=101, facecolor="white",
                      edgecolor=COLORS[i], lw=1.55, zorder=4)
        handles.append(Line2D([], [], marker=marker, ls="none", color=COLORS[i],
                              label=f"vs. {NAMES[i]}", markersize=5))
    handles.append(Line2D([], [], marker="o", ls="none", markerfacecolor="white",
                          markeredgecolor="#718395", label="Mean", markersize=7))
    trade.set(xlabel=r"Additional time to $x=14$ m (s)",
              ylabel="Reduction in exposure (pp)")
    trade.legend(handles=handles, loc="upper center", bbox_to_anchor=(.5, -.25),
                 ncols=3, frameon=False, fontsize=9.5)
    for ax, title in zip((profile, trade), ("(a) Flight speed along the route",
                                           "(b) Matched time and exposure")):
        ax.set_title(title, loc="left", fontweight="bold")
        ax.grid(axis="y", alpha=.23)
        ax.set_axisbelow(True)
    OUT.mkdir(exist_ok=True, parents=True)
    fig.savefig(OUT/"section54_forest_seed_study.png", dpi=600)
    plt.close(fig)


def main():
    times, exposures, speed = [], [], []
    max_sensitivity = max_gap = 0.0
    for method in METHODS:
        method_times, method_exposures, method_speed = [], [], []
        for seed in range(1, 11):
            rows, duration = load_flight(method, seed)
            phi, gap = exposure(rows, "last")
            alternative, _ = exposure(rows, "first")
            max_sensitivity = max(max_sensitivity, abs(phi-alternative))
            max_gap = max(max_gap, gap)
            method_times.append(duration)
            method_exposures.append(phi)
            method_speed.append(first_crossing_speed(rows))
        times.append(method_times)
        exposures.append(method_exposures)
        speed.append(method_speed)
    times, exposures, speed = map(np.array, (times, exposures, speed))
    print("Means and sample SDs in the order Fixed EGO, Speed-only, Proposed")
    for label, values in (("Time (s)", times), ("Exposure (%)", exposures)):
        print(label, "mean", values.mean(axis=1), "SD", values.std(axis=1, ddof=1))
    signs = np.array(list(product((-1, 1), repeat=10)))
    p_values = []
    for i in (0, 1):
        for values in (times, exposures):
            delta = values[-1]-values[i]
            p = np.mean(np.abs(signs@delta) >= abs(delta.sum())-1e-10)
            p_values.append(p)
            print(NAMES[i], "mean", delta.mean(), "median", np.median(delta),
                  "n_lower", sum(delta < 0), "p", p)
    order = np.argsort(p_values)
    adjusted = np.empty(4)
    adjusted[order] = np.minimum(1, np.maximum.accumulate(np.array(p_values)[order]*np.arange(4, 0, -1)))
    print("Holm-adjusted p:", adjusted)
    print("Max exposure sensitivity (pp):", max_sensitivity, "Max valid-data gap (s):", max_gap)
    plot(times, exposures, speed)


if __name__ == "__main__":
    main()
