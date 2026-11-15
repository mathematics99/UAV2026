"""Update the fixed-throat comparison using all 30 matched forest layouts."""

import csv
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent.parent / "ego-planner-swarm/scripts/section54"))
import plot_forest_seed_study as original

OUT = HERE / "section542_updated"
SEEDS = tuple(range(1, 31))
KEYS = ("T14", "exposure", "RMSE", "jerk")
DIAGNOSTICS = (
    "travel_time_s", "mean_speed_m_s", "min_clearance_m", "braking_margin_M_min_m",
    "braking_margin_M_negative_time_ratio", "mean_Rb", "Rb_saturation_ratio", "replan_count",
)


def read(path):
    with path.open(encoding="utf-8-sig", newline="") as file:
        return list(csv.DictReader(file))


def exact_sign_flip(delta):
    """Exact two-sided test on the paired mean via two half-sum distributions."""
    halves = []
    for block in np.array_split(delta, 2):
        sums = np.array([0.0])
        for value in block:
            sums = np.r_[sums + value, sums - value]
        halves.append(sums)
    first, second = halves
    second.sort()
    cutoff = abs(delta.sum()) - 1e-10
    if cutoff <= 0:
        return 1.0
    lower = np.searchsorted(second, -cutoff - first, side="right")
    upper = len(second) - np.searchsorted(second, cutoff - first, side="left")
    return float((lower + upper).sum() / (len(first) * len(second)))


def paired_tests(data):
    n = data.shape[1]
    samples = np.random.default_rng(5422026).integers(0, n, size=(20_000, n))
    tests = {}
    for i, method in enumerate(original.METHODS[:2]):
        tests[method] = {}
        for j, key in enumerate(KEYS):
            delta = data[-1, :, j] - data[i, :, j]
            tests[method][key] = dict(
                mean=float(delta.mean()), median=float(np.median(delta)),
                lower=int(np.count_nonzero(delta < 0)), higher=int(np.count_nonzero(delta > 0)),
                ci=np.quantile(delta[samples].mean(axis=1), [.025, .975]).tolist(),
                raw_p=exact_sign_flip(delta),
            )
    ordered = sorted((v for group in tests.values() for v in group.values()),
                     key=lambda row: row["raw_p"])
    adjusted = 0.
    for n_remaining, row in zip(range(len(ordered), 0, -1), ordered):
        adjusted = min(1., max(adjusted, row["raw_p"] * n_remaining))
        row["holm_p"] = adjusted
    return tests


def flight(method, seed, saved):
    folder = original.SOURCE / method / f"seed{seed:02d}"
    log, = folder.glob("*_timeseries.csv")
    rows = read(log)
    times = np.array([float(row["t_s"]) for row in rows])
    assert np.all(np.diff(times) >= 0) and {int(row["seed"]) for row in rows} == {seed}
    start = next(i for i, row in enumerate(rows) if all(row[k] for k in ("cmd_x", "cmd_y", "cmd_z")))
    end = next(i for i in range(start, len(rows)) if float(rows[i]["x"]) >= 14)
    window = rows[start:end+1]
    exposure, gap = original.exposure(window, "last")
    alternative, _ = original.exposure(window, "first")
    speed = original.first_crossing_speed(window)

    x0, y0 = (float(rows[0][k]) for k in ("x", "y"))
    motion = next(i for i, row in enumerate(rows)
                  if np.hypot(float(row["x"]) - x0, float(row["y"]) - y0) > .3)
    tracking = rows[motion:end+1]
    axes = ("x", "y", "z")
    differences = [[float(row[k]) - float(row[f"cmd_{k}"]) for k in axes]
                   for row in tracking if all(row[f"cmd_{k}"] for k in axes)]
    rmse = float(np.sqrt(np.mean(np.sum(np.square(differences), axis=1))))
    sensor_min = min(float(row["d_min"]) for row in tracking if row["d_min"])
    margins = np.asarray([float(row["d_min"]) - float(row["d_brake"]) for row in tracking
                          if row["d_min"] and row["d_brake"]])
    risk = np.asarray([float(row["R_b"]) for row in tracking if row["R_b"]])
    recomputed = dict(
        travel_time_s=times[end] - times[motion], tracking_rmse_m=rmse, min_clearance_m=sensor_min,
        braking_margin_M_min_m=margins.min(), braking_margin_M_negative_time_ratio=(margins < 0).mean(),
        mean_Rb=risk.mean(), Rb_saturation_ratio=(risk > .8).mean(),
    )
    for key, value in recomputed.items():
        assert abs(value - float(saved[key])) <= 5.01e-7, (method, seed, key, value, saved[key])
    archive = {row["metric"]: row["value"] for row in read(folder / "run_summary.csv")}
    assert all(float(saved[k]) == float(archive[k]) for k in (*DIAGNOSTICS, "mean_abs_jerk_m_s3"))
    return (times[end] - times[start], exposure, float(saved["tracking_rmse_m"]),
            float(saved["mean_abs_jerk_m_s3"])), speed, gap, abs(exposure - alternative)


def main():
    keys = {(m, i) for m in original.METHODS for i in SEEDS}
    rows = read(original.SOURCE / "trials_wide.csv")
    by_key = {(r["method"], int(r["seed"])): r for r in rows}
    assert set(by_key) == keys and len(by_key) == len(rows) == 90
    metrics, profiles, issues = [], [], []
    sensitivities = []
    for method in original.METHODS:
        method_data, method_speed = [], []
        for seed in SEEDS:
            result, speed, gap, sensitivity = flight(method, seed, by_key[method, seed])
            method_data.append(result)
            method_speed.append(speed)
            sensitivities.append(sensitivity)
            if gap >= 1:
                issues.append((method, seed, gap))
        metrics.append(method_data)
        profiles.append(method_speed)
    data = np.asarray(metrics)
    profiles = np.asarray(profiles)
    stats = {}
    for i, method in enumerate(original.METHODS):
        stats[method] = {}
        for j, key in enumerate(KEYS):
            values = data[i, :, j]
            stats[method][key] = dict(
                mean=float(values.mean()), sd=float(values.std(ddof=1)),
                median=float(np.median(values)), iqr=np.quantile(values, [.25, .75]).tolist(),
            )
        for key in DIAGNOSTICS:
            stats[method][key] = float(np.mean([float(by_key[method, n][key]) for n in SEEDS]))
    keep = [n not in {s for _, s, _ in issues} for n in SEEDS]
    report = dict(
        n=len(SEEDS), diagnostics=stats, paired=paired_tests(data),
        gap_sensitivity=paired_tests(data[:, keep, :]), gaps_at_least_one_s=issues,
        maximum_duplicate_handling_change_pp=max(sensitivities),
    )
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "fixed_throat_statistics.json").write_text(json.dumps(report, indent=2) + "\n",
                                                     encoding="utf-8")
    original.OUT = OUT / "figures"
    original.plot(data[:, :, 0], data[:, :, 1], profiles)


if __name__ == "__main__":
    main()
