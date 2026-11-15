"""Build the Section 5.3 evidence tables and figures from the frozen batch."""
from collections import Counter
import csv
import json
from pathlib import Path
import shutil

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "experiments/section53_batch10_20261106"
DELIVERY = ROOT / "experiments/section53_paper_final_20261107"
BUILD = ROOT / "tmp/section53_paper_final"
METHODS = ["fixed_ego_swarm", "speed_only", "coupled_legacy", "gate_off", "risk_decoupled"]
LABELS = ["Fixed EGO", "Speed-only", "Legacy", "Gate-OFF", "Proposed"]
COLORS = ["#4477AA", "#EE6677", "#228833", "#CCBB44", "#AA3377"]
WIDTHS = (1.0, 1.4)


def read_csv(path):
    with path.open(encoding="utf-8-sig", newline="") as file:
        return list(csv.DictReader(file))


def write_csv(path, rows):
    with path.open("w", encoding="utf-8-sig", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def audit_flight(row, events):
    width = float(row["width_m"])
    run = SOURCE / "runs" / f"{row['algorithm']}_w{width:.1f}_r{int(row['trial']):02d}"
    keys = ["t_wall_s", "x_m", "y_m", "z_m", "speed_twist_m_s",
            "center_wall_distance_m", "mission_window"]
    raw = run / "raw.csv"
    with raw.open(encoding="utf-8") as file:
        header = file.readline().strip().split(",")
        samples = np.loadtxt(file, delimiter=",", usecols=[header.index(key) for key in keys])
    t, x, y, z, speed, distance, mission_flag = samples.T
    position = samples[:, 1:4]
    assert np.all(np.diff(t) > 0), run
    motion = mission_flag.astype(bool)
    section = motion & (x >= -3.0) & (x <= 3.0)
    low_speed = section & (speed < 0.2)
    boundaries = np.flatnonzero(np.diff(np.r_[False, low_speed, False]))
    low_speed_duration = max((t[end - 1] - t[start] for start, end in
                              boundaries.reshape(-1, 2)), default=0.0)
    crossings = np.flatnonzero(motion[:-1] & motion[1:] & (x[:-1] < 0) & (x[1:] >= 0))
    assert len(crossings), run
    i = crossings[0]
    fraction = -x[i] / (x[i + 1] - x[i])
    _, throat_y, throat_z = position[i] + fraction * (position[i + 1] - position[i])
    passage_exit = np.any(motion[i + 1:] & (x[i + 1:] >= 3))
    passage_completed = abs(throat_y) < width / 2 and 0 <= throat_z <= 2.5 and passage_exit
    path = position[motion]
    chord = float(np.linalg.norm(path[-1] - path[0]))
    length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
    assert abs(length - float(row["path_length_m"])) < 1e-8, run
    radius = json.loads((run / "run_parameters.json").read_text())["body_collision_radius_m"]
    assert radius == 0.322
    # A segment point is within half the segment length of at least one endpoint.
    max_step = float(np.linalg.norm(np.diff(position, axis=0), axis=1).max())
    lower_bound = float(distance.min()) - max_step / 2 - radius
    assert lower_bound > 0 and row["collision"] == "false", run
    local = [e for e in events if e["event_kind"] == "local_replan"]
    failures = sum(e["success"] == "0" for e in local)
    assert len(local) == int(row["replanning_attempt_count"]), run
    assert failures == int(row["replanning_failure_count"]), run
    return {
        "passage_completed": int(passage_completed),
        "throat_crossing_y_m": float(throat_y),
        "throat_crossing_z_m": float(throat_z),
        "passage_peak_abs_y_m": float(np.max(np.abs(y[section]))),
        "longest_sampled_passage_low_speed_s": float(low_speed_duration),
        "passage_stall_0p2mps_1s": int(low_speed_duration >= 1.0),
        "endpoint_chord_length_m": chord,
        "excess_path_over_endpoint_chord_m": max(0.0, length - chord),
        "full_record_body_clearance_lower_bound_m": lower_bound,
        "local_calls_return_failed": failures,
    }


def values(rows, key):
    return np.array([float(row[key]) for row in rows])


def mean_sd(rows, key, decimals, scale=1):
    data = values(rows, key) * scale
    return f"{data.mean():.{decimals}f} ± {data.std(ddof=1):.{decimals}f}"


def save_figures(groups):
    target = DELIVERY / "figures"
    target.mkdir(exist_ok=True)
    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 10.5, "axes.spines.top": False,
        "axes.spines.right": False, "pdf.fonttype": 42, "savefig.bbox": "tight",
        "axes.labelcolor": "#222222", "axes.edgecolor": "#444444", "grid.color": "#dddddd",
    })
    figure, axes = plt.subplots(1, 2, figsize=(9.1, 3.55), sharex=True, sharey=True,
                               layout="constrained")
    for panel, (axis, width) in enumerate(zip(axes, WIDTHS)):
        means = []
        for method, color in zip(METHODS, COLORS):
            rows = groups[method, width]
            t = values(rows, "traversal_time_wall_s")
            phi = 100 * values(rows, "body_margin_negative_fraction")
            means.append((t.mean(), phi.mean()))
            axis.scatter(t, phi, color=color, alpha=0.30, s=19, linewidth=0, zorder=2)
        frontier = sorted(
            point for point in means
            if not any(other != point and other[0] <= point[0] and other[1] <= point[1]
                       for other in means)
        )
        axis.plot(*zip(*frontier), color="#515151", ls="--", lw=1.1, zorder=1)
        for method, label, color, (mt, mp), marker in zip(
                METHODS, LABELS, COLORS, means, ["s", "^", "D", "P", "o"]):
            rows = groups[method, width]
            axis.errorbar(mt, mp, xerr=values(rows, "traversal_time_wall_s").std(ddof=1),
                          yerr=100 * values(rows, "body_margin_negative_fraction").std(ddof=1),
                          fmt=marker, ms=6.8, capsize=2.8, lw=1.0, color=color,
                          label=label, zorder=3)
        axis.set_title(f"({chr(97 + panel)}) Throat {width:.1f} m", loc="left", weight="bold")
        axis.set_xlabel("Traversal time (s)")
        axis.set_xlim(22.35, 27.05)
        axis.set_ylim(4, 17.5)
        axis.xaxis.set_major_locator(MultipleLocator(1))
        axis.yaxis.set_major_locator(MultipleLocator(2))
        axis.grid(alpha=0.70)
        axis.set_axisbelow(True)
    axes[0].set_ylabel("Negative body-margin time (%)")
    handles, labels = axes[0].get_legend_handles_labels()
    figure.legend(handles, labels, loc="outside lower center", ncols=5, frameon=False)
    figure.savefig(target / "section53_pareto.png", dpi=600)
    plt.close(figure)
    for source, name in [
        ("fig53_full_flight.png", "section53_flight_distributions.png"),
        ("fig53_replanning_ecdf.png", "section53_planning_ecdf.png"),
    ]:
        shutil.copy2(SOURCE / "figures" / source, target / name)


def main():
    DELIVERY.mkdir(exist_ok=True)
    BUILD.mkdir(exist_ok=True, parents=True)
    events_by_run = {}
    for event in read_csv(SOURCE / "all_replan_events.csv"):
        key = (event["algorithm"], event["width_m"], event["trial"])
        events_by_run.setdefault(key, []).append(event)
    flights = read_csv(SOURCE / "all_flight_metrics.csv")
    assert len(flights) == len(events_by_run) == 100
    for row in flights:
        key = (row["algorithm"], row["width_m"], row["trial"])
        row.update(audit_flight(row, events_by_run[key]))
        row.pop("normalized_safety_efficiency_cost")
    write_csv(DELIVERY / "flight_results.csv", flights)
    groups = {}
    summaries = []
    for width in WIDTHS:
        for method, label in zip(METHODS, LABELS):
            attempts = [r for r in flights if r["algorithm"] == method and float(r["width_m"]) == width]
            completed = [r for r in attempts if r["task_success"] == "1"]
            assert len(attempts) == 10
            groups[method, width] = completed
            entry = {
                "通道喉宽(m)": width, "算法": label, "尝试数": len(attempts),
                "通过通道数": sum(r["passage_completed"] for r in attempts),
                "严格完成数": len(completed),
                "几何碰撞数": sum(r["collision"] == "true" for r in attempts),
                "通道低速停滞数": sum(r["passage_stall_0p2mps_1s"] for r in attempts),
                "局部重规划失败调用数": sum(r["local_calls_return_failed"] for r in attempts),
                "局部重规划调用总数": sum(int(r["replanning_attempt_count"]) for r in attempts),
                "最大超出端点直线距离的路径(mm)": f"{1000 * values(attempts, 'excess_path_over_endpoint_chord_m').max():.4f}",
            }
            for header, key, precision, scale in [
                ("穿越时间(s)", "traversal_time_wall_s", 2, 1),
                ("实际路径长度(m)", "path_length_m", 4, 1),
                ("实际平均速度(m/s)", "mean_actual_speed_twist_wallweighted_m_s", 3, 1),
                ("实际峰值速度(m/s)", "max_actual_speed_twist_m_s", 3, 1),
                ("跟踪RMSE(m)", "tracking_rmse_m", 4, 1),
                ("最小机体包络净距(m)", "min_true_body_clearance_m", 3, 1),
                ("最小质心制动裕度(m)", "min_dynamic_center_margin_m", 3, 1),
                ("最小机体制动裕度(m)", "min_dynamic_body_margin_m", 3, 1),
                ("质心负裕度时间(%)", "center_margin_negative_fraction", 2, 100),
                ("机体负裕度时间(%)", "body_margin_negative_fraction", 2, 100),
                ("累计机体裕度亏缺(m·s)", "body_braking_deficit_m_s", 3, 1),
                ("重规划均值(ms)", "replan_mean_ms", 3, 1),
                ("重规划P50(ms)", "replan_p50_ms", 3, 1),
                ("重规划P95(ms)", "replan_p95_ms", 3, 1),
                ("重规划P99(ms)", "replan_p99_ms", 3, 1),
            ]:
                population = attempts if key.startswith("replan_") else completed
                entry[header] = mean_sd(population, key, precision, scale)
            entry["统计口径"] = "均值±样本标准差，飞行指标限严格完成，计数和调用耗时含全部尝试"
            summaries.append(entry)
    write_csv(DELIVERY / "summary.csv", summaries)
    save_figures(groups)
    outcome = {
        "attempts": len(flights),
        "passage_completed": sum(r["passage_completed"] for r in flights),
        "local_calls": sum(int(r["replanning_attempt_count"]) for r in flights),
        "local_failures": sum(r["local_calls_return_failed"] for r in flights),
        "longest_in_passage_slow_interval_s": float(values(flights, "longest_sampled_passage_low_speed_s").max()),
        "stall_count": sum(r["passage_stall_0p2mps_1s"] for r in flights),
        "maximum_abs_passage_y_m": float(values(flights, "passage_peak_abs_y_m").max()),
        "maximum_excess_path_mm": float(values(flights, "excess_path_over_endpoint_chord_m").max() * 1000),
        "minimum_full_record_clearance_bound_m": float(values(flights, "full_record_body_clearance_lower_bound_m").min()),
        "max_reported_twist_speed_m_s": float(values(flights, "max_actual_speed_twist_m_s").max()),
        "maximum_walltime_fraction_over_105pct_cap": float(values(flights, "velocity_violation_5pct_wall_fraction").max()),
        "terminal_failures": [
            {key: row[key] for key in ("algorithm", "trial", "terminal_goal_distance_m", "terminal_speed_m_s")}
            for row in flights if row["task_success"] != "1"
        ],
        "event_totals": {" ".join(key): count for key, count in Counter(
            (r["event_kind"], r["success"]) for events in events_by_run.values() for r in events).items()},
    }
    (BUILD / "operational_audit.json").write_text(json.dumps(outcome, indent=2), encoding="utf-8")
    print(json.dumps(outcome, indent=2))


if __name__ == "__main__":
    main()
