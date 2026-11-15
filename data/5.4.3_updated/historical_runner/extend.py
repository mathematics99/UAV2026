"""Stage and audit the thirty additional matched Section 5.4.3 scenes."""
import argparse
import csv
import fcntl
import io
import json
import os
from pathlib import Path
import random
import shutil
import subprocess
import sys
import traceback

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent / "ego-planner-swarm"
EXPERIMENTS = ROOT / "experiments/section54_v2"
FORMAL = EXPERIMENTS / "formal"
SNAPSHOT = HERE / "original"
STAGED = HERE / "scenes"
READY = HERE / "ready"
LAUNCH = HERE / "launch"
SCENE_IDS = tuple(range(21, 51))
SEED = 540050
sys.path.insert(0, str(ROOT / "scripts/section54"))
import run as experiment
from scenes import materialize, sha


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path, data):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def read_rows(path):
    with path.open(encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def prepare():
    SNAPSHOT.mkdir(exist_ok=True)
    STAGED.mkdir(exist_ok=True)
    for name in ("flight_results.csv", "frozen_matrix.json", "COMPLETE"):
        source = FORMAL / name
        saved = SNAPSHOT / name
        if not saved.exists():
            shutil.copy2(source, saved)
        assert sha(source) == sha(saved), f"Original archive changed: {name}"
    old = read_json(SNAPSHOT / "frozen_matrix.json")
    assert [cfg["scene_id"] for cfg in old] == list(range(1, 21))
    original_rows = read_rows(SNAPSHOT / "flight_results.csv")
    assert len(original_rows) == 60 and {
        (int(r["scene_id"]), r["method"]) for r in original_rows
    } == {(i, method) for i in range(1, 21) for method in experiment.METHODS}
    density, gap, margin = 20, 1.2, 0.1
    assert all((c["forest_density"], c["forest_gap_m"], c["route_margin_m"]) ==
               (density, gap, margin) for c in old)

    experiment.configure_launch(LAUNCH)
    for name in ("advanced_param.launch.py", "simulator.launch.py"):
        assert (LAUNCH / name).read_text() == (EXPERIMENTS / "launch" / name).read_text(), name
    binary_hashes = experiment.runtime()
    selected = subprocess.check_output(["ros2", "pkg", "prefix", "ego_planner"], text=True).strip()
    assert selected == "/home/ubuntu/ros2_ws/install/ego_planner", "Unexpected EGO binary prefix"
    selected = subprocess.check_output(["ros2", "pkg", "prefix", "so3_quadrotor_simulator"], text=True).strip()
    assert selected == "/tmp/section54_overlay/install/so3_quadrotor_simulator", "Unexpected truth simulator prefix"
    write_json(HERE / "runtime_sha256.json", binary_hashes)

    rng = random.Random(SEED)
    n = len(SCENE_IDS)
    widths = [round(1 + 0.6 * (i + rng.random()) / n, 3) for i in range(n)]
    lengths = [round(4 + 3 * (i + rng.random()) / n, 3) for i in range(n)]
    rng.shuffle(widths)
    rng.shuffle(lengths)
    conditions = [(v, noise) for v in (0., .5, 1.) for noise in ("none", "low", "medium")] * 3
    # Add one of each speed and noise level without repeating either extra pair
    # in the archived twenty-scene design.
    conditions += [(0., "low"), (.5, "medium"), (1., "none")]
    rng.shuffle(conditions)
    design = []
    for i, width, length, (vx, noise) in zip(SCENE_IDS, widths, lengths, conditions):
        order = rng.sample(experiment.METHODS, len(experiment.METHODS))
        design.append(dict(scene_id=i, width_m=width, contraction_m=length,
                           initial_vx_m_s=vx, noise=noise, order=order,
                           forest_gap_m=gap, route_margin_m=margin))
    if (HERE / "design.json").is_file():
        assert read_json(HERE / "design.json") == design, "Extension design changed"
    else:
        write_json(HERE / "design.json", design)

    sweep = read_rows(EXPERIMENTS / "sweep/sweep_results.csv")
    map_hashes = {c["map_sha256"] for c in old} | {r["map_sha256"] for r in sweep}
    matrix = []
    for cfg in design:
        i = cfg["scene_id"]
        directory = STAGED / f"scene_{i:02d}" / "scene"
        if (directory / "scenario.json").is_file():
            frozen = read_json(directory / "scenario.json")
            assert all(frozen[key] == value for key, value in cfg.items())
        else:
            print("FREEZE", i, flush=True)
            frozen = materialize(directory, cfg, 542000 + 1000 * i, densities=(density,))
        for key, filename in (("map", "map.xyz"), ("noise", "noise.txt"),
                              ("geometry", "geometry.json")):
            assert sha(directory / filename) == frozen[f"{key}_sha256"]
        assert frozen["map_sha256"] not in map_hashes, f"Duplicate map for Scene {i}"
        map_hashes.add(frozen["map_sha256"])
        matrix.append(frozen)
    write_json(HERE / "extension_matrix.json", matrix)
    print("FROZEN", len(matrix), "new scenes", flush=True)


def parameters(run, node):
    dump = run / "parameters" / f"{node}_parameters.yaml"
    return next(iter(yaml.safe_load(dump.read_text()).values()))["ros__parameters"]


def audit_flight(block, cfg, method):
    run = block / "runs" / method
    row = read_json(run / "result.json")
    for key in ("map_sha256", "noise_sha256"):
        assert row[key] == cfg[key], key
    assert row["initial_vx_measured_m_s"] == cfg["initial_vx_m_s"], "Initial speed"
    assert row["noise_max_error"] < 1e-12, "Noise mismatch"
    assert row["truth_max_gap_s"] < 1.0, "Source timestamp gap of at least 1 s"
    assert bool(row["collision"]) == (row["min_body_clearance_m"] <= 0), "Clearance sign"
    assert row["collision"] or row["clearance_lower_bound_m"] > 0, "Clearance certificate"
    local = [e for e in read_rows(run / "replans.csv") if e["kind"] == "local"]
    assert len(local) == row["local_calls"], "Missing local calls"
    assert sum(e["success"] == "0" for e in local) == row["local_failures"], "Failed call count"
    with np.load(run / "raw.npz") as raw:
        stamps, steps, truth, observed, commands = (
            raw[k] for k in ("stamp_ns", "step", "truth", "observed", "commands"))
        assert len(stamps) == len(truth) == row["truth_samples"] > 100
        assert len(observed) == row["paired_samples"] and len(stamps) - len(observed) in (0, 1)
        assert np.all(np.diff(stamps) > 0) and np.all(np.diff(steps) >= 0)
        gap = float(np.diff(stamps).max() * 1e-9)
        assert gap < 1.0 and np.isclose(gap, row["truth_max_gap_s"], rtol=0, atol=1e-10)
        initial = cfg["start"] + [cfg["initial_vx_m_s"], 0., 0.]
        assert steps[0] == 0 and np.all(truth[steps == 0] == initial)
        noise = np.loadtxt(block / "scene/noise.txt")
        assert float(np.max(abs(observed - truth[:len(observed)] -
                                noise[steps[:len(observed)] // 10]))) < 1e-12
        start = max(0, int(np.searchsorted(stamps, commands[0, 0])) - 1) if len(commands) else 0
        geometry = read_json(block / "scene/geometry.json")
        clearance, bound, collision = experiment.swept_clearance(truth[start:, :3], geometry)
        assert np.isclose(clearance, row["min_body_clearance_m"], rtol=0, atol=1e-8)
        assert np.isclose(bound, row["clearance_lower_bound_m"], rtol=0, atol=1e-8)
        assert collision == row["collision"]

    old_run = FORMAL / "scene_02/runs" / method
    for node in ("drone_0_ego_planner_node", "risk_evaluator", "drone_0_pcl_render_node"):
        assert parameters(run, node) == parameters(old_run, node), f"Changed {node} parameters"
    if method != "fixed_ego_swarm":
        assert parameters(run, "adaptive_planner") == parameters(old_run, "adaptive_planner")
    return row


def retain_rejected(run):
    target = HERE / "rejected" / run.parent.parent.name / run.name
    target.mkdir(parents=True, exist_ok=True)
    attempt = 1 + len(list(target.glob("attempt_*")))
    run.rename(target / f"attempt_{attempt:02d}")


def work(ids, domain):
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    configs = {c["scene_id"]: c for c in read_json(HERE / "extension_matrix.json")}
    binaries = read_json(HERE / "runtime_sha256.json")
    locks = HERE / "scene_locks"
    locks.mkdir(exist_ok=True)
    for i in ids:
        with (locks / f"scene_{i:02d}.lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            work_scene(configs[i], domain, binaries)


def work_scene(cfg, domain, binaries):
    i = cfg["scene_id"]
    block = STAGED / f"scene_{i:02d}"
    for method in cfg["order"]:
        run = block / "runs" / method
        if (run / "AUDITED.json").is_file():
            continue
        for attempt in range(1, 9):
            try:
                if not (run / "result.json").is_file():
                    if run.exists():
                        retain_rejected(run)
                    print("START", i, method, "domain", domain, "attempt", attempt, flush=True)
                    experiment.fly(block, method, LAUNCH, binaries)
                result = audit_flight(block, cfg, method)
                write_json(run / "AUDITED.json", dict(scene=i, method=method,
                    max_source_gap_s=result["truth_max_gap_s"],
                    map_sha256=result["map_sha256"], noise_sha256=result["noise_sha256"]))
                print("AUDITED", i, method, flush=True)
                break
            except Exception as error:
                traceback.print_exc()
                if run.exists():
                    (run / "audit_error.txt").write_text(traceback.format_exc(), encoding="utf-8")
                    retain_rejected(run)
                interrupted = str(error) in {"ROS launch exited: -15", "Parameter audit failed"}
                if attempt == 8 or (attempt >= 3 and not interrupted):
                    raise


def finalize():
    old = read_json(SNAPSHOT / "frozen_matrix.json")
    matrix = read_json(HERE / "extension_matrix.json")
    assert [c["scene_id"] for c in matrix] == list(SCENE_IDS)
    old_bytes = (SNAPSHOT / "flight_results.csv").read_bytes()
    old_rows = read_rows(SNAPSHOT / "flight_results.csv")
    fields = list(old_rows[0])
    line_ending = "\r\n" if b"\r\n" in old_bytes.splitlines(keepends=True)[0] else "\n"
    append = io.StringIO(newline="")
    writer = csv.DictWriter(append, fields, lineterminator=line_ending, extrasaction="ignore")
    audits = []
    for cfg in matrix:
        i = cfg["scene_id"]
        block = STAGED / f"scene_{i:02d}"
        for method in cfg["order"]:
            run = block / "runs" / method
            marker = read_json(run / "AUDITED.json")
            row = read_json(run / "result.json")
            assert marker["scene"] == i and marker["method"] == method
            assert row["truth_max_gap_s"] == marker["max_source_gap_s"] < 1
            row.update(scene_id=i, method=method, execution_group="extension",
                       passage_success=int(row["settled"] and row["passed_passage"] and not row["collision"]),
                       **{key: cfg[key] for key in ("seed", "width_m", "contraction_m", "noise",
                          "position_sigma_m", "velocity_sigma_m_s", "forest_density")})
            assert set(fields).issubset(row)
            writer.writerow(row)
            audits.append(marker)
    assert len(audits) == 90
    READY.mkdir(exist_ok=True)
    separator = b"" if old_bytes.endswith(b"\n") else line_ending.encode("ascii")
    (READY / "flight_results.csv").write_bytes(old_bytes + separator + append.getvalue().encode("utf-8"))
    write_json(READY / "frozen_matrix.json", old + matrix)
    (READY / "COMPLETE").write_text("150 audited attempts (50 matched scenes)\n", encoding="utf-8")
    write_json(READY / "extension_audit_21_50.json", dict(seed=SEED, scenes=list(SCENE_IDS),
        original_excluded_scene_ids=[11], new_valid_flights=len(audits), primary_scene_count=49,
        primary_flight_count=147, runtime_sha256=read_json(HERE / "runtime_sha256.json"), flights=audits))
    rows = read_rows(READY / "flight_results.csv")
    assert rows[:60] == old_rows and len(rows) == 150
    assert len({(r["scene_id"], r["method"]) for r in rows}) == 150
    print("READY: 150 rows, 50 matched scenes", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=("prepare", "worker", "finalize"))
    parser.add_argument("ids", type=int, nargs="*")
    parser.add_argument("--domain", type=int)
    args = parser.parse_intermixed_args()
    if args.action == "prepare":
        prepare()
    elif args.action == "worker":
        work(args.ids, args.domain)
    else:
        finalize()
