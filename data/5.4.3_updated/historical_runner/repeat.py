"""Run and validate matched repeats of six archived Section 5.4.3 scenes."""
import argparse
import csv
import itertools
import os
from pathlib import Path
import random
import shutil
import subprocess
import sys
import traceback

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "section543_extension"))
import extend as ext

experiment = ext.experiment
read_json, write_json, sha = ext.read_json, ext.write_json, ext.sha
ARCHIVE = ext.ROOT / "experiments/5.4.3"
FORMAL = ARCHIVE / "formal"
ext.FORMAL = FORMAL
SCENES = (46, 22, 32, 50, 23, 31)
ROUNDS = tuple(range(1, 6))
STAGED = HERE / "scenes"
LAUNCH = HERE / "launch"
DESIGN = HERE / "repeat_design.json"
BINARIES = HERE / "runtime_sha256.json"
ARCHIVED_FILES = ("flight_results.csv", "frozen_matrix.json")
ORDER_SEED = 5430625


def prepare():
    hashes = {name: sha(FORMAL / name) for name in ARCHIVED_FILES}
    matrix = {c["scene_id"]: c for c in read_json(FORMAL / "frozen_matrix.json")}
    rows = ext.read_rows(FORMAL / "flight_results.csv")
    assert set(matrix) == set(range(1, 51))
    assert len(rows) == 150 and {
        (int(row["scene_id"]), row["method"]) for row in rows
    } == {(i, method) for i in matrix for method in experiment.METHODS}
    LAUNCH.mkdir(exist_ok=True)
    for name in ("advanced_param.launch.py", "simulator.launch.py"):
        source = ARCHIVE / "launch" / name
        assert sha(source) == sha(HERE.parent / "section543_extension/launch" / name), name
        shutil.copyfile(source, LAUNCH / name)
    binaries = experiment.runtime()
    assert binaries == read_json(HERE.parent / "section543_extension/runtime_sha256.json")
    for package, prefix in (
        ("ego_planner", "/home/ubuntu/ros2_ws/install/ego_planner"),
        ("so3_quadrotor_simulator", "/tmp/section54_overlay/install/so3_quadrotor_simulator"),
    ):
        actual = subprocess.check_output(["ros2", "pkg", "prefix", package], text=True).strip()
        assert actual == prefix, (package, actual)
    write_json(BINARIES, binaries)

    seed = ORDER_SEED
    rng = random.Random(seed)
    design = []
    for scene in SCENES:
        original = FORMAL / f"scene_{scene:02d}" / "scene"
        config = read_json(original / "scenario.json")
        assert config == matrix[scene]
        for key, file in (("map", "map.xyz"), ("noise", "noise.txt"), ("geometry", "geometry.json")):
            assert sha(original / file) == config[f"{key}_sha256"], (scene, file)
        permutations = list(itertools.permutations(experiment.METHODS))
        orders = []
        while len(orders) < len(ROUNDS):
            batch = permutations.copy()
            rng.shuffle(batch)
            orders.extend(batch)
        for round_id, order in zip(ROUNDS, orders):
            block = STAGED / f"scene_{scene:02d}" / f"round_{round_id:02d}"
            if not (block / "scene").exists():
                block.mkdir(parents=True, exist_ok=True)
                temporary = block / "scene.pending"
                temporary.mkdir(exist_ok=True)
                for file in ("scenario.json", "map.xyz", "geometry.json", "noise.txt"):
                    shutil.copyfile(original / file, temporary / file)
                temporary.rename(block / "scene")
            assert read_json(block / "scene/scenario.json") == config
            design.append(dict(scene_id=scene, replicate=round_id, order=list(order),
                               config=config))
    planned = dict(original_archive_sha256=hashes, order_seed=seed, rounds=design)
    if DESIGN.exists():
        assert read_json(DESIGN) == planned, "Frozen design or original archive changed"
    else:
        write_json(DESIGN, planned)
    print(f"READY TO RUN: {len(design)} repeat rounds, "
          f"{len(design) * len(experiment.METHODS)} flights", flush=True)


def retain_rejected(run):
    round_name, scene_name = run.parent.parent.name, run.parent.parent.parent.name
    target = HERE / "rejected" / scene_name / round_name / run.name
    target.mkdir(parents=True, exist_ok=True)
    run.rename(target / f"attempt_{1 + len(list(target.glob('attempt_*'))):02d}")


def check_parameters(run, scene, method):
    reference = FORMAL / f"scene_{scene:02d}" / "runs" / method
    nodes = experiment.NODES + (["adaptive_planner"] if method != experiment.METHODS[0] else [])
    for node in nodes:
        actual = ext.parameters(run, node)
        expected = ext.parameters(reference, node)
        if node == "drone_0_quadrotor_simulator_so3":
            noise_file = Path(actual.pop("experiment/noise_file"))
            assert noise_file == run.parent.parent / "scene/noise.txt", noise_file
            expected.pop("experiment/noise_file")
        assert actual == expected, node


def run_round(item, binaries):
    scene, round_id = item["scene_id"], item["replicate"]
    block = STAGED / f"scene_{scene:02d}" / f"round_{round_id:02d}"
    for method in item["order"]:
        run = block / "runs" / method
        marker = run / "AUDITED.json"
        if marker.exists():
            continue
        for attempt in range(1, 5):
            try:
                if not (run / "result.json").is_file():
                    if run.exists():
                        retain_rejected(run)
                    print(f"START {scene:02d}/{round_id:02d} {method} attempt {attempt}", flush=True)
                    experiment.fly(block, method, LAUNCH, binaries)
                row = ext.audit_flight(block, item["config"], method)
                check_parameters(run, scene, method)
                write_json(marker, dict(scene_id=scene, replicate=round_id, method=method,
                                        source_gap_s=row["truth_max_gap_s"],
                                        map_sha256=row["map_sha256"], noise_sha256=row["noise_sha256"]))
                print(f"AUDITED {scene:02d}/{round_id:02d} {method}", flush=True)
                break
            except Exception:
                traceback.print_exc()
                if run.exists():
                    (run / "audit_error.txt").write_text(traceback.format_exc(), encoding="utf-8")
                    retain_rejected(run)
                if attempt == 4:
                    raise


def worker(scene, domain):
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    binaries = read_json(BINARIES)
    for item in read_json(DESIGN)["rounds"]:
        if item["scene_id"] == scene:
            run_round(item, binaries)


def finalize():
    design = read_json(DESIGN)
    assert all(sha(FORMAL / name) == value
               for name, value in design["original_archive_sha256"].items())
    fields = list(ext.read_rows(FORMAL / "flight_results.csv")[0]) + ["replicate", "order_position"]
    rows, audits = [], []
    for item in design["rounds"]:
        scene, round_id = item["scene_id"], item["replicate"]
        block = STAGED / f"scene_{scene:02d}" / f"round_{round_id:02d}"
        cfg = item["config"]
        for position, method in enumerate(item["order"], 1):
            run = block / "runs" / method
            row = read_json(run / "result.json")
            audit = read_json(run / "AUDITED.json")
            assert (audit["scene_id"], audit["replicate"], audit["method"]) == (scene, round_id, method)
            assert audit["source_gap_s"] == row["truth_max_gap_s"] < 1.0
            row.update(scene_id=scene, replicate=round_id, order_position=position, method=method,
                       execution_group="fixed_scene_repeat",
                       passage_success=int(row["settled"] and row["passed_passage"] and not row["collision"]),
                       **{key: cfg[key] for key in ("seed", "width_m", "contraction_m", "noise",
                          "position_sigma_m", "velocity_sigma_m_s", "forest_density")})
            assert set(fields).issubset(row)
            rows.append(row)
            audits.append(audit)
    assert len(rows) == len(audits) == len(SCENES) * len(ROUNDS) * len(experiment.METHODS) and {
        (row["scene_id"], row["replicate"], row["method"]) for row in rows
    } == set(itertools.product(SCENES, ROUNDS, experiment.METHODS))
    with (HERE / "flight_repeats.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    write_json(HERE / "repeat_audit.json", {"flights": audits})
    (HERE / "COMPLETE").write_text(f"{len(rows)} audited flights, "
                                   f"{len(design['rounds'])} paired rounds, {len(SCENES)} frozen scenes\n",
                                   encoding="utf-8")
    print(f"COMPLETE: {len(rows)} audited flights", flush=True)


if __name__ == "__main__":
    cli = argparse.ArgumentParser()
    cli.add_argument("action", choices=("prepare", "worker", "finalize"))
    cli.add_argument("--scene", type=int, choices=SCENES)
    cli.add_argument("--domain", type=int)
    args = cli.parse_args()
    if args.action == "prepare":
        prepare()
    elif args.action == "worker":
        assert args.scene is not None and args.domain is not None
        worker(args.scene, args.domain)
    else:
        finalize()
