"""Export archived trajectories and local obstacle views for MATLAB."""

import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.io import savemat

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent / "ego-planner-swarm"
ARCHIVE = REPO / "experiments/5.4.3/formal"
sys.path.insert(0, str(REPO / "scripts/section54"))
from scenes import RADIUS, distance_to_solids


def recorded_path(block, method):
    with np.load(block / "runs" / method / "raw.npz") as log:
        first = max(0, int(np.searchsorted(log["stamp_ns"], log["commands"][0, 0])) - 1)
        return log["truth"][first:, :3]


def crossing(path, station):
    x = path[:, 0]
    right = np.searchsorted(np.maximum.accumulate(x), station)
    assert 0 < right < len(path)
    left = right - 1
    ratio = (station - x[left]) / (x[right] - x[left])
    return path[left] + ratio * (path[right] - path[left])


def obstacle_row(obstacle):
    x, y = obstacle["x"], obstacle["y"]
    if obstacle["shape"] == "cylinder":
        radius, half_height = obstacle["r"], obstacle["h"] / 2
        return [1, x, y, half_height, radius, radius, half_height, 0]
    return [2, x, y, obstacle["z"], obstacle["a"], obstacle["b"], obstacle["c"], obstacle["yaw"]]


def export_scene(scene_id, detail_method, methods):
    block = ARCHIVE / f"scene_{scene_id:02d}"
    config = json.loads((block / "scene/scenario.json").read_text(encoding="utf-8"))
    geometry = json.loads((block / "scene/geometry.json").read_text(encoding="utf-8"))
    paths = {method: recorded_path(block, method) for method in methods}
    detail_path = paths[detail_method]
    closest = detail_path[np.argmin(distance_to_solids(detail_path, geometry))]
    station = closest[0]
    assert abs(station) > config["contraction_m"]
    local = np.stack([crossing(path, station) for path in paths.values()])
    distances = [distance_to_solids(closest[None, :], dict(geometry, obstacles=[o]))[0]
                 for o in geometry["obstacles"]]
    target = int(np.argmin(distances))
    assert np.isclose(distances[target], distance_to_solids(closest[None, :], geometry)[0])
    obstacles = np.array([obstacle_row(o) for o in geometry["obstacles"]])
    target_row = obstacles[target]
    lateral_radius = np.hypot(target_row[4] * np.sin(target_row[7]),
                              target_row[5] * np.cos(target_row[7]))
    y_min = min(local[:, 1].min() - RADIUS, target_row[2] - lateral_radius) - .25
    y_max = max(local[:, 1].max() + RADIUS, target_row[2] + lateral_radius) + .25
    clearances = distance_to_solids(local, geometry) - RADIUS
    print(f"Scene {scene_id}, x={station:.2f} m: local body clearances {clearances.round(3)} m")
    result = dict(
        scene_id=scene_id, width=config["width_m"], base=paths["fixed_ego_swarm"],
        proposed=paths["risk_decoupled"], start=config["start"], goal=config["goal"],
        wall_nodes=geometry["wall_nodes"], wall_height=geometry["wall_height_m"],
        wall_thickness=geometry["wall_thickness_m"], obstacles=obstacles,
        station=station, target=target + 1, local_points=local, body_radius=RADIUS,
        detail_x_limits=[station - 1.75, station + 1.75],
        detail_y_limits=[np.floor(2 * y_min) / 2, np.ceil(2 * y_max) / 2],
        detail_title="Front forest detail" if station < 0 else "Rear forest detail",
    )
    if "speed_only" in paths:
        result["speed_only"] = paths["speed_only"]
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=int, choices=(13,))
    parser.add_argument("--with-speed-only", action="store_true",
                        help="Export a separate three-method dataset for Scene 13.")
    args = parser.parse_args()
    if args.with_speed_only and args.scene != 13:
        parser.error("--with-speed-only requires --scene 13")
    if args.scene == 13:
        if args.with_speed_only:
            methods = ("fixed_ego_swarm", "speed_only", "risk_decoupled")
            output = "scan_13_speed_only_data.mat"
        else:
            methods = ("fixed_ego_swarm", "risk_decoupled")
            output = "scan_13_data.mat"
        cases = {"case13": export_scene(13, "fixed_ego_swarm", methods)}
    else:
        methods = ("fixed_ego_swarm", "speed_only", "risk_decoupled")
        output = "scan_4622_data.mat"
        cases = {f"case{i}": export_scene(i, "speed_only", methods) for i in (46, 22)}
    with (HERE / output).open("wb") as out:
        savemat(out, cases, do_compression=True)
