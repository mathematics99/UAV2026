"""Serial matched composite flights. On any capture/audit error, stop the batch."""
import argparse
from collections import Counter
import csv
import json
import os
from pathlib import Path
import random
import re
import shutil
import signal
import subprocess
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float64
from traj_utils.msg import ReplanEvent

from scenes import GOAL, RADIUS, distance_to_solids, materialize, sha

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / "experiments/section53_batch10_20261106/reproducibility"
OUT = ROOT / "experiments/section54_v2"
METHODS = ["fixed_ego_swarm", "speed_only", "risk_decoupled"]
NODES = ["drone_0_ego_planner_node", "drone_0_quadrotor_simulator_so3",
         "drone_0_so3_control_component", "risk_evaluator", "sensor_adapter",
         "drone_0_traj_server", "drone_0_pcl_render_node"]


def json_write(path, data):
    path.write_text(json.dumps(data, indent=2), encoding="utf-8")


def write_csv(path, rows, fieldnames=None):
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames or list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def xyz(message):
    return message.x, message.y, message.z


class Capture(Node):
    def __init__(self, scene):
        super().__init__("section54_capture")
        self.truth, self.obs, self.commands, self.events = [], {}, [], []
        self.goal = np.asarray(GOAL)
        self.settling_since, self.settled = None, False
        self.map_hashes = set()
        self.map = PointCloud2()
        self.map.header.frame_id = "world"
        self.map.height = 1
        self.map.data = (scene/"map.xyz").read_bytes()
        self.map.width = len(self.map.data)//12
        self.map.fields = [PointField(name=k, offset=i*4, datatype=PointField.FLOAT32, count=1)
                           for i,k in enumerate("xyz")]
        self.map.point_step, self.map.row_step, self.map.is_dense = 12, len(self.map.data), True
        self.publisher = self.create_publisher(PointCloud2, "/map_generator/global_cloud", 1)
        self.create_timer(.5, self.send_map)
        self.create_subscription(PointCloud2, "/map_generator/global_cloud", self.see_map, 10)
        self.create_subscription(Odometry, "/experiment/truth", self.see_truth, 1000)
        self.create_subscription(Odometry, "/drone_0_visual_slam/odom", self.see_obs, 1000)
        self.create_subscription(PositionCommand, "/drone_0_planning/pos_cmd", self.see_cmd, 1000)
        self.create_subscription(ReplanEvent, "/planner/replan_event", self.see_event, 1000)
        self.adaptive = {k: [] for k in ("safe_distance", "collision_weight",
                                          "max_velocity", "smoothed_risk", "contraction_gate")}
        for topic, key in (("/adaptive/safe_distance", "safe_distance"),
                           ("/adaptive/collision_weight", "collision_weight"),
                           ("/adaptive/max_velocity", "max_velocity"),
                           ("/adaptive/smoothed_risk", "smoothed_risk"),
                           ("/adaptive/contraction_gate", "contraction_gate")):
            self.create_subscription(Float64, topic,
                                     lambda m, k=key: self.adaptive[k].append(
                                         (self.get_clock().now().nanoseconds, m.data)), 1000)

    def send_map(self):
        self.map.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.map)

    def see_map(self, msg):
        import hashlib
        self.map_hashes.add(hashlib.sha256(msg.data).hexdigest())

    def see_truth(self, msg):
        stamp, step = ns(msg.header.stamp), int(msg.child_frame_id)
        p, v = xyz(msg.pose.pose.position), xyz(msg.twist.twist.linear)
        self.truth.append((stamp, step, *p, *v))
        in_goal = np.linalg.norm(np.asarray(p)-self.goal) <= .2
        if step and in_goal and np.linalg.norm(v) < .2:
            if self.settling_since is None:
                self.settling_since = stamp
            self.settled = stamp - self.settling_since >= 10**9
        else:
            self.settling_since = None

    def see_obs(self, msg):
        stamp = ns(msg.header.stamp)
        if stamp in self.obs:
            raise RuntimeError("Duplicate observed odometry timestamp")
        self.obs[stamp] = (*xyz(msg.pose.pose.position), *xyz(msg.twist.twist.linear))

    def see_cmd(self, msg):
        self.commands.append((ns(msg.header.stamp), *xyz(msg.position)))

    def see_event(self, msg):
        self.events.append(dict(stamp_ns=ns(msg.stamp), count=msg.count,
                                time_ms=msg.planning_time_ms, success=int(msg.success)))


def classify_events(events, log):
    log_calls = []
    for time_s, duration in re.findall(r"\[INFO\]\s+\[(\d+\.\d+)\][^\n]*\[planning_time\]\s+([\d.]+) ms", log):
        sec, frac = time_s.split(".")
        log_calls.append((int(sec)*10**9+int(frac.ljust(9,"0")[:9]),float(duration)))
    used = set()
    for event in events:
        matches = [i for i,(stamp,ms) in enumerate(log_calls) if i not in used
                   and 0 <= event["stamp_ns"]-stamp < 100_000_000
                   and abs(event["time_ms"]-ms) <= .000501]
        if matches:
            used.add(min(matches,key=lambda i:event["stamp_ns"]-log_calls[i][0]))
            event["kind"] = "local"
        else:
            event["kind"] = "global"
    if len(used) != len(log_calls) or [e["count"] for e in events] != list(range(1,len(events)+1)):
        raise RuntimeError("Replanning events do not match the simulator log")


def swept_clearance(p, scene):
    lengths = np.linalg.norm(np.diff(p,axis=0),axis=1)
    step = .005
    while True:
        divisions = np.maximum(1,np.ceil(lengths/step).astype(int))
        extras = [p[i] + (np.arange(1,n)/n)[:,None]*(p[i+1]-p[i])
                  for i,n in enumerate(divisions) if n > 1]
        pts = np.vstack([p,*extras])
        clearance = float(distance_to_solids(pts,scene).min()-RADIUS)
        bound = clearance - float((lengths/divisions).max(initial=0))/2
        if clearance <= 0 or bound > 0:
            return clearance,bound,int(clearance <= 0)
        step /= 4
        if step < .00005:
            raise RuntimeError("Swept-clearance sign unresolved at evaluation tolerance")


def process(run, cfg, capture):
    stamps = np.array([r[0] for r in capture.truth],dtype=np.int64)
    assert len(stamps) > 100 and np.all(np.diff(stamps)>0), "Truth clock/coverage invalid"
    step = np.array([r[1] for r in capture.truth],dtype=int)
    state = np.array([r[2:] for r in capture.truth])
    assert step[0]==0 and np.all(np.diff(step)>=0)
    expected = np.array([*cfg["start"],cfg["initial_vx_m_s"],0,0])
    assert np.allclose(state[step==0],expected,rtol=0,atol=1e-12), "Initial state differs"
    assert capture.map_hashes == {cfg["map_sha256"]}, "Wrong map or multiple maps"
    # A trailing truth/observed message can arrive after stop; interior drops fail.
    matched = [i for i,t in enumerate(stamps) if int(t) in capture.obs]
    assert matched and matched == list(range(matched[-1]+1)) and len(stamps)-len(matched)<=1, "Unpaired odometry"
    n = len(matched)
    observed = np.array([capture.obs[int(t)] for t in stamps[:n]])
    noise = np.loadtxt(run.parent.parent/"scene"/"noise.txt")
    error = float(np.abs((observed-state[:n])-noise[step[:n]//10]).max())
    assert error < 1e-12, "Observation perturbation mismatch"
    log = (run/"simulation.log").read_text(errors="replace").split("[section54] intentional shutdown",1)[0]
    classify_events(capture.events,log)
    local = [e for e in capture.events if e["kind"]=="local"]
    durations = np.array([e["time_ms"] for e in local])
    start = max(0,int(np.searchsorted(stamps,capture.commands[0][0]))-1) if capture.commands else 0
    first = np.flatnonzero(np.linalg.norm(state[start:,:3]-GOAL,axis=1)<=.2)
    arrived = bool(len(first))
    end = start+int(first[0]) if arrived else len(stamps)-1
    t = (stamps-stamps[start])*1e-9
    p = state[:,:3]
    scene = json.loads((run.parent.parent/"scene"/"geometry.json").read_text())
    clearance,bound,collision = swept_clearance(p[start:],scene)
    window = slice(start,end+1)
    pm = p[window]
    tv = t[window]
    speed = np.linalg.norm(state[:,3:],axis=1)
    d = distance_to_solids(pm,scene)
    lat = np.abs(pm[:, 1])
    def _series(key):
        return np.array([v for _, v in capture.adaptive.get(key, [])])
    def _stat(key, fn):
        a = _series(key)
        return fn(a) if a.size else None
    ds_peak = _stat("safe_distance", np.max)
    ds_range = _stat("safe_distance", lambda a: a.max() - a.min())
    lam_peak = _stat("collision_weight", np.max)
    lam_range = _stat("collision_weight", lambda a: a.max() - a.min())
    vmax_min = _stat("max_velocity", np.min)
    rbar_peak = _stat("smoothed_risk", np.max)
    gate_min = _stat("contraction_gate", np.min)
    margin = d-RADIUS-.3*speed[window]-speed[window]**2/4
    path = float(np.linalg.norm(np.diff(pm,axis=0),axis=1).sum())
    chord = float(np.linalg.norm(pm[-1]-pm[0]))
    low = (np.abs(pm[:,0]) <= cfg["contraction_m"]) & (speed[window]<.2)
    idx = np.flatnonzero(np.diff(np.r_[False,low,False])).reshape(-1,2)
    longest = max((tv[j-1]-tv[i] for i,j in idx),default=0.)
    crossing = np.flatnonzero((pm[:-1,0]<0)&(pm[1:,0]>=0))
    through = False
    for i in crossing:
        a,b = pm[i:i+2]
        q = a + (-a[0]/(b[0]-a[0]))*(b-a)
        if abs(q[1])<cfg["width_m"]/2 and 0<=q[2]<=1.9 and np.any(pm[i+1:,0]>=cfg["contraction_m"]):
            through = True
    tracking = None
    if len(capture.commands)>1:
        ct = (np.array([c[0] for c in capture.commands],dtype=np.int64)-stamps[start])*1e-9
        cp = np.array([c[1:] for c in capture.commands])
        assert np.all(np.diff(ct)>0), "Reference clock not increasing"
        good = (tv>=ct[0]) & (tv<=ct[-1])
        if np.count_nonzero(good)>1:
            ref = np.column_stack([np.interp(tv[good],ct,cp[:,i]) for i in range(3)])
            tracking = float(np.sqrt(np.trapz(np.sum((pm[good]-ref)**2,axis=1),tv[good])/np.ptp(tv[good])))
    row = dict(arrived=int(arrived),settled=int(capture.settled),collision=collision,
               success=int(capture.settled and not collision),passed_passage=int(through),
               stalled_in_passage=int(longest>=1),passage_low_speed_s=float(longest),
               traversal_s=float(tv[-1]) if arrived else None,observed_s=float(tv[-1]),
               path_m=path,excess_path_m=max(0,path-chord),
               mean_speed_m_s=float(np.trapz(speed[window],tv)/np.ptp(tv)),
               peak_speed_m_s=float(speed[window].max()),
               min_body_clearance_m=clearance,clearance_lower_bound_m=bound,
               lateral_offset_mean_m=float(lat.mean()),lateral_offset_peak_m=float(lat.max()),
               d_safe_peak_m=ds_peak,d_safe_range_m=ds_range,lambda_peak=lam_peak,
               lambda_range=lam_range,v_max_min_m_s=vmax_min,smoothed_risk_peak=rbar_peak,
               contraction_gate_min=gate_min,
               body_negative_margin_fraction=float(np.trapz((margin<0).astype(float),tv)/np.ptp(tv)),
               body_deficit_m_s=float(np.trapz(np.maximum(-margin,0),tv)),tracking_rmse_m=tracking,
               local_calls=len(local),local_failures=sum(e["success"]==0 for e in local),
               local_mean_ms=float(durations.mean()) if len(local) else None,
               local_p50_ms=float(np.quantile(durations,.5)) if len(local) else None,
               local_p95_ms=float(np.quantile(durations,.95)) if len(local) else None,
               local_p99_ms=float(np.quantile(durations,.99)) if len(local) else None,
               initial_vx_measured_m_s=float(state[0,3]),noise_max_error=error,
               truth_samples=len(stamps),paired_samples=n,truth_max_gap_s=float(np.diff(stamps).max()*1e-9),
               map_sha256=cfg["map_sha256"],noise_sha256=cfg["noise_sha256"])
    np.savez_compressed(run/"raw.npz",stamp_ns=stamps,step=step,truth=state,
                        observed=observed,commands=capture.commands)
    ad = {k: np.asarray(v) if len(v) else np.zeros((0, 2)) for k, v in capture.adaptive.items()}
    np.savez_compressed(run/"adaptive.npz",
                        **{k: ad[k][:, 0].astype(np.int64) for k in ad},
                        **{k + "_v": ad[k][:, 1].astype(float) for k in ad})
    write_csv(run/"replans.csv",capture.events,["stamp_ns","count","time_ms","success","kind"])
    json_write(run/"result.json",row)
    return row


def configure_launch(directory):
    directory.mkdir(exist_ok=True)
    frozen = BASE/"original_launch"
    shutil.copy2(frozen/"advanced_param.launch.py",directory)
    sim = (frozen/"simulator.launch.py").read_text()
    old = "{'rate/odom': 100.0},"
    assert sim.count(old)==1
    sim = sim.replace(old,old+"\n                    {'experiment/initial_vx': float(os.environ['S54_VX'])},"
                      "\n                    {'experiment/noise_file': os.environ['S54_NOISE']},")
    (directory/"simulator.launch.py").write_text(sim)


def fly(block, method, launch, binaries):
    scene = block/"scene"
    cfg = json.loads((scene/"scenario.json").read_text())
    for name in ("map","noise","geometry"):
        file = {"map":"map.xyz","noise":"noise.txt","geometry":"geometry.json"}[name]
        assert sha(scene/file)==cfg[f"{name}_sha256"], f"Changed scene file: {name}"
    assert all(sha(Path(p))==h for p,h in binaries.items()), "Runtime binary changed"
    run = block/"runs"/method
    run.mkdir(parents=True,exist_ok=False)
    env = dict(os.environ, SECTION53_ALGORITHM=method, SECTION53_FROZEN_LAUNCH=str(launch),
               S54_VX=str(cfg["initial_vx_m_s"]),S54_NOISE=str(scene/"noise.txt"))
    rclpy.init()
    capture = Capture(scene)
    sim = None
    try:
        with (run/"simulation.log").open("w") as log:
            sim = subprocess.Popen(["ros2","launch",str(ROOT/"scripts/section53_batch10/controlled.launch.py")],
                                   env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            start = time.monotonic()
            dumped = False
            while time.monotonic()-start < 80 and not capture.settled:
                if sim.poll() is not None:
                    raise RuntimeError(f"ROS launch exited: {sim.returncode}")
                rclpy.spin_once(capture,timeout_sec=.02)
                if not dumped and time.monotonic()-start > 9:
                    nodes = NODES + (["adaptive_planner"] if method!=METHODS[0] else [])
                    params = run/"parameters"
                    params.mkdir()
                    with (run/"parameters.log").open("w") as plog:
                        audit = subprocess.Popen(["/usr/bin/python3",str(ROOT/"scripts/section53_batch10/dump_parameters.py"),str(params),*nodes],
                                                 env=env,stdout=plog,stderr=subprocess.STDOUT)
                        deadline=time.monotonic()+20
                        while audit.poll() is None and time.monotonic()<deadline:
                            rclpy.spin_once(capture,timeout_sec=.02)
                        if audit.poll() is None:
                            audit.terminate()
                            audit.wait()
                        assert audit.returncode==0, "Parameter audit failed"
                    dumped = True
            log.write("[section54] intentional shutdown\n")
            log.flush()
            os.killpg(sim.pid,signal.SIGINT)
            until=time.monotonic()+.4
            while time.monotonic()<until:
                rclpy.spin_once(capture,timeout_sec=.01)
            try:
                sim.wait(timeout=8)
            except subprocess.TimeoutExpired:
                # Capture has ended. A stuck teardown must not discard a flight.
                os.killpg(sim.pid,signal.SIGTERM)
                try:
                    sim.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    os.killpg(sim.pid,signal.SIGKILL)
                    sim.wait()
        assert dumped, "Run ended before parameter audit"
        log_text = (run/"simulation.log").read_text(errors="replace")
        assert "process has died" not in log_text.split("[section54] intentional shutdown",1)[0], "ROS node crashed before shutdown"
        row = process(run,cfg,capture)
        print("END",block.name,method,json.dumps(row),flush=True)
        return row
    finally:
        if sim is not None and sim.poll() is None:
            os.killpg(sim.pid,signal.SIGTERM)
            sim.wait(timeout=10)
        capture.destroy_node()
        rclpy.try_shutdown()


def runtime():
    binaries = json.loads((BASE/"runtime_binary_sha256.json").read_text())
    overlay = Path("/tmp/section54_overlay/install/so3_quadrotor_simulator/lib/so3_quadrotor_simulator/so3_quadrotor_simulator")
    binaries[str(overlay)] = sha(overlay)
    assert all(sha(Path(p))==h for p,h in binaries.items())
    return binaries


def sweep(launch, binaries, densities, gaps, route_margin, reps):
    """Pilot scan over (density, gap). reps=1 coarse, reps>1 refine on shortlisted cells."""
    rows = []
    base = 542000
    for di, density in enumerate(densities):
        for gi, gap in enumerate(gaps):
            for rep in range(reps):
                seed = base + (di * len(gaps) + gi) * 100 + rep
                block = OUT / "sweep" / f"d{density}_g{gap}_r{rep}"
                config = dict(width_m=1.3, contraction_m=5.5, initial_vx_m_s=0.5, noise="low",
                              forest_gap_m=gap, route_margin_m=route_margin)
                try:
                    cfg = materialize(block / "scene", config, seed, densities=(density,))
                except RuntimeError as exc:
                    print(f"INFEASIBLE d{density} g{gap} r{rep}: {exc}", flush=True)
                    continue
                for method in METHODS:
                    print("START", block.name, method, flush=True)
                    result = fly(block, method, launch, binaries)
                    rows.append(dict(density=density, gap=gap, rep=rep, method=method, **result))
                    print("END", block.name, method, json.dumps(result), flush=True)
    (OUT / "sweep").mkdir(parents=True, exist_ok=True)
    if rows:
        write_csv(OUT / "sweep" / "sweep_results.csv", rows)
    _sweep_summary(rows)
    return rows


def _sweep_summary(rows):
    print("\n=== SWEEP SUMMARY (collision/success, mean min_body_clearance m) ===")
    grouped = {}
    for r in rows:
        grouped.setdefault((r["density"], r["gap"]), {}).setdefault(r["method"], []).append(r)
    header = " | ".join(f"{m:>28}" for m in METHODS)
    print(f"{'density':>7} {'gap':>5} | {header}")
    for (density, gap) in sorted(grouped):
        by_method = grouped[(density, gap)]
        cells = []
        for m in METHODS:
            rs = by_method.get(m, [])
            if not rs:
                cells.append(f"{'--':>28}")
                continue
            coll = sum(int(r["collision"]) for r in rs)
            succ = sum(int(r["success"]) for r in rs)
            clr = float(np.mean([r["min_body_clearance_m"] for r in rs]))
            cells.append(f"{coll}/{succ}  {clr:+.3f}".rjust(28))
        print(f"{density:>7} {gap:>5.1f} | " + " | ".join(cells))
    bad = [r for r in rows if r["local_calls"] == 0]
    if bad:
        print(f"WARNING: {len(bad)} flights had zero local-planning calls")


def freeze(density, gap, route_margin):
    """Balance discrete factors, stratify the two continuous factors independently."""
    rng = random.Random(540020)
    n = 20
    widths = [round(1+.6*(i+rng.random())/n,3) for i in range(n)]
    lengths = [round(4+3*(i+rng.random())/n,3) for i in range(n)]
    rng.shuffle(widths)
    rng.shuffle(lengths)
    combinations = [(v,noise) for v in [0.,.5,1.] for noise in ["none","low","medium"]]*2
    combinations += [(0.,"none"),(.5,"low")]
    rng.shuffle(combinations)
    matrix = []
    for i,(width,length,(vx,noise)) in enumerate(zip(widths,lengths,combinations),1):
        block = OUT/"formal"/f"scene_{i:02d}"
        order = rng.sample(METHODS,len(METHODS))
        config = dict(scene_id=i,width_m=width,contraction_m=length,initial_vx_m_s=vx,noise=noise,
                      order=order,forest_gap_m=gap,route_margin_m=route_margin)
        print("FREEZE",block.name,flush=True)
        matrix.append(materialize(block/"scene",config,542000+1000*i,densities=(density,)))
    json_write(OUT/"formal"/"frozen_matrix.json",matrix)
    return matrix


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("phase", choices=["sweep", "formal"])
    parser.add_argument("--density", type=int, default=None)
    parser.add_argument("--gap", type=float, default=None)
    parser.add_argument("--route-margin", type=float, default=0.1)
    parser.add_argument("--densities", type=str, default="15,20,25")
    parser.add_argument("--gaps", type=str, default="0.8,1.0,1.2")
    parser.add_argument("--reps", type=int, default=1)
    args = parser.parse_args()
    OUT.mkdir(exist_ok=True)
    launch = OUT / "launch"
    configure_launch(launch)
    binaries = runtime()
    if args.phase == "sweep":
        densities = [int(x) for x in args.densities.split(",")]
        gaps = [float(x) for x in args.gaps.split(",")]
        sweep(launch, binaries, densities, gaps, args.route_margin, args.reps)
        return
    assert args.density is not None and args.gap is not None, "formal needs --density and --gap (locked)"
    matrix_file = OUT / "formal" / "frozen_matrix.json"
    matrix = json.loads(matrix_file.read_text()) if matrix_file.exists() else freeze(args.density, args.gap, args.route_margin)
    assert len(matrix) == 20 and all(
        c["forest_density"] == args.density and c["forest_gap_m"] == args.gap for c in matrix), (
        "frozen_matrix.json params differ from --density/--gap; "
        "delete experiments/section54_v2/formal to re-freeze")
    all_rows = []
    for config in matrix:
        block = OUT / "formal" / f"scene_{config['scene_id']:02d}"
        for method in config["order"]:
            run = block / "runs" / method
            print("START", block.name, method, flush=True)
            result = json.loads((run / "result.json").read_text()) if (run / "result.json").exists() else fly(block, method, launch, binaries)
            all_rows.append(dict(scene_id=config["scene_id"], method=method, **result))
            write_csv(OUT / "formal" / "flight_results.csv", all_rows)
    assert len(all_rows) == 60
    (OUT / "formal" / "COMPLETE").write_text("60 audited attempts")


if __name__=="__main__":
    main()
