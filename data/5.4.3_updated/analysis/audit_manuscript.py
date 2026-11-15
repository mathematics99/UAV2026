"""Audit the v2 recordings and produce the matched-scene manuscript evidence."""
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
from scipy.stats import binomtest, rankdata
from scenes import RADIUS, distance_to_solids

ROOT = Path(__file__).resolve().parents[2]
FORMAL = ROOT / "experiments/section54_v2/formal"
SWEEP = FORMAL.parent / "sweep"
OUT = ROOT / "experiments/section53_54_manuscript_20261108"
AUDIT = OUT / "audit"
METHODS = ("fixed_ego_swarm", "speed_only", "risk_decoupled")
NAMES = ("Fixed EGO", "Speed-only", "Proposed")
COLORS = ("#4477AA", "#EE6677", "#AA3377")
LEGACY_PROFILE = Path(
    "F:/无人机暑假开发/看实验数据/data/复合环境/"
    "position_domain_mechanism_bundle/position_domain_outputs/position_profile_0p1m.csv"
)
KEYS = ("passage_success", "collision", "min_body_clearance_m",
        "traversal_s", "tracking_rmse_m")


def read(path):
    with path.open(encoding="utf-8-sig", newline="") as file:
        return list(csv.DictReader(file))


def write(path, rows):
    with path.open("w", encoding="utf-8-sig", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit():
    matrix = json.loads((FORMAL / "frozen_matrix.json").read_text(encoding="utf-8"))
    assert {c["scene_id"] for c in matrix} == set(range(1, 21))
    raw_table = read(FORMAL / "flight_results.csv")
    indexed = {(int(r["scene_id"]), r["method"]): r for r in raw_table}
    assert len(indexed) == len(raw_table) == 60
    sweep_hashes = {r["map_sha256"] for r in read(SWEEP / "sweep_results.csv")}
    flagged_scenes = set()
    audits = []
    for cfg in matrix:
        i = cfg["scene_id"]
        block = FORMAL / f"scene_{i:02d}"
        assert json.loads((block/"scene/scenario.json").read_text(encoding="utf-8")) == cfg
        assert set(cfg["order"]) == set(METHODS)
        for name, filename in [("map", "map.xyz"), ("noise", "noise.txt"),
                               ("geometry", "geometry.json")]:
            assert sha(block/"scene"/filename) == cfg[name+"_sha256"]
        assert cfg["map_sha256"] not in sweep_hashes, "Sweep map reused in formal evaluation"
        noise = np.loadtxt(block/"scene/noise.txt")
        geometry = json.loads((block/"scene/geometry.json").read_text(encoding="utf-8"))
        for method in METHODS:
            row = indexed[i, method]
            run = block/"runs"/method
            result = json.loads((run/"result.json").read_text(encoding="utf-8"))
            for k,v in result.items():
                stored = row[k]
                if isinstance(v, (float, int)):
                    assert float(stored) == v, (i, method, k)
                else:
                    assert stored == ("" if v is None else v), (i, method, k)
            for k in ("map_sha256", "noise_sha256"):
                assert row[k] == cfg[k]
            with np.load(run/"raw.npz") as z:
                stamps, step, truth, observed, commands = (
                    z[k] for k in ("stamp_ns", "step", "truth", "observed", "commands"))
            assert len(stamps) == result["truth_samples"] == len(truth)
            assert len(observed) == result["paired_samples"] and len(stamps)-len(observed) <= 1
            assert np.all(np.diff(stamps)>0) and np.all(np.diff(step)>=0)
            assert step[0] == 0 and np.array_equal(
                truth[step==0], np.tile(cfg["start"]+[cfg["initial_vx_m_s"],0,0],
                                       (np.count_nonzero(step==0),1)))
            replay_error = float(np.max(abs(observed-truth[:len(observed)]-
                                            noise[step[:len(observed)]//10])))
            assert replay_error < 1e-12
            gaps = np.diff(stamps)
            idx = int(gaps.argmax())
            gap = float(gaps[idx]*1e-9)
            assert np.isclose(gap, result["truth_max_gap_s"], rtol=0, atol=1e-10)
            if gap >= 1.0:
                flagged_scenes.add(i)
            distance = np.linalg.norm(truth[:,:3]-cfg["goal"],axis=1)
            speed = np.linalg.norm(truth[:,3:],axis=1)
            settled = (step>0) & (distance<=.2) & (speed<.2)
            boundaries = np.flatnonzero(np.diff(np.r_[False,settled,False])).reshape(-1,2)
            settled_check = any(stamps[b-1]-stamps[a] >= 10**9 for a,b in boundaries)
            assert bool(result["arrived"]) == bool(np.any(distance<=.2))
            assert bool(result["settled"]) == settled_check
            start = max(0, int(np.searchsorted(stamps, commands[0,0]))-1) if len(commands) else 0
            section = np.flatnonzero((truth[start:-1,0]<0)&(truth[start+1:,0]>=0)) + start
            first_entry = np.flatnonzero(distance[start:]<=.2)
            end = start+int(first_entry[0]) if len(first_entry) else len(stamps)-1
            through = False
            for j in section[section<end]:
                a,b = truth[j:j+2,:3]
                crossing = a + (-a[0]/(b[0]-a[0]))*(b-a)
                if (abs(crossing[1])<cfg["width_m"]/2 and 0<=crossing[2]<=1.9 and
                        np.any(truth[j+1:end+1,0]>=cfg["contraction_m"])):
                    through = True
            assert bool(result["passed_passage"]) == through
            path = truth[start:,:3]
            segment_lengths = np.linalg.norm(np.diff(path,axis=0),axis=1)
            # Replay the certified 5 mm sweep on the original positions.
            divisions = np.maximum(1,np.ceil(segment_lengths/.005).astype(int))
            interpolated = [path[j] + (np.arange(1,n)/n)[:,None]*(path[j+1]-path[j])
                            for j,n in enumerate(divisions) if n>1]
            clearance = float(distance_to_solids(np.vstack([path,*interpolated]),geometry).min()-RADIUS)
            assert np.isclose(clearance,result["min_body_clearance_m"],atol=1e-8,rtol=0), (i,method,clearance)
            assert bool(result["collision"]) == (result["min_body_clearance_m"] <= 0)
            assert result["collision"] or result["clearance_lower_bound_m"] > 0
            assert int(row["passage_success"]) == int(settled_check and through and not result["collision"])
            events = [e for e in read(run/"replans.csv") if e["kind"]=="local"]
            assert len(events) == result["local_calls"]
            assert sum(e["success"]=="0" for e in events) == result["local_failures"]
            audits.append(dict(scene=i, method=method, max_source_gap_s=gap,
                               integration_step_gap=int(step[idx+1]-step[idx]),
                               noise_replay_error=replay_error,recomputed_min_clearance_m=clearance))
    assert flagged_scenes == {11}
    for a in audits:
        a["included_in_primary"] = int(a["scene"] not in flagged_scenes)
    return matrix, raw_table, flagged_scenes, audits


def numbers(rows, key):
    return np.asarray([float(r[key]) for r in rows if r[key] != ""])


def median_iqr(rows, key):
    a = numbers(rows,key)
    return np.quantile(a,[.25,.5,.75]).tolist()


def signed_rank_exact(d):
    d = np.asarray(d)
    d = d[(d != 0) & np.isfinite(d)]
    if not len(d):
        return 1.
    ranks = (rankdata(abs(d))*2).astype(int)
    threshold = abs(ranks[d>0].sum()-ranks[d<0].sum())
    counts = np.zeros(ranks.sum()+1, dtype=np.int64)
    counts[0] = 1
    for r in ranks:
        counts[r:] += counts[:-r].copy()
    return float(counts[abs(2*np.arange(len(counts))-ranks.sum())>=threshold].sum()/2**len(d))


def paired_tests(rows, set_name):
    indexed = {(r["scene_id"],r["method"]):r for r in rows}
    scenes = sorted({r["scene_id"] for r in rows},key=int)
    tests = []
    for other in METHODS[:2]:
        for key in KEYS:
            pairs = [(indexed[s,other],indexed[s,METHODS[-1]]) for s in scenes]
            if key in ("traversal_s","tracking_rmse_m"):
                pairs = [(a,b) for a,b in pairs if a["passage_success"]==b["passage_success"]=="1"]
            d = np.asarray([float(b[key])-float(a[key]) for a,b in pairs])
            positive,negative = int((d>0).sum()),int((d<0).sum())
            if not len(d):
                p = None
            elif key in ("collision","passage_success"):
                p = float(binomtest(positive,positive+negative).pvalue) if positive+negative else 1.
            else:
                p = signed_rank_exact(d)
            tests.append(dict(analysis_set=set_name,comparator=other,metric=key,pairs=len(d),
                              positive=positive,negative=negative,
                              mean_delta=float(d.mean()) if len(d) else None,
                              median_delta=float(np.median(d)) if len(d) else None,
                              raw_p=p,holm_p=None))
    high = 0.
    for i,t in enumerate(sorted((r for r in tests if r["raw_p"] is not None),key=lambda r:r["raw_p"])):
        high = max(high,min(1.,(len(tests)-i)*t["raw_p"]))
        t["holm_p"] = high
    return tests


def summary(rows):
    groups = [[r for r in rows if r["method"]==m] for m in METHODS]
    results = []
    for name,group in zip(NAMES,groups):
        counts = {key:int(numbers(group,key).sum())
                  for key in ("arrived","settled","passed_passage","collision","passage_success",
                              "stalled_in_passage","local_failures")}
        counts["flights_with_local_failures"] = int(np.count_nonzero(numbers(group,"local_failures")))
        metrics = {key:median_iqr(group,key) for key in (
            "min_body_clearance_m","traversal_s","tracking_rmse_m","path_m",
            "body_negative_margin_fraction","body_deficit_m_s","local_p95_ms",
        )}
        results.append(dict(method=name,n=len(group),counts=counts,iqr=metrics))
    return results


def flight_truth(run):
    with np.load(run/"raw.npz") as log:
        start = max(0, int(np.searchsorted(log["stamp_ns"], log["commands"][0, 0])) - 1)
        return log["truth"][start:]


def draw(primary, matrix):
    plt.rcParams.update({"font.family":"DejaVu Sans", "font.size":11,
                         "axes.spines.top":False, "axes.spines.right":False})
    fig = plt.figure(figsize=(10.4, 4.6))
    fig.subplots_adjust(left=.08, right=.985, top=.91, bottom=.16, wspace=.40, hspace=.30)
    grid = fig.add_gridspec(2, 2, width_ratios=(1.55, 1))
    speed = fig.add_subplot(grid[0, 0])
    overlap = fig.add_subplot(grid[1, 0], sharex=speed)
    paired = fig.add_subplot(grid[:, 1])
    stations = np.linspace(-1.95, 1.95, 157)
    bins = np.linspace(-2, 2, 41)
    configs = {cfg["scene_id"]:cfg for cfg in matrix}
    indexed = {(int(row["scene_id"]), row["method"]):row for row in primary}
    ids = sorted({int(row["scene_id"]) for row in primary})
    handles = []
    for i in (0, 2):
        speeds, counts = [], []
        for scene_id in ids:
            cfg = configs[scene_id]
            block = FORMAL/f"scene_{scene_id:02d}"
            geometry = json.loads((block/"scene/geometry.json").read_text(encoding="utf-8"))
            truth = flight_truth(block/"runs"/METHODS[i])
            length = cfg["contraction_m"]
            boundaries = [cfg["start"][0], -length, 0, length, cfg["goal"][0]]
            x = truth[:, 0]
            x_query = np.interp(stations, [-2, -1, 0, 1, 2], boundaries)
            right = np.searchsorted(np.maximum.accumulate(x), x_query)
            assert np.all((right > 0) & (right < len(x)))
            left = right - 1
            # Interpolate truth velocity at the first forward crossing of each station.
            fraction = (x_query-x[left])/(x[right]-x[left])
            velocity = truth[left, 3:] + fraction[:, None]*(truth[right, 3:]-truth[left, 3:])
            speeds.append(np.linalg.norm(velocity, axis=1))
            clear = distance_to_solids(truth[:, :3], geometry)-RADIUS
            sampled_overlap = x[clear <= 0]
            assert bool(len(sampled_overlap)) == (indexed[scene_id, METHODS[i]]["collision"] == "1")
            positions = np.interp(sampled_overlap, boundaries, [-2, -1, 0, 1, 2])
            counts.append(np.histogram(positions, bins)[0] > 0)
        color, style = COLORS[i], "--" if i == 0 else "-"
        low, median, high = np.quantile(speeds, [.25, .5, .75], axis=0)
        speed.fill_between(stations, low, high, color=color, alpha=.14, linewidth=0)
        speed.plot(stations, median, color=color, ls=style, lw=1.8)
        overlap.stairs(np.sum(counts, axis=0), bins, color=color, ls=style, lw=1.8,
                       zorder=4-i)
        handles.append(Line2D([], [], color=color, ls=style, lw=1.8, label=NAMES[i]))

    for ax in (speed, overlap):
        ax.axvspan(-1, 1, color="#e6edf4", alpha=.67, zorder=0)
        ax.axvline(0, color="#718395", ls=":", lw=.9, zorder=1)
        ax.set_xlim(-2, 2)
        ax.set_xticks([-2, -1, 0, 1, 2], ["Start", "Entrance", "Throat", "Exit", "Goal"])
    speed.tick_params(labelbottom=False, bottom=False)
    speed.set_ylabel("Speed (m/s)")
    overlap.set_ylabel("Scenes with overlap")
    overlap.set_ylim(bottom=-.15)
    overlap.yaxis.set_major_locator(plt.MaxNLocator(integer=True))

    for scene_id in ids:
        fixed, proposed = (indexed[scene_id, METHODS[i]] for i in (0, 2))
        delta_t = float(proposed["traversal_s"])-float(fixed["traversal_s"])
        delta_c = float(proposed["min_body_clearance_m"])-float(fixed["min_body_clearance_m"])
        gain = fixed["passage_success"] == "0" and proposed["passage_success"] == "1"
        paired.scatter(delta_t, delta_c, marker="D" if gain else "o", s=31,
                       color=COLORS[2] if gain else "#718395", alpha=.85,
                       edgecolor="white", linewidth=.45, zorder=3)
    paired.axhline(0, color="#718395", ls="--", lw=.8)
    paired.set(xlabel="Additional first-entry time (s)", ylabel="Change in minimum clearance (m)")
    for ax, title in zip((speed, overlap, paired), (
        "(a) Executed speed", "(b) Sampled geometric overlap", "(c) Matched trade-off",
    )):
        ax.set_title(title, loc="left", weight="bold", fontsize=11.5)
        ax.grid(axis="y", alpha=.22)
        ax.set_axisbelow(True)
    handles.append(Line2D([], [], marker="D", ls="none", color=COLORS[2],
                          label="New strict completion", markersize=5))
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(.5,.01),
               ncols=3, frameon=False, fontsize=9.5)
    fig.savefig(OUT/"figures/section54_matched_clearance.png",dpi=600)
    plt.close(fig)


def mechanism():
    data = read(LEGACY_PROFILE)
    x = numbers(data,"x_bin")
    fig,axes = plt.subplots(1,3,figsize=(10.4,3.1),sharex=True,layout="constrained")
    axes[0].plot(x,numbers(data,"v"),color=COLORS[0],label="Actual speed")
    axes[0].plot(x,numbers(data,"v_max"),color=COLORS[2],label="Adaptive speed cap")
    axes[1].plot(x,numbers(data,"R_c"),color="#228833",label=r"$R_c$")
    axes[1].plot(x,numbers(data,"R_b"),color=COLORS[1],label=r"$R_b$")
    axes[2].plot(x,numbers(data,"rho_contrib"),color=COLORS[0],label=r"$0.20\rho$")
    axes[2].plot(x,numbers(data,"Rb_contrib"),color=COLORS[1],label=r"$0.55R_b$")
    axes[2].plot(x,numbers(data,"Rc_contrib"),color="#228833",label=r"$0.25R_c$")
    axes[0].set_ylabel("Speed (m/s)")
    axes[1].set_ylabel("Risk signal")
    axes[2].set_ylabel("Weighted contribution")
    for ax,title,legend_columns in zip(
        axes, ("(a) Speed response", "(b) Raw risk signals", "(c) Risk contributions"), (2,2,3)
    ):
        ax.axvspan(-5,5,color="#bbbbbb",alpha=.16,zorder=0)
        ax.axvspan(-1,1,color="#999999",alpha=.2,zorder=0)
        ax.grid(alpha=.23)
        ax.set(xlabel="Longitudinal position (m)",xlim=(-15,15))
        ax.set_xticks([-15,-10,-5,0,5,10,15])
        ax.set_title(title,loc="left",weight="bold",fontsize=10.5)
        ax.legend(loc="upper center", bbox_to_anchor=(.5, -.22), ncols=legend_columns,
                  frameon=False, fontsize=8.5)
    fig.savefig(OUT/"figures/section54_mechanism.png",dpi=600)
    plt.close(fig)


def main():
    matrix,rows,flagged,audits = audit()
    primary = [r for r in rows if int(r["scene_id"]) not in flagged]
    assert len(primary)==57
    (OUT/"figures").mkdir(parents=True,exist_ok=True)
    AUDIT.mkdir(exist_ok=True)
    write(AUDIT/"section54_clock_audit.csv",audits)
    tests = paired_tests(primary,"19 valid matched scenes")
    all_outcome_tests = [t for t in paired_tests(rows,"20 recorded matched scenes")
                         if t["metric"] in ("passage_success","collision","min_body_clearance_m")]
    # Time-linked tests from the full set are invalid. Do not report their correction family.
    for t in all_outcome_tests:
        t["holm_p"] = None
    write(AUDIT/"section54_paired_tests.csv",tests+all_outcome_tests)
    info = dict(primary=summary(primary),all_records=summary(rows),tests=tests,
                matrix_sha256=sha(FORMAL/"frozen_matrix.json"),excluded_scenes=sorted(flagged),
                width_range=[min(c["width_m"] for c in matrix),max(c["width_m"] for c in matrix)],
                length_range=[min(c["contraction_m"] for c in matrix),
                              max(c["contraction_m"] for c in matrix)])
    (AUDIT/"section54_audit_summary.json").write_text(json.dumps(info,indent=2),encoding="utf-8")
    supplement = []
    for set_name, groups in (("19 valid matched scenes",info["primary"]),
                             ("20 recorded scenes; outcomes only",info["all_records"])):
        for g in groups:
            row = dict(analysis_set=set_name,method=g["method"],n=g["n"],**g["counts"])
            for key,values in g["iqr"].items():
                for quantile,value in zip(("q1","median","q3"),values):
                    row[f"{key}_{quantile}"] = value if set_name.startswith("19") else ""
            supplement.append(row)
    write(AUDIT/"section54_all_methods_summary.csv",supplement)
    draw(primary, matrix)
    mechanism()
    print(json.dumps(info,indent=2))


if __name__=="__main__":
    main()
