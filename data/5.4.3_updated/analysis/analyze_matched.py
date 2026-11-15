"""Rebuild the Section 5.4.3 evidence from the expanded matched archive."""

import csv
import json
from pathlib import Path
import sys

import numpy as np
from scipy.stats import binomtest, rankdata


HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent / "ego-planner-swarm"
OUT = HERE / "section543_updated"
sys.path.insert(0, str(REPO / "scripts/section54"))
import audit_manuscript as original

METHODS = original.METHODS
CONTINUOUS = (
    ("min_body_clearance_m", 1),
    ("traversal_s", 1),
    ("path_m", 1),
    ("tracking_rmse_m", 1),
    ("body_negative_margin_fraction", 100),
)
BINARY = ("collision", "passage_success")
ZONES = ("front forest", "passage", "rear forest")


def signed_rank(differences):
    """Exact two-sided sign permutation on the nonzero averaged ranks."""
    nonzero = differences[differences != 0]
    if not len(nonzero):
        return 1.0
    ranks = np.rint(2 * rankdata(np.abs(nonzero))).astype(int)
    total = ranks.sum()
    observed = ranks[nonzero > 0].sum()
    counts = np.zeros(total + 1, dtype=np.int64)
    counts[0] = 1
    for rank in ranks:
        counts[rank:] += counts[:-rank].copy()
    tails = np.abs(2 * np.arange(total + 1) - total) >= abs(2 * observed - total)
    return float(counts[tails].sum() / (2 ** len(nonzero)))


def analyse(index, ids, seed):
    rng = np.random.default_rng(seed)
    samples = rng.integers(0, len(ids), size=(20_000, len(ids)))
    tests = {}
    for comparator in METHODS[:2]:
        tests[comparator] = {}
        for key, scale in (*((key, 100) for key in BINARY), *CONTINUOUS):
            delta = np.round(
                [float(index[i, METHODS[-1]][key]) - float(index[i, comparator][key])
                 for i in ids], decimals=12
            ) * scale
            gain = int(np.count_nonzero(delta > 0))
            loss = int(np.count_nonzero(delta < 0))
            if key in BINARY:
                p = float(binomtest(gain, gain + loss).pvalue) if gain + loss else 1.0
                statistic = np.mean
            else:
                p = signed_rank(delta)
                statistic = np.median
            interval = np.quantile(statistic(delta[samples], axis=1), [.025, .975])
            tests[comparator][key] = dict(
                pairs=len(ids), positive=gain, negative=loss, raw_p=p,
                mean=float(delta.mean()), effect=float(statistic(delta)),
                ci=interval.tolist(), quartiles=np.quantile(delta, [.25, .75]).tolist(),
            )
    family = sorted((entry for outcomes in tests.values() for entry in outcomes.values()),
                    key=lambda entry: entry["raw_p"])
    previous = 0.
    for remaining, entry in zip(range(len(family), 0, -1), family):
        previous = min(1., max(previous, remaining * entry["raw_p"]))
        entry["holm_p"] = previous
    return tests


def main():
    with (original.FORMAL / "flight_results.csv").open(encoding="utf-8-sig", newline="") as f:
        rows = list(csv.DictReader(f))
    matrix = json.loads((original.FORMAL / "frozen_matrix.json").read_text(encoding="utf-8"))
    configs = {c["scene_id"]: c for c in matrix}
    index = {(int(row["scene_id"]), row["method"]): row for row in rows}
    assert len(index) == len(rows) == 150 and set(index) == {
        (i, method) for i in range(1, 51) for method in METHODS
    }
    excluded = {i for (i, _), row in index.items() if float(row["truth_max_gap_s"]) >= 1}
    assert excluded == {11}
    ids = sorted(configs.keys() - excluded)
    primary = [row for row in rows if int(row["scene_id"]) in ids]
    assert len(ids) == 49 and len(primary) == 147
    assert all(row["arrived"] == row["passed_passage"] == "1" for row in primary)
    assert all(row["passage_success"] == str(
        int(row["settled"] == "1" and row["collision"] == "0")
    ) for row in primary)
    for (i, _), row in index.items():
        for key in ("map_sha256", "noise_sha256"):
            assert row[key] == configs[i][key]

    overlap = {method: dict.fromkeys(ZONES, 0) for method in METHODS}
    for i in ids:
        for method in METHODS:
            block = original.FORMAL / f"scene_{i:02d}"
            geo = json.loads((block / "scene/geometry.json").read_text(encoding="utf-8"))
            truth = original.flight_truth(block / "runs" / method)
            x = truth[original.distance_to_solids(truth[:, :3], geo) <= original.RADIUS, 0]
            length = configs[i]["contraction_m"]
            counts = (np.any(x < -length), np.any(np.abs(x) <= length), np.any(x > length))
            for zone, present in zip(ZONES, counts):
                overlap[method][zone] += int(present)

    workloads = {}
    for method in METHODS[:2]:
        changes = [int(index[i, METHODS[-1]]["local_calls"]) - int(index[i, method]["local_calls"])
                   for i in ids]
        workloads[method] = float(np.median(changes))
    report = dict(
        scene_count=len(configs), excluded_scene_ids=sorted(excluded), valid_scenes=ids,
        bootstrap_seed=5432026, bootstrap_resamples=20_000,
        descriptive=original.summary(primary), paired=analyse(index, ids, 5432026),
        new_batch=analyse(index, [i for i in ids if i >= 21], 5432027),
        sampled_overlap_by_zone=overlap, median_paired_local_call_change=workloads,
        narrowest=sorted(((c["width_m"], i) for i, c in configs.items()))[:4],
    )
    figures = OUT / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    (OUT / "matched_statistics.json").write_text(json.dumps(report, indent=2) + "\n",
                                                encoding="utf-8")
    original.OUT = OUT
    original.draw(primary, matrix)


if __name__ == "__main__":
    main()
