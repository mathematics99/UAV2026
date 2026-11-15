# -*- coding: utf-8 -*-
"""Paired (same-seed) comparison of adaptive vs baseline / speed_only.
Reads trials_wide.csv, computes per-seed differences and sign-test +
Wilcoxon signed-rank p-values. Temporary analysis helper."""
import math
import numpy as np
import pandas as pd

CSV = r"F:\无人机暑假开发\看实验数据\data\复合环境\multi_trials\trials_wide.csv"

# direction: +1 = larger is better (safer/faster-success), -1 = smaller is better
METRICS = {
    "braking_margin_M_min_m": +1,
    "braking_margin_M_p5_m": +1,
    "min_clearance_m": +1,
    "min_safety_distance_m": +1,
    "braking_margin_M_negative_time_ratio": -1,
    "mean_Rb": -1,
    "Rb_saturation_ratio": -1,
    "tracking_rmse_m": -1,
    "travel_time_s": -1,
    "mean_speed_m_s": +1,
    "planning_time_p95_ms": -1,
}


def sign_test_p(k, n):
    """Two-sided exact binomial sign test."""
    if n == 0:
        return float("nan")
    tail = sum(math.comb(n, i) for i in range(k, n + 1))
    return min(1.0, 2.0 * tail * 0.5 ** n)


def wilcoxon_p(d):
    d = np.asarray([x for x in d if x != 0], dtype=float)
    n = len(d)
    if n == 0:
        return float("nan")
    order = np.argsort(np.abs(d))
    ranks = np.empty(n, dtype=float)
    i = 0
    while i < n:
        j = i
        while j < n and abs(d[order[j]]) == abs(d[order[i]]):
            j += 1
        ranks[order[i:j]] = (i + 1 + j) / 2.0  # average rank for ties
        i = j
    W = ranks[d > 0].sum()
    Wmin = min(W, n * (n + 1) / 2.0 - W)
    mu = n * (n + 1) / 4.0
    sigma = math.sqrt(n * (n + 1) * (2 * n + 1) / 24.0)
    z = (Wmin - mu) / sigma if sigma > 0 else 0.0
    return 1.0 - math.erf(abs(z) / math.sqrt(2.0))


df = pd.read_csv(CSV)
piv = {m: df.pivot(index="seed", columns="method", values=m) for m in METRICS}

print("Paired (same-seed) comparison, reference = adaptive")
print(f"{'metric':38s} {'vs':10s} {'n':>2s} {'wins':>6s} {'meanDiff':>9s} {'stdDiff':>8s} {'sign_p':>7s} {'wilcox_p':>8s}")
for metric, better in METRICS.items():
    p = piv[metric]
    for b in ("baseline", "speed_only"):
        d = ((p["adaptive"] - p[b]) * better).dropna()
        n = len(d)
        k = int((d > 0).sum())
        print(
            f"{metric:38s} {b:10s} {n:2d} {k:>3d}/{n:<3d} "
            f"{d.mean():+.4f} {d.std(ddof=1):.4f} "
            f"{sign_test_p(k, n):.4f} {wilcoxon_p(d):.4f}"
        )
