#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Post-process data_logger output -> run_summary.csv + console report.

Usage: python3 analyze_csv.py <dir_with_risk_run_csvs>
"""
import argparse
import csv
import math
import os

V_HIGH = 2.0          # adaptive_planner speed/v_high
ADAPT_THRESH = 0.95   # I_adapt = 1(v_max < 0.95 * v_high)

REGIONS = ['forest_before', 'contraction', 'throat', 'expansion', 'forest_after']


def pct(vals, p):
    vals = [v for v in vals if v == v]  # drop NaN
    if not vals:
        return float('nan')
    s = sorted(vals)
    k = (len(s) - 1) * p / 100.0
    f = int(k)
    c = min(f + 1, len(s) - 1)
    return s[f] + (s[c] - s[f]) * (k - f)


def col(rows, name):
    out = []
    for r in rows:
        v = r.get(name, '')
        try:
            out.append(float(v))
        except (ValueError, TypeError):
            out.append(float('nan'))
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument('csv_dir', help='directory containing *_timeseries.csv / *_replan_events.csv')
    args = p.parse_args()

    ts_files = sorted(f for f in os.listdir(args.csv_dir) if f.endswith('_timeseries.csv'))
    rp_files = sorted(f for f in os.listdir(args.csv_dir) if f.endswith('_replan_events.csv'))
    if not ts_files:
        raise SystemExit('no *_timeseries.csv found in %s' % args.csv_dir)
    ts_path = os.path.join(args.csv_dir, ts_files[-1])
    rp_path = os.path.join(args.csv_dir, rp_files[-1]) if rp_files else None

    with open(ts_path) as f:
        rows = list(csv.DictReader(f))

    t = col(rows, 't_s')
    x = col(rows, 'x')
    y = col(rows, 'y')
    z = col(rows, 'z')
    v = col(rows, 'v')
    dmin = col(rows, 'd_min')
    Rb = col(rows, 'R_b')
    Rg = col(rows, 'R_g')
    Rc = col(rows, 'R_c')
    vmax = col(rows, 'v_max')
    region = [r.get('scene_region', '') for r in rows]

    # --- global motion metrics ---
    def firstlast(a):
        a = [v for v in a if v == v]
        return (a[0], a[-1]) if a else (float('nan'), float('nan'))

    t0, t1 = firstlast(t)
    travel_time = t1 - t0
    path_length = 0.0
    for i in range(1, len(rows)):
        dx = x[i] - x[i - 1]
        dy = y[i] - y[i - 1]
        dz = z[i] - z[i - 1]
        if dx == dx and dy == dy and dz == dz:
            path_length += math.sqrt(dx * dx + dy * dy + dz * dz)

    speed = [vv for vv in v if vv == vv]
    mean_speed = sum(speed) / len(speed) if speed else float('nan')
    max_speed = max(speed) if speed else float('nan')
    min_clearance = min((d for d in dmin if d == d), default=float('nan'))

    # --- risk metrics ---
    rb = [r for r in Rb if r == r]
    rg = [r for r in Rg if r == r]
    rc = [r for r in Rc if r == r]
    mean_Rb = sum(rb) / len(rb) if rb else float('nan')
    p95_Rb = pct(Rb, 95)
    max_Rb = max(rb) if rb else float('nan')
    rb_sat = sum(1 for r in rb if r > 0.8) / len(rb) if rb else float('nan')
    mean_Rg = sum(rg) / len(rg) if rg else float('nan')
    p95_Rg = pct(Rg, 95)
    max_Rg = max(rg) if rg else float('nan')
    max_Rc = max(rc) if rc else float('nan')

    # --- adaptive / planning metrics ---
    vm = [vv for vv in vmax if vv == vv]
    adapt_active = sum(1 for vv in vm if vv < ADAPT_THRESH * V_HIGH) / len(vm) if vm else float('nan')

    replan_count = 0
    plan_times = []
    if rp_path:
        with open(rp_path) as f:
            rp_rows = list(csv.DictReader(f))
        replan_count = len(rp_rows)
        for r in rp_rows:
            try:
                plan_times.append(float(r['planning_time_ms']))
            except (ValueError, KeyError, TypeError):
                pass
    mean_pt = sum(plan_times) / len(plan_times) if plan_times else float('nan')
    p95_pt = pct(plan_times, 95)
    max_pt = max(plan_times) if plan_times else float('nan')

    # --- write run_summary.csv (long format) ---
    summary = {
        'travel_time_s': travel_time,
        'path_length_m': path_length,
        'mean_speed': mean_speed,
        'max_speed': max_speed,
        'min_clearance_m': min_clearance,
        'mean_Rb': mean_Rb,
        'p95_Rb': p95_Rb,
        'max_Rb': max_Rb,
        'Rb_saturation_ratio': rb_sat,
        'mean_Rg': mean_Rg,
        'p95_Rg': p95_Rg,
        'max_Rg': max_Rg,
        'max_Rc': max_Rc,
        'adaptive_active_ratio': adapt_active,
        'replan_count': replan_count,
        'mean_planning_time_ms': mean_pt,
        'p95_planning_time_ms': p95_pt,
        'max_planning_time_ms': max_pt,
    }
    out_path = os.path.join(args.csv_dir, 'run_summary.csv')
    with open(out_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['metric', 'value'])
        for k, val in summary.items():
            w.writerow([k, '' if val != val else ('%.6f' % val)])

    # --- console report ---
    def fmt(v):
        return '  n/a' if (v != v or v is None) else '%8.3f' % v

    print('=== global summary ===')
    print('travel_time_s       %s' % fmt(travel_time))
    print('path_length_m       %s' % fmt(path_length))
    print('mean_speed          %s' % fmt(mean_speed))
    print('max_speed           %s' % fmt(max_speed))
    print('min_clearance_m     %s' % fmt(min_clearance))
    print('mean_Rb             %s' % fmt(mean_Rb))
    print('p95_Rb              %s' % fmt(p95_Rb))
    print('max_Rb              %s' % fmt(max_Rb))
    print('Rb_saturation_ratio %s' % fmt(rb_sat))
    print('mean_Rg             %s' % fmt(mean_Rg))
    print('p95_Rg              %s' % fmt(p95_Rg))
    print('max_Rg              %s' % fmt(max_Rg))
    print('max_Rc              %s' % fmt(max_Rc))
    print('adaptive_active_ratio%s' % fmt(adapt_active))
    print('replan_count        %s' % ('  n/a' if replan_count is None else '%8d' % replan_count))
    print('mean_planning_ms    %s' % fmt(mean_pt))
    print('p95_planning_ms     %s' % fmt(p95_pt))
    print('max_planning_ms     %s' % fmt(max_pt))

    # --- per-region table ---
    print()
    hdr = '%-14s %10s %10s %10s %10s %10s %10s %8s' % (
        'region', 'n_samp', 'mean_v', 'max_v', 'mean_Rb', 'mean_Rg', 'min_clear', 'adapt_act')
    print(hdr)
    print('-' * len(hdr))
    for reg in REGIONS:
        idx = [i for i, rr in enumerate(region) if rr == reg]
        if not idx:
            print('%-14s %10s' % (reg, '(empty)'))
            continue
        n = len(idx)
        mean_v = sum(v[i] for i in idx if v[i] == v[i]) / max(1, sum(1 for i in idx if v[i] == v[i]))
        max_v = max((v[i] for i in idx if v[i] == v[i]), default=float('nan'))
        mean_rb = sum(Rb[i] for i in idx if Rb[i] == Rb[i]) / max(1, sum(1 for i in idx if Rb[i] == Rb[i]))
        mean_rg = sum(Rg[i] for i in idx if Rg[i] == Rg[i]) / max(1, sum(1 for i in idx if Rg[i] == Rg[i]))
        min_cl = min((dmin[i] for i in idx if dmin[i] == dmin[i]), default=float('nan'))
        act = sum(1 for i in idx if vmax[i] == vmax[i] and vmax[i] < ADAPT_THRESH * V_HIGH) / \
            max(1, sum(1 for i in idx if vmax[i] == vmax[i]))
        print('%-14s %10d %10.3f %10.3f %10.3f %10.3f %10.3f %8.3f' % (
            reg, n, mean_v, max_v, mean_rb, mean_rg, min_cl, act))

    print()
    print('run_summary written -> %s' % out_path)


if __name__ == '__main__':
    main()
