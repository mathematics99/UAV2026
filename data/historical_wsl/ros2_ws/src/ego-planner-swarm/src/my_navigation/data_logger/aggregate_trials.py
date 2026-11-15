#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Aggregate per-trial run_summary.csv into (1) a wide trials CSV and
(2) a per-method statistics CSV (mean / std / min / max / n).

Usage: python3 aggregate_trials.py <multi_trials_root> [--methods a,b,c]
"""
import argparse
import csv
import os
import statistics


def main():
    p = argparse.ArgumentParser()
    p.add_argument('root', help='multi_trials root: <root>/<method>/seedNN/run_summary.csv')
    p.add_argument('--methods', default='adaptive,baseline,speed_only')
    args = p.parse_args()
    methods = args.methods.split(',')

    trials = []          # (method, seed, metric_dict)
    metric_order = None
    for m in methods:
        mdir = os.path.join(args.root, m)
        if not os.path.isdir(mdir):
            continue
        for seed_dir in sorted(os.listdir(mdir)):
            rp = os.path.join(mdir, seed_dir, 'run_summary.csv')
            if not os.path.isfile(rp):
                continue
            d = {}
            with open(rp, encoding='utf-8') as f:
                for r in csv.DictReader(f):
                    d[r['metric']] = r['value']
            if metric_order is None:
                metric_order = list(d.keys())
            seed = seed_dir.replace('seed', '')
            trials.append((m, seed, d))

    if not trials:
        raise SystemExit('no run_summary.csv found under %s' % args.root)

    # (1) wide CSV: one row per trial
    wide_path = os.path.join(args.root, 'trials_wide.csv')
    with open(wide_path, 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.writer(f)
        w.writerow(['method', 'seed'] + metric_order)
        for m, seed, d in trials:
            w.writerow([m, seed] + [d.get(k, '') for k in metric_order])

    # (2) statistics CSV: per metric, per method -> mean/std/min/max/n
    numeric_metrics = []
    for metric in metric_order:
        vals = []
        for m, seed, d in trials:
            try:
                vals.append(float(d.get(metric, '')))
            except (ValueError, TypeError):
                pass
        if vals:
            numeric_metrics.append(metric)

    header = ['metric']
    for m in methods:
        header += [m + '_mean', m + '_std', m + '_min', m + '_max', m + '_n']
    stats_path = os.path.join(args.root, 'statistics_by_method.csv')
    with open(stats_path, 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.writer(f)
        w.writerow(header)
        for metric in numeric_metrics:
            row = [metric]
            for m in methods:
                vals = []
                for mm, seed, d in trials:
                    if mm != m:
                        continue
                    try:
                        vals.append(float(d.get(metric, '')))
                    except (ValueError, TypeError):
                        pass
                if vals:
                    row += ['%.6f' % statistics.mean(vals),
                            '%.6f' % (statistics.pstdev(vals) if len(vals) > 1 else 0.0),
                            '%.6f' % min(vals),
                            '%.6f' % max(vals),
                            str(len(vals))]
                else:
                    row += ['', '', '', '', '0']
            w.writerow(row)

    print('trials_wide -> %s (%d trials)' % (wide_path, len(trials)))
    print('statistics  -> %s (%d metrics)' % (stats_path, len(numeric_metrics)))


if __name__ == '__main__':
    main()
