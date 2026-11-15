# Section 5.3 — Controlled flight comparison

The final tables use 100 flight attempts, with five configurations, two throat widths, and ten repeats per condition. The widths are 1.0 m and 1.4 m. The flight-level CSV contains 100 throat crossings, 97 strict completions, and zero geometric-envelope collisions.

## Tables 2–4

| Material | Source |
| --- | --- |
| Table 2, method configuration | `config/experiment_matrix.json` and the archived run/ROS parameter snapshots under `config/` |
| Tables 3–4, final summarized results | `paper_summary.csv`, one row per method–width condition |
| Tables 3–4, flight-level inputs | `paper_flight_results.csv`, one row per attempt |
| Evaluation formulas and scope | `config/analysis_plan_frozen.json`, including the common braking rule and statistics |

The five methods are `fixed_ego_swarm` (Fixed EGO), `speed_only` (adaptive speed only), `coupled_legacy` (static braking offset), `gate_off` (contraction gate disabled), and `risk_decoupled` (Proposed). Traversal and braking metrics in the manuscript summary use strict completions. The planning-time statistics use every attempt. `body_margin_negative_fraction` is a proportion, with percent values obtained by multiplying by 100. The deficit field `body_braking_deficit_m_s` is integrated in m·s. Scripts and configuration metadata from the live batch are archived in `historical_runner/`.

## Regenerate the summary and Figures 12–13

Use the offline environment in the root README and start Python from this section directory. The recipe reuses the original Figure 12 source block and the original Table 3–4 and Figure 13 routines. Full execution of `plots.py` also calculates unrelated position plots from the historical per-sample ROS logs. The documented call selects just its existing timing-distribution block so that the deposited event CSV is sufficient. The original code and figure assets are unchanged.

```python
import inspect
from pathlib import Path
import shutil
import sys
from textwrap import dedent

root = Path.cwd()
output = root / 'reproduced'
output.mkdir(exist_ok=True)
(output / 'figures').mkdir(exist_ok=True)
sys.path.insert(0, str(root / 'analysis'))
import plots
import paper_final

# Figure 12 uses local_replan events grouped by flight and a 99.5th percentile display limit.
shutil.copy2(root / 'analysis' / 'all_replan_events.csv', output)
source = inspect.getsource(plots.main)
first = source.index('    plt.rcParams.update')
last = source.index('    fig,axes=', first)
timing = source.index('    fig,axes=plt.subplots(1,2,figsize=(9,3.5)')
end = source.index("    print('Four research", timing)
scope = dict(vars(plots), OUT=output, out=output / 'figures')
exec(dedent(source[first:last]), scope)
exec(dedent(source[timing:end]), scope)

# Tables 3–4 and Figure 13 use the audited flight-level records already in the release.
flights = paper_final.read_csv(root / 'paper_flight_results.csv')
for flight in flights:
    for key in ('passage_completed', 'passage_stall_0p2mps_1s', 'local_calls_return_failed'):
        flight[key] = int(flight[key])
source = inspect.getsource(paper_final.main)
first = source.index('    groups = {}')
end = source.index('    save_figures(groups)', first)
scope = dict(vars(paper_final), flights=flights, DELIVERY=output)
exec(dedent(source[first:end]), scope)
paper_final.DELIVERY = output
paper_final.SOURCE = root / 'analysis' / 'archived_inputs'
paper_final.save_figures(scope['groups'])
```

Figure 12 regenerated from the events is `reproduced/figures/fig53_replanning_ecdf.png`. `save_figures` also restores the archived manuscript timing PNG to its submission filename. Figure 13 is `reproduced/figures/section53_pareto.png` and the recalculated table is `reproduced/summary.csv`. `analysis/replanning_ecdf_by_flight.csv` contains the saved curve values for cross-checking. The exact manuscript assets are [Figure 12](../paper_figures/figure_12.png) and [Figure 13](../paper_figures/figure_13.png).

## Historical simulation setup

The archived `historical_runner/run_batch.sh` and `batch.py` document the original overlay and output layout. The runner used a locally built Section 5.3 overlay and emitted per-flight bag, telemetry, and parameter snapshots. The full underlying ROS bags and raw per-sample logs are in the original working archive. The deposited rows and timing events support offline figure and table reproduction. A fresh simulation run requires building an equivalent overlay and setting the paths documented in the archived launcher. Live timing can change with process scheduling and hardware.
