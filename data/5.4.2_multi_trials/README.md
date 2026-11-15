# Section 5.4.2 — Fixed-throat forest experiment

The experiment uses a 1.4 m throat and 30 forest layouts. `baseline/`, `speed_only/`, and `adaptive/` correspond to Fixed EGO, Speed-only, and Proposed. Each method directory contains `seed01` through `seed30`. The per-seed `*_timeseries.csv`, `*_replan_events.csv`, and `run_summary.csv` are original records. `_batch_config.txt` documents the configuration in Table 6. `trials_wide.csv` contains 90 per-flight rows. `statistics_by_method.csv` retains the acquisition-era aggregate. `fixed_throat_statistics.json` contains the manuscript analysis used for Table 7.

## Reproduce Table 7 and Figure 17

Run the block from a Python session with this section directory as the working directory. The updated analysis uses all 30 seeds and calls the original plotting function. Its recorded flight time ends at the first crossing of `x = 14 m`. It evaluates negative perceived center-margin exposure and uses paired layouts for comparisons.

```python
from pathlib import Path
import sys

root = Path.cwd()
sys.path.insert(0, str(root / 'analysis'))
import analyze_fixed_throat as analysis

analysis.original.SOURCE = root
analysis.OUT = root / 'reproduced'
analysis.main()
```

The resulting JSON and `reproduced/figures/section54_forest_seed_study.png` correspond to Table 7 and [Figure 17](../paper_figures/figure_17.png). Check that the negative perceived center-margin means round to 61.93%, 56.43%, and 44.40% in the method order above. `position_domain_analysis/` retains side analyses for the Proposed and Speed-only flights on seed 05.

The [historical WSL scripts](../historical_wsl/README.md) record the initial seed 1–10 run and the later seed 11–30 extension.
