# Sections 5.1–5.2 — Controlled corridor experiments

| Figure | Input | Original analysis entry point |
| --- | --- | --- |
| 7 | Controlled single-contraction scene | [Archived simulator view](../paper_figures/figure_07.png) |
| 8 | `data/exp1_funnel_timeseries.csv`, with `exp3_g*_timeseries.csv` for the width comparison | `analysis/plot_contraction.py` |
| 9 | Analytic linear corridor with 1.0 m and 1.4 m throats | `analysis/plot_linear_gate.py` and `data/linear_gate/` |
| 10 | `data/exp2_v0.5_timeseries.csv` through `exp2_v2.5_timeseries.csv` | `analysis/plot_braking_sweep.py` |
| 11 | First complete forward passage in `data/exp3_g1.4_timeseries.csv` | `analysis/plot_braking_ablation.py` |

The sweep setpoints are 0.5, 1.0, 1.5, 2.0, and 2.5 m/s with a 1.4 m throat. The Figure 11 script evaluates the dynamic and static-offset formulas on the same trajectory. The mean braking risk from this computation rounds to 0.252 and 0.368. Additional width-sweep and early full-flight files are retained under `data/` with their original names. Tables 2–4 use the later [100-flight batch](../5.3_batch10/README.md).

## Re-run the original plotting functions

Run the following with the analysis environment active and this directory as the working directory. Runtime path bindings connect the archived functions to this checkout while preserving the original plotting files. The Python block can be pasted into a Python session. The speed sweep creates its default output directory on import, so use a scratch copy if the checkout is read-only.

```python
from pathlib import Path
import runpy
import sys

root = Path.cwd()
out = root / 'reproduced'
out.mkdir(exist_ok=True)
sys.path.insert(0, str(root / 'analysis'))
import plot_contraction as contraction
import plot_braking_ablation as ablation
import plot_braking_sweep as sweep

contraction.DATA_DIR = root / 'data'
contraction.make_figure(outdir=out)
ablation.DATA_FILE = root / 'data' / 'exp3_g1.4_timeseries.csv'
ablation.make_figure(outdir=out)
sweep.OUT_DIR = out
sweep.FILES = {speed: root / 'data' / f'exp2_v{speed:.1f}_timeseries.csv'
               for speed in (0.5, 1.0, 1.5, 2.0, 2.5)}
data, summary = sweep.load_data()
sweep.plot_risk_distribution(data, summary)
sweep.plot_same_clearance(data, summary)
runpy.run_path(str(root / 'analysis' / 'plot_linear_gate.py'), run_name='__main__')
```

The linear gate script writes to `analysis/section64_linear_outputs/`. The archived corridor runner, map publishers, extraction code, and summaries document the acquisition. `run_exp.sh` retains the original workspace paths and process management. The [historical WSL README](../historical_wsl/README.md) describes this environment.
