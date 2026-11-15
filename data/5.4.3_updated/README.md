# Section 5.4.3 — Randomized composite experiments

## Archived flight cohorts

| Cohort | Input | Manuscript use |
| --- | --- | --- |
| One run per method and layout | `archive/one_run_per_layout/flight_results.csv` and `frozen_matrix.json` | Table 8 design, Figure 18, and the original case visualizations |
| Initial flights plus repeats | `data.csv` | Values in Tables 9–10 |
| Repeat flight records | `archive/repeat_flight_results.csv` and `archive/repeated_layouts/` | The 90 repeated-flight rows included in `data.csv` |

The one-run-per-layout archive contains 50 layouts with three methods each. `archive/one_run_per_layout/` holds trajectory and event records, `result.json`, scene geometry, and scenario configurations. `archive/frozen_map_and_noise_inputs.zip` contains the original `map.xyz` and `noise.txt` for each layout, with the same relative paths. Extract it into a scratch copy of `archive/one_run_per_layout/` for use with the historical simulator. The six repeated layouts use the corresponding frozen map and noise inputs. `historical_runner/repeat_design.json` records the selected layouts, rounds, and orders. `archive/nominal_parameters/` contains the ROS parameter snapshots for the archived Proposed Scene 13 run. `historical_runner/` preserves the archived simulation and preparation code, including its original workspace references.

The results file `data.csv` combines 60 initial flights from 20 layouts with 90 flights from layouts 22, 23, 31, 32, 46, and 50, each repeated five times across three methods. Use `(scene_id, replicate)` as the pairing key. Initial flights have `replicate=0`. `included_in_analysis=0` marks the three Scene 11 flights removed after the clock audit. The effective dataset has 147 flights, 49 matched rounds, and 25 distinct layouts. The root-level `matched_statistics.json` describes the one-run-per-layout cohort used for Figure 18.

## Tables 9 and 10

From this section directory, use the original statistical routines and bind the published CSV to the pairing keys above. The recipe saves the summaries as JSON for inspection.

```python
import json
from pathlib import Path
import sys

root = Path.cwd()
sys.path.insert(0, str(root / 'analysis'))
import analyze_matched as matched

rows = [row for row in matched.original.read(root / 'data.csv')
        if row['included_in_analysis'] == '1']
pairs = sorted({(int(row['scene_id']), int(row['replicate'])) for row in rows})
pair_ids = {pair: index for index, pair in enumerate(pairs)}
index = {(pair_ids[int(row['scene_id']), int(row['replicate'])], row['method']): row
         for row in rows}
assert len(rows) == len(index) == 147 and len(pairs) == 49
result = dict(descriptive=matched.original.summary(rows),
              paired=matched.analyse(index, list(range(len(pairs))), 5432026))
(root / 'reproduced_tables_09_10.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
```

In the order Fixed EGO, Speed-only, and Proposed, check collision counts of 30, 23, and 20, strict completion counts of 18, 26, and 29, and median first-entry times of 23.38, 27.57, and 27.80 s. The median paired minimum-clearance gains are 51.82 mm versus Fixed EGO and 28.97 mm versus Speed-only. The paired median time change versus Fixed EGO is 4.31 s in this cohort. The two Holm-adjusted clearance p-values in Table 10 round to 0.0025 and 0.010 with the archived 14-comparison family. The manuscript caption says 12 tests. The original 50-layout analysis yields the separate 4.61 s median paired delay. The recipe uses the archived bootstrap seed and order specified here, so the regenerated interval endpoints may differ slightly from the manuscript if its original ordering differed.

## Figure 18

Run the original drawing function on the one-run-per-layout cohort. The drawing reads stored truth trajectories, scenario geometry, and published per-flight metrics.

```python
import json
from pathlib import Path
import sys

root = Path.cwd()
sys.path.insert(0, str(root / 'analysis'))
import audit_manuscript as drawing

drawing.FORMAL = root / 'archive' / 'one_run_per_layout'
drawing.OUT = root / 'reproduced'
(drawing.OUT / 'figures').mkdir(parents=True, exist_ok=True)
matrix = json.loads((drawing.FORMAL / 'frozen_matrix.json').read_text(encoding='utf-8'))
rows = [row for row in drawing.read(drawing.FORMAL / 'flight_results.csv') if row['scene_id'] != '11']
assert len(rows) == 147
drawing.draw(rows, matrix)
```

The exact reference image is [Figure 18](../paper_figures/figure_18.png). See the [MATLAB README](<Trajectory Reproduction in MATLAB/README.md>) for Figures 19 and 20.
