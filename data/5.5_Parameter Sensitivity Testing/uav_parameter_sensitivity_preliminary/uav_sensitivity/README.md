# Fixed-trajectory parameter replay

`data/risk_run_20260904_212411_timeseries.csv` is the sole input log. With the analysis environment active, run `python reproduce_sensitivity.py` and then `python reproduce_contraction_sensitivity.py` from this directory. Run in a scratch copy to preserve the archived outputs.

The first script generates the risk-fusion and EMA sweeps for Figures 22 and 23. The second generates the contraction fusion and gate sweeps for Figures 24 and 25. They save PNGs under `figures/` and write the corresponding numerical CSVs in this directory. See the [Section 5.5 README](../../README.md) for the manuscript mapping. `ros2/` contains the historical launch template and notes for a separate closed-loop run.
