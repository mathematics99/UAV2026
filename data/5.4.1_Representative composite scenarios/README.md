# Section 5.4.1 — Representative composite flight

The manuscript uses the flight recorded on 4 September 2026. `risk_run_20260904_212411_timeseries.csv` contains position, velocity, and risk channels. `risk_run_20260904_212411_replan_events.csv` contains 45 planning events. `run_summary.csv` records the 37.660527 s travel time and peak contraction risk of 0.304012.

Run the original analysis from this directory with the package's Python environment active.

```sh
python position_domain_mechanism_bundle/position_domain_mechanism_analysis.py \
  --timeseries risk_run_20260904_212411_timeseries.csv \
  --events risk_run_20260904_212411_replan_events.csv \
  --out-dir reproduced
```

The outputs `fig_position_speed.png` and `fig_position_risk.png` support Figure 15. `fig_position_contraction_components.png` and `fig_position_risk_contributions.png` support Figure 16. `position_region_summary.csv` contains the regional means for Table 5. The corresponding input columns include `mean_speed`, `mean_vmax`, `mean_Rc`, `mean_Rb`, `mean_Rg`, and `min_dmin`. `position_profile_0p1m.csv` stores the spatial profile used for plotting. The [Figure 14 simulator view](../paper_figures/figure_14.png) is an archived image.
