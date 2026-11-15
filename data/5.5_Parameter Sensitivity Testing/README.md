# Section 5.5 — Parameter sensitivity

The numerical figures use a fixed-trajectory replay of the recorded flight `risk_run_20260904_212411_timeseries.csv`. `uav_parameter_sensitivity_preliminary/uav_sensitivity/` contains the original input copy, scripts, tabulated outputs, and figures. Figure 21 is provided as the [archived parameter workflow diagram](../paper_figures/figure_21.png).

| Figures | Calculation | Output |
| --- | --- | --- |
| 22 | Risk fusion weight sweep | `weight_sensitivity_replay.csv` and `figures/Fig_S1_weight_sensitivity.png` |
| 23 | EMA coefficient sweep | `ema_sensitivity_replay.csv` and `figures/Fig_S2_EMA_sensitivity.png` |
| 24–25 | Single-parameter and joint contraction/gate sensitivity | `contraction_sensitivity_replay.csv`, `figures/Fig_S3_contraction_sensitivity.png`, and `figures/Fig_S4_contraction_joint_sensitivity.png` |

Run both original scripts from a scratch copy of the `uav_sensitivity` directory with the offline environment active. They write into their own directory.

```sh
python reproduce_sensitivity.py
python reproduce_contraction_sensitivity.py
```

The auxiliary `ros2/risk_sensitivity.launch.py` exposes live parameter selection. It documents a separate closed-loop setup. The manuscript sensitivity figures and CSVs come from the recorded-trajectory replay.
