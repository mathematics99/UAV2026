# Figures 19–20 — MATLAB LiDAR and trajectory views

| Figure | Scene | Function | Required input | Output |
| --- | --- | --- | --- | --- |
| 19 | Scene 22 | `render_scan_4622(22)` | `scan_4622_data.mat` | `scene_22_lidar_map.png` |
| 20 | Scene 13 | `render_scan_13` | `scan_13_data.mat` | `scene_13_lidar_map.png` |

Both functions call `render_scan_maps.m`. Use MATLAB R2024b with the files in the same directory. To preserve the archived PNGs, run in a scratch copy of this directory. Set that copy as the MATLAB current folder and run the following commands.

```matlab
render_scan_4622(22)
render_scan_13
```

The `.mat` files contain the scene geometry and executed trajectories used by the original simulated scan and plotting functions. The Figure 19 trajectories come from the archived Scene 22 run in the one-run-per-layout cohort. The published Figure 20 uses the default function call and the two-method view for Scene 13. The optional `scan_13_speed_only_data.mat` supports its three-method view. Additional `.mat` and `.fig` files record supplemental trajectory views.
