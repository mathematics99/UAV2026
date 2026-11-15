# Section 3 — Mechanistic simulations

| Figure | Script | Analytic or synthetic input |
| --- | --- | --- |
| 3 | `Simulation of Channel Characteristic Mechanisms.py` | Three predefined ten-slice openness profiles |
| 4 | `Numerical simulation of contractile gating.py` | NumPy grids for speed, obstacle distance, and risk |
| 5 | `Monte Carlo.py` | A parameter grid and 30,000 Gaussian perturbations with seed `20260824` |

The scripts generate their own inputs. `paper_style.py` is their shared plotting helper. Figure 3 uses a contraction regularizer of `1e-6` in the numerical example. The archived ROS flight configuration uses `0.1` for its contraction regularizer. The Monte Carlo risk distributions round to means of 0.615 and 0.935 for the decoupled and static-offset cases respectively.

Run these commands in a scratch copy of this directory. The original scripts write their PNGs into `outputs/` adjacent to the scripts.

```sh
python "Simulation of Channel Characteristic Mechanisms.py"
python "Numerical simulation of contractile gating.py"
python "Monte Carlo.py"
```

The manuscript-ready copies are [Figure 3](../paper_figures/figure_03.png), [Figure 4](../paper_figures/figure_04.png), and [Figure 5](../paper_figures/figure_05.png).
