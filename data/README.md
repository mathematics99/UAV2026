# Reproduction package for risk-decoupled UAV trajectory planning

This package accompanies the manuscript *A risk-decoupled adaptive trajectory planning method for UAVs in constricting narrow passages* by Tianyu Shi and Huifeng Tian. Figure and table numbers refer to the 15 November 2026 manuscript. The package groups the original ROS 2 source, offline analysis scripts, experiment inputs and results, and the exact manuscript figure assets. Scripts and data retain their archived contents.

## Contents

| Location | Manuscript material |
| --- | --- |
| [Core implementation and ROS 2 environment](src/README.md) | Sections 2 and 4, algorithm architecture, topics, build and launch |
| [Mechanistic simulations](<3_mechanistic simulation/README.md>) | Section 3, Figures 3–5 |
| [Corridor mechanism experiments](<5.1&5.2_ego_risk_experiments/README.md>) | Sections 5.1–5.2, Figures 7–11 |
| [Controlled flight comparison](5.3_batch10/README.md) | Section 5.3, Figures 12–13 and Tables 2–4 |
| [Representative composite flight](<5.4.1_Representative composite scenarios/README.md>) | Section 5.4.1, Figures 14–16 and Table 5 |
| [Fixed-throat forest comparison](5.4.2_multi_trials/README.md) | Section 5.4.2, Figure 17 and Tables 6–7 |
| [Randomized composite experiments](5.4.3_updated/README.md) | Section 5.4.3, Figures 18–20 and Tables 8–10 |
| [Parameter sensitivity](<5.5_Parameter Sensitivity Testing/README.md>) | Section 5.5, Figures 21–25 |
| [Manuscript figure assets](paper_figures/README.md) | Figure files numbered 01–25, including individual panels |
| [Historical WSL entry points](historical_wsl/README.md) | Original launch and acquisition files from the experiment workspace |

## Offline environment

Use a separate Python environment for analysis. The Windows analysis environment used for checking this package had Python 3.11.6 and the following package versions. MATLAB R2024b is the version available for Figures 19–20. The ROS 2 build and simulation environment is documented in [src/README.md](src/README.md).

```sh
python -m venv .venv
# Linux or macOS
source .venv/bin/activate
# Windows PowerShell equivalent
# .\.venv\Scripts\Activate.ps1
python -m pip install numpy==2.4.6 scipy==1.17.1 pandas==2.3.3 matplotlib==3.11.2 Pillow==12.3.0 PyYAML==6.0.3
```

Run the section recipes from the working directories specified in their READMEs. Use a scratch directory or checkout for commands that write into their script directory. Figure files under `paper_figures/` are the published-layout assets. Re-running the original plotting code can produce minor font, raster-size, or layout differences across systems. The original schematic diagrams and simulator screenshots are available as figure assets.

## Table provenance

| Table | Source |
| --- | --- |
| 1 | [Notation defined by the method](src/README.md#table-1-notation) |
| 2 | [Controlled-flight comparison settings](5.3_batch10/README.md#tables-2-4) |
| 3–4 | [100 controlled flights and manuscript summary](5.3_batch10/README.md#tables-2-4) |
| 5 | [Regional statistics for the representative flight](<5.4.1_Representative composite scenarios/README.md>) |
| 6–7 | [Fixed-throat design and 90-flight analysis](5.4.2_multi_trials/README.md) |
| 8 | [Frozen configurations for the 50-layout design](5.4.3_updated/README.md#archived-flight-cohorts) |
| 9–10 | [Results from 60 initial flights plus 90 repeat flights](5.4.3_updated/README.md#tables-9-and-10) |

**Section 5.4.3 uses two archived cohorts.** Figure 18 and the design in Table 8 correspond to the 50-layout, one-run-per-method archive. The numbers in Tables 9 and 10 match `5.4.3_updated/data.csv`, which combines the 20 initial layouts with five repeats of each of six additional layouts. After excluding the three Scene 11 flights, the latter has 147 flights in 49 matched rounds over 25 distinct layouts. The [Section 5.4.3 README](5.4.3_updated/README.md) gives the pairing keys, the observed table values, and the difference between the 12-test caption and the 14-comparison calculation.

The offline recipes use the deposited inputs. The historical simulation scripts and parameter snapshots document how flights were obtained. The earlier ROS bags and the full Section 5.3 raw, per-sample run folders are held in the working archive. Section 5.3 deposited timing events and final per-flight records suffice for its manuscript figures and result tables. A clean-host rebuild and complete simulation rerun are separate validation steps.

## Verification of the release

The offline recipes for the numerical figures were executed in a scratch copy on the documented Python environment. The Section 5.3 summary matched the archived table and the 3,500 timing curve rows matched numerically. The Section 5.4.2 mean exposure values and Section 5.4.3 collision and strict completion counts matched the manuscript. Both MATLAB rendering commands completed in R2024b. Where a new rendering has different raster details, the manuscript assets remain the reference.

## License and attribution

The ROS source contains derivatives of EGO-Swarm. The repository carries the upstream-derived [LICENSE](LICENSE) and [NOTICE](NOTICE). Preserve the existing source credits and cite the accompanying manuscript when using the data and figures.
