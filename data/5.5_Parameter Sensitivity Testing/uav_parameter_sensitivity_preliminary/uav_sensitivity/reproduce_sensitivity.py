#!/usr/bin/env python3
"""Risk weights + EMA one-factor-at-a-time sensitivity: offline fixed-trajectory replay.

This reproduces the fusion and EMA+slew logic of risk_evaluator.cpp and
adaptive_planner.cpp. It DOES NOT rerun EGO-Swarm, update UAV motion, or
produce independent flight outcomes. The measured raw risk channels are fixed.
"""
from pathlib import Path
import matplotlib
import numpy as np
import pandas as pd

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parent
DATA=ROOT/'data'/'risk_run_20260904_212411_timeseries.csv'
FIG=ROOT/'figures'; FIG.mkdir(parents=True,exist_ok=True)
TEAL = '#14687C'
RUST = '#A55124'
PURPLE = '#66548E'
INK = '#263646'
plt.rcParams.update({
 'font.family':'DejaVu Sans', 'font.size':10, 'axes.labelsize':10, 'axes.titlesize':10.5,
 'axes.titleweight':'bold', 'axes.linewidth':0.7, 'axes.edgecolor':'#55616B',
 'xtick.labelsize':9, 'ytick.labelsize':9, 'legend.fontsize':8.5,
 'axes.spines.top':False, 'axes.spines.right':False,
 'savefig.dpi':600, 'savefig.facecolor':'white',
})

d=pd.read_csv(DATA)
d=d.dropna(subset=['rho','R_b','R_c','R_g','r_bar','rc_bar','v_max','t_s']).reset_index(drop=True)
# The 50-Hz state logger samples asynchronously with the risk callbacks.
# Infer a risk update when the recorded raw risk or recorded smoothed risk changes.
events=((d.R_g.diff().abs()>1e-6) | (d.r_bar.diff().abs()>1e-6)).to_numpy()
events[0]=True
e_idx=np.flatnonzero(events)
time=d.t_s.to_numpy(float)
med_event_interval=float(np.median(np.diff(time[e_idx])))


def replay(raw:np.ndarray,lam:float):
    smoothed=np.empty_like(raw,dtype=float)
    deltas=[]; limit_hits=0; v=0.0; started=False
    for k,r in enumerate(raw):
        if events[k]:
            cand=r if not started else lam*v+(1-lam)*r
            started=True
            dv=cand-v
            if abs(dv)>0.05:limit_hits+=1
            v=v+np.clip(dv,-0.05,0.05)
            deltas.append(v-(smoothed[k-1] if k>0 else 0.0))
        smoothed[k]=v
    return smoothed,np.array(deltas),limit_hits


def signals(wrho,wb,wc,lam):
    raw=(wrho*d.rho+wb*d.R_b+wc*d.R_c).to_numpy(float)
    rbar, dr, nhits=replay(raw,lam)
    vmax=2.0-1.2*rbar
    return raw,rbar,vmax,dr,nhits

# Cross-check with the original recorded parameters.
raw0,sm0,v0,delta0,n0=signals(.2,.55,.25,.9)
raw_mae=float(np.mean(abs(raw0-d.R_g.to_numpy(float))))
sm_mae=float(np.mean(abs(sm0-d.r_bar.to_numpy(float))))
v_mae=float(np.mean(abs(v0-d.v_max.to_numpy(float))))
if raw_mae>1e-4 or sm_mae>.01:
    raise RuntimeError(f'Baseline reconstruction poor: raw_mae={raw_mae}, sm_mae={sm_mae}')

ws=[.4,.475,.55,.625,.7]
weight_rows=[]; weight_series={}
for wb in ws:
    wrho=.2; wc=.8-wb
    raw,smooth,vmax,dr,nhits=signals(wrho,wb,wc,.9)
    weight_series[wb]=vmax
    row=dict(w_density=wrho,w_braking=wb,w_contraction=wc,ema_lambda=.9,
             mean_Rg=float(raw.mean()),p95_Rg=float(np.percentile(raw,95)),
             mean_smoothed_risk=float(smooth.mean()),
             mean_vlimit_mps=float(vmax.mean()),min_vlimit_mps=float(vmax.min()),
             vlimit_throat_mps=float(np.mean(vmax[d.scene_region=='throat'])),
             vlimit_contraction_mps=float(np.mean(vmax[d.scene_region=='contraction'])),
             vlimit_forest_before_mps=float(np.mean(vmax[d.scene_region=='forest_before'])),
             p95_abs_delta_rbar=float(np.percentile(abs(dr),95)),
             num_slew_limited_updates=int(nhits))
    weight_rows.append(row)
wdf=pd.DataFrame(weight_rows)
wdf.to_csv(ROOT/'weight_sensitivity_replay.csv',index=False,float_format='%.8f')

lams=[.7,.8,.9,.95]
ema_rows=[]; ema_series={}
for lam in lams:
    raw,rbar,vmax,dr,nhits=signals(.2,.55,.25,lam)
    ema_series[lam]=rbar
    # Ideal EMA 90% step response: constant update interval and no slew cap.
    t90=float(np.log(.1)/np.log(lam)*med_event_interval)
    row=dict(ema_lambda=lam,mean_vlimit_mps=float(vmax.mean()),
             min_vlimit_mps=float(vmax.min()),p95_abs_delta_rbar=float(np.percentile(abs(dr),95)),
             mean_abs_delta_rbar=float(np.mean(abs(dr))),
             max_abs_delta_rbar=float(np.max(abs(dr))),
             theoretical_90pct_step_time_s=t90,
             num_slew_limited_updates=int(nhits))
    ema_rows.append(row)
edf=pd.DataFrame(ema_rows)
edf.to_csv(ROOT/'ema_sensitivity_replay.csv',index=False,float_format='%.8f')


def style(ax):
    ax.set_axisbelow(True)
    ax.grid(axis='y', color='#DCE4EA', linewidth=0.65)
    ax.tick_params(direction='out', width=0.7, length=3.5)


def mark_nominal(ax, x, y, color):
    ax.scatter(x, y, s=38, facecolor='white', edgecolor=color, linewidth=1.5, zorder=5)


# Both panels use the same speed scale so regional slopes are comparable.
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.45, 3.15), sharey=True, layout='constrained')
fig.get_layout_engine().set(w_pad=0.065, h_pad=0.065, wspace=0.08)
nominal_index = ws.index(0.55)
overall = wdf.mean_vlimit_mps.to_numpy()
ax1.plot(ws, overall, color=TEAL, lw=2.0, marker='o', markersize=4)
mark_nominal(ax1, 0.55, overall[nominal_index], TEAL)
ax1.annotate(r'Nominal $w_b=0.55$', (0.55, overall[nominal_index]), xytext=(8, 13),
             textcoords='offset points', color=INK, fontsize=8.5)
ax1.set_ylabel('Mean prescribed speed limit (m/s)')
ax1.set_title('(a) Whole flight', loc='left', pad=8)

for label, color, linestyle, marker, field in (
    ('Contraction', TEAL, '-', 'o', 'vlimit_contraction_mps'),
    ('Front forest', PURPLE, '-.', '^', 'vlimit_forest_before_mps'),
    ('Throat', RUST, '--', 's', 'vlimit_throat_mps'),
):
    regional = wdf[field].to_numpy()
    ax2.plot(ws, regional, color=color, linestyle=linestyle, lw=1.8,
             marker=marker, markersize=3.8, label=label)
    mark_nominal(ax2, 0.55, regional[nominal_index], color)
ax2.set_title('(b) By scene region', loc='left', pad=8)
ax2.legend(loc='lower left', frameon=False, handlelength=2.2, borderaxespad=0.7)

for ax in (ax1, ax2):
    ax.axvline(0.55, color='#87949C', linestyle=':', linewidth=0.9, zorder=0)
    ax.set(xlabel=r'Braking-risk weight $w_b$', xlim=(0.39, 0.71), ylim=(1.26, 1.75),
           xticks=ws, xticklabels=[f'{w:g}' for w in ws])
    style(ax)
fig.savefig(FIG/'Fig_S1_weight_sensitivity.png')
plt.close(fig)

# Align the transient to the largest recorded raw-risk update.
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.45, 3.15), layout='constrained',
                               gridspec_kw={'width_ratios': [1.18, 1]})
fig.get_layout_engine().set(w_pad=0.065, h_pad=0.065, wspace=0.08)
raw_changes = abs(np.diff(d.R_g.to_numpy(), prepend=d.R_g.iloc[0]))
relative_time = time - time[int(np.argmax(raw_changes))]
mask = (relative_time >= -1.45) & (relative_time <= 2.1)
ax1.step(relative_time[mask], raw0[mask], where='post', color='#697680',
         lw=1.0, alpha=0.8, label=r'Raw $R_g$')
ema_styles = ((RUST, '--', 'o'), (PURPLE, '-.', '^'), (TEAL, '-', 'o'), (INK, ':', 's'))
for lam, (color, linestyle, _) in zip(lams, ema_styles):
    ax1.step(relative_time[mask], ema_series[lam][mask], where='post',
             color=color, linestyle=linestyle, lw=2.05 if lam == 0.9 else 1.45,
             label=fr'$\lambda={lam:.2f}$')
ax1.axvline(0, color='#87949C', linestyle=':', linewidth=0.9, zorder=0)
ax1.set(xlabel='Time relative to risk change (s)', ylabel='Composite risk',
        xlim=(-1.45, 2.1), ylim=(0.23, 0.65), xticks=[-1, 0, 1, 2])
ax1.set_title('(a) Risk transient', loc='left', pad=8)
ax1.legend(loc='lower left', frameon=False, handlelength=1.8, labelspacing=0.26)

# One point per setting in the smoothing-versus-lag plane.
lag = edf.theoretical_90pct_step_time_s.to_numpy()
ripple = edf.p95_abs_delta_rbar.to_numpy()
ax2.plot(lag, ripple, color='#A7B4BD', lw=1.2, zorder=1)
for lam, x, y, (color, _, marker) in zip(lams, lag, ripple, ema_styles):
    if lam == 0.9:
        mark_nominal(ax2, x, y, color)
    else:
        ax2.scatter(x, y, color=color, marker=marker, s=26, zorder=3)
    offset = (-5, 16) if lam == 0.95 else (6, 6)
    ax2.annotate(fr'$\lambda={lam:.2f}$', (x, y), xytext=offset,
                 textcoords='offset points', ha='right' if lam == 0.95 else 'left',
                 fontsize=8.7, color=color)
ax2.set(xlabel='Ideal 90% step-response time (s)', ylabel=r'P95 of $|\Delta\bar{R}|$ per update',
        xlim=(0.34, 3.85), ylim=(0.008, 0.056), yticks=[0.01, 0.02, 0.03, 0.04, 0.05])
ax2.set_title('(b) Smoothing and lag', loc='left', pad=8)
for ax in (ax1, ax2):
    style(ax)
fig.savefig(FIG/'Fig_S2_EMA_sensitivity.png')
plt.close(fig)


report=f'''SENSITIVITY AUDIT — fixed-trajectory replay (NOT closed-loop flight)
Input: {DATA.name}
Usable samples: {len(d)}; inferred risk-update events: {len(e_idx)}
Median inferred event interval: {med_event_interval:.4f} s
Baseline reconstruction MAE: raw Rg={raw_mae:.8f}; filtered Rbar={sm_mae:.8f}; speed cap={v_mae:.8f} m/s

Weight scan: w_rho=0.20, w_b in {ws}, w_c=0.80-w_b; lambda=0.90.
EMA scan: lambda in {lams}, (w_rho,w_b,w_c)=(0.20,0.55,0.25).

The 95th percentile of |delta Rbar| is measured at inferred risk-update events.
Step response is a closed-form approximation for the EMA without slew limit,
using the median inferred update interval; it is not measured flight latency.

All risk signals, geometry, states and actual trajectories are held fixed. The
speed limit is recalculated, but the UAV is NOT simulated again. No collision,
flight duration, success rate, or safety improvement can be inferred.

Nominal values are supported as an interpretable design compromise, NOT
verified as mathematically optimal. Varying weights changes the risk score's
meaning, so lower mean Rg cannot be interpreted as safer flight.
'''
(ROOT/'AUDIT_AND_LIMITATIONS.txt').write_text(report,encoding='utf-8')
print(report)
print('WEIGHTS\n',wdf.round(5).to_string(index=False))
print('EMA\n',edf.round(5).to_string(index=False))
