"""Publication figures from flight replicates, never from selected best runs."""
import csv,json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from finalize import OUT,METHODS,LABELS,read,number,write
COLORS=['#4477AA','#EE6677','#228833','#CCBB44','#AA3377']
def main():
    assert (OUT/'BATCH_COMPLETE').exists()
    out=OUT/'figures';out.mkdir(exist_ok=True);rows=read(OUT/'all_flight_metrics.csv');agg=json.loads((OUT/'aggregate_results.json').read_text());groups={(m,w):[r for r in rows if r['algorithm']==m and float(r['width_m'])==w and r['task_success']=='1'] for w in [1.,1.4] for m in METHODS}
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'axes.spines.top':False,'axes.spines.right':False,'pdf.fonttype':42,'ps.fonttype':42,'savefig.bbox':'tight'})
    def save(fig,name):
        fig.savefig(out/(name+'.pdf'));fig.savefig(out/(name+'.png'),dpi=600);plt.close(fig)
    fig,axes=plt.subplots(2,3,figsize=(10.5,6.5),constrained_layout=True)
    keys=[('traversal_time_wall_s','Traversal time (s)',1),('tracking_rmse_m','Tracking RMSE (m)',1),('center_margin_negative_fraction','Negative center-margin time (%)',100)]
    for row,w in enumerate([1.,1.4]):
        for col,(key,label,scale) in enumerate(keys):
            ax=axes[row,col];data=[np.array([number(r,key) for r in groups[m,w]])*scale for m in METHODS]
            box=ax.boxplot(data,positions=np.arange(5),widths=.55,patch_artist=True,showfliers=False,medianprops=dict(color='black'))
            for i,(values,patch,color) in enumerate(zip(data,box['boxes'],COLORS)):
                patch.set_facecolor(color);patch.set_alpha(.3);ax.scatter(i+np.linspace(-.14,.14,len(values)),values,s=12,color=color,zorder=3,edgecolor='white',linewidth=.3)
            ax.set_xticks(range(5));ax.set_xticklabels([LABELS[m] for m in METHODS],rotation=22,ha='right');ax.set_ylabel(label);ax.set_title(f'({chr(97+row*3+col)}) Throat {w:.1f} m');ax.grid(axis='y',alpha=.2)
    save(fig,'fig53_full_flight')
    if '--performance-only' in sys.argv:return
    fig,axes=plt.subplots(2,2,figsize=(9,6.5),constrained_layout=True)
    for row,w in enumerate([1.,1.4]):
        for col,(key,label,scale) in enumerate([('center_margin_negative_fraction','Negative center-margin time (%)',100),('tracking_rmse_m','Tracking RMSE (m)',1)]):
            ax=axes[row,col]
            for m,color in zip(METHODS,COLORS):
                x=np.array([number(r,'traversal_time_wall_s') for r in groups[m,w]]);y=np.array([number(r,key)*scale for r in groups[m,w]])
                ax.scatter(x,y,s=14,alpha=.35,color=color)
                ax.errorbar(x.mean(),y.mean(),xerr=x.std(ddof=1),yerr=y.std(ddof=1),fmt='o',ms=6,capsize=3,color=color,label=LABELS[m])
            ax.set_xlabel('Traversal time (s)');ax.set_ylabel(label);ax.set_title(f'({chr(97+row*2+col)}) Throat {w:.1f} m');ax.grid(alpha=.2)
    axes[0,0].legend(fontsize=10);save(fig,'fig53_tradeoff')
    # Spatial profiles from first passage through each longitudinal position.
    xgrid=np.arange(-14,14.001,.2);profiles={};profile_rows=[]
    for run in sorted((OUT/'runs').iterdir()):
        if read(run/'batch_metrics.csv')[0]['task_success']!='1':continue
        cfg=json.loads((run/'run_parameters.json').read_text());a=[]
        with (run/'raw.csv').open(buffering=8*1024*1024) as f:
            for r in csv.DictReader(f):
                if r['mission_window']=='1':a.append([float(r[k]) for k in ['x_m','speed_twist_m_s','braking_margin_center_m','tracking_error_m']])
        a=np.array(a);keep=np.r_[True,a[1:,0]>np.maximum.accumulate(a[:,0])[:-1]];a=a[keep];key=(cfg['algorithm'],cfg['width_m']);values=np.column_stack([np.interp(xgrid,a[:,0],a[:,i]) for i in range(1,4)]);profiles.setdefault(key,[]).append(values)
    fig,axes=plt.subplots(2,3,figsize=(10.5,6),constrained_layout=True)
    for row,w in enumerate([1.,1.4]):
        for col,label in enumerate(['Actual speed (m/s)','Center braking margin (m)','Tracking error (m)']):
            ax=axes[row,col]
            for m,color in zip(METHODS,COLORS):
                v=np.array(profiles[m,w])[:,:,col];med=np.nanmedian(v,axis=0);q25,q75=np.nanpercentile(v,[25,75],axis=0);ax.plot(xgrid,med,color=color,lw=1.5,label=LABELS[m]);ax.fill_between(xgrid,q25,q75,color=color,alpha=.15)
                for x,y,lo,hi in zip(xgrid,med,q25,q75):profile_rows.append(dict(algorithm=m,width_m=w,metric=label,x_m=x,median=y,q25=lo,q75=hi,n=len(v)))
            ax.axvspan(-3,3,color='gray',alpha=.08);ax.axvline(0,color='gray',ls=':',lw=.6);ax.set_xlabel('Longitudinal position x (m)');ax.set_ylabel(label);ax.set_title(f'({chr(97+row*3+col)}) Throat {w:.1f} m');ax.grid(alpha=.15)
            if col==1:ax.axhline(0,color='black',ls='--',lw=.7)
    axes[0,0].legend(fontsize=10,loc='lower left');save(fig,'fig53_profiles');write(OUT/'spatial_profiles.csv',profile_rows)
    fig,axes=plt.subplots(1,2,figsize=(9,3.5),constrained_layout=True);events=read(OUT/'all_replan_events.csv')
    all_local=[float(e['planning_time_ms']) for e in events if e['event_kind']=='local_replan'];top=np.percentile(all_local,99.5);grid=np.linspace(0,top,350);ecdf_rows=[]
    for ax,w in zip(axes,[1.,1.4]):
        for m,color in zip(METHODS,COLORS):
            curves=[]
            for trial in range(1,11):
                a=np.sort([float(e['planning_time_ms']) for e in events if e['algorithm']==m and float(e['width_m'])==w and int(e['trial'])==trial and e['event_kind']=='local_replan']);curves.append(np.searchsorted(a,grid,side='right')/len(a))
            curves=np.array(curves);med=np.median(curves,axis=0);q25,q75=np.percentile(curves,[25,75],axis=0);ax.plot(grid,med,color=color,label=LABELS[m]);ax.fill_between(grid,q25,q75,color=color,alpha=.15)
            for x,y,lo,hi in zip(grid,med,q25,q75):ecdf_rows.append(dict(algorithm=m,width_m=w,planning_time_ms=x,median_flight_ecdf=y,q25=lo,q75=hi,n_flights=10))
        ax.set_xlabel('Local rebound planning time (ms)');ax.set_ylabel('Within-flight cumulative fraction');ax.set_title(f'Throat {w:.1f} m');ax.grid(alpha=.2);ax.set_ylim(0,1.02)
    axes[1].legend(fontsize=10);save(fig,'fig53_replanning_ecdf');write(OUT/'replanning_ecdf_by_flight.csv',ecdf_rows)
    print('Four research figures exported as PDF and 600-dpi PNG; source data retained')
if __name__=='__main__':main()
