"""Flight-level statistics and LaTeX tables; run only after 100 captured flights."""
import csv,json,math,shutil,itertools
from pathlib import Path
import numpy as np
import yaml
from scipy.stats import beta,rankdata
S=Path(__file__).resolve().parent;ROOT=S.parents[1];OUT=ROOT/'experiments/section53_batch10_20261106'
METHODS=['fixed_ego_swarm','speed_only','coupled_legacy','gate_off','risk_decoupled']
LABELS={'fixed_ego_swarm':'Fixed EGO','speed_only':'Speed-only','coupled_legacy':'Legacy $+d_0$','gate_off':'Gate-OFF','risk_decoupled':'Proposed'}
def read(p):
    with p.open(encoding='utf-8-sig') as f:return list(csv.DictReader(f))
def write(p,rows):
    with p.open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=list(dict.fromkeys(k for r in rows for k in r)));w.writeheader();w.writerows(rows)
def number(r,k):
    try:return float(r[k])
    except (ValueError,KeyError):return math.nan
def summarize(rows,key):
    a=np.array([number(r,key) for r in rows]);a=a[np.isfinite(a)]
    return dict(n=len(a),mean=float(np.mean(a)) if len(a) else math.nan,sd=float(np.std(a,ddof=1)) if len(a)>1 else math.nan,median=float(np.median(a)) if len(a) else math.nan,q25=float(np.percentile(a,25)) if len(a) else math.nan,q75=float(np.percentile(a,75)) if len(a) else math.nan)
def signed_rank_p(d):
    z=d[d!=0]
    if not len(z):return 1.
    ranks=rankdata(np.abs(z));total=ranks.sum();observed=min(ranks[z>0].sum(),ranks[z<0].sum())
    sums=np.asarray(list(itertools.product([0.,1.],repeat=len(z))))@ranks
    return float(np.mean(np.minimum(sums,total-sums)<=observed+1e-12))
def main():
    assert (OUT/'BATCH_COMPLETE').exists(),'Do not draft outcome claims from a partial batch'
    runs=sorted((OUT/'runs').iterdir());rows=[read(run/'batch_metrics.csv')[0] for run in runs];assert len(rows)==100
    # Correct the display-order counter from the first in-memory batch version.
    # The randomized condition sequence and trial IDs were unaffected.
    matrix=json.loads((OUT/'experiment_matrix.json').read_text());corrections=[]
    for i,item in enumerate(matrix,1):
        captured=next(r for r in rows if int(r['trial'])==item['trial'] and r['algorithm']==item['algorithm'] and float(r['width_m'])==item['width_m'])
        corrections.append(dict(trial=item['trial'],algorithm=item['algorithm'],width_m=item['width_m'],logged_order=captured['order'],actual_serial_order=i));item['order']=i
    order_lookup={(str(x['trial']),x['algorithm'],float(x['width_m'])):x['order'] for x in matrix}
    for run,row in zip(runs,rows):
        row['order']=order_lookup[row['trial'],row['algorithm'],float(row['width_m'])]
        base=read(run/'statistics_summary.csv')[0]
        with (run/'raw.csv').open() as f:
            for last in csv.DictReader(f):pass
        row.update(time_to_first_goal_entry_wall_s=base['traversal_time_wall_s'] if row['goal_reached']=='1' else '',terminal_goal_distance_m=float(last['goal_distance_m']),terminal_speed_m_s=float(last['speed_twist_m_s']),outcome_reason='successful_settled_completion' if row['task_success']=='1' else ('terminal_settling_not_met_after_goal_entry' if row['goal_reached']=='1' else 'goal_not_reached'))
        write(run/'batch_metrics.csv',[row]);cfg=json.loads((run/'run_parameters.json').read_text());cfg['order']=row['order'];(run/'run_parameters.json').write_text(json.dumps(cfg,indent=2))
    (OUT/'experiment_matrix.json').write_text(json.dumps(matrix,indent=2));write(OUT/'order_label_audit.csv',corrections)
    (OUT/'progress.json').write_text(json.dumps(dict(completed=100,total=100,status='all_flights_primary_validated'),indent=2))
    write(OUT/'all_flight_metrics.csv',rows);grouped={};aggregate=[];validation=[]
    keys=['traversal_time_wall_s','traversal_time_integration_s','path_length_m','tracking_rmse_m','min_true_center_obstacle_distance_m','min_true_body_clearance_m','min_dynamic_center_margin_m','min_dynamic_body_margin_m','center_margin_negative_fraction','body_margin_negative_fraction','tracking_ise_m2_s','body_braking_deficit_m_s','normalized_safety_efficiency_cost','mean_actual_speed_twist_wallweighted_m_s','max_actual_speed_twist_m_s','velocity_violation_5pct_wall_fraction','replanning_attempt_count','replan_mean_ms','replan_p50_ms','replan_p95_ms','replan_p99_ms','replan_max_ms','time_scale_integration_over_wall']
    successrows=[]
    for width in [1.,1.4]:
        for method in METHODS:
            selected=[r for r in rows if float(r['width_m'])==width and r['algorithm']==method];assert len(selected)==10 and len(set(r['trial'] for r in selected))==10
            good=[r for r in selected if r['task_success']=='1'];grouped[method,width]=good;n=len(selected);k=len(good)
            successrows.append(dict(algorithm=method,width_m=width,n=n,successes=k,goal_entries=sum(r['goal_reached']=='1' for r in selected),collisions=sum(r['collision']=='true' for r in selected),success_ci95_low=0 if k==0 else float(beta.ppf(.025,k,n-k+1)),success_ci95_high=1 if k==n else float(beta.ppf(.975,k+1,n-k))))
            for key in keys:aggregate.append(dict(algorithm=method,width_m=width,metric=key,**summarize(selected if key.startswith('replan') else good,key)))
    write(OUT/'group_summary.csv',aggregate);write(OUT/'success_collision_summary.csv',successrows)
    entry_summaries=[]
    for width in [1.,1.4]:
        for method in METHODS:
            entered=[r for r in rows if float(r['width_m'])==width and r['algorithm']==method and r['goal_reached']=='1']
            for key in ['time_to_first_goal_entry_wall_s','path_length_m','tracking_rmse_m','center_margin_negative_fraction','tracking_ise_m2_s']:
                entry_summaries.append(dict(algorithm=method,width_m=width,metric=key,scope='post hoc diagnostic: all goal-entry flights including failed terminal settling; not substituted for frozen primary analysis',**summarize(entered,key)))
    write(OUT/'all_goal_entries_diagnostic_summary.csv',entry_summaries)
    stats={(r['algorithm'],r['width_m'],r['metric']):r for r in aggregate}
    wide=[];display=[]
    for w in [1.,1.4]:
        for m in METHODS:
            status=next(r for r in successrows if r['algorithm']==m and r['width_m']==w);item=dict(status);pretty={'喉宽_m':w,'方法':LABELS[m].replace('$',''),'成功次数':f"{status['successes']}/10",'碰撞次数':status['collisions']}
            for key,title,digits,scale in [('traversal_time_wall_s','穿越时间_s',2,1),('path_length_m','路径长度_m',3,1),('tracking_rmse_m','跟踪RMSE_m',4,1),('min_true_body_clearance_m','最小机体净空_m',3,1),('center_margin_negative_fraction','中心裕度不足时间百分比',2,100),('tracking_ise_m2_s','累计跟踪误差_m2_s',4,1),('normalized_safety_efficiency_cost','归一化安全效率成本',3,1),('replan_p95_ms','单次飞行P95规划耗时_ms',3,1)]:
                z=stats[m,w,key];item[key+'_mean']=z['mean'];item[key+'_sd']=z['sd'];pretty[title]=f"{z['mean']*scale:.{digits}f} ± {z['sd']*scale:.{digits}f}"
            wide.append(item);display.append(pretty)
    write(OUT/'summary_table_numeric.csv',wide);write(OUT/'统计汇总表.csv',display)
    contrast=[];rng=np.random.RandomState(1)
    for width in [1.,1.4]:
        for method in METHODS[:-1]:
            aa={r['trial']:r for r in grouped['risk_decoupled',width]};bb={r['trial']:r for r in grouped[method,width]};paired=sorted(set(aa)&set(bb))
            for key in ['tracking_rmse_m','center_margin_negative_fraction']:
                d=np.array([number(aa[j],key)-number(bb[j],key) for j in paired]);resamples=d[rng.randint(0,len(d),(10000,len(d)))].mean(axis=1) if len(d) else np.array([math.nan])
                pv=signed_rank_p(d)
                contrast.append(dict(width_m=width,comparator=method,metric=key,paired_completed_flights=len(d),nonzero_pairs=int(np.count_nonzero(d)),test='two-sided Wilcoxon signed-rank; exact conditional sign enumeration; zero differences dropped; tied ranks averaged',difference_direction='proposed minus comparator; negative favors proposed',mean_difference=float(d.mean()) if len(d) else '',ci95_low=float(np.percentile(resamples,2.5)),ci95_high=float(np.percentile(resamples,97.5)),p_raw=pv))
    order=sorted(range(len(contrast)),key=lambda i:contrast[i]['p_raw']);previous=0
    for rank,index in enumerate(order):previous=max(previous,min(1.,contrast[index]['p_raw']*(len(contrast)-rank)));contrast[index]['p_holm_16']=previous
    write(OUT/'paired_contrasts.csv',contrast)
    events=[]
    for run in runs:
        cfg=json.loads((run/'run_parameters.json').read_text());v=json.loads((run/'validation.json').read_text());m=json.loads((run/'metric_validation.json').read_text())
        validation.append(dict(run=run.name,source_monotone=v['source_stamps_strictly_increasing'],event_log_match=v['event_log_count_matches'],bag_counts_match=v['raw_bag_counts_match_recorder'],missing_mission_integration_pairs=v['missing_mission_integration_pairs'],unpaired_only_at_boundary=v['unpaired_only_at_recording_boundary'],twist_integration_fd_median_error_m_s=v['twist_vs_integration_fd_median_abs_error_m_s'],tracking_identity_error=m['tracking_identity_abs_error']))
        for e in read(run/'replan_events.csv'):events.append(dict(e,trial=cfg['trial'],block=cfg['trial']))
    write(OUT/'validation_summary.csv',validation);write(OUT/'all_replan_events.csv',events)
    assert all(r['source_monotone'] and r['event_log_match'] and r['bag_counts_match'] for r in validation)
    maps={}
    import hashlib
    for width in [1.,1.4]:
        hashes={hashlib.sha256((run/'map/wall_points.npy').read_bytes()).hexdigest() for run in runs if json.loads((run/'run_parameters.json').read_text())['width_m']==width};assert len(hashes)==1;maps[str(width)]=next(iter(hashes))
    (OUT/'paired_map_validation.json').write_text(json.dumps(maps,indent=2))
    parameter_checks={}
    for node in ['drone_0_quadrotor_simulator_so3','drone_0_so3_control_component','drone_0_pcl_render_node','sensor_adapter','drone_0_traj_server']:
        values=[yaml.safe_load((run/'logs'/(node+'_parameters.yaml')).read_text()) for run in runs]
        parameter_checks[node]=all(v==values[0] for v in values)
    (OUT/'common_parameters_validation.json').write_text(json.dumps(parameter_checks,indent=2));assert all(parameter_checks.values())
    # All manuscript numbers are formatted from the same flight-level aggregates.
    manuscript=OUT/'manuscript';manuscript.mkdir(exist_ok=True)
    def cell(method,width,key,digits=3,scale=1.):
        x=stats[method,width,key];return f"${x['mean']*scale:.{digits}f} \\pm {x['sd']*scale:.{digits}f}$" if x['n'] else '--'
    def table(name,caption,headers,columns,label):
        text=['\\begin{table}[!ht]','\\centering','\\footnotesize','\\setlength{\\tabcolsep}{5pt}','\\caption{\\textbf{'+caption+'}}','\\begin{tabular}{ll'+'r'*(len(headers)-2)+'}','\\hline',' & '.join(headers)+' \\\\','\\hline']
        for width in [1.,1.4]:
            for method in METHODS:
                record=next(x for x in successrows if x['algorithm']==method and x['width_m']==width)
                values=[f'{width:.1f}',LABELS[method]]
                for col in columns:
                    if col=='counts':values.append(f"{record['successes']}/10; {record['collisions']}/10")
                    else:values.append(cell(method,width,*col))
                text.append(' & '.join(values)+' \\\\')
            text.append('\\hline')
        text+=['\\end{tabular}','\\label{'+label+'}','\\end{table}']
        (manuscript/name).write_text('\n'.join(text))
    table('table_full_flight.tex','Full-flight performance over ten repetitions per condition. Values are mean $\\pm$ sample SD over successful flights; $\\bar{v}$ is the time-weighted actual twist speed. The final column lists successful missions and observed collisions.',['Width (m)','Method','$T$ (s)','$L$ (m)','$\\bar{v}$ (m/s)','RMSE (m)','$c_{\\min}$ (m)','Success; collision'],[('traversal_time_wall_s',2),('path_length_m',3),('mean_actual_speed_twist_wallweighted_m_s',3),('tracking_rmse_m',4),('min_true_body_clearance_m',3),'counts'],'tab:s53full')
    table('table_safety.tex','Common geometric--braking diagnostics and exploratory trade-off costs. Negative margins are conservative isotropic diagnostics, not directional stopping guarantees.',['Width (m)','Method','$m_{c,\\min}$ (m)','$\\phi_c$ (\\%)','$\\phi_b$ (\\%)','$J_{\\mathrm{track}}$ (m$^2$s)','$J_{\\mathrm{SE}}$'],[('min_dynamic_center_margin_m',3),('center_margin_negative_fraction',2,100),('body_margin_negative_fraction',2,100),('tracking_ise_m2_s',4),('normalized_safety_efficiency_cost',3)],'tab:s53safety')
    table('table_timing.tex','Local rebound replanning workload. Each statistic is first computed within a flight, then summarized across flights; durations exclude the risk layer, global initialization, sensing, and controller execution.',['Width (m)','Method','Attempts','Mean (ms)','P50 (ms)','P95 (ms)','P99 (ms)'],[('replanning_attempt_count',1),('replan_mean_ms',3),('replan_p50_ms',3),('replan_p95_ms',3),('replan_p99_ms',3)],'tab:s53timing')
    (OUT/'aggregate_results.json').write_text(json.dumps(dict(groups=aggregate,success=successrows,contrasts=contrast,validation=validation),indent=2))
    print('100 flight-level records, 10 groups, 16 corrected planned contrasts, tables generated')
if __name__=='__main__':main()
