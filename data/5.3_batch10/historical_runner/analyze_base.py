#!/usr/bin/env python3
"""Recompute all metrics from CDR, preserving source and reception clocks."""
import csv
import json
import math
import re
import sys
import shutil
import tempfile
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

def write(path,rows,header=None):
    if not rows and header is None:
        header=['planning_time_ms','cumulative_probability'] if 'ecdf' in str(path) else ['source_stamp_ns','count','planning_time_ms','success','event_kind']
    with tempfile.NamedTemporaryFile(mode='w',newline='',delete=False,dir='/tmp',suffix='.csv') as f:
        w=csv.DictWriter(f,fieldnames=header or list(rows[0]));w.writeheader();w.writerows(rows);temporary=f.name
    shutil.copyfile(temporary,path)

def ns(stamp):return int(stamp.sec)*1000000000+int(stamp.nanosec)
def xyz(p):return [p.x,p.y,p.z]

def clearance(p,gap):
    """Exact Euclidean center distance to continuous polygonal wall prisms."""
    p=np.asarray(p,dtype=float);result=np.full(len(p),np.inf)
    for sign in [-1,1]:
        q=p[:,:2]*np.array([1,sign])
        poly=np.array([[-3,1.5],[0,gap/2],[3,1.5],[3,12],[-3,12]])
        d=np.full(len(p),np.inf);inside=np.ones(len(p),dtype=bool)
        for a,b in zip(poly,np.roll(poly,-1,axis=0)):
            ab=b-a;ap=q-a;t=np.clip((ap@ab)/(ab@ab),0,1)
            d=np.minimum(d,np.linalg.norm(ap-t[:,None]*ab,axis=1))
            inside &= ab[0]*ap[:,1]-ab[1]*ap[:,0]>=-1e-12
        xy=np.where(inside,0,d);dz=np.maximum(np.maximum(-p[:,2],p[:,2]-2.5),0)
        dist=np.hypot(xy,dz)
        # Negative signed distance only for a center inside the solid prism.
        within=inside&(p[:,2]>=0)&(p[:,2]<=2.5)
        dist[within]=-np.minimum(d[within],np.minimum(p[within,2],2.5-p[within,2]))
        result=np.minimum(result,dist)
    return result

def stats(values,prefix):
    v=np.asarray(values,dtype=float);v=v[np.isfinite(v)]
    if not len(v):return {prefix+'_count':0}
    return {prefix+'_count':len(v),prefix+'_mean_ms':float(np.mean(v)),prefix+'_std_ms':float(np.std(v)),prefix+'_min_ms':float(np.min(v)),prefix+'_p50_ms':float(np.percentile(v,50)),prefix+'_p90_ms':float(np.percentile(v,90)),prefix+'_p95_ms':float(np.percentile(v,95)),prefix+'_p99_ms':float(np.percentile(v,99)),prefix+'_max_ms':float(np.max(v))}

def main():
    run=Path(sys.argv[1]);cfg=json.loads((run/'run_parameters.json').read_text());gap=cfg['width_m']
    # Offline only: read SQLite from Linux tmp, avoiding NTFS per-message latency.
    # The exported bag remains the authoritative, unchanged recording.
    local_path=run/'staging_bag_path.txt'
    staged=Path(local_path.read_text()) if local_path.exists() else None
    cached_bag=staged if staged and staged.exists() else Path(tempfile.mkdtemp(prefix='section53_analysis_'))/'rosbag'
    if cached_bag!=staged:shutil.copytree(run/'rosbag',cached_bag)
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(cached_bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
    types={t.name:t.type for t in reader.get_all_topics_and_types()};classes={t:get_message(v) for t,v in types.items()}
    od=[];cmd=[];events=[];scalar=[];integr={};counts={t:0 for t in types}
    while reader.has_next():
        topic,data,receipt=reader.read_next();counts[topic]+=1
        if types[topic]=='sensor_msgs/msg/PointCloud2' or types[topic]=='traj_utils/msg/Bspline':continue
        m=deserialize_message(data,classes[topic])
        if topic=='/drone_0_visual_slam/odom':od.append((ns(m.header.stamp),receipt,xyz(m.pose.pose.position),xyz(m.twist.twist.linear),[m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w]))
        elif topic=='/experiment/integration_odom':integr[int(m.child_frame_id)]=ns(m.header.stamp)
        elif topic=='/drone_0_planning/pos_cmd':cmd.append((ns(m.header.stamp),receipt,xyz(m.position),xyz(m.velocity),xyz(m.acceleration),m.trajectory_id))
        elif topic=='/planner/replan_event':events.append(dict(source_stamp_ns=ns(m.stamp),receipt_stamp_ns=receipt,count=m.count,planning_time_ms=m.planning_time_ms,success=int(m.success),event_kind='global_initial' if m.count==1 else 'local_replan'))
        elif types[topic]=='std_msgs/msg/Float64':scalar.append(dict(receipt_stamp_ns=receipt,topic=topic,value=m.data))
    del reader
    if cached_bag!=staged:shutil.rmtree(cached_bag.parent)
    external=(run/'native_replan_events.csv').exists()
    if external:
        with (run/'native_replan_events.csv').open() as f:
            for e in csv.DictReader(f):
                events.append(dict(source_stamp_ns=int(e['source_stamp_ns']),receipt_stamp_ns='',count=int(e['count']),planning_time_ms=float(e['planning_time_ms']),success=int(e['success']),event_kind=e['event_kind'],return_code=int(e['return_code'])))
    if not od or not cmd:raise RuntimeError('No odometry or command data')
    log=(run/'logs/simulation.log').read_text(errors='replace')
    log_local=[]
    for stamp,duration in re.findall(r'\[INFO\]\s+\[(\d+\.\d+)\][^\n]*\[planning_time\]\s+([\d.]+)\s+ms',log):
        sec,nsec=stamp.split('.');log_local.append((int(sec)*10**9+int(nsec.ljust(9,'0')[:9]),float(duration)))
    unmatched=set(range(len(log_local)))
    for e in ([] if external else events):
        candidates=[j for j in unmatched if 0<=e['source_stamp_ns']-log_local[j][0]<100000000 and abs(e['planning_time_ms']-log_local[j][1])<=.000501]
        if candidates:
            j=min(candidates,key=lambda j:e['source_stamp_ns']-log_local[j][0]);unmatched.remove(j);e['event_kind']='local_replan'
        else:e['event_kind']='global_initial' if e['count']==1 else 'global_goal_reset'
    stamps=np.array([r[0] for r in od],dtype=np.int64);base=int(stamps[0]);t=(stamps-base)*1e-9
    recv=np.array([r[1] for r in od],dtype=np.int64);pos=np.array([r[2] for r in od]);vel=np.array([r[3] for r in od]);speed=np.linalg.norm(vel,axis=1)
    sim=np.array([integr.get(int(s),np.nan)*1e-9 for s in stamps]);simrel=sim-sim[np.where(np.isfinite(sim))[0][0]]
    ct=np.array([(r[0]-base)*1e-9 for r in cmd]);cp=np.array([r[2] for r in cmd]);cv=np.array([r[3] for r in cmd])
    ref=np.column_stack([np.interp(t,ct,cp[:,j]) for j in range(3)])
    refspeed=np.linalg.norm(np.column_stack([np.interp(t,ct,cv[:,j]) for j in range(3)]),axis=1)
    refvalid=(t>=ct[0])&(t<=ct[-1]);err=np.linalg.norm(pos-ref,axis=1);err[~refvalid]=np.nan
    fd=np.full(len(t),np.nan);fdsim=fd.copy();ds=np.linalg.norm(np.diff(pos,axis=0),axis=1)
    dt=np.diff(t);dtsim=np.diff(sim);fd[1:]=np.divide(ds,dt,out=np.full(len(ds),np.nan),where=dt>0);fdsim[1:]=np.divide(ds,dtsim,out=np.full(len(ds),np.nan),where=dtsim>0)
    d=clearance(pos,gap);radius=cfg['body_collision_radius_m'];body=d-radius
    brake=speed*.3+speed**2/4; margin=d-brake;bodymargin=body-brake
    telemetry={};ages={}
    for topic in sorted(set(r['topic'] for r in scalar)):
        rs=[r for r in scalar if r['topic']==topic];rt=np.array([r['receipt_stamp_ns'] for r in rs],dtype=np.int64);values=np.array([r['value'] for r in rs]);idx=np.searchsorted(rt,recv,side='right')-1
        val=np.full(len(t),np.nan);age=val.copy();valid=idx>=0;val[valid]=values[idx[valid]];age[valid]=(recv[valid]-rt[idx[valid]])*1e-9
        telemetry[topic]=val;ages[topic]=age
    vmax=telemetry.get('/adaptive/max_velocity',np.full(len(t),2.0))
    vmax=np.where(np.isfinite(vmax),vmax,2)
    start=max(0,int(np.searchsorted(t,ct[0]))-1)
    goalerr=np.linalg.norm(pos-np.array(cfg['goal']),axis=1)
    reached=np.where((np.arange(len(t))>=start)&(goalerr<=cfg['goal_tolerance_m']))[0]
    finish=int(reached[0]) if len(reached) else len(t)-1
    mission=np.zeros(len(t),dtype=bool);mission[start:finish+1]=True
    def integral_mean(v,clock):
        sl=slice(start,finish+1);tt=clock[sl];vv=v[sl];good=np.isfinite(tt)&np.isfinite(vv)
        return float(np.trapz(vv[good],tt[good])/(tt[good][-1]-tt[good][0])) if good.sum()>1 else float('nan')
    # Densely inspect every inter-sample segment and retain a Lipschitz lower bound.
    swept_min=float(d[mission].min());segments=ds[start:finish]
    subdivisions=np.maximum(1,np.ceil(segments/.005).astype(int))
    max_spacing=float(np.max(segments/subdivisions)) if len(segments) else 0.0
    extras=[]
    for offset in np.where(subdivisions>1)[0]:
        i=start+int(offset);n=int(subdivisions[offset])
        extras.append(pos[i]+(np.arange(1,n)/n)[:,None]*(pos[i+1]-pos[i]))
    if extras:swept_min=min(swept_min,float(clearance(np.vstack(extras),gap).min()))
    lower=swept_min-max_spacing/2
    body_collision='true' if swept_min<=radius else ('false' if lower>radius else 'uncertain')
    raw=[]
    for i,r in enumerate(od):
        row=dict(source_stamp_ns=r[0],receipt_stamp_ns=r[1],t_wall_s=t[i],integration_time_s=sim[i],t_integration_s=simrel[i],receipt_latency_s=(r[1]-r[0])*1e-9,x_m=pos[i,0],y_m=pos[i,1],z_m=pos[i,2],vx_m_s=vel[i,0],vy_m_s=vel[i,1],vz_m_s=vel[i,2],speed_twist_m_s=speed[i],speed_fd_wall_m_s=fd[i],speed_fd_integration_m_s=fdsim[i],qx=r[4][0],qy=r[4][1],qz=r[4][2],qw=r[4][3],reference_x_m=ref[i,0],reference_y_m=ref[i,1],reference_z_m=ref[i,2],reference_speed_m_s=refspeed[i],tracking_error_m=err[i],center_wall_distance_m=d[i],body_clearance_m=body[i],braking_distance_twist_m=brake[i],braking_margin_center_m=margin[i],braking_margin_body_m=bodymargin[i],velocity_limit_m_s=vmax[i],speed_excess_m_s=speed[i]-vmax[i],speed_violation_5pct=int(speed[i]>vmax[i]*1.05),center_collision=int(d[i]<=0),body_collision=int(body[i]<=0),goal_distance_m=goalerr[i],mission_window=int(mission[i]),algorithm=cfg['algorithm'],width_m=gap,seed=cfg['seed'])
        for topic,val in telemetry.items():
            key=topic.strip('/').replace('/','_');row[key]=val[i];row[key+'_age_s']=ages[topic][i]
        raw.append(row)
    write(run/'raw.csv',raw)
    cr=[]
    for stamp,receipt,p,v,acc,traj in cmd:
        cr.append(dict(source_stamp_ns=stamp,receipt_stamp_ns=receipt,t_wall_s=(stamp-base)*1e-9,x_m=p[0],y_m=p[1],z_m=p[2],vx_m_s=v[0],vy_m_s=v[1],vz_m_s=v[2],ax_m_s2=acc[0],ay_m_s2=acc[1],az_m_s2=acc[2],trajectory_id=traj))
    write(run/'commands.csv',cr);write(run/'scalar_events.csv',scalar)
    for e in events:e.update(t_wall_s=(e['source_stamp_ns']-base)*1e-9,algorithm=cfg['algorithm'],width_m=gap,seed=cfg['seed'])
    write(run/'replan_events.csv',events)
    locals_=[e for e in events if e['event_kind'] in ['local_replan','tunnel_replan','initial_local_plan']];dur=[e['planning_time_ms'] for e in locals_]
    write(run/'replan_time_ecdf.csv',[dict(planning_time_ms=v,cumulative_probability=(i+1)/len(dur)) for i,v in enumerate(sorted(dur))])
    bins=[0,.1,.2,.5,1,2,5,10,20,50,100,1000,float('inf')];hist,_=np.histogram(dur,bins=bins)
    write(run/'replan_time_histogram.csv',[dict(lower_ms=bins[i],upper_ms=bins[i+1],count=int(v)) for i,v in enumerate(hist)])
    path=float(ds[start:finish].sum());walltime=float(t[finish]-t[start]);simtime=float(sim[finish]-sim[start])
    status=json.loads((run/'record_status.json').read_text())
    summary=dict(algorithm=cfg['algorithm'],width_m=gap,seed=cfg['seed'],trial=1,task_success=int(status['reason']=='goal_settled' and body_collision=='false'),goal_reached=int(bool(len(reached))),goal_settled=int(status['reason']=='goal_settled'),collision=body_collision,point_collision=int(swept_min<=0),traversal_time_wall_s=walltime,traversal_time_integration_s=simtime,path_length_m=path,mean_speed_path_over_wall_m_s=path/walltime,mean_speed_path_over_integration_m_s=path/simtime,mean_actual_speed_twist_wallweighted_m_s=integral_mean(speed,t),mean_actual_speed_twist_integrationweighted_m_s=integral_mean(speed,sim),max_actual_speed_twist_m_s=float(speed[mission].max()),max_speed_fd_wall_m_s=float(np.nanmax(fd[mission])),max_speed_fd_integration_m_s=float(np.nanmax(fdsim[mission])),min_true_center_obstacle_distance_m=swept_min,min_true_body_clearance_m=swept_min-radius,swept_clearance_lower_bound_m=lower-radius,min_braking_margin_center_m=float(margin[mission].min()),min_braking_margin_body_m=float(bodymargin[mission].min()),tracking_rmse_m=math.sqrt(integral_mean(err**2,t)),tracking_rmse_integrationweighted_m=math.sqrt(integral_mean(err**2,sim)),velocity_violation_5pct_samples=int(np.sum(speed[mission]>1.05*vmax[mission])),velocity_violation_5pct_wall_fraction=integral_mean((speed>1.05*vmax).astype(float),t),max_velocity_excess_m_s=float((speed-vmax)[mission].max()),replanning_attempt_count=len(locals_),replanning_success_count=sum(e['success'] for e in locals_),replanning_failure_count=sum(1-e['success'] for e in locals_),global_initial_event_count=sum(e['event_kind']=='global_initial' for e in events),global_goal_reset_event_count=sum(e['event_kind']=='global_goal_reset' for e in events),time_scale_integration_over_wall=simtime/walltime,raw_samples=len(od),mission_samples=int(mission.sum()),record_stop_reason=status['reason'])
    summary.update(stats(dur,'replan'));summary.update(stats([e['planning_time_ms'] for e in events if e['event_kind']=='global_initial'],'initial_global'))
    for topic,key in [('/risk/braking_risk','Rb'),('/risk/risk_score','Rg'),('/risk/contraction_risk','Rc')]:
        v=telemetry.get(topic,np.full(len(t),np.nan));summary[key+'_max']=float(np.nanmax(v[mission]));summary[key+'_mean_wallweighted']=integral_mean(v,t)
    # Paper-comparable -14 to +14 crossing interval, interpolated in source time.
    def crossing(x):
        ix=np.where((pos[:-1,0]<x)&(pos[1:,0]>=x)&(np.arange(len(t)-1)>=start))[0]
        if not len(ix):return None
        i=int(ix[0]);f=(x-pos[i,0])/(pos[i+1,0]-pos[i,0]);return t[i]+f*(t[i+1]-t[i])
    c0,c1=crossing(-14),crossing(14)
    summary['crossing_neg14_pos14_wall_s']=c1-c0 if c0 is not None and c1 is not None else ''
    write(run/'statistics_summary.csv',[summary])
    logevents=len(locals_) if external else len(log_local)
    finite_indices=np.where(np.isfinite(sim))[0]
    missing_only_in_prefix_suffix=bool(len(finite_indices) and np.all(np.isfinite(sim[finite_indices[0]:finite_indices[-1]+1])))
    validation=dict(source_stamps_strictly_increasing=bool(np.all(np.diff(stamps)>0)),duplicate_source_stamps=int(len(stamps)-len(np.unique(stamps))),max_source_gap_s=float(dt.max()),integration_clock_pairs=len(integr),missing_integration_pairs=int(np.isnan(sim).sum()),missing_mission_integration_pairs=int(np.isnan(sim[mission]).sum()),unpaired_only_at_recording_boundary=missing_only_in_prefix_suffix,integration_time_strictly_increasing=bool(np.all(np.diff(sim[np.isfinite(sim)])>0)),event_counts_contiguous=[e['count'] for e in events]==list(range(1,len(events)+1)),local_event_count=len(locals_),local_events_in_log=logevents,event_log_count_matches=logevents==len(locals_),raw_bag_counts_match_recorder=counts==status['counts'],map_cloud_publisher_expected_count=1,swept_segment_sampling_max_spacing_m=max_spacing,collision_test='continuous wall prism vs conservative radius .322; swept samples <=.005m; Lipschitz lower bound excludes missed intersections when positive',tracking_reference='source-stamp linear interpolation of PositionCommand; no receipt-time matching; only command-covered mission window',float_topic_timestamp_basis='receipt only (Float64 has no source timestamp); ages and scalar_events retained',speed_limit_test='actual twist norm > received active limit * 1.05; lagged constraint can legitimately be exceeded, never clipped',dynamics_to_wall_time_ratio=simtime/walltime,twist_vs_integration_fd_median_abs_error_m_s=float(np.nanmedian(abs(speed[mission]-fdsim[mission]))),twist_vs_wall_fd_median_abs_error_m_s=float(np.nanmedian(abs(speed[mission]-fd[mission]))))
    (run/'validation.json').write_text(json.dumps(validation,indent=2))
    print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
