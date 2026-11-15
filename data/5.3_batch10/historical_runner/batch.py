"""100 serial full-flight repetitions using the previously validated protocol."""
import argparse,csv,hashlib,json,os,random,shutil,signal,subprocess,time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2];S=Path(__file__).resolve().parent
OUT=ROOT/'experiments/section53_batch10_20261106';OLD=ROOT/'experiments/5.3加统计数据'
METHODS=['fixed_ego_swarm','speed_only','coupled_legacy','gate_off','risk_decoupled']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    p=argparse.ArgumentParser();p.add_argument('--resume',action='store_true');args=p.parse_args()
    OUT.mkdir(exist_ok=True);frozen=OUT/'reproducibility';frozen.mkdir(exist_ok=True)
    launch=frozen/'original_launch';launch.mkdir(exist_ok=True)
    for name in ['simulator.launch.py','advanced_param.launch.py']:shutil.copy2(OLD/'reproducibility/original_launch'/name,launch/name)
    integrity=[]
    with (OLD/'algorithm_integrity.csv').open() as f:
        for e in csv.DictReader(f):
            current=sha(ROOT/e['source']);runtime=Path('/home/ubuntu/ros2_ws/src/ego-planner-swarm')/e['source']
            integrity.append(dict(source=e['source'],old=e['workspace_sha256'],current=current,runtime=sha(runtime),same=current==e['workspace_sha256']==sha(runtime)))
    (frozen/'algorithm_integrity.json').write_text(json.dumps(integrity,indent=2));assert all(e['same'] for e in integrity)
    bins=[Path('/home/ubuntu/ros2_ws/install')/p for p in ['ego_planner/lib/ego_planner/ego_planner_node','ego_planner/lib/ego_planner/traj_server','risk_evaluator/lib/risk_evaluator/risk_evaluator_node','adaptive_planner/lib/adaptive_planner/adaptive_planner_node']]
    expected={str(p):sha(p) for p in bins};(frozen/'runtime_binary_sha256.json').write_text(json.dumps(expected,indent=2))
    shutil.copytree(S,frozen/'scripts',dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
    shutil.copy2(ROOT/'scripts/section53/map.py',frozen/'map.py')
    rng=random.Random(1);matrix=[]
    for trial in range(1,11):
        conditions=[(m,w) for w in [1.0,1.4] for m in METHODS];rng.shuffle(conditions)
        offset=len(matrix)
        matrix.extend([dict(order=offset+i+1,trial=trial,algorithm=m,width_m=w,seed=1) for i,(m,w) in enumerate(conditions)])
    (OUT/'experiment_matrix.json').write_text(json.dumps(matrix,indent=2))
    env=os.environ.copy();env.update(ROS_DOMAIN_ID='176',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',PYTHONNOUSERSITE='1',PYTHONUNBUFFERED='1',SECTION53_FROZEN_LAUNCH=str(launch))
    for row in matrix:
        label=f"{row['algorithm']}_w{row['width_m']:.1f}_r{row['trial']:02d}";run=OUT/'runs'/label
        if args.resume and (run/'batch_metrics.csv').exists():
            v=json.loads((run/'validation.json').read_text());assert v['source_stamps_strictly_increasing'] and v['event_log_count_matches'] and v['raw_bag_counts_match_recorder'],'Existing run failed validation: '+label
            print('SKIP validated existing attempt',label,flush=True);continue
        if args.resume and (run/'capture_complete').exists() and (run/'rosbag/metadata.yaml').exists():
            state=json.loads((run/'record_status.json').read_text());assert state['reason'] in ['goal_settled','timeout'],'Incomplete interrupted flight needs separate audit'
            for script in ['analyze_base.py','metrics.py']:
                with (run/(script+'.log')).open('w') as f:subprocess.run(['/usr/bin/python3',str(S/script),str(run)],env=env,stdout=f,stderr=subprocess.STDOUT,check=True)
            v=json.loads((run/'validation.json').read_text());assert v['source_stamps_strictly_increasing'] and v['event_log_count_matches'] and v['raw_bag_counts_match_recorder']
            print('RECOVERED complete saved recording',label,flush=True);continue
        if run.exists():raise RuntimeError('Existing incomplete run requires audit: '+label)
        assert all(sha(Path(p))==h for p,h in expected.items()),'Runtime changed during batch'
        logs=run/'logs';logs.mkdir(parents=True);cfg=dict(row,start=[-15.,0.,.1],goal=[15.,0.,1.],map_size_m=[50.,25.,2.],max_velocity=2.,max_acceleration=6.,body_collision_radius_m=.322,goal_tolerance_m=.2,goal_speed_tolerance_m_s=.2,settle_s=1.,timeout_s=80.,domain_id=176,braking_tau=.3,braking_a_b=2.,braking_d0_m=.3,braking_d0_enabled=row['algorithm']=='coupled_legacy',planner_rng='unchanged glibc default rand seed 1, fresh process; same deterministic map',repeat_scope='conditional repeatability under OS/ROS scheduling; not independent maps',protocol='original section53 first command to first goal entry; success additionally requires 1 s settling; no shared-preflight adapter')
        (run/'run_parameters.json').write_text(json.dumps(cfg,indent=2));env['SECTION53_ALGORITHM']=row['algorithm']
        processes=[];handles=[]
        def start(cmd,name):
            h=(logs/(name+'.log')).open('w');handles.append(h);q=subprocess.Popen(cmd,env=env,stdout=h,stderr=subprocess.STDOUT,start_new_session=True);processes.append((name,q));(logs/(name+'_command.json')).write_text(json.dumps(cmd));return q
        print('START',row['order'],'/100',label,time.strftime('%H:%M:%S'),flush=True)
        try:
            start(['/usr/bin/python3',str(ROOT/'scripts/section53/map.py'),'--gap',str(row['width_m']),'--out',str(run/'map')],'map')
            rec=start(['/usr/bin/python3',str(S/'record.py'),'--out',str(run),'--timeout','80'],'recorder')
            until=time.monotonic()+20
            while not (run/'recorder_ready').exists():
                if rec.poll() is not None or time.monotonic()>until:raise RuntimeError('Recorder initialization failed')
                time.sleep(.1)
            sim=start(['ros2','launch',str(S/'controlled.launch.py')],'simulation');time.sleep(9)
            nodes=['drone_0_ego_planner_node','drone_0_quadrotor_simulator_so3','drone_0_so3_control_component','risk_evaluator','sensor_adapter','drone_0_traj_server','drone_0_pcl_render_node']+([] if row['algorithm']=='fixed_ego_swarm' else ['adaptive_planner'])
            with (logs/'parameter_audit.log').open('w') as f:subprocess.run(['/usr/bin/python3',str(S/'dump_parameters.py'),str(logs),*nodes],env=env,stdout=f,stderr=subprocess.STDOUT,timeout=22,check=True)
            while not (run/'capture_complete').exists():
                if sim.poll() is not None or rec.poll() is not None:raise RuntimeError('Capture/simulation exited before completion')
                time.sleep(.2)
        finally:
            # Stop physics as soon as local recording closes, before NTFS bag copying.
            for name,q in reversed(processes):
                if name!='recorder' and q.poll() is None:os.killpg(q.pid,signal.SIGINT)
            for name,q in processes:
                try:q.wait(timeout=60 if name=='recorder' else 8)
                except subprocess.TimeoutExpired:
                    os.killpg(q.pid,signal.SIGTERM)
                    try:q.wait(timeout=3)
                    except subprocess.TimeoutExpired:os.killpg(q.pid,signal.SIGKILL);q.wait()
            for h in handles:h.close()
            (run/'process_status.json').write_text(json.dumps({n:q.returncode for n,q in processes},indent=2))
        for script in ['analyze_base.py','metrics.py']:
            with (run/(script+'.log')).open('w') as f:subprocess.run(['/usr/bin/python3',str(S/script),str(run)],env=env,stdout=f,stderr=subprocess.STDOUT,check=True)
        validation=json.loads((run/'validation.json').read_text())
        assert validation['source_stamps_strictly_increasing'] and validation['event_log_count_matches'] and validation['raw_bag_counts_match_recorder'],'Primary recording audit failed'
        assert all(sha(Path(p))==h for p,h in expected.items()),'Runtime changed during trial'
        with (run/'batch_metrics.csv').open() as f:summary=next(csv.DictReader(f))
        print('END',label,'success',summary['task_success'],'T',summary['traversal_time_wall_s'],'RMSE',summary['tracking_rmse_m'],flush=True)
        (OUT/'progress.json').write_text(json.dumps(dict(completed=row['order'],total=100,last=label),indent=2))
    (OUT/'BATCH_COMPLETE').write_text('100 recorded and primary-validated flights')

if __name__=='__main__':main()
