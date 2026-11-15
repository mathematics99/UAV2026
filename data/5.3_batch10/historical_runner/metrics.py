"""Frozen physical diagnostics; no parameter selection from new-batch rankings."""
import csv,json,math,sys
from pathlib import Path
import numpy as np
import yaml
from analyze_base import write
run=Path(sys.argv[1]);cfg=json.loads((run/'run_parameters.json').read_text())
with (run/'statistics_summary.csv').open() as f:s=next(csv.DictReader(f))
with (run/'raw.csv').open() as f:r=list(csv.DictReader(f))
active=[x for x in r if x['mission_window']=='1'];t=np.array([float(x['t_wall_s']) for x in active]);duration=t[-1]-t[0]
def array(k):return np.array([float(x[k]) for x in active])
def integral(x):
    good=np.isfinite(x);return float(np.trapz(x[good],t[good])) if np.count_nonzero(good)>1 else math.nan
v=array('speed_twist_m_s');d=array('center_wall_distance_m');body=d-.322;brake=.3*v+v*v/4;mc=d-brake;mb=body-brake;err=array('tracking_error_m')
jtrack=integral(err**2);coverage=t[np.isfinite(err)][-1]-t[np.isfinite(err)][0];jbody=integral(np.maximum(-mb,0));jcenter=integral(np.maximum(-mc,0));fraccenter=integral((mc<0).astype(float))/duration;fracbody=integral((mb<0).astype(float))/duration
success=s['task_success']=='1'
s.update(trial=cfg['trial'],block=cfg['trial'],order=cfg['order'],observed_duration_wall_s=duration,observed_partial_path_length_m=s['path_length_m'],tracking_reference_coverage_s=coverage,tracking_ise_m2_s=jtrack,tracking_identity_abs_error=abs(jtrack-coverage*float(s['tracking_rmse_m'])**2),center_margin_negative_fraction=fraccenter,body_margin_negative_fraction=fracbody,center_braking_deficit_m_s=jcenter,body_braking_deficit_m_s=jbody,normalized_safety_efficiency_cost=(duration+jbody/.322)/15 if success else '',safety_efficiency_valid=success,reference_time_s=15.,reference_clearance_m=.322,min_dynamic_center_margin_m=float(mc.min()),min_dynamic_body_margin_m=float(mb.min()),configured_min_center_margin_m=float((mc-(.3 if cfg['algorithm']=='coupled_legacy' else 0)).min()),complete_record_duration_s=float(r[-1]['t_wall_s'])-float(r[0]['t_wall_s']),evaluation_scope='first command to first goal entry; settled completion separately required',clock_primary='source wall time; integration-time diagnostics retained',risk_scores_comparable=False)
s.update(time_to_first_goal_entry_wall_s=s['traversal_time_wall_s'] if s['goal_reached']=='1' else '',terminal_goal_distance_m=float(r[-1]['goal_distance_m']),terminal_speed_m_s=float(r[-1]['speed_twist_m_s']),outcome_reason='successful_settled_completion' if success else ('terminal_settling_not_met_after_goal_entry' if s['goal_reached']=='1' else 'goal_not_reached'))
if not success:s['traversal_time_wall_s']='';s['traversal_time_integration_s']=''
for k,val in list(s.items()):
    if isinstance(val,str) and val.lower() in ['nan','inf','-inf']:s[k]=''
write(run/'batch_metrics.csv',[s])
# Fixed mathematical settings; check actual parameter dumps rather than inferred labels.
def params(name):return next(iter(yaml.safe_load((run/'logs'/(name+'_parameters.yaml')).read_text()).values()))['ros__parameters']
risk=params('risk_evaluator');assert risk['braking/use_d0']==(cfg['algorithm']=='coupled_legacy')
checks=dict(risk_static_distance_switch_correct=True,tracking_identity_abs_error=s['tracking_identity_abs_error'],same_dynamic_braking_definition_for_all=True,clock_pairing=json.loads((run/'validation.json').read_text())['missing_mission_integration_pairs'])
if cfg['algorithm']!='fixed_ego_swarm':
    a=params('adaptive_planner');checks['gate_switch_correct']=a['gate/enabled']==(cfg['algorithm']!='gate_off');checks['mapping_upper_bounds_correct']=a['safe/d_max']==(.5 if cfg['algorithm']=='speed_only' else 1.) and a['weight/lambda_max']==(.5 if cfg['algorithm']=='speed_only' else 2.)
    assert checks['gate_switch_correct'] and checks['mapping_upper_bounds_correct']
assert s['tracking_identity_abs_error']<1e-8
(run/'metric_validation.json').write_text(json.dumps(checks,indent=2))
print(json.dumps(s,indent=2))
