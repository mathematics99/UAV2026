"""Bounded direct ROS parameter-service collection, avoiding CLI discovery hangs."""
import json,sys,time
from pathlib import Path
import rclpy,yaml
from rcl_interfaces.srv import ListParameters,GetParameters
from rclpy.parameter import parameter_value_to_python
rclpy.init();n=rclpy.create_node('section53_parameter_audit');out=Path(sys.argv[1]);nodes=sys.argv[2:];pending={};results={}
def request(name,phase):
    typ=ListParameters if phase=='list' else GetParameters
    c=n.create_client(typ,'/'+name+('/list_parameters' if phase=='list' else '/get_parameters'))
    req=typ.Request()
    if phase=='list':req.depth=0
    else:req.names=results[name]['names']
    return dict(client=c,future=c.call_async(req),phase=phase,start=time.monotonic(),tries=1)
for name in nodes:pending[name]=request(name,'list')
until=time.monotonic()+18
while pending and time.monotonic()<until:
    rclpy.spin_once(n,timeout_sec=.05)
    for name,state in list(pending.items()):
        f=state['future']
        if f.done():
            if state['phase']=='list':results[name]={'names':list(f.result().result.names)};pending[name]=request(name,'get')
            else:
                values={k:parameter_value_to_python(v) for k,v in zip(results[name]['names'],f.result().values)}
                (out/(name+'_parameters.yaml')).write_text(yaml.safe_dump({'/'+name:{'ros__parameters':values}},sort_keys=True));del pending[name]
        elif time.monotonic()-state['start']>5 and state['tries']==1:
            pending[name]=request(name,state['phase']);pending[name]['tries']=2
n.destroy_node();rclpy.try_shutdown()
if pending:raise RuntimeError('Parameter service unavailable: '+str(list(pending)))
print(json.dumps({'dumped':nodes}))
