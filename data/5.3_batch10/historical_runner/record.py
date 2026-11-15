"""Lossless CDR recording; one callback per source sample, no polling of cached odometry."""
import argparse
import json
import time
import shutil
import tempfile
import queue
import threading
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

FLOATS=['/risk/'+s for s in ['min_distance','obstacle_density','contraction_global','contraction_gradient','contraction_risk','braking_risk','risk_score','current_velocity','braking_distance','timing_ms']]+['/adaptive/'+s for s in ['smoothed_risk','smoothed_contraction','max_velocity','safe_distance','collision_weight','contraction_gate','timing_ms']]
TOPICS={t:'std_msgs/msg/Float64' for t in FLOATS}
TOPICS.update({'/drone_0_visual_slam/odom':'nav_msgs/msg/Odometry','/experiment/integration_odom':'nav_msgs/msg/Odometry','/drone_0_planning/pos_cmd':'quadrotor_msgs/msg/PositionCommand','/planner/replan_event':'traj_utils/msg/ReplanEvent','/drone_0_planning/bspline':'traj_utils/msg/Bspline','/drone_0_pcl_render_node/cloud':'sensor_msgs/msg/PointCloud2','/map_generator/global_cloud':'sensor_msgs/msg/PointCloud2'})

def main():
    p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--timeout',type=float,default=100);a=p.parse_args()
    out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
    rclpy.init();n=Node('section53_recorder');w=rosbag2_py.SequentialWriter()
    staging=Path(tempfile.mkdtemp(prefix='section53_record_'))
    (out/'staging_bag_path.txt').write_text(str(staging/'rosbag'))
    w.open(rosbag2_py.StorageOptions(uri=str(staging/'rosbag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
    counts={t:0 for t in TOPICS};state={'done':False,'settled':None,'reason':'timeout','max_x':-15.0}
    pending=queue.Queue();writer_errors=[]
    def write_pending():
        try:
            while True:
                item=pending.get()
                if item is None:break
                w.write(*item)
        except BaseException as e:writer_errors.append(str(e))
    worker=threading.Thread(target=write_pending)
    cutoff=None
    def cb(topic,msg):
        nonlocal cutoff
        if cutoff is not None:
            if topic=='/drone_0_visual_slam/odom' and msg.header.stamp.sec*10**9+msg.header.stamp.nanosec>cutoff:return
            if topic=='/experiment/integration_odom' and int(msg.child_frame_id)>cutoff:return
        # Static map content is immutable; one lossless map sample fully defines the run.
        if topic=='/map_generator/global_cloud' and counts[topic]:return
        pending.put((topic,serialize_message(msg),n.get_clock().now().nanoseconds));counts[topic]+=1
        if topic=='/drone_0_visual_slam/odom':
            p=msg.pose.pose.position;v=msg.twist.twist.linear
            state['max_x']=max(state['max_x'],p.x)
            if ((p.x-15)**2+p.y**2+(p.z-1)**2)**.5<.2 and (v.x*v.x+v.y*v.y+v.z*v.z)**.5<.2:
                if state['settled'] is None:state['settled']=time.monotonic()
                elif time.monotonic()-state['settled']>=1.0:state.update(done=True,reason='goal_settled')
            else:state['settled']=None
            if state['done'] and cutoff is None:cutoff=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
    for topic,typ in TOPICS.items():
        w.create_topic(rosbag2_py.TopicMetadata(name=topic,type=typ,serialization_format='cdr'))
        reliability=ReliabilityPolicy.BEST_EFFORT if topic in ['/drone_0_pcl_render_node/cloud','/super/pos_cmd','/super/poly_traj'] else ReliabilityPolicy.RELIABLE
        n.create_subscription(get_message(typ),topic,lambda msg,t=topic:cb(t,msg),QoSProfile(depth=10000,reliability=reliability))
    worker.start()
    (out/'recorder_ready').write_text(str(time.time_ns()))
    start=time.monotonic()
    try:
        while rclpy.ok() and not state['done'] and time.monotonic()-start<a.timeout:rclpy.spin_once(n,timeout_sec=.1)
        # Drain diagnostics already in flight, while holding the final odometry cutoff.
        if state['done']:
            # Keep collecting diagnostics for 0.3 seconds; pairing is audited offline.
            until=time.monotonic()+.3
            while rclpy.ok() and time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.01)
    except KeyboardInterrupt:state['reason']='interrupted'
    finally:
        pending.put(None);worker.join()
        if writer_errors:raise RuntimeError(writer_errors)
        (out/'record_status.json').write_text(json.dumps(dict(state,counts=counts,wall_elapsed_s=time.monotonic()-start,writer_queue_drained=True),indent=2))
        del w;n.destroy_node();rclpy.try_shutdown()
        (out/'capture_complete').write_text('writer closed; immutable local bag ready for export')
        shutil.copytree(staging/'rosbag',out/'rosbag')

if __name__=='__main__':main()
