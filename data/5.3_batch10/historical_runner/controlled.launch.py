"""Use frozen original launch parameters; start perception before the planner."""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    base=os.environ['SECTION53_FROZEN_LAUNCH']
    def include(name,args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(base,name)),launch_arguments=args.items())
    sim=include('simulator.launch.py',dict(use_dynamic='true',drone_id='0',map_size_x_='50.0',map_size_y_='25.0',map_size_z_='2.0',init_x_='-15.0',init_y_='0.0',init_z_='0.1',odometry_topic='visual_slam/odom'))
    planner=include('advanced_param.launch.py',dict(drone_id='0',map_size_x_='50.0',map_size_y_='25.0',map_size_z_='2.0',odometry_topic='visual_slam/odom',obj_num_set='0',camera_pose_topic='pcl_render_node/camera_pose',depth_topic='pcl_render_node/depth',cloud_topic='pcl_render_node/cloud',cx='321.04638671875',cy='243.44969177246094',fx='387.229248046875',fy='387.229248046875',max_vel='2.0',max_acc='6.0',planning_horizon='7.5',use_distinctive_trajs='true',flight_type='2',point_num='1',point0_x='15.0',point0_y='0.0',point0_z='1.0'))
    server=Node(package='ego_planner',executable='traj_server',name='drone_0_traj_server',output='screen',remappings=[('position_cmd','drone_0_planning/pos_cmd'),('planning/bspline','drone_0_planning/bspline')],parameters=[{'traj_server/time_forward':1.0}])
    sensor=Node(package='sensor_adapter',executable='sensor_adapter_node',name='sensor_adapter',output='screen',parameters=[{'input_odom_topic':'/drone_0_visual_slam/odom','input_cloud_topic':'/drone_0_pcl_render_node/cloud'}])
    algorithm=os.environ['SECTION53_ALGORITHM']
    risk_params=[{'braking/use_d0':True}] if algorithm=='coupled_legacy' else []
    risk=Node(package='risk_evaluator',executable='risk_evaluator_node',name='risk_evaluator',output='screen',parameters=risk_params)
    actions=[sim,server,sensor,risk]
    if algorithm not in ['fixed_ego_swarm','risk_decoupled','speed_only','coupled_legacy','gate_off']:
        raise ValueError('Unknown ablation '+algorithm)
    adaptive_params=[]
    if algorithm=='speed_only':
        adaptive_params=[{'safe/d_max':0.5,'weight/lambda_max':0.5}]
    elif algorithm=='gate_off':
        adaptive_params=[{'gate/enabled':False}]
    if algorithm!='fixed_ego_swarm':actions.append(Node(package='adaptive_planner',executable='adaptive_planner_node',name='adaptive_planner',output='screen',parameters=adaptive_params))
    actions.append(TimerAction(period=5.0,actions=[planner]))
    return LaunchDescription(actions)
