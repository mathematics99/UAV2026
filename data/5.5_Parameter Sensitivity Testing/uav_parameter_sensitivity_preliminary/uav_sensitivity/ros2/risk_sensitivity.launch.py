"""Minimal ROS 2 launch to test risk weights and EMA on the supplied EGO map.

Copy into ego_planner/launch within the original ROS 2 source workspace.
Relies on the original packages already built and sourced.
This file was syntax-checked without ROS; runtime verification is required.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ego_share = get_package_share_directory('ego_planner')
    gap = LaunchConfiguration('gap')
    wd = LaunchConfiguration('w_density')
    wb = LaunchConfiguration('w_braking')
    wc = LaunchConfiguration('w_contraction')
    ema = LaunchConfiguration('ema')

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ego_share, 'launch', 'single_run_in_sim.launch.py')),
        launch_arguments={
            'use_wall_map': 'true',
            'point_num': '1',
            'gap': gap,
            'max_vel': '2.0',
        }.items(),
    )
    sensor = Node(
        package='sensor_adapter', executable='sensor_adapter_node',
        name='sensor_adapter', output='screen',
        parameters=[{
            'input_odom_topic': 'drone_0_visual_slam/odom',
            'input_cloud_topic': 'drone_0_pcl_render_node/cloud',
        }],
    )
    risk = Node(
        package='risk_evaluator', executable='risk_evaluator_node',
        name='risk_evaluator', output='screen',
        parameters=[{
            'weight/density': ParameterValue(wd, value_type=float),
            'weight/braking': ParameterValue(wb, value_type=float),
            'weight/contraction': ParameterValue(wc, value_type=float),
        }],
    )
    planner = Node(
        package='adaptive_planner', executable='adaptive_planner_node',
        name='adaptive_planner', output='screen',
        parameters=[{'smoothing/lambda': ParameterValue(ema, value_type=float)}],
    )
    return LaunchDescription([
        DeclareLaunchArgument('gap', default_value='1.4'),
        DeclareLaunchArgument('w_density', default_value='0.20'),
        DeclareLaunchArgument('w_braking', default_value='0.55'),
        DeclareLaunchArgument('w_contraction', default_value='0.25'),
        DeclareLaunchArgument('ema', default_value='0.90'),
        base, sensor, risk, planner,
    ])
