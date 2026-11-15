import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Fixed-Parameter EGO-Swarm baseline (paper §5.3 control arm).

    Same full-flight stack as risk_data_collect.launch.py EXCEPT the
    adaptive_planner node is omitted, so the planner keeps its fixed
    max_vel / safe_distance / collision_weight. sensor_adapter + risk_evaluator
    still run to *measure* risk for a fair metric comparison.
    """
    point_num = LaunchConfiguration('point_num', default='1')
    use_wall_map = LaunchConfiguration('use_wall_map', default='true')
    use_dynamic = LaunchConfiguration('use_dynamic', default='true')
    gap = LaunchConfiguration('gap', default='1.4')
    seed = LaunchConfiguration('seed', default='42')

    ego_share = get_package_share_directory('ego_planner')
    single_run = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ego_share, 'launch', 'single_run_in_sim.launch.py')),
        launch_arguments={
            'use_wall_map': use_wall_map,
            'point_num': point_num,
            'use_dynamic': use_dynamic,
            'gap': gap,
            'seed': seed,
        }.items(),
    )

    sensor_adapter = Node(
        package='sensor_adapter', executable='sensor_adapter_node',
        name='sensor_adapter', output='screen',
        parameters=[
            {'input_odom_topic': 'drone_0_visual_slam/odom'},
            {'input_cloud_topic': 'drone_0_pcl_render_node/cloud'},
        ],
    )

    risk_evaluator = Node(
        package='risk_evaluator', executable='risk_evaluator_node',
        name='risk_evaluator', output='screen',
    )

    # NOTE: adaptive_planner intentionally omitted -> fixed-parameter baseline.

    return LaunchDescription([
        single_run,
        sensor_adapter,
        risk_evaluator,
    ])
