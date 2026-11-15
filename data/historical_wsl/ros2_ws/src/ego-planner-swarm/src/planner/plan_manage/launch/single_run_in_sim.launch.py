import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # 定义参数的 LaunchConfiguration
    obj_num = LaunchConfiguration('obj_num', default=10)
    drone_id = LaunchConfiguration('drone_id', default=0)
    point_num = LaunchConfiguration('point_num', default=4)

    map_size_x = LaunchConfiguration('map_size_x', default = 50.0)
    map_size_y = LaunchConfiguration('map_size_y', default = 25.0)
    map_size_z = LaunchConfiguration('map_size_z', default = 2.0)
    odom_topic = LaunchConfiguration('odom_topic', default = 'visual_slam/odom')
    
    
    # 声明全局参数
    obj_num_cmd = DeclareLaunchArgument('obj_num', default_value=obj_num, description='Number of objects')
    drone_id_cmd = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')
    point_num_cmd = DeclareLaunchArgument('point_num', default_value=point_num, description='Number of waypoints')

    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')

    # 地图属性以及是否使用动力学仿真
    use_mockamap = LaunchConfiguration('use_mockamap', default=False) # map_generator or mockamap

    use_mockamap_cmd = DeclareLaunchArgument('use_mockamap', default_value=use_mockamap, description='Choose map type, map_generator or mockamap')

    use_wall_map = LaunchConfiguration('use_wall_map', default=False) # 窄通道压测: 外部窄通道地图发布器

    use_wall_map_cmd = DeclareLaunchArgument('use_wall_map', default_value=use_wall_map, description='Use external narrow-channel map (skip map_generator/mockamap)')

    use_dynamic = LaunchConfiguration('use_dynamic', default=False)
    use_dynamic_cmd = DeclareLaunchArgument('use_dynamic', default_value=use_dynamic, description='Use Drone Simulation Considering Dynamics or Not')

    max_vel = LaunchConfiguration('max_vel', default=2.0) # 最大速度(入口速度扫参实验用)
    max_vel_cmd = DeclareLaunchArgument('max_vel', default_value=max_vel, description='Maximum velocity')

    init_y = LaunchConfiguration('init_y', default=0.0) # 起始横向偏移 (seed 扰动用)
    init_y_cmd = DeclareLaunchArgument('init_y', default_value=init_y, description='Initial lateral offset (seed perturbation)')

    gap = LaunchConfiguration('gap', default=1.0) # 漏斗喉部宽度 (use_wall_map:=true 时生效)
    gap_cmd = DeclareLaunchArgument('gap', default_value=gap, description='Funnel throat width in meters (use_wall_map:=true)')

    wide = LaunchConfiguration('wide', default=5.0) # 漏斗开口宽度 (收缩力度 = wide/gap)
    wide_cmd = DeclareLaunchArgument('wide', default_value=wide, description='Funnel opening width in meters (use_wall_map:=true)')

    seed = LaunchConfiguration('seed', default='42') # 窄通道地图随机种子 (障碍布局随机化)
    seed_cmd = DeclareLaunchArgument('seed', default_value=seed, description='Random seed for the narrow-corridor map (obstacle layout)')

    # map
    map_generator_node = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='screen',
        parameters=[
            {'map/x_size': 26.0},
            {'map/y_size': 20.0},
            {'map/z_size': 3.0},
            {'map/resolution': 0.1},
            {'ObstacleShape/seed': 1.0},
            {'map/obs_num': 250},
            {'ObstacleShape/lower_rad': 0.5},
            {'ObstacleShape/upper_rad': 0.7},
            {'ObstacleShape/lower_hei': 0.0},
            {'ObstacleShape/upper_hei': 3.0},
            {'map/circle_num': 250},
            {'ObstacleShape/radius_l': 0.7},
            {'ObstacleShape/radius_h': 0.5},
            {'ObstacleShape/z_l': 0.7},
            {'ObstacleShape/z_h': 0.8},
            {'ObstacleShape/theta': 0.5},
            {'pub_rate': 1.0},
            {'min_distance': 0.8}
        ],
        condition = IfCondition(PythonExpression([
            "not (('", use_mockamap, "').lower() == 'true') and "
            "not (('", use_wall_map, "').lower() == 'true')"
        ]))
    )

    mockamap_node = Node(
        package='mockamap',
        executable='mockamap_node',
        name='mockamap_node',
        output='screen',
        remappings=[
            ('/mock_map', '/map_generator/global_cloud')
        ],
        parameters=[
            {'seed': 127},
            {'update_freq': 0.5},
            {'resolution': 0.1},
            {'x_length': PythonExpression(['int(', map_size_x, ')'])},
            {'y_length': PythonExpression(['int(', map_size_y, ')'])},
            {'z_length': PythonExpression(['int(', map_size_z, ')'])},
            {'type': 1},
            {'complexity': 0.05},
            {'fill': 0.12},
            {'fractal': 1},
            {'attenuation': 0.1}
        ],
        condition = IfCondition(PythonExpression([
            "(('", use_mockamap, "').lower() == 'true') and "
            "not (('", use_wall_map, "').lower() == 'true')"
        ]))
    )

    # 窄通道地图发布器 (use_wall_map:=true 时替代 map_generator/mockamap)
    # 注意: LaunchConfiguration 作为 Substitution 必须独立成列表元素, 不能与字符串直接 + 拼接
    _pkg_share = get_package_share_directory('ego_planner')
    _script_path = os.path.join(_pkg_share, 'launch', 'narrow_corridor_map.py')
    wall_map_pub = ExecuteProcess(
        cmd=[
            'python3', _script_path,
            '--gap',         PythonExpression(["str(", gap, ")"]),
            '--wide',        PythonExpression(["str(", wide, ")"]),
            '--init-x',      '-15.0',
            '--goal-x',      '15.0',
            '--funnel-x',    '0.0',
            '--funnel-len',  '5.0',
            '--seed',        PythonExpression(["str(", seed, ")"]),
            '--rate',        '2.0',
        ],
        output='screen',
        name='narrow_corridor_map_exec',
        condition=IfCondition(PythonExpression(["'", use_wall_map, "' == 'True' or '", use_wall_map, "' == 'true'"]))
    )

    # Include advanced parameters
    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': odom_topic,
            'obj_num_set': obj_num,
            
            'camera_pose_topic': 'pcl_render_node/camera_pose',
            'depth_topic': 'pcl_render_node/depth',
            'cloud_topic': 'pcl_render_node/cloud',
            
            'cx': str(321.04638671875),
            'cy': str(243.44969177246094),
            'fx': str(387.229248046875),
            'fy': str(387.229248046875),
            'max_vel': max_vel,
            'max_acc': str(6.0),
            'planning_horizon': str(7.5),
            'use_distinctive_trajs': 'True',
            'flight_type': str(2),
            'point_num': point_num,
            'point0_x': str(15.0),
            'point0_y': str(0.0),
            'point0_z': str(1.0),
            
            'point1_x': str(-15.0),
            'point1_y': str(0.0),
            'point1_z': str(1.0),
            
            'point2_x': str(15.0),
            'point2_y': str(0.0),
            'point2_z': str(1.0),
            
            'point3_x': str(-15.0),
            'point3_y': str(0.0),
            'point3_z': str(1.0),
            
            'point4_x': str(15.0),
            'point4_y': str(0.0),
            'point4_z': str(1.0),
        }.items()
    )
    
    # Trajectory server node
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        remappings=[
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline'])
        ],
        parameters=[
            {'traj_server/time_forward': 1.0}
        ]
    )
    
    # Include simulator 
    simulator_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ego_planner'), 'launch', 'simulator.launch.py')),
        launch_arguments={
            'use_dynamic': use_dynamic,
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'init_x_': str(-15.0),
            'init_y_': init_y,
            'init_z_': str(0.1),
            'odometry_topic': odom_topic
        }.items()
    )
    
    ld = LaunchDescription()
        
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(obj_num_cmd)
    ld.add_action(drone_id_cmd)
    ld.add_action(point_num_cmd)
    ld.add_action(use_dynamic_cmd)
    ld.add_action(use_mockamap_cmd)
    ld.add_action(use_wall_map_cmd)
    ld.add_action(max_vel_cmd)
    ld.add_action(init_y_cmd)
    ld.add_action(gap_cmd)
    ld.add_action(wide_cmd)
    ld.add_action(seed_cmd)

    # 添加 Map Generator 节点
    ld.add_action(map_generator_node)
    ld.add_action(mockamap_node)
    ld.add_action(wall_map_pub)
    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)
    ld.add_action(simulator_include)

    return ld