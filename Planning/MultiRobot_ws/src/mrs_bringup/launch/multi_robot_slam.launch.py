"""Bring up the multi-robot 2D SLAM testbed.

Launches Gazebo with the warehouse world, spawns `n_robots` differential-drive
UGVs, gives each one an independent slam_toolbox instance, and starts the
coordination layer (comms link model + map merger).

Everything is namespaced per robot (/robotN/...) while TF stays global with
per-robot frame prefixes (robotN/odom, robotN/base_footprint, robotN/map), so a
single RViz instance can display the whole fleet.

    ros2 launch mrs_bringup multi_robot_slam.launch.py n_robots:=2
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def _load_fleet(bringup_share):
    with open(os.path.join(bringup_share, 'config', 'fleet.yaml')) as handle:
        return yaml.safe_load(handle)['fleet']['robots']


def _spawn_fleet(context, *args, **kwargs):
    bringup_share = get_package_share_directory('mrs_bringup')
    description_share = get_package_share_directory('mrs_description')

    n_robots = int(LaunchConfiguration('n_robots').perform(context))
    def flag(name):
        return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')

    use_explorer = flag('use_explorer')
    use_slam = flag('use_slam')

    xacro_file = os.path.join(description_share, 'urdf', 'ugv.urdf.xacro')
    slam_params = os.path.join(bringup_share, 'config', 'slam_toolbox_async.yaml')

    robots = _load_fleet(bringup_share)
    if n_robots > len(robots):
        raise RuntimeError(
            f'requested {n_robots} robots but fleet.yaml only defines {len(robots)}')
    robots = robots[:n_robots]

    actions = []
    for robot in robots:
        name = robot['name']
        prefix = f'{name}/'
        pose = robot['initial_pose']

        # ParameterValue(..., str) is required: the expanded URDF is XML and
        # launch would otherwise try to parse it as YAML.
        robot_description = ParameterValue(
            Command([
                'xacro ', xacro_file,
                ' namespace:=', name,
                ' prefix:=', prefix,
            ]),
            value_type=str)

        group = [
            PushRosNamespace(name),

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'robot_description': robot_description,
                    'frame_prefix': prefix,
                }],
            ),

            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_entity',
                output='screen',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', name,
                    '-robot_namespace', name,
                    '-x', str(pose['x']),
                    '-y', str(pose['y']),
                    '-z', '0.05',
                    '-Y', str(pose['yaw']),
                ],
            ),

        ]

        if use_slam:
            group.append(Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_params,
                    {
                        'use_sim_time': True,
                        'odom_frame': f'{prefix}odom',
                        'base_frame': f'{prefix}base_footprint',
                        'map_frame': f'{prefix}map',
                        'scan_topic': f'/{name}/scan',
                        'map_name': f'/{name}/map',
                    },
                ],
            ))

        if use_explorer:
            group.append(Node(
                package='mrs_coordination',
                executable='explorer_node',
                name='explorer',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'robot_name': name,
                    'base_frame': f'{prefix}base_footprint',
                }],
            ))

        actions.append(GroupAction(group))

    # Coordination layer: shared across the whole fleet, so it lives outside the
    # per-robot namespaces.
    robot_names = [r['name'] for r in robots]
    init_poses = []
    for r in robots:
        init_poses += [
            float(r['initial_pose']['x']),
            float(r['initial_pose']['y']),
            float(r['initial_pose']['yaw']),
        ]

    actions.append(Node(
        package='mrs_coordination',
        executable='comms_link_node',
        name='comms_link',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_names': robot_names,
            'dropout_enabled': LaunchConfiguration('comms_dropout'),
            'uptime_sec': 45.0,
            'downtime_sec': 25.0,
            'phase_offset_sec': 12.0,
        }],
    ))

    actions.append(Node(
        package='mrs_coordination',
        executable='map_merger_node',
        name='map_merger',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_names': robot_names,
            'initial_poses': init_poses,
            'global_frame': 'map',
            'merge_rate': 1.0,
            'resolution': 0.05,
            'respect_comms': True,
        }],
    ))

    return actions


def generate_launch_description():
    bringup_share = get_package_share_directory('mrs_bringup')
    gazebo_share = get_package_share_directory('gazebo_ros')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(bringup_share, 'worlds', 'warehouse.world'),
        description='Gazebo world file to load.')

    n_robots_arg = DeclareLaunchArgument(
        'n_robots', default_value='2',
        description='Number of UGVs to spawn (bounded by config/fleet.yaml).')

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Start the Gazebo client. Set false for headless runs.')

    rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Start RViz with the fleet configuration.')

    explorer_arg = DeclareLaunchArgument(
        'use_explorer', default_value='true',
        description='Start the reactive explorer so the robots drive themselves.')

    slam_arg = DeclareLaunchArgument(
        'use_slam', default_value='true',
        description='Start a slam_toolbox instance per robot. Disable to run '
                    'the simulation layer on its own.')

    comms_arg = DeclareLaunchArgument(
        'comms_dropout', default_value='false',
        description='Cycle inter-robot connectivity on and off to emulate '
                    'restricted communication.')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'true',
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(bringup_share, 'rviz', 'multi_robot.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        world_arg,
        n_robots_arg,
        gui_arg,
        rviz_arg,
        explorer_arg,
        slam_arg,
        comms_arg,
        gzserver,
        gzclient,
        OpaqueFunction(function=_spawn_fleet),
        rviz,
    ])
