from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    map_yaml_file = '/home/haniel/Projects/ROS-Mobile/Planning/Workspaces/bumperbot_ws/src/maps/map.yaml'

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=map_yaml_file,
        description='Full path to map YAML file'
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'topic_name': 'map',
            'frame_id': 'map',
            'use_sim_time': False 
        }],
        arguments=['--ros-args', '--log-level', 'debug']
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
            'bond_timeout': 4.0
        }]
    )

    return LaunchDescription([
        map_yaml_arg,
        map_server_node,
        lifecycle_manager_node,
        rviz_node
    ])