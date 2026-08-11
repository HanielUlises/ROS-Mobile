import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map_yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true"
    )

    map_yaml_arg = DeclareLaunchArgument(
        "map_yaml",
        default_value="/home/haniel/Projects/ROS-Mobile/Planning/Workspaces/bumperbot_ws/src/maps/map.yaml",
        description="Full path to the map yaml file"
    )

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"yaml_filename": map_yaml},
            {"topic_name": "map"},
            {"frame_id": "map"}
        ],
        arguments=["--ros-args", "--log-level", "info"]
    )

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["map_server"]},
            {"bond_timeout": 4.0}
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_yaml_arg,
        map_server_node,
        lifecycle_manager_node,
        rviz_node
    ])
