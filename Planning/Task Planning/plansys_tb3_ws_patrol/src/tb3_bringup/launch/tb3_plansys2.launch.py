import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tb3_gazebo_dir   = get_package_share_directory('turtlebot3_gazebo')
    tb3_planning_dir = get_package_share_directory('tb3_planning')
    plansys2_dir     = get_package_share_directory('plansys2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation clock')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')))

    plansys2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(plansys2_dir, 'launch', 'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
            'model_file':  os.path.join(tb3_planning_dir, 'pddl', 'tb3_domain.pddl'),
            'params_file': os.path.join(tb3_planning_dir, 'config', 'planner_config.yaml'),
        }.items())

    def action_node(executable):
        return Node(
            package='tb3_actions',
            executable=executable,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}])

    planner_client_node = Node(
        package='tb3_bringup',
        executable='planner_client',
        name='planner_client',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    return LaunchDescription([
        declare_use_sim_time,
        gazebo_launch,
        TimerAction(period=5.0,  actions=[plansys2_launch]),
        TimerAction(period=8.0,  actions=[action_node('move_action')]),
        TimerAction(period=8.0,  actions=[action_node('inspect_action')]),
        TimerAction(period=8.0,  actions=[action_node('report_action')]),
        TimerAction(period=15.0, actions=[planner_client_node]),
    ])
