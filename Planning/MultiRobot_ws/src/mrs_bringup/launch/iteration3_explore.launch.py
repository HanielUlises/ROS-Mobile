"""Third iteration: coordinated frontier exploration in Freiburg building 079.

A thin wrapper over `multi_robot_slam.launch.py` that pins the scenario the
third iteration is defined by — the fr079 world, its three-agent fleet file and
the deliberative planner — and leaves every earlier command line alone. The
first two iterations keep the base file's defaults (the warehouse, two agents,
the reactive policy), so nothing in this iteration can change what they do.

The one argument worth touching is `explorer`. Both policies are run on this
scenario and compared, and the only difference between the two runs is that
word:

    # deliberative, coordinated frontier exploration
    ros2 launch mrs_bringup iteration3_explore.launch.py \\
        record:=true record_dir:=/tmp/mrs_iter3_frontier

    # the reactive baseline, same world, same poses, same link process
    ros2 launch mrs_bringup iteration3_explore.launch.py explorer:=reactive \\
        record:=true record_dir:=/tmp/mrs_iter3_reactive
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('mrs_bringup')
    world = os.path.join(bringup_share, 'worlds', 'fr079_office.world')

    forwarded = ['gui', 'use_rviz', 'record', 'record_dir', 'comms_dropout',
                 'explorer', 'n_robots']

    arguments = [
        DeclareLaunchArgument('explorer', default_value='frontier',
                              description="'frontier' for the deliberative "
                                          "planner, 'reactive' for the "
                                          'baseline of iterations 1 and 2.'),
        DeclareLaunchArgument('n_robots', default_value='3'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('record_dir', default_value='/tmp/mrs_iter3'),
        DeclareLaunchArgument('comms_dropout', default_value='true'),
    ]

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'multi_robot_slam.launch.py')),
        launch_arguments=dict(
            [('world', world), ('fleet', 'fleet_fr079.yaml')]
            + [(name, LaunchConfiguration(name)) for name in forwarded]
        ).items(),
    )

    return LaunchDescription(arguments + [base])
