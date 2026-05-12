from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    ExecuteProcess,
    TimerAction,
)
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
import os


def generate_launch_description():

    bumperbot_description_dir = get_package_share_directory("bumperbot_description")

    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    is_ignition = "True" if ros_distro == "humble" else "False"

    model_argument = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            bumperbot_description_dir, "urdf", "bumperbot.urdf.xacro"
        ),
        description="Absolute path to robot URDF/xacro",
    )

    world_name_argument = DeclareLaunchArgument(
        name="world_name",
        default_value="empty",
        description="World file name without extension",
    )

    share_dir = os.path.dirname(bumperbot_description_dir)

    models_dir = os.path.join(bumperbot_description_dir, "models")

    resource_paths = share_dir + ":" + models_dir

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=resource_paths,
    )

    ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=resource_paths,
    )

    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    world_path = PathJoinSubstitution([
        bumperbot_description_dir,
        "worlds",
        PythonExpression([
            "'", LaunchConfiguration("world_name"), "' + '.world'"
        ]),
    ])

    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", world_path],
        output="screen",
    )

    gz_spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-topic", "robot_description",
                    "-name", "bumperbot",
                    "-z", "0.1",
                ],
            )
        ],
    )

    ign_plugin_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
        value="/opt/ros/humble/lib",
    )

    return LaunchDescription([
        model_argument,
        world_name_argument,
        gz_resource_path,
        ign_resource_path,
        ign_plugin_path, 
        robot_state_publisher,
        gazebo,
        gz_spawn_entity,
    ])