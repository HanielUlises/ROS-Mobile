from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = "test_package",
            executable = "rpm_pub",
            name = "rpm_pub_node",
            parameters = [
                {"rpm_val": 5.0},
                {"param_2": "test"}
            ]
        ),
        ExecuteProcess(
            cmd = ['ros2', 'topic', 'list'],
            output = 'screen'
        )
    ])
        
 
