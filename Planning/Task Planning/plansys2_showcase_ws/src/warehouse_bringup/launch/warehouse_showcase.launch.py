"""Bring up the whole showcase for a fleet of `n_robots`.

One launch file covers both halves of the study, because the point of the
multi-agent report is that *nothing* changes between them except a number:

    ros2 launch warehouse_bringup warehouse_showcase.launch.py n_robots:=1
    ros2 launch warehouse_bringup warehouse_showcase.launch.py n_robots:=3

What it starts, in order: Gazebo with the AWS RoboMaker small warehouse; one
TurtleBot3 Waffle per robot, each in its own ROS namespace; the PlanSys2 stack
with this workspace's PDDL domain; one `move` performer and one `pick`/`drop`
performer per robot, each specialised on its own robot so no performer bids for
another's work; the fleet monitor; and the mission controller, which is what
actually poses the problem and asks for a plan.

The per-robot SDF is generated here rather than shipped, because the stock
TurtleBot3 model publishes on global topic names and three copies of it would
fight over `/cmd_vel`, `/odom` and `/scan`. The generated copy carries a ROS
namespace and prefixed frames, and nothing else about the model is touched.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml

# Stations the robots start from, in the order the problem generator hands them
# out. Kept here as a fallback only: the mission controller is told the real
# assignment through parameters below, generated from the same roadmap.
TURTLEBOT3_MODEL = 'turtlebot3_waffle'


def _namespaced_model(source_sdf, namespace):
    """Return an SDF for one robot, on its own topics and TF frames."""
    with open(source_sdf) as handle:
        sdf = handle.read()

    # The stock model carries the namespace as a comment in every plugin — the
    # upstream way of saying "put yours here". Replacing the comment rather than
    # inserting a tag keeps the edit to one line per plugin and leaves the
    # geometry, the inertias and the sensor parameters exactly as shipped.
    sdf = sdf.replace('<!-- <namespace>/tb3</namespace> -->',
                      f'<namespace>/{namespace}</namespace>')
    sdf = sdf.replace('<odometry_frame>odom</odometry_frame>',
                      f'<odometry_frame>{namespace}/odom</odometry_frame>')
    sdf = sdf.replace('<robot_base_frame>base_footprint</robot_base_frame>',
                      f'<robot_base_frame>{namespace}/base_footprint</robot_base_frame>')
    sdf = sdf.replace('<frame_name>base_scan</frame_name>',
                      f'<frame_name>{namespace}/base_scan</frame_name>')
    sdf = sdf.replace('<frame_name>imu_link</frame_name>',
                      f'<frame_name>{namespace}/imu_link</frame_name>')

    out_dir = os.path.join(tempfile.gettempdir(), 'warehouse_showcase_models', namespace)
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, 'model.sdf')
    with open(out_path, 'w') as handle:
        handle.write(sdf)
    return out_path


def _scenario(roadmap_path, n_robots, n_crates):
    """Reproduce `warehouse_tools::buildScenario` for the launch parameters.

    The C++ generator is the definition; this is the launch file reading the
    same roadmap to reach the same answer, so that the mission controller and
    the benchmark pose the same problem. The two agree because they follow the
    same two rules: robots spread over corridor nodes by graph travel time,
    crates dealt round-robin over the storage bays.
    """
    with open(roadmap_path) as handle:
        roadmap = yaml.safe_load(handle)

    waypoints = roadmap['waypoints']
    edges = roadmap['edges']
    corridors = sorted(n for n, v in waypoints.items() if v['kind'] == 'corridor')
    bays = sorted(n for n, v in waypoints.items() if v['kind'] == 'storage')
    docks = sorted(n for n, v in waypoints.items() if v['kind'] == 'dock')

    adjacency = {}
    for edge in edges:
        adjacency.setdefault(edge['from'], []).append((edge['to'], edge['travel_time_s']))
        adjacency.setdefault(edge['to'], []).append((edge['from'], edge['travel_time_s']))

    def shortest(a, b):
        import heapq
        best = {a: 0.0}
        queue = [(0.0, a)]
        while queue:
            cost, node = heapq.heappop(queue)
            if node == b:
                return cost
            if cost > best.get(node, float('inf')):
                continue
            for nxt, weight in adjacency.get(node, []):
                if cost + weight < best.get(nxt, float('inf')):
                    best[nxt] = cost + weight
                    heapq.heappush(queue, (best[nxt], nxt))
        return best.get(b, float('inf'))

    starts = [corridors[0]]
    while len(starts) < n_robots:
        best, best_distance = None, -1.0
        for candidate in corridors:
            if candidate in starts:
                continue
            nearest = min(shortest(candidate, chosen) for chosen in starts)
            if nearest > best_distance:
                best, best_distance = candidate, nearest
        if best is None:
            break
        starts.append(best)
    while len(starts) < n_robots:
        starts.append(corridors[len(starts) % len(corridors)])

    robots = [f'r{i + 1}' for i in range(n_robots)]
    crates = [f'crate{i + 1}' for i in range(n_crates)]
    crate_bay = [bays[i % len(bays)] for i in range(n_crates)]
    return {
        'robots': robots,
        'robot_start': starts[:n_robots],
        'crates': crates,
        'crate_bay': crate_bay,
        'dock': docks[0],
        'positions': {name: (v['x'], v['y']) for name, v in waypoints.items()},
    }


def _setup(context, *args, **kwargs):
    n_robots = int(LaunchConfiguration('n_robots').perform(context))
    n_crates = int(LaunchConfiguration('n_crates').perform(context))
    log_dir = LaunchConfiguration('log_dir').perform(context)
    os.makedirs(log_dir, exist_ok=True)

    planning_share = get_package_share_directory('warehouse_planning')
    roadmap = os.path.join(planning_share, 'config', 'roadmap.yaml')
    domain = os.path.join(planning_share, 'pddl', 'warehouse_domain.pddl')
    scenario = _scenario(roadmap, n_robots, n_crates)

    tb3_sdf = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models', TURTLEBOT3_MODEL, 'model.sdf')

    actions = []

    # PlanSys2: domain expert, problem expert, planner and executor, all on the
    # workspace's own domain file.
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch', 'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={'model_file': domain}.items()))

    for index, (robot, start) in enumerate(zip(scenario['robots'], scenario['robot_start'])):
        x, y = scenario['positions'][start]
        # Robots that share a start node are offset along x so that Gazebo
        # does not spawn one inside another; the symbolic start is still the
        # same waypoint, which is the configuration the planner was given.
        offset = 0.0
        for earlier in scenario['robot_start'][:index]:
            if earlier == start:
                offset += 0.6
        actions.append(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'spawn_{robot}',
            output='screen',
            arguments=[
                '-entity', robot,
                '-file', _namespaced_model(tb3_sdf, robot),
                '-robot_namespace', robot,
                '-x', str(x + offset), '-y', str(y), '-z', '0.01',
                '-Y', '0.0',
            ]))

        actions.append(Node(
            package='warehouse_actions',
            executable='move_action',
            name=f'move_{robot}',
            output='screen',
            parameters=[{
                'action_name': 'move',
                'specialized_arguments': [robot],
                'robot_name': robot,
                'roadmap': roadmap,
            }]))

        for action_name, dwell, carrying in (('pick', 8.0, True), ('drop', 8.0, False)):
            actions.append(Node(
                package='warehouse_actions',
                executable='manipulate_action',
                name=f'{action_name}_{robot}',
                output='screen',
                parameters=[{
                    'action_name': action_name,
                    'specialized_arguments': [robot],
                    'robot_name': robot,
                    'roadmap': roadmap,
                    'dwell_seconds': dwell,
                    'carrying_after': carrying,
                }]))

    actions.append(Node(
        package='warehouse_bringup',
        executable='fleet_monitor',
        name='fleet_monitor',
        output='screen',
        parameters=[{
            'robots': scenario['robots'],
            'roadmap': roadmap,
            'log_dir': log_dir,
        }]))

    actions.append(Node(
        package='warehouse_bringup',
        executable='mission_controller',
        name='mission_controller',
        output='screen',
        parameters=[{
            'roadmap': roadmap,
            'robots': scenario['robots'],
            'robot_start': scenario['robot_start'],
            'crates': scenario['crates'],
            'crate_bay': scenario['crate_bay'],
            'dock': scenario['dock'],
            'log_dir': log_dir,
            # Gazebo has to have spawned every robot and every performer has to
            # have reached Inactive before the first action is dispatched.
            'startup_delay': 12.0 + 2.0 * n_robots,
        }]))

    return actions


def generate_launch_description():
    world = os.path.join(
        get_package_share_directory('aws_robomaker_small_warehouse_world'),
        'worlds', 'no_roof_small_warehouse', 'no_roof_small_warehouse.world')

    return LaunchDescription([
        DeclareLaunchArgument('n_robots', default_value='1',
                              description='Size of the fleet.'),
        DeclareLaunchArgument('n_crates', default_value='4',
                              description='Number of crates to deliver.'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start the Gazebo client.'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('log_dir', default_value='/tmp/warehouse_showcase'),
        DeclareLaunchArgument('world', default_value=world),

        ExecuteProcess(
            cmd=['gzserver', '--verbose', LaunchConfiguration('world'),
                 '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('gui'))),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('warehouse_bringup'), 'rviz', 'showcase.rviz')],
            condition=IfCondition(LaunchConfiguration('use_rviz'))),

        OpaqueFunction(function=_setup),
    ])
