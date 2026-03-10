import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_plansys')
    domain_file = os.path.join(pkg_dir, 'pddl', 'domain.pddl')

    # ── PlanSys2 core nodes (replaces plansys2_bringup include) ─────────────
    domain_expert_node = Node(
        package='plansys2_domain_expert',
        executable='domain_expert_node',
        name='domain_expert',
        output='screen',
        parameters=[{'model_file': domain_file}],
    )

    problem_expert_node = Node(
        package='plansys2_problem_expert',
        executable='problem_expert_node',
        name='problem_expert',
        output='screen',
        parameters=[{'model_file': domain_file}],
    )

    planner_node = Node(
        package='plansys2_planner',
        executable='planner_node',
        name='planner',
        output='screen',
        parameters=[{'plan_solver_plugins': ['POPF']}],
    )

    executor_node = Node(
        package='plansys2_executor',
        executable='executor_node',
        name='executor',
        output='screen',
        parameters=[{
            'default_action_bt_xml_filename': os.path.join(
                get_package_share_directory('plansys2_executor'),
                'behavior_trees', 'plansys2_action_bt.xml'),
        }],
    )

    # ── Action executor nodes ────────────────────────────────────────────────
    move_action_node = Node(
        package='robot_plansys',
        executable='move_action',
        name='move_action',
        output='screen',
    )

    pick_action_node = Node(
        package='robot_plansys',
        executable='pick_action',
        name='pick_action',
        output='screen',
    )

    place_action_node = Node(
        package='robot_plansys',
        executable='place_action',
        name='place_action',
        output='screen',
    )

    # ── Controller delayed 5 s to let PlanSys2 fully start ──────────────────
    robot_controller_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='robot_plansys',
                executable='robot_controller',
                name='robot_controller',
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        domain_expert_node,
        problem_expert_node,
        planner_node,
        executor_node,
        move_action_node,
        pick_action_node,
        place_action_node,
        robot_controller_node,
    ])
