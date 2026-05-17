from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('ackermann_control')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_dir, 'maps', 'map.yaml')
    bt_xml = os.path.join(pkg_dir, 'config', 'minimal_bt.xml')

    return LaunchDescription([

        # 1. Map Server (Loads your saved .yaml map)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file}]
        ),

        # 2. AMCL (Localizes the robot on the saved map)
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params]
        ),

        # 3. Lifecycle Manager (Wakes up Nav2 nodes in the correct order)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator'
                ]
            }]
        ),

        # 4. Planner (Calculates the path avoiding obstacles)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params]
        ),

        # 5. Controller (Regulated Pure Pursuit - sends speed commands)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params]
        ),

        # 6. Behavior Server (Recovery actions - backup, wait)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params]
        ),

        # 7. BT Navigator (Executes the minimal XML logic)
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                nav2_params,
                {
                    'default_nav_to_pose_bt_xml': bt_xml,
                    'default_nav_through_poses_bt_xml': bt_xml,
                    'plugin_lib_names': [
                        'nav2_compute_path_to_pose_action_bt_node',
                        'nav2_follow_path_action_bt_node',
                        'nav2_pipeline_sequence_bt_node',
                        'nav2_rate_controller_bt_node',
                        'nav2_recovery_node_bt_node',     # Added Recovery Plugin
                        'nav2_back_up_action_bt_node',    # Added Reversing Plugin
                        'nav2_wait_action_bt_node'        # Added Wait Plugin
                    ]
                }
            ]
        )
    ])
