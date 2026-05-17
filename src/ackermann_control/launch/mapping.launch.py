from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('ackermann_control')
    slam_params = os.path.join(pkg_dir, 'config', 'slam_toolbox.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz_slam.rviz')

    return LaunchDescription([

        # 1. SLAM Toolbox (Builds the map and provides map -> odom TF)
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params],
            remappings=[
                ('/scan', '/ouster/scan')
            ]
        ),

        # 2. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
