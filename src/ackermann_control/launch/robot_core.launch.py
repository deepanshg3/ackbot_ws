from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('ackermann_control')
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf.yaml')

    # Path to the Ouster ROS launch file
    ouster_launch_file = os.path.join(
        get_package_share_directory('ouster_ros'),
        'launch',
        'sensor.launch.xml'
    )

    return LaunchDescription([

        # ==================================================
        # 1. SENSORS (The Eyes)
        # ==================================================
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(ouster_launch_file),
            launch_arguments={
                'sensor_hostname': 'os-122103000257.local',
                'udp_dest': '169.254.102.124',
                'timestamp_mode': 'TIME_FROM_ROS_TIME'
            }.items()
        ),

        # ==================================================
        # 2. HARDWARE CONTROL (The Muscles)
        # ==================================================
        Node(
            package='ackermann_control',
            executable='ackermann_controller',
            name='ackermann_controller',
            output='screen'
        ),

        Node(
            package='ackermann_control',
            executable='odometry_node',
            name='odometry_node',
            output='screen'
        ),

        # ==================================================
        # 3. TF & LOCALIZATION (The Inner Ear)
        # ==================================================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf',
            output='screen',
            # 🔥 FIXED: New syntax, and pointed back to os_sensor!
            arguments=[
                '--x', '0', 
                '--y', '0', 
                '--z', '0.165', 
                '--roll', '0', 
                '--pitch', '0', 
                '--yaw', '0', 
                '--frame-id', 'base_link', 
                '--child-frame-id', 'os_sensor'
            ]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        )
    ])
