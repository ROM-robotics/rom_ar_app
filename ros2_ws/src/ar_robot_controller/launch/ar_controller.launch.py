"""
Main launch file for AR Robot Controller

Launches:
  1. rosbridge_websocket_launch (WebSocket bridge)
  2. gesture_controller_node (gesture → cmd_vel)
  3. lidar_relay_node (LiDAR → web app friendly format)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    pkg_dir = get_package_share_directory('ar_robot_controller')
    rosbridge_dir = get_package_share_directory('rosbridge_server')

    # Launch arguments
    port_arg = DeclareLaunchArgument(
        'port', default_value='9090',
        description='WebSocket port for rosbridge'
    )

    max_linear_vel_arg = DeclareLaunchArgument(
        'max_linear_vel', default_value='0.5',
        description='Maximum linear velocity (m/s)'
    )

    max_angular_vel_arg = DeclareLaunchArgument(
        'max_angular_vel', default_value='1.0',
        description='Maximum angular velocity (rad/s)'
    )

    # rosbridge WebSocket server
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(rosbridge_dir, 'launch', 'rosbridge_websocket_launch.xml')
        ),
        launch_arguments={'port': LaunchConfiguration('port')}.items()
    )

    # Gesture Controller Node
    gesture_controller = Node(
        package='ar_robot_controller',
        executable='gesture_controller_node.py',
        name='gesture_controller_node',
        output='screen',
        parameters=[{
            'max_linear_vel': LaunchConfiguration('max_linear_vel'),
            'max_angular_vel': LaunchConfiguration('max_angular_vel'),
            'dead_zone': 0.15,
            'smoothing': 0.3,
            'publish_rate': 20.0,
            'emergency_stop_enabled': True,
        }]
    )

    # LiDAR Relay Node
    lidar_relay = Node(
        package='ar_robot_controller',
        executable='lidar_relay_node.py',
        name='lidar_relay_node',
        output='screen',
        parameters=[{
            'downsample_factor': 4,
            'max_range_viz': 5.0,
            'relay_rate': 10.0,
        }]
    )

    return LaunchDescription([
        port_arg,
        max_linear_vel_arg,
        max_angular_vel_arg,
        rosbridge_launch,
        gesture_controller,
        lidar_relay,
    ])
