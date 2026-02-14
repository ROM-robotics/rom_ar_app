"""
Web Server launch file

Quick HTTP server to serve the web AR app.
In production, use nginx or similar.
"""

import os
import subprocess
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('ar_robot_controller')
    web_dir = os.path.join(pkg_dir, 'web_app')

    # Python HTTP server to serve the web app
    web_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8080', '--directory', web_dir],
        output='screen',
        name='web_server'
    )

    return LaunchDescription([
        web_server,
    ])
