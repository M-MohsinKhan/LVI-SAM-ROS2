"""
Modular launch file: Robot State Publisher
Mirrors: module_robot_state_publisher.launch (ROS 1)

Publishes the robot URDF/TF tree so that all frames (base_link, lidar,
camera, imu) are broadcast to /tf_static.

Accepted launch arguments (forwarded from the parent launch file):
  None — uses fixed paths relative to the package share directory.
"""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('lvi_sam')

    xacro_file = os.path.join(
        pkg_share, 'launch', 'include', 'config', 'robot.urdf.xacro'
    )

    # Process xacro → URDF string at launch time
    robot_description = subprocess.check_output(
        ['xacro', xacro_file]
    ).decode('utf-8')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='lvi_sam_robot_state_publisher',
        output='screen',
        respawn=True,
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([
        robot_state_publisher,
    ])
