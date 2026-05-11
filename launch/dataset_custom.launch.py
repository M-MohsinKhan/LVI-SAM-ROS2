"""
Dataset launch file for the custom sensor configuration.

Sensor setup:
  - 32-channel LSLiDAR           (topic: /lslidar_point_cloud)
  - IMU                          (topic: /camera/imu)
  - RealSense infra camera       (topic: /camera/infra1/image_rect_raw, PINHOLE 640x480)

Usage:
  ros2 launch lvi_sam dataset_custom.launch.py
  ros2 launch lvi_sam dataset_custom.launch.py use_rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('lvi_sam')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Whether to launch RViz2'
    )

    return LaunchDescription([
        use_rviz_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'run.launch.py')
            ),
            launch_arguments={
                'lidar_params': os.path.join(
                    pkg_share, 'config', 'custom', 'params_lidar.yaml'
                ),
                'camera_params': os.path.join(
                    pkg_share, 'config', 'custom', 'params_camera.yaml'
                ),
                'use_rviz': LaunchConfiguration('use_rviz'),
            }.items()
        ),
    ])
