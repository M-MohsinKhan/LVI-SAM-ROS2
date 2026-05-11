"""
Top-level launch file for LVI-SAM (ROS 2 / Humble).
Mirrors: run.launch (ROS 1)

Composes the three modular sub-launches:
  include/module_robot_state_publisher.launch.py
  include/module_rviz.launch.py
  include/module_sam.launch.py

Accepted launch arguments:
  lidar_params   — path to params_lidar.yaml
  camera_params  — path to params_camera.yaml
  rviz_config    — path to rviz.rviz config
  use_rviz       — 'true'/'false'  (default: 'true')

Typical usage (via a dataset wrapper):
  ros2 launch lvi_sam dataset_lvisam.launch.py
  ros2 launch lvi_sam dataset_custom.launch.py use_rviz:=false
  ros2 launch lvi_sam run.launch.py             # uses default config paths
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_share = get_package_share_directory('lvi_sam')
    include_dir = os.path.join(pkg_share, 'launch', 'include')

    # ── Arguments ─────────────────────────────────────────────────────────────
    lidar_params_arg = DeclareLaunchArgument(
        'lidar_params',
        default_value=os.path.join(pkg_share, 'config', 'params_lidar.yaml'),
        description='Path to lidar/IMU parameter YAML file'
    )
    camera_params_arg = DeclareLaunchArgument(
        'camera_params',
        default_value=os.path.join(pkg_share, 'config', 'params_camera.yaml'),
        description='Path to camera/VINS parameter YAML file'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(include_dir, 'config', 'rviz.rviz'),
        description='Path to RViz2 config file'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description="Set to 'false' to skip launching RViz2"
    )

    # ── Robot State Publisher ─────────────────────────────────────────────────
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(include_dir, 'module_robot_state_publisher.launch.py')
        )
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(include_dir, 'module_rviz.launch.py')
        ),
        launch_arguments={
            'rviz_config': LaunchConfiguration('rviz_config'),
            'use_rviz':    LaunchConfiguration('use_rviz'),
        }.items()
    )

    # ── SAM nodes (lidar + visual odometry stack) ─────────────────────────────
    sam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(include_dir, 'module_sam.launch.py')
        ),
        launch_arguments={
            'lidar_params':  LaunchConfiguration('lidar_params'),
            'camera_params': LaunchConfiguration('camera_params'),
        }.items()
    )

    return LaunchDescription([
        lidar_params_arg,
        camera_params_arg,
        rviz_config_arg,
        use_rviz_arg,
        robot_state_publisher_launch,
        rviz_launch,
        sam_launch,
    ])
