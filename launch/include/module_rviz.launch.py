"""
Modular launch file: RViz2
Mirrors: module_rviz.launch (ROS 1)

Launches RViz2 with the package's default rviz config file.

Accepted launch arguments:
  rviz_config  — path to a .rviz config file
                 (default: launch/include/config/rviz.rviz)
  use_rviz     — 'true'/'false' to conditionally enable RViz2
                 (default: 'true')
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('lvi_sam')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            pkg_share, 'launch', 'include', 'config', 'rviz.rviz'
        ),
        description='Path to the RViz2 config file'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Set to false to skip launching RViz2'
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='lvi_sam_rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        rviz_config_arg,
        use_rviz_arg,
        rviz2_node,
    ])
