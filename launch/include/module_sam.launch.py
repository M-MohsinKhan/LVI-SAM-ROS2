"""
Modular launch file: SAM nodes (Lidar + Visual odometry stack)
Mirrors: module_sam.launch (ROS 1)

Starts all seven LVI-SAM processing nodes.  Parameter paths are passed
in as launch arguments so that dataset_lvisam / dataset_custom wrapper
files can override them without touching this file.

Accepted launch arguments:
  lidar_params   — path to params_lidar.yaml   (default: config/params_lidar.yaml)
  camera_params  — path to params_camera.yaml  (default: config/params_camera.yaml)
    use_image_republish  — enable image_transport republisher (default: false)
    image_topic          — base image topic used by republisher (default: /camera/image_raw)
    image_transport_in   — input transport plugin name (default: compressed)
    image_transport_out  — output transport plugin name (default: raw)
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
    use_image_republish_arg = DeclareLaunchArgument(
        'use_image_republish',
        default_value='false',
        description='Enable image_transport republisher for compressed camera topics'
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Base camera image topic for image_transport republisher'
    )
    image_transport_in_arg = DeclareLaunchArgument(
        'image_transport_in',
        default_value='compressed',
        description='Input image transport plugin (e.g. compressed, raw, theora)'
    )
    image_transport_out_arg = DeclareLaunchArgument(
        'image_transport_out',
        default_value='raw',
        description='Output image transport plugin (typically raw)'
    )

    # ── Lidar odometry nodes ───────────────────────────────────────────────────
    imu_preintegration = Node(
        package='lvi_sam',
        executable='lvi_sam_imuPreintegration',
        name='lvi_sam_imuPreintegration',
        output='screen',
        respawn=True,
        parameters=[LaunchConfiguration('lidar_params')]
    )

    image_projection = Node(
        package='lvi_sam',
        executable='lvi_sam_imageProjection',
        name='lvi_sam_imageProjection',
        output='screen',
        respawn=True,
        parameters=[LaunchConfiguration('lidar_params')]
    )

    feature_extraction = Node(
        package='lvi_sam',
        executable='lvi_sam_featureExtraction',
        name='lvi_sam_featureExtraction',
        output='screen',
        respawn=True,
        parameters=[LaunchConfiguration('lidar_params')]
    )

    map_optimization = Node(
        package='lvi_sam',
        executable='lvi_sam_mapOptmization',
        name='lvi_sam_mapOptmization',
        output='screen',
        respawn=True,
        parameters=[LaunchConfiguration('lidar_params')]
    )

    # ── Visual odometry nodes ──────────────────────────────────────────────────
    visual_feature = Node(
        package='lvi_sam',
        executable='lvi_sam_visual_feature',
        name='lvi_sam_visual_feature',
        output='screen',
        respawn=True,
        parameters=[{'vins_config_file': LaunchConfiguration('camera_params')}]
    )

    visual_odometry = Node(
        package='lvi_sam',
        executable='lvi_sam_visual_odometry',
        name='lvi_sam_visual_odometry',
        output='screen',
        respawn=True,
        parameters=[{'vins_config_file': LaunchConfiguration('camera_params')}]
    )

    visual_loop = Node(
        package='lvi_sam',
        executable='lvi_sam_visual_loop',
        name='lvi_sam_visual_loop',
        output='screen',
        respawn=True,
        parameters=[{'vins_config_file': LaunchConfiguration('camera_params')}]
    )

    # Optional image conversion for compressed camera streams
    image_republish = Node(
        package='image_transport',
        executable='republish',
        name='lvi_sam_republish',
        output='screen',
        # keep disabled to avoid endless restart loops on missing plugins
        respawn=False,
        condition=IfCondition(LaunchConfiguration('use_image_republish')),
        arguments=[
            LaunchConfiguration('image_transport_in'),
            LaunchConfiguration('image_transport_out'),
        ],
        remappings=[
            ('in', LaunchConfiguration('image_topic')),
            ('out', LaunchConfiguration('image_topic')),
        ]
    )

    return LaunchDescription([
        lidar_params_arg,
        camera_params_arg,
        use_image_republish_arg,
        image_topic_arg,
        image_transport_in_arg,
        image_transport_out_arg,
        imu_preintegration,
        image_projection,
        feature_extraction,
        map_optimization,
        visual_feature,
        visual_odometry,
        visual_loop,
        image_republish,
    ])
