
import os
from pathlib import Path

from rospkg import get_ros_package_path, get_ros_root
from ament_index_python import get_package_prefix, get_resource
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic' : '/scan',
                'odom_topic' : '/odom_rf2o',
                'publish_tf' : True,
                'base_frame_id' : 'base_link',
                'odom_frame_id' : 'map',
                'init_pose_from_topic' : '',
                'freq' : 1.0}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'velodyne']
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/velodyne_points'),
                        ('scan', '/scan')],
            parameters=[{
                'target_frame': 'velodyne',
                'transform_tolerance': 0.1,
                'min_height': 0.0,
                'max_height': 100.0,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 100.0,
                'use_inf': False,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        ),
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', str(os.path.join(get_package_share_directory('odom_bringup'), 'config', 'show_all.rviz.yaml'))]
        ),
    ])
