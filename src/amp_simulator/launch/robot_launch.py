#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Webots Tesla driver."""

import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('amp_simulator')
    world = LaunchConfiguration('world')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    robot_description_path = os.path.join(package_dir, 'resource', 'tesla_webots.urdf')
    tesla_driver = WebotsController(
        robot_name='vehicle',
        parameters=[
            {'robot_description': robot_description_path}
        ],
        respawn=False
    )
    # depth_to_pc = Node(
    #     package='depth_image_proc',
    #     executable='point_cloud_xyzrgb',
    #     remappings=[
    #         ('rgb/camera_info', '/vehicle/range_finder/camera_info'),
    #         ('rgb/image_rect_color', '/vehicle/camera/image_color'),
    #         ('depth_registered/image_rect', 'masked_depth/image')
    #     ]
    # )
    depth_to_pc = ComposableNodeContainer(
        name='depth_to_pc',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Driver itself
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='point_cloud_xyzrgb_node',
                remappings=[
                    ('rgb/camera_info', '/vehicle/camera/camera_info'),
                    ('rgb/image_rect_color', '/vehicle/camera/image_color'),
                    ('depth_registered/image_rect', '/vehicle/camera/image_raw'),
                    ('points', 'resultpoints')
                ],
                parameters=[
                    {'queue_size': 25}
                ]
            ),
        ],
        output='screen',
    )

    lane_follower = Node(
        package='amp_lane',
        executable='lane_follower',
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        emulate_tty=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='track.wbt',
            description='Choose one of the world files from `/amp_simulator/worlds` directory'
        ),
        webots,
        webots._supervisor,
        tesla_driver,
        depth_to_pc,
        lane_follower,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    ])
