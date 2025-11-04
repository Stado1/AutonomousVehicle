#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Joep Tool, Hyungyu Kim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node




def generate_launch_description():

    # --- RTAB-Map stereo node ---
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            {'frame_id': 'base_link'},
            {'subscribe_stereo': True},
            {'visual_odometry': True},
            {'use_sim_time': True},
            {'subscribe_depth': False},
            {'subscribe_rgbd': False},
            {'subscribe_rgb': False},
        ],
        remappings=[
            # Stereo image topics
            ('left/image_rect', '/stereo/left/image_raw'),
            ('right/image_rect', '/stereo/right/image_raw'),
            ('left/camera_info', '/stereo/left/camera_info'),
            ('right/camera_info', '/stereo/right/camera_info'),
        ]
    )
    # Static TF: base_link → camera_left_frame
    static_tf_left = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_left',
        arguments=['0.064', '-0.065', '0.094', '0', '0', '0', 'base_link', 'camera_left_frame'],
        output='screen'
    )

    # Static TF: base_link → camera_right_frame
    static_tf_right = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_right',
        arguments=['0.124', '-0.065', '0.094', '0', '0', '0', 'base_link', 'camera_right_frame'],
        output='screen'
    )



    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(static_tf_left)
    ld.add_action(static_tf_right)
    ld.add_action(rtabmap_node)



    return ld
