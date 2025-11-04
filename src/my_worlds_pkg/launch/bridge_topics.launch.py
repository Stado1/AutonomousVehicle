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

    # Bridge velocity
    velocity_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='velocity_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )

    # Bridge left stereo camera
    left_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='left_camera_bridge',
        arguments=['/stereo/left/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen'
    )

    # Bridge right stereo camera
    right_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='right_camera_bridge',
        arguments=['/stereo/right/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen'
    )

    # Bridge left camera info
    left_camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='left_camera_info_bridge',
        arguments=['/stereo/left/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        output='screen'
    )

    # Bridge right camera info
    right_camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='right_camera_info_bridge',
        arguments=['/stereo/right/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        output='screen'
    )

    # Bridge odometry topic
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        output='screen'
    )

    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='tf_bridge',
        output='screen',
        arguments=[
            # Dynamic TF (regular transforms)
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Static TF
            '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ]
    )

    # clock_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='clock_bridge',
    #     output='screen',
    #     arguments=['/world/default/clock@rosgraph_msgs/msg/Clock2[gz.msgs.Clock'],
    # )





    ld = LaunchDescription()

    # ld.add_action(clock_bridge)
    ld.add_action(velocity_bridge)
    ld.add_action(left_camera_bridge)
    ld.add_action(right_camera_bridge)
    ld.add_action(left_camera_info_bridge)
    ld.add_action(right_camera_info_bridge)
    ld.add_action(odom_bridge)
    ld.add_action(tf_bridge)



    return ld
