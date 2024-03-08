# Copyright (c) 2022，Horizon Robotics.
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

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():

    web_show = os.getenv('WEB_SHOW')
    print("web_show is ", web_show)

    # jpeg图片编码&发布pkg
    jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'shared_mem',
            'codec_out_mode': 'ros',
            'codec_sub_topic': '/hbmem_img',
            'codec_pub_topic': '/image_jpeg',
        }.items()
    )

    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image_jpeg',
            'websocket_smart_topic': '/racing_track_center_detection'
        }.items()
    )

    racing_track_detection_resnet18_node = Node(
        package='racing_track_detection_resnet',
        executable='racing_track_detection_resnet',
        output='screen',
        parameters=[
            {"sub_img_topic": "/hbmem_img"},
            {"model_path": "/opt/nodehub_model/race_detection/race_track_detection.bin"}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    if web_show == "TRUE":
        return LaunchDescription([
            racing_track_detection_resnet18_node,
            jpeg_codec_node,
            web_node
        ])
    else:
        return LaunchDescription([
            racing_track_detection_resnet18_node
        ])