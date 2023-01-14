#!/usr/bin/env python
# -*- coding:utf-8 -*-

# Copyright (c) 2022 SoftBank Corp.
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

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    calibrated_tf = Node(package='tf2_ros',
                         executable='static_transform_publisher',
                         name='calibrated_imu_tf',
                         output='screen',
                         arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'imu_link'])

    return LaunchDescription([
        calibrated_tf
    ])
