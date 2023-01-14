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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    duration_arg = DeclareLaunchArgument('duration',
                                         default_value='60')
    rate_arg = DeclareLaunchArgument('rate',
                                     default_value='100')

    analyzer = Node(package='imu_calibration',
                    executable='analyze_imu',
                    output='screen',
                    parameters=[{'duration': LaunchConfiguration('duration'),
                                 'rate': LaunchConfiguration('rate')}])

    return LaunchDescription([
        duration_arg,
        rate_arg,
        analyzer
    ])
