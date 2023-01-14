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

import pathlib

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_path = pathlib.Path(FindPackageShare('imu_calibration').find('imu_calibration'))
    duration_arg = DeclareLaunchArgument('duration',
                                         default_value='60')
    velocity_arg = DeclareLaunchArgument('velocity',
                                         default_value='0.2')
    visualize_arg = DeclareLaunchArgument('visualize',
                                          default_value='false')
    output_file_arg = DeclareLaunchArgument('output_file',
                                            default_value=str(pkg_path / 'config/mag.yaml'))

    calibrate_mag = Node(package='imu_calibration',
                         executable='calibrate_mag',
                         output='screen',
                         parameters=[{'duration': LaunchConfiguration('duration'),
                                      'velocity': LaunchConfiguration('velocity'),
                                      'visualize': LaunchConfiguration('visualize'),
                                      'output_file': LaunchConfiguration('output_file')}])

    return LaunchDescription([
        duration_arg,
        velocity_arg,
        visualize_arg,
        output_file_arg,
        calibrate_mag
    ])
