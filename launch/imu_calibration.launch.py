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
    base_frame_arg = DeclareLaunchArgument('base_frame',
                                           default_value='base_link')
    result_file_arg = DeclareLaunchArgument('result_file',
                                            default_value=str(pkg_path / 'launch/calibrated_tf.launch.py'))

    calibrate_imu = Node(package='imu_calibration',
                         executable='calibrate_imu',
                         output='screen',
                         parameters=[{'duration': LaunchConfiguration('duration'),
                                      'base_frame': LaunchConfiguration('base_frame'),
                                      'result_file': LaunchConfiguration('result_file')}])

    return LaunchDescription([
        duration_arg,
        base_frame_arg,
        result_file_arg,
        calibrate_imu
    ])
