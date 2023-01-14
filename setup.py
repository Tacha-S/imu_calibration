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

import glob

from setuptools import find_packages
from setuptools import setup

package_name = 'imu_calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', glob.glob('./launch/*.launch.py')),
        (f'share/{package_name}/config', glob.glob('./config/*.yaml')),
    ],
    maintainer='Tatsuro Sakaguchi',
    maintainer_email='tacchan.mello.ioiq@gmail.com',
    description='The imu_calibration package',
    license='Apache License, Version2.0',
    entry_points={
        'console_scripts': [
            f'analyze_imu = {package_name}.analyze_imu:main',
            f'calibrate_imu = {package_name}.calibrate_imu:main',
            f'calibrate_mag = {package_name}.calibrate_mag:main'
        ],
    }
)
