#!/usr/bin/env python
# -*- coding:utf-8 -*-

import re
import sys

import numpy as np
import rclpy
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from pyquaternion import Quaternion
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu


class CalibrateIMU(Node):
    def __init__(self):
        super().__init__('calibrate_imu')
        self.data = []
        self.declare_parameters(namespace='',
                                parameters=[('duration', 60),
                                            ('base_frame', 'base_link'),
                                            ('result_file', get_package_share_directory('imu_calibration')
                                             + '/launch/calibrated_tf.launch.py')])
        self.duration = Duration(seconds=self.get_parameter('duration').value)
        self.base_frame = self.get_parameter('base_frame').value
        self.result_file = self.get_parameter('result_file').value
        self.start_t = None
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.sub = self.create_subscription(Imu, 'data', self.cb, 10)

    def cb(self, msg):
        if self.start_t is None:
            self.start_t = Time.from_msg(msg.header.stamp)
        if Time.from_msg(msg.header.stamp) - self.start_t < self.duration:
            self.data.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            return

        self.sub.destroy()
        mean = np.mean(self.data, axis=0)
        unit_z = np.array([0, 0, 1])
        a = np.cross(mean, unit_z)
        q = np.array([a[0], a[1], a[2], np.linalg.norm(mean) + np.dot(mean, unit_z)])
        q = Quaternion(q / np.linalg.norm(q))
        t = self.buffer.lookup_transform(msg.header.frame_id, self.base_frame, Time())
        q_base = Quaternion([t.transform.rotation.x, t.transform.rotation.y,
                            t.transform.rotation.z, t.transform.rotation.w])
        q_new = q_base * q
        with open(self.result_file, 'r') as f:
            context = f.read()
            m = re.search(r'(arguments=\[)(.*)(\])', context)
            new_tf = m.group(2).split(', ')
            t2 = self.buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time())
            new_tf[:3] = [f"'{t2.transform.translation.x}'",
                          f"'{t2.transform.translation.y}'",
                          f"'{t2.transform.translation.z}'"]
            new_tf[3:7] = [f"'{q}'" for q in q_new.elements]
            new_tf[7] = f"'{self.base_frame}'"
            new_tf[8] = f"'{msg.header.frame_id}'"
            replaced = re.sub(r'arguments=\[.*\]', m.group(1) + ', '.join(new_tf) + m.group(3), context)
            self.get_logger().info(replaced)
        with open(self.result_file, 'w') as f:
            f.write(replaced)

        raise KeyboardInterrupt


def main():
    rclpy.init()
    try:
        node = CalibrateIMU()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
