#!/usr/bin/env python
# -*- coding:utf-8 -*-

import re

import numpy as np
import rospkg
import rospy
import tf
import tf2_ros
from sensor_msgs.msg import Imu


class CalibrateIMU(object):
    def __init__(self):
        self.data = []
        self.duration = rospy.Duration(rospy.get_param('~duration', 60.0))
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.result_file = rospy.get_param('~result_file',
                                           rospkg.RosPack().get_path('imu_calibration')
                                           + '/launch/calibrated_tf.launch')
        self.start_t = None
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.sub = rospy.Subscriber('data', Imu, self.cb)

    def cb(self, msg):
        if self.start_t is None:
            self.start_t = msg.header.stamp
        if msg.header.stamp - self.start_t < self.duration:
            self.data.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            return

        self.sub.unregister()
        mean = np.mean(self.data, axis=0)
        unit_z = np.array([0, 0, 1])
        a = np.cross(mean, unit_z)
        q = np.array([a[0], a[1], a[2], np.linalg.norm(mean) + np.dot(mean, unit_z)])
        q /= np.linalg.norm(q)

        t = self.buffer.lookup_transform(msg.header.frame_id, self.base_frame, rospy.Time())
        q_base = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        q_new = tf.transformations.quaternion_multiply(q_base, q)
        with open(self.result_file, 'r') as f:
            context = f.read()
            m = re.search(r'(name="calibrated_imu_tf".*args=")(.*)(")', context)
            new_tf = m.group(2).split(' ')
            t2 = self.buffer.lookup_transform(self.base_frame, msg.header.frame_id, rospy.Time())
            new_tf[:3] = [str(t2.transform.translation.x),
                          str(t2.transform.translation.y),
                          str(t2.transform.translation.z)]
            new_tf[3:7] = map(str, q_new)
            new_tf[7] = self.base_frame
            new_tf[8] = msg.header.frame_id
            replaced = re.sub(m.group(0), m.group(1) + ' '.join(new_tf) + m.group(3), context)
        with open(self.result_file, 'w') as f:
            f.write(replaced)

        rospy.signal_shutdown('Calibration finished')


if __name__ == '__main__':
    rospy.init_node('calibrator')
    _ = CalibrateIMU()
    rospy.spin()
