#!/usr/bin/env python
# -*- coding:utf-8 -*-

import pathlib

import numpy as np
import plotly.graph_objects as go
import rospkg
import rospy
import yaml
from geometry_msgs.msg import Twist
from sensor_msgs.msg import MagneticField


class CalibrateMag(object):
    def __init__(self):
        self.data = []
        self.duration = rospy.Duration(rospy.get_param('~duration', 60.0))
        self.velocity = rospy.get_param('~velocity', 0.2)
        self.output = rospy.get_param('~output_file',
                                      rospkg.RosPack().get_path('imu_calibration') + '/config/mag.yaml')
        self.visualize = rospy.get_param('~visualize', False)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(20)
        self.sub = rospy.Subscriber('mag', MagneticField, self.cb)

    def cb(self, msg):
        self.data.append([msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])

    def run(self):
        start = rospy.Time.now()
        twist = Twist()
        twist.angular.z = self.velocity
        while not rospy.is_shutdown() and rospy.Time.now() - start < self.duration:
            self.pub.publish(twist)
            self.rate.sleep()

        self.sub.unregister()
        data = np.array(self.data)

        a = np.vstack([data[:, 0], data[:, 1], np.ones(len(data))]).T
        b = data[:, 2]
        result = np.linalg.lstsq(a, b, rcond=None)

        a1 = np.vstack([-2 * data[:, 0], -2 * data[:, 1], -2 * data[:, 2], np.ones(len(data))])
        a2 = np.vstack([result[0][0] * np.ones(len(data)), result[0][1]
                        * np.ones(len(data)), -np.ones(len(data)), np.zeros(len(data))])
        a = np.hstack([a1, a2]).T
        b1 = -(np.power(data[:, 0], 2) + np.power(data[:, 1], 2) + np.power(data[:, 2], 2))
        b2 = result[0][0] * data[:, 0] + result[0][1] * data[:, 1] - data[:, 2]
        b = np.concatenate([b1, b2], 0)
        result2 = np.linalg.lstsq(a, b, rcond=None)
        bias = result2[0]
        rospy.loginfo(f'magnetic bias: {bias[:3]}')

        if not pathlib.Path(self.output).exists():
            with open(self.output, 'w') as f:
                f.write(f'mag_bias_x: {bias[0]}\n')
                f.write(f'mag_bias_y: {bias[1]}\n')
                f.write(f'mag_bias_z: {bias[2]}\n')
        else:
            params = yaml.safe_load(open(self.output, 'r'))
            params['mag_bias_x'] = float(bias[0])
            params['mag_bias_y'] = float(bias[1])
            params['mag_bias_z'] = float(bias[2])
            yaml.safe_dump(params, open(self.output, 'w'))

        if self.visualize:
            r = np.sqrt(np.power(bias[0], 2) + np.power(bias[1], 2) + np.power(bias[2], 2) - bias[3])

            xmesh, ymesh = np.meshgrid(np.linspace(np.min(data[:, 0]), np.max(data[:, 0]), 20),
                                       np.linspace(np.min(data[:, 1]), np.max(data[:, 1]), 20))
            zmesh = (result[0][0] * xmesh.ravel() + result[0][1] * ymesh.ravel() + result[0][2]).reshape(xmesh.shape)
            theta = np.linspace(-np.pi, np.pi, 50)
            x = r * np.cos(theta) + bias[0]
            y = r * np.sin(theta) + bias[1]
            fig = go.Figure()
            fig.add_trace(go.Scatter3d(name='Data', x=data[:, 0], y=data[:, 1], z=data[:, 2], marker=dict(size=2)))
            fig.add_trace(go.Scatter3d(name='Bias', x=[bias[0]], y=[bias[1]], z=[bias[2]], marker=dict(size=5)))
            fig.add_trace(go.Scatter3d(name='Approximate circle',
                                       x=x,
                                       y=y,
                                       z=result[0][0] * x + result[0][1] * y + result[0][2],
                                       marker=dict(size=3)))
            fig.add_trace(go.Surface(x=xmesh, y=ymesh, z=zmesh, opacity=0.7, showscale=False))
            max_range = np.max(np.max(data, axis=0) - np.min(data, axis=0))
            mean = np.mean(data, axis=0)
            fig.update_layout(scene=dict(xaxis=dict(nticks=5, range=[mean[0] - max_range / 2 * 1.2,
                                                                     mean[0] + max_range / 2 * 1.2]),
                                         yaxis=dict(nticks=5, range=[mean[1] - max_range / 2 * 1.2,
                                                                     mean[1] + max_range / 2 * 1.2]),
                                         zaxis=dict(nticks=5, range=[mean[2] - max_range / 2 * 1.2,
                                                                     mean[2] + max_range / 2 * 1.2])))
            fig.show()


if __name__ == '__main__':
    rospy.init_node('calibrator')
    node = CalibrateMag()
    node.run()
