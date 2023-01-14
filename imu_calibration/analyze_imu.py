#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys

import allantools
import numpy as np
import plotly.graph_objects as go
import rclpy
from plotly.subplots import make_subplots
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu


class AnalyzeIMU(Node):
    def __init__(self):
        super().__init__('analyze_imu')
        self.acc = []
        self.vel = []

        self.declare_parameters(namespace='',
                                parameters=[('duration', 60),
                                            ('rate', 100)])
        self.duration = Duration(seconds=self.get_parameter('duration').value)
        self.rate = self.get_parameter('rate').value
        self.start_t = None
        self.sub = self.create_subscription(Imu, 'data', self.cb, 10)

    def cb(self, msg):
        if self.start_t is None:
            self.start_t = msg.header.stamp
        if msg.header.stamp - self.start_t < self.duration:
            self.acc.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            self.vel.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            return

        self.sub.destroy()
        acc = np.array(self.acc)
        vel = np.array(self.vel)
        fig = make_subplots(rows=2,
                            cols=3,
                            subplot_titles=['Acceleration X',
                                            'Acceleration Y',
                                            'Acceleration Z',
                                            'Angular Velocity X',
                                            'Angular Velocity Y',
                                            'Angular Velocity Z'])

        for i in range(3):
            allan = allantools.oadev(acc[:, i],
                                     rate=self.rate,
                                     data_type='freq',
                                     taus=np.logspace(-int(np.log10(self.rate)) - 1,
                                                      int(np.log10(self.duration.to_sec())) + 1,
                                                      100))
            fig.add_trace(go.Scatter(x=allan[0],
                                     y=allan[1],
                                     error_y=dict(type='data',
                                                  array=allan[2],
                                                  visible=True)),
                          row=1,
                          col=i + 1)
            allan = allantools.oadev(vel[:, i],
                                     rate=self.rate,
                                     data_type='freq',
                                     taus=np.logspace(-int(np.log10(self.rate)) - 1,
                                                      int(np.log10(self.duration.to_sec())) + 1,
                                                      100))
            fig.add_trace(go.Scatter(x=allan[0],
                                     y=allan[1],
                                     error_y=dict(type='data',
                                                  array=allan[2],
                                                  visible=True)),
                          row=2,
                          col=i + 1)
        fig.update_xaxes(type="log")
        fig.update_yaxes(type="log")
        fig.update_layout(title_text="Allan Deviation", showlegend=False, template='seaborn')
        fig.show()

        raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)
    try:
        node = AnalyzeIMU()
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
