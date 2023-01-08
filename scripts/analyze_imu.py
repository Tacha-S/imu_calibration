#!/usr/bin/env python
# -*- coding:utf-8 -*-

import allantools
import numpy as np
import plotly.graph_objects as go
import rospy
from plotly.subplots import make_subplots
from sensor_msgs.msg import Imu


class AnalyzeIMU(object):
    def __init__(self):
        self.acc = []
        self.vel = []
        self.duration = rospy.Duration(rospy.get_param('~duration', 60.0))
        self.rate = rospy.get_param('~rate', 100)
        self.start_t = None
        self.sub = rospy.Subscriber('data', Imu, self.cb)

    def cb(self, msg):
        if self.start_t is None:
            self.start_t = msg.header.stamp
        if msg.header.stamp - self.start_t < self.duration:
            self.acc.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            self.vel.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            return

        self.sub.unregister()
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

        rospy.signal_shutdown('Done')


if __name__ == '__main__':
    rospy.init_node('calibrator')
    _ = AnalyzeIMU()
    rospy.spin()
