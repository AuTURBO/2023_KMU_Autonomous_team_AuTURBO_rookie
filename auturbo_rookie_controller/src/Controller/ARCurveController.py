#! /usr/bin/env python
# -*- coding:utf-8 -*-

from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

class ARCurveController(object):
    '''
    Speed and Steer controller for precise parking using AR tag
    '''
    def __init__(self):
        self.target = 0.43
        self.markers_x_point_list = []
        self.markers_y_point_list = []
        self.markers_yaw_point_list = []

    def __call__(self, ar_msg):
        '''
        return angle and speed from x, y, yaw of a AR tag
        '''
        for i in ar_msg.markers:     
            pose = i.pose.pose
            self.markers_x_point_list.append(pose.position.x)
            self.markers_y_point_list.append(pose.position.z)
            self.markers_yaw_point_list.append(euler_from_quaternion((pose.orientation.x, pose.orientation.y,
                                              pose.orientation.z, pose.orientation.w))[1])

        # termination
        if len(self.markers_x_point_list) == 0:
            print('there is no detecting AR tag')
            return 0, 0

        # direction
        elif len(self.markers_x_point_list) > 0:
            error = self.markers_x_point_list[0] - self.target
            print(f"error: {error}")
            angle = int(error * 170) 
            print(angle)

        return angle, 1