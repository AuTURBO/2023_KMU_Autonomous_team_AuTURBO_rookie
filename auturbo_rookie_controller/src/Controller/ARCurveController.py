#! /usr/bin/env python
# -*- coding:utf-8 -*-

from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

class ARCurveController(object):
    '''
    Speed and Steer controller for precise parking using AR tag
    '''
    def __init__(self):
        self.flag = 0
        # self.global_marker_id = [0]*10
        self.angle = 10

    def __call__(self, ar_msg):
        '''
        return self.angle and speed from x, y, yaw of a AR tag
        '''
        markers_point_list = []

        for i in ar_msg.markers:
            pose = i.pose.pose
            markers_point_list.append(pose.position.y)
        markers_point_list = sorted(markers_point_list, reverse=True)

        if len(markers_point_list) == 1 and markers_point_list[0] < 0.2:
            # self.angle = 10
            self.flag = 1
            print("ar marker count : ", len(markers_point_list))
            return self.flag, 25 
        elif len(markers_point_list) != 0 and  markers_point_list[0] > 0.3:
            return self.flag, 0
        else:
            # self.flag = 0
            return self.flag, -25

