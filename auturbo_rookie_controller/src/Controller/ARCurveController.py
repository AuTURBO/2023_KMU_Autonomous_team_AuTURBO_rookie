#! /usr/bin/env python
# -*- coding:utf-8 -*-

from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

class ARCurveController(object):
    '''
    Speed and Steer controller for precise parking using AR tag
    '''
    def __init__(self):
        self.kp = 10
        self.angle = 0

    def __call__(self, ar_msg):
        '''
        return self.angle and speed from x, y, yaw of a AR tag
        '''
        markers_x_point_list = []
        markers_y_point_list = []
        tuple_markers = []

        for i in ar_msg.markers:
            pose = i.pose.pose
            markers_x_point_list.append(pose.position.x)
            markers_y_point_list.append(pose.position.z)
            tuple_markers.append((pose.position.x, pose.position.z))
        
        flag = 0
        if len(markers_y_point_list) > 0:
            tuple_markers.sort(key=lambda x: x[1], reverse=True)

        if len(markers_x_point_list) > 1:
            while tuple_markers[0][1] < 0.85:
                tuple_markers.pop(0)
            self.angle = -1 * tuple_markers[0][0] *  self.kp
            flag = 1
        elif len(markers_x_point_list) == 1 and tuple_markers[0][1] < 0.85:
            self.angle = -15
            flag = 1
        else:# 0인 경우
            self.angle = 15
            flag = 0     
        


        return int(self.angle), flag