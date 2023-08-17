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
        self.flag = 0
        self.global_marker_id = [0]*10
        self.angle = 0

    def __call__(self, ar_msg):
        '''
        return self.angle and speed from x, y, yaw of a AR tag
        '''
        local_marker_id = [0]*10
        markers_x_point_list = []
        markers_y_point_list = []
        markers_yaw_point_list = []

        for i in ar_msg.markers:
            self.global_marker_id[i.id] = 1
            local_marker_id[i.id] = 1

            pose = i.pose.pose
            markers_x_point_list.append(pose.position.x)
            markers_y_point_list.append(pose.position.z)
            markers_yaw_point_list.append(euler_from_quaternion((pose.orientation.x, pose.orientation.y,
                                              pose.orientation.z, pose.orientation.w))[1])
        markers_x_point_list.sort(reverse=True)
        print('detecting AR tag: ', len(markers_x_point_list))

        # start
        if len(markers_x_point_list) == 0 and self.flag == 0:
            self.angle = 0
        
        # detection
        elif len(markers_x_point_list) > 0 and self.flag == 0:
            for i in range(10):
                if self.global_marker_id[i] == 1 and local_marker_id == 0:
                    print('AR Curve start: ', len(markers_x_point_list))
                    self.flag = 1
                    self.angle = -45

        # curve controll 
        elif len(markers_x_point_list) > 0 and self.flag == 1:
            error = markers_x_point_list[0] - self.target
            self.angle = int(error * 170) 
            self.flag = 2
            print(f"error: {error}, self.angle: {self.angle}")

        # termination
        if len(markers_x_point_list) == 0 and self.flag == 2:
            self.flag = 3
            self.angle = 0

        return self.flag, int(self.angle)