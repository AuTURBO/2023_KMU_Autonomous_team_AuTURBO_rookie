#! /usr/bin/env python
# -*- coding:utf-8 -*-

import numpy as np
import rospy

class ObjectDetector(object):
    '''
    Detects obstacle using lidar data
    '''
    def __init__(self, timer, car_target, car_rest):
        self.direction = 'none'
        self.timer = timer
        self.car_target = car_target
        self.car_rest = car_rest
        self.threshold = 10

    def __call__(self, detect, x_mid, y):
        '''
        returns direction to avoid obstacles from lidar inputs
        '''
        if (detect[self.car_target] and detect[self.car_rest] and y[self.car_target] > 30):
            if (x_mid[self.car_target] < x_mid[self.car_rest]):
                self.direction = "left"
            elif (x_mid[self.car_target] > x_mid[self.car_rest]):
                self.direction = "right"
        rospy.loginfo("self.direction: %s, y[self.car_target]: %f", self.direction, y[self.car_target])
        return self.direction
