#! /usr/bin/env python
# -*- coding:utf-8 -*-

import numpy as np
import rospy

class ObjectDetector(object):
    '''
    Detects obstacle using lidar data
    '''
    def __init__(self, timer, specfic_car):
        self.direction = 'none'
        self.timer = timer
        self.specfic_car = specfic_car
        self.threshold = 10

    def __call__(self, count, sum_x):
        '''
        returns direction to avoid obstacles from lidar inputs
        '''
        if (count[self.specfic_car] == 0):
            direction = "none"
            return direction
        elif (count[self.specfic_car] <= self.threshold):
            xavg = sum_x[self.specfic_car] / count[self.specfic_car]
            direction = "none"
        elif (count[self.specfic_car] > self.threshold):
            xavg = sum_x[self.specfic_car] / count[self.specfic_car]
            if (xavg >= 320):
                direction = "right"
            else:
                direction = "left"
        rospy.loginfo("self.specfic_car: %s, direction: %s, xavg: %f, count: %d", self.specfic_car, direction, xavg, count[self.specfic_car])
        return direction
