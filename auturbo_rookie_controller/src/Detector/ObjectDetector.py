#! /usr/bin/env python
# -*- coding:utf-8 -*-

import numpy as np
import rospy

class ObjectDetector(object):
    '''
    Detects obstacle using lidar data
    '''
    def __init__(self, timer, car_target, car_rest):
        self.timer = timer
        self.direction = 'none'
        self.car_target = car_target
        self.car_rest = car_rest
        self.x_mid = {"grandeur":0.0, "avante":0.0, "sonata":0.0}

    def __call__(self, detect, x_mid, y):
        '''
        returns direction to avoid obstacles from lidar inputs
        '''
        if (detect[self.car_target]): self.x_mid[self.car_target] = x_mid[self.car_target]
        if (detect[self.car_rest]): self.x_mid[self.car_rest] = x_mid[self.car_rest]

        if (y[self.car_target] > 30 or y[self.car_rest] > 30):
            if (self.x_mid[self.car_target] != 0 and self.x_mid[self.car_rest] != 0):
                if (self.x_mid[self.car_target] < self.x_mid[self.car_rest]):
                    self.direction = "left"
                elif (self.x_mid[self.car_target] >= self.x_mid[self.car_rest]):
                    self.direction = "right"
            elif (self.x_mid[self.car_target] == 0): # target 객체를 인식하지 못함
                if (self.x_mid[self.car_rest] < 320):
                    self.direction = "right"
                elif (self.x_mid[self.car_rest] >= 320):
                    self.direction = "left"
            elif (self.x_mid[self.car_rest] == 0): # rest 객체를 인식하지 못함
                if (self.x_mid[self.car_target] < 320):
                    self.direction = "left"
                elif (self.x_mid[self.car_target] >= 320):
                    self.direction = "right"
            
        rospy.loginfo("self.direction: %s, y[self.car_target]: %f", self.direction, y[self.car_target])
        return self.direction
