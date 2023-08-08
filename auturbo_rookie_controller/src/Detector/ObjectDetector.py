#! /usr/bin/env python
# -*- coding:utf-8 -*-

import numpy as np

class ObstacleParking(object):
    '''
    Detects obstacle using lidar data
    '''
    def __init__(self, timer):

        self.timer = timer

    def __call__(self, count, sum_x, specfic_car):
        '''
        returns direction to avoid obstacles from lidar inputs
        '''
        x = sum_x[specfic_car] / count[specfic_car]
        if (x >= 320):
            direction = "right"
        else:
            direction = "left"
        return direction
