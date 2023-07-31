#! /usr/bin/env python
# -*- coding:utf-8 -*-

import numpy as np

class ObstacleDetector(object):
    '''
    Detects obstacle using lidar data
    '''
    def __init__(self, timer):

        self.avoid_direction = 'middle'
        self.obstacle_counter = 0
        self.timer = timer
        self.obs_dict = {1:1.0, 2:2.6, 3:4.0}

    def __call__(self, ranges, angle_increment):
        '''
        returns direction to avoid obstacles from lidar inputs
        '''

        if self.obstacle_counter == 0 and (self.timer() > 0.35):
            ranges = np.array(ranges)
            ranges[:len(ranges)//4] = 0.0
            ranges[3*len(ranges)//4:] = 0.0
            deg = np.arange(len(ranges)) * angle_increment - 252 * angle_increment
            mask = (np.abs(ranges * np.sin(deg)) < 0.4) & (0.2 < ranges * np.cos(deg)) & (ranges * np.cos(deg) < 0.4)
            filtered = np.where(mask, ranges, 0.0)
            nz = np.nonzero(filtered)[0]
            if len(nz) > 5:
                if np.median(nz) > len(ranges)/2:
                    self.avoid_direction = 'right'
                else:
                    self.avoid_direction = 'left'
                print('avoid to ' + self.avoid_direction)
                self.timer.update()
                self.obstacle_counter += 1

        elif self.obstacle_counter != 0:
            if self.timer() > self.obs_dict[self.obstacle_counter]:
                if self.obstacle_counter == 5:
                    self.avoid_direction = 'middle'                    
                else:
                    self.avoid_direction = 'left' if self.avoid_direction == 'right' else 'right'
                print('avoid to ' + self.avoid_direction)
                self.obstacle_counter += 1

        return self.avoid_direction
