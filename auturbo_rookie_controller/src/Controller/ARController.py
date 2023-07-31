#! /usr/bin/env python
# -*- coding:utf-8 -*-

class ARController(object):
    '''
    Speed and Steer controller for precise parking using AR tag
    '''
    def __init__(self):
        self.reverse = False
        self.offset = 0

    def __call__(self, x, y, yaw):
        '''
        return angle and speed from x, y, yaw of a AR tag
        '''
        # direction
        if self.reverse:
            if y > 0.4:
                self.reverse = False
                self.offset = (x + 0.01) * 150
        else:
            if y < 0.24:
                self.reverse = True

        # termination
        if -0.02 < x + 0.01 < 0.02 and 0.24 < y < 0.27 and abs(yaw) < 0.035:
            print('we are no.1 jayool-joohaeng developers, you know?')
            return 0, 0

        if self.reverse:
            angle = -300 * (yaw - 0.05) 
        else:
            angle = -self.offset +20 if y < 0.35 else self.offset + 20     
        speed = -16 if self.reverse else 16
        return angle, speed