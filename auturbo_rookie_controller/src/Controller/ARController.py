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
        if x != None and y != None and yaw != None:
            # direction
            print("x :{}".format(x))

            print("y :{}".format(y))

            print("yaw :{}".format(yaw))

            if self.reverse:
                if y > 0.4:
                    self.reverse = False
                    self.offset = (x + 0.01) * 150
            else:
                if y < 0.24:
                    self.reverse = True

            # termination
            if 0.05 < x + 0.25 < 0.02 and 0.44 < y < 0.60 and abs(yaw) < 0.1:
                print('ar 주차 성공')
                return 0, 0

            if self.reverse:
                angle = -300 * (yaw - 0.05) 
            else:
                angle = -self.offset +20 if y < 0.35 else self.offset + 20     
            speed = 4 if self.reverse else  0
            return int(angle), int(speed)
        else:
            print('no tag')
            return 0, 4