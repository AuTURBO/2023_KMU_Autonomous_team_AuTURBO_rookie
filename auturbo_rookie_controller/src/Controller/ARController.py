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
            # print("x :{}".format(x))

            # print("y :{}".format(y))

            # print("yaw :{}".format(yaw))

            if self.reverse:
                if y > 0.8:
                    self.reverse = False
            else:
                if y < 0.66:
                    self.reverse = True
   #x  0.22868945620672587   y  0.9174609773533844  w  0.20812997531422267  id  1
    #x  0.1765239034700272   y  0.716902588854083  w  0.24787252483351582  id  1

    #x  0.11365149612910468   y  0.39251462070955173  w  0.11461349366373742  id  1

            # termination
            if 0.66 < y < 0.8:
                print('ar 주차 성공')
                return 0, 0

            speed = -3 if self.reverse else  3
            if speed == -3:
                return int(-8), int(speed)
            else:
                return int(8), int(speed)
        else:
            print('no tag')
            return 0, 0