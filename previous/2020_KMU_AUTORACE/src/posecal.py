
#!/usr/bin/env python

import numpy as np
import math
import cv2

class Pose:

    def __init__(self, x=0, y=0):
        self.pose = [x, y]

    def clear(self):
        self.pose = [0.0, 0.0]


    def calc_ahead(self, angle, speed):
        dt_x = ((speed) * math.sin(angle * np.pi / 180) * 0.01)
        dt_y = ((speed) * math.cos(angle * np.pi / 180) * 0.01)

        self.pose[0] += dt_x
        self.pose[1] -= dt_y

        return self.pose

    def calc_behind(self, angle, speed):
        if angle > 0:
            dt_x = -((speed) * math.sin(angle * np.pi / 180) * 0.01)
            dt_y = ((speed) * math.cos(angle * np.pi / 180) * 0.01)
        else:
            dt_x = ((speed) * math.sin(angle * np.pi / 180) * 0.01)
            dt_y = ((speed) * math.cos(angle * np.pi / 180) * 0.01)

        self.pose[0] += dt_x
        self.pose[1] -= dt_y

        return self.pose

    def get_curpose(self):
        return self.pose


    def get_midpoint(self, circle, MODE="LEFT"):
        if MODE == "LEFT":
            mid_pose = [circle.x + 0.25, circle.y]
        else:
            mid_pose = [circle.x - 0.25, circle.y]
        return mid_pose

    def get_goalpoint(self, circle, MODE="LEFT"):
        if MODE == "LEFT":
            goal_pose = [0, circle.y - 0.38]
        else:

            goal_pose = [0, circle.y - 0.38]

        return goal_pose



