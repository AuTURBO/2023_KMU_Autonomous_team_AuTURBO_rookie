#!/usr/bin/env python
#-*- coding:utf-8 -*-

import enum
import math

class Position(enum.Enum):
    NO = 0
    LEFT = 1
    RIGHT = 2

class ObstacleDetector:
    def __init__(self):
        self.mode = Position.NO
        self.previous_time = 0
        self.cnt = 0

    # return EnumClass Position
    def check(self, obstacles):
        center = None
        d = None
        for circle in obstacles.circles:
            p = circle.center

            distance = math.sqrt(math.pow(p.x, 2) + math.pow(p.y, 2))

            if -0.6 < p.y < -0.1:
                d = distance
                center = p
                if abs(p.x) < 0.35:
                    if p.x > 0.08:
                        self.mode = Position.RIGHT
                    elif p.x < -0.08:
                        self.mode = Position.LEFT
                    self.cnt += 1
                    break
            else:
                self.mode = Position.NO


        return self.mode, center, d


    def wallcheck(self, obstacles):
        for circle in obstacles.circles:
            p = circle.center

            if -0.1 < p.y < 0.25 and 0.1 <= p.x < 0.6:
                return True

        return False

    def get_circle(self, obstacles):
        circle_parm = None
        for circle in obstacles.circles:
            p = circle.center
            if -0.8 < p.y <= 0.1 and abs(p.x) < 0.3:
                circle_parm = p
                break

        return circle_parm




obstacles = None

def obstacle_callback(data):
    global obstacles
    obstacles = data

